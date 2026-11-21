// Standalone CRANE-X7 bus emulator: serves the motor set from
// config/crane_x7.toml over a pseudo-terminal so any Dynamixel client
// (dxl_inspect, the bring-up apps, rt tools) can run without hardware.
//
// Usage: dxl_emu [--config config/crane_x7.toml] [--link /tmp/ttyDXL]
//                [--firmware N]
//
// --firmware sets every motor's Firmware Version (default 44); 45 and
// later enable Drive Mode's Torque On by Goal Update.
//
// Timing (emu::StandaloneLoop): input is read when it arrives, not on a
// polling sleep; the motor model ticks every 1 ms of wall time (steady
// clock), each step running its due ticks before the commands it reads.
// If the loop falls more than kMaxCatchUpTicks ticks behind (it was kept
// from running for longer than that), model time cannot be kept on wall
// time: dxl_emu says so and exits with status 3. SIGINT or SIGTERM stops
// it (status 0); a pty failure exits with status 1. At exit it prints its
// model time against the wall time since it started serving.
//
// Opt-in adapter cadence (emu::CompletionCadence; without --cadence the
// emulator answers as above, after the pty's own delays):
//   --cadence independent|request-tied
//   --cadence-ppm X         the adapter clock's drift on the host's (0)
//   --cadence-phase-us X    the first completion after the start (0)
//   --cadence-ready-us X    a reply is ready X us after its request (0)
//   --cadence-byte-ns X     plus X ns per reply byte (0)
//   --cadence-delay-every N --cadence-delay-us X
//                           every Nth completion delivers X us late
//   --cadence-miss-every N  every Nth completion delivers nothing (N >= 2)
//   --cadence-impair-from-s T
//                           delays and misses only from T s on (0)
//   --cadence-step T:S      from T s after the start, completions move by
//                           S us (repeatable, up to 16, in time order)
//   --cadence-tie-us X      request-tied: each reply X us after its request
//   --cadence-log F         every held reply's release, as CSV (created
//                           exclusively; written at exit)
// Independent: every reply waits for the first completion of a 1 ms
// lattice (drifting by --cadence-ppm) at or after its ready time;
// request-tied (a negative control) sends each reply a fixed delay after
// its request. Replies go in order. Times in the log are CLOCK_MONOTONIC
// nanoseconds. With --cadence the emulator's timer slack is 1 ns, so its
// releases are not deferred by the default 50 us slack. Invalid cadence
// options exit with status 2 before serving.

#include <sys/prctl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <toml++/toml.hpp>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"
#include "rtctrl/emu/standalone_loop.hpp"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

bool parseDouble(const char* text, double* out) {
  char* end = nullptr;
  const double v = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(v)) return false;
  *out = v;
  return true;
}

bool parseCount(const char* text, std::uint64_t* out) {
  char* end = nullptr;
  if (*text == '-') return false;
  const unsigned long long v = std::strtoull(text, &end, 10);
  if (end == text || *end != '\0') return false;
  *out = v;
  return true;
}

std::int64_t usToNs(double us) {
  return static_cast<std::int64_t>(std::llround(us * 1e3));
}

// The cadence options (all opt-in). Returns false with *why set if one is
// malformed or misplaced.
struct CadenceArgs {
  bool on = false;
  rtctrl::emu::CadenceConfig config;
  std::string log_path;
};

bool parseCadence(int argc, char* argv[], CadenceArgs* c, std::string* why) {
  bool any_option = false;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag.rfind("--cadence", 0) != 0) continue;
    if (i + 1 >= argc) {
      *why = flag + ": needs a value";
      return false;
    }
    const char* value = argv[++i];
    double x = 0.0;
    std::uint64_t n = 0;
    if (flag == "--cadence") {
      if (std::strcmp(value, "independent") == 0) {
        c->config.mode = rtctrl::emu::CadenceMode::kIndependent;
      } else if (std::strcmp(value, "request-tied") == 0) {
        c->config.mode = rtctrl::emu::CadenceMode::kRequestTied;
      } else {
        *why = "--cadence: independent or request-tied";
        return false;
      }
      c->on = true;
      continue;
    }
    any_option = true;
    if (flag == "--cadence-log") {
      c->log_path = value;
    } else if (flag == "--cadence-ppm" && parseDouble(value, &x)) {
      c->config.ppm = x;
    } else if (flag == "--cadence-phase-us" && parseDouble(value, &x)) {
      c->config.phase_ns = usToNs(x);
    } else if (flag == "--cadence-ready-us" && parseDouble(value, &x)) {
      c->config.ready_ns = usToNs(x);
    } else if (flag == "--cadence-byte-ns" && parseDouble(value, &x)) {
      c->config.byte_ns = static_cast<std::int64_t>(std::llround(x));
    } else if (flag == "--cadence-delay-every" && parseCount(value, &n)) {
      c->config.delay_every = n;
    } else if (flag == "--cadence-delay-us" && parseDouble(value, &x)) {
      c->config.delay_ns = usToNs(x);
    } else if (flag == "--cadence-miss-every" && parseCount(value, &n)) {
      c->config.miss_every = n;
    } else if (flag == "--cadence-impair-from-s" && parseDouble(value, &x)) {
      c->config.impair_from_ns = static_cast<std::int64_t>(std::llround(x * 1e9));
    } else if (flag == "--cadence-tie-us" && parseDouble(value, &x)) {
      c->config.tie_ns = usToNs(x);
    } else if (flag == "--cadence-step") {
      const std::string v = value;
      const auto colon = v.find(':');
      double at_s = 0.0, shift_us = 0.0;
      if (colon == std::string::npos ||
          !parseDouble(v.substr(0, colon).c_str(), &at_s) ||
          !parseDouble(v.substr(colon + 1).c_str(), &shift_us)) {
        *why = "--cadence-step: T:S (seconds after the start, a shift in us)";
        return false;
      }
      if (c->config.step_count >= rtctrl::emu::CadenceConfig::kMaxSteps) {
        *why = "--cadence-step: at most 16";
        return false;
      }
      c->config.steps[c->config.step_count++] = {
          static_cast<std::int64_t>(std::llround(at_s * 1e9)),
          usToNs(shift_us)};
    } else {
      *why = flag + ": unknown, or not a number: " + value;
      return false;
    }
  }
  if (any_option && !c->on) {
    *why = "the --cadence-* options need --cadence";
    return false;
  }
  if (c->on) {
    const std::string e = rtctrl::emu::cadenceConfigError(c->config);
    if (!e.empty()) {
      *why = "--cadence: " + e;
      return false;
    }
  }
  return true;
}

void writeCadenceLog(std::FILE* f, const rtctrl::emu::PtyBus& pty) {
  std::fprintf(f,
               "arrival_ns,instruction,id,data_length,bytes,ready_ns,index,"
               "completion_ns,release_ns,delayed,missed,due_ns,written_ns\n");
  for (const auto& r : pty.cadenceRecords()) {
    std::fprintf(f, "%lld,%u,%u,%u,%zu,%lld,%lld,%lld,%lld,%d,%u,%lld,%lld\n",
                 static_cast<long long>(r.arrival_ns), r.instruction, r.id,
                 r.data_length, r.bytes,
                 static_cast<long long>(r.release.ready_ns),
                 static_cast<long long>(r.release.index),
                 static_cast<long long>(r.release.completion_ns),
                 static_cast<long long>(r.release.release_ns),
                 r.release.delayed ? 1 : 0, r.release.missed,
                 static_cast<long long>(r.due_ns),
                 static_cast<long long>(r.written_ns));
  }
}

// The held replies' lateness at the write (written - due), for the exit
// line: the emulator's own wake precision, not the pty's.
void printCadenceSummary(const rtctrl::emu::PtyBus& pty) {
  std::vector<std::int64_t> late;
  std::uint64_t delayed = 0, missed = 0, unsent = 0;
  for (const auto& r : pty.cadenceRecords()) {
    if (r.release.delayed) ++delayed;
    missed += r.release.missed;
    if (r.written_ns < 0) {
      ++unsent;
      continue;
    }
    late.push_back(r.written_ns - r.due_ns);
  }
  std::sort(late.begin(), late.end());
  const auto q = [&](double p) {
    if (late.empty()) return 0.0;
    return static_cast<double>(
               late[static_cast<std::size_t>(p * static_cast<double>(late.size() - 1))]) /
           1e3;
  };
  std::printf(
      "dxl_emu: cadence: %llu replies held, %zu recorded (%llu beyond the "
      "record), %llu on delayed completions, %llu missed completions passed "
      "over, %llu unsent at exit; written after due by p50 %.1f, p99 %.1f, "
      "max %.1f us\n",
      static_cast<unsigned long long>(pty.cadenceReplies()),
      pty.cadenceRecords().size(),
      static_cast<unsigned long long>(pty.cadenceRecordsDropped()),
      static_cast<unsigned long long>(delayed),
      static_cast<unsigned long long>(missed),
      static_cast<unsigned long long>(unsent), q(0.5), q(0.99), q(1.0));
}
}  // namespace

int main(int argc, char* argv[]) {
  std::string config_path = "config/crane_x7.toml";
  std::string link_path;
  int firmware = rtctrl::emu::MotorEmulator::Config{}.firmware_version;
  for (int i = 1; i < argc - 1; ++i) {
    if (std::strcmp(argv[i], "--config") == 0) config_path = argv[i + 1];
    if (std::strcmp(argv[i], "--link") == 0) link_path = argv[i + 1];
    if (std::strcmp(argv[i], "--firmware") == 0) {
      char* end = nullptr;
      const long value = std::strtol(argv[i + 1], &end, 10);
      if (end == argv[i + 1] || *end != '\0' || value < 0 || value > 255) {
        std::fprintf(stderr, "--firmware: an integer in [0, 255]\n");
        return 2;
      }
      firmware = static_cast<int>(value);
    }
  }
  CadenceArgs cadence;
  {
    std::string why;
    if (!parseCadence(argc, argv, &cadence, &why)) {
      std::fprintf(stderr, "dxl_emu: %s\n", why.c_str());
      return 2;
    }
  }

  rtctrl::emu::MotorBus bus;
  try {
    const auto tbl = toml::parse_file(config_path);
    const auto* joints = tbl["joint"].as_array();
    if (joints == nullptr) throw std::runtime_error("no [[joint]] entries");
    for (const auto& entry : *joints) {
      const auto* joint = entry.as_table();
      rtctrl::emu::MotorEmulator::Config config;
      config.id = static_cast<std::uint8_t>(
          joint->at("id").value<int>().value());
      const auto model = joint->at("model").value<std::string>().value();
      config.model_number = (model == "XM540-W270")
                                ? rtctrl::dxl::kModelXm540W270
                                : rtctrl::dxl::kModelXm430W350;
      config.firmware_version = static_cast<std::uint8_t>(firmware);
      bus.add(config);
      std::printf("motor id %u (%s, firmware v%d)\n", config.id,
                  model.c_str(), firmware);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error reading %s: %s\n", config_path.c_str(),
                 e.what());
    return 1;
  }

  std::FILE* cadence_log = nullptr;
  if (!cadence.log_path.empty()) {
    cadence_log = std::fopen(cadence.log_path.c_str(), "wx");
    if (cadence_log == nullptr) {
      std::fprintf(stderr, "dxl_emu: cannot create %s (it must not exist)\n",
                   cadence.log_path.c_str());
      return 2;
    }
  }

  rtctrl::emu::PtyBus pty(bus);
  std::printf("serving on %s\n", pty.slavePath().c_str());
  if (!link_path.empty()) {
    unlink(link_path.c_str());
    if (symlink(pty.slavePath().c_str(), link_path.c_str()) == 0) {
      std::printf("linked as %s\n", link_path.c_str());
    }
  }
  std::fflush(stdout);

  // No SA_RESTART: a signal ends the loop's wait at once (EINTR).
  struct sigaction action {};
  action.sa_handler = onSignal;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);

  constexpr std::uint64_t kMaxCatchUpTicks = 50;  // 50 ms of model time
  rtctrl::emu::StandaloneLoop::Config loop_config;
  loop_config.tick = std::chrono::milliseconds(1);
  loop_config.max_catch_up_ticks = kMaxCatchUpTicks;
  loop_config.max_read_bytes = 4096;
  const auto start = rtctrl::emu::StandaloneLoop::Clock::now();
  if (cadence.on) {
    constexpr std::size_t kCadenceRecords = 1000000;
    pty.setCadence(cadence.config, start,
                   cadence_log != nullptr ? kCadenceRecords : 0);
    prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL);
    const auto& c = cadence.config;
    if (c.mode == rtctrl::emu::CadenceMode::kRequestTied) {
      std::printf("cadence: request-tied, each reply %.1f us after its "
                  "request\n",
                  static_cast<double>(c.tie_ns) / 1e3);
    } else {
      std::printf(
          "cadence: independent, period %.3f ns (%+.3f ppm), first "
          "completion %.1f us after the start, ready %.1f us + %lld ns per "
          "byte, delayed every %llu by %.1f us, missed every %llu (from %.3f "
          "s), %zu phase steps\n",
          rtctrl::emu::cadencePeriodNs(c), c.ppm,
          static_cast<double>(c.phase_ns) / 1e3,
          static_cast<double>(c.ready_ns) / 1e3,
          static_cast<long long>(c.byte_ns),
          static_cast<unsigned long long>(c.delay_every),
          static_cast<double>(c.delay_ns) / 1e3,
          static_cast<unsigned long long>(c.miss_every),
          static_cast<double>(c.impair_from_ns) / 1e9, c.step_count);
      for (std::size_t i = 0; i < c.step_count; ++i) {
        std::printf("cadence: step at %.6f s by %+.1f us\n",
                    static_cast<double>(c.steps[i].at_ns) / 1e9,
                    static_cast<double>(c.steps[i].shift_ns) / 1e3);
      }
    }
    std::printf("cadence: origin %lld ns (CLOCK_MONOTONIC)\n",
                static_cast<long long>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        start.time_since_epoch())
                        .count()));
    std::fflush(stdout);
  }
  rtctrl::emu::StandaloneLoop loop(pty, bus, loop_config, start);
  const auto status = loop.run([] { return g_stop != 0; });
  const double wall_s = std::chrono::duration<double>(
                            rtctrl::emu::StandaloneLoop::Clock::now() - start)
                            .count();
  if (!link_path.empty()) unlink(link_path.c_str());

  const auto& st = loop.stats();
  const double tick_s =
      std::chrono::duration<double>(loop_config.tick).count();
  std::printf(
      "dxl_emu: model time %.3f s (%llu ticks of %.3f ms) over %.3f s of "
      "wall time; steps %llu, of them late (more than one tick) %llu, most "
      "ticks at once %llu; reads at the %zu-byte bound %llu; waits by time "
      "alone (no client) %llu\n",
      static_cast<double>(st.ticks) * tick_s,
      static_cast<unsigned long long>(st.ticks), tick_s * 1e3, wall_s,
      static_cast<unsigned long long>(st.steps),
      static_cast<unsigned long long>(st.late_steps),
      static_cast<unsigned long long>(st.max_ticks_in_step),
      loop_config.max_read_bytes,
      static_cast<unsigned long long>(st.limited_reads),
      static_cast<unsigned long long>(st.hangup_waits));
  if (cadence.on) {
    printCadenceSummary(pty);
    if (cadence_log != nullptr) {
      writeCadenceLog(cadence_log, pty);
      if (std::fclose(cadence_log) != 0) {
        std::fprintf(stderr, "dxl_emu: writing %s failed\n",
                     cadence.log_path.c_str());
        return 1;
      }
    }
  }
  std::fflush(stdout);
  switch (status) {
    case rtctrl::emu::StandaloneLoop::Status::kOk:
      return 0;
    case rtctrl::emu::StandaloneLoop::Status::kOverrun:
      std::fprintf(stderr,
                   "dxl_emu: %llu model ticks were due at once, more than "
                   "the cap of %llu (%.0f ms): it was kept from running too "
                   "long to keep model time on wall time; stopping\n",
                   static_cast<unsigned long long>(st.overrun_due),
                   static_cast<unsigned long long>(kMaxCatchUpTicks),
                   static_cast<double>(kMaxCatchUpTicks) * tick_s * 1e3);
      return 3;
    case rtctrl::emu::StandaloneLoop::Status::kIoError:
      std::fprintf(stderr, "dxl_emu: the pseudo-terminal failed; stopping\n");
      return 1;
  }
  return 1;
}
