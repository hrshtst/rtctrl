// Standalone CRANE-X7 bus emulator: serves the motor set from
// config/crane_x7.toml over a pseudo-terminal so any Dynamixel client
// (dxl_inspect, the bring-up apps, rt tools) can run without hardware.
//
// Usage: dxl_emu [--config config/crane_x7.toml] [--link /tmp/ttyDXL]
//                [--firmware N] [--arrival-log F]
//
// --firmware sets every motor's Firmware Version (default 44); 45 and
// later enable Drive Mode's Torque On by Goal Update.
//
// --arrival-log F (for tests) writes every instruction the emulator
// received, in order, as CSV at exit (created exclusively; exit status 2
// if it exists): arrival_ns (CLOCK_MONOTONIC, when the emulator extracted
// it), instruction, id, addr (a single read's or write's, or a grouped
// instruction's), data_length (a grouped read's), and for a grouped
// instruction its listed IDs (space-separated) and a Sync Write's data
// bytes (hex, each listed servo's in turn).
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
// options exit with status 2 before serving: times are bounded (1e18 ns)
// before they are scaled, then rounded to the nanosecond, and counts are
// at most 64 bits (bus/dxl_emu_cadence.hpp). A held reply is written in
// full or the run fails: if the pty does not accept all of its bytes (a
// client that stops reading fills it), dxl_emu says so and exits with
// status 1, the record keeping the bytes accepted and the error, the
// later replies unsent (emu::PtyBus::cadenceWriteFailure).

#include <sys/prctl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <toml++/toml.hpp>

#include "bus/dxl_emu_cadence.hpp"
#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"
#include "rtctrl/emu/standalone_loop.hpp"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

namespace cad = rtctrl::apps::dxl_emu_cadence;

void writeArrivalLog(std::FILE* f, const rtctrl::emu::PtyBus& pty) {
  std::fprintf(f, "arrival_ns,instruction,id,addr,data_length,ids,data\n");
  for (const auto& a : pty.arrivals()) {
    std::string ids, data;
    for (const std::uint8_t id : a.ids) {
      ids += (ids.empty() ? "" : " ") + std::to_string(id);
    }
    for (const std::uint8_t b : a.data) {
      char hex[3];
      std::snprintf(hex, sizeof hex, "%02x", b);
      data += hex;
    }
    std::fprintf(f, "%lld,%u,%u,%u,%u,%s,%s\n",
                 static_cast<long long>(
                     std::chrono::duration_cast<std::chrono::nanoseconds>(
                         a.at.time_since_epoch())
                         .count()),
                 a.instruction, a.id, a.addr, a.data_length, ids.c_str(),
                 data.c_str());
  }
}

void writeCadenceLog(std::FILE* f, const rtctrl::emu::PtyBus& pty) {
  std::fprintf(f,
               "arrival_ns,instruction,id,data_length,bytes,ready_ns,index,"
               "completion_ns,release_ns,delayed,missed,due_ns,written_ns,"
               "written_bytes,write_error\n");
  for (const auto& r : pty.cadenceRecords()) {
    std::fprintf(f,
                 "%lld,%u,%u,%u,%zu,%lld,%lld,%lld,%lld,%d,%u,%lld,%lld,%zu,"
                 "%d\n",
                 static_cast<long long>(r.arrival_ns), r.instruction, r.id,
                 r.data_length, r.bytes,
                 static_cast<long long>(r.release.ready_ns),
                 static_cast<long long>(r.release.index),
                 static_cast<long long>(r.release.completion_ns),
                 static_cast<long long>(r.release.release_ns),
                 r.release.delayed ? 1 : 0, r.release.missed,
                 static_cast<long long>(r.due_ns),
                 static_cast<long long>(r.written_ns), r.written_bytes,
                 r.write_error);
  }
}

// The exit line: the held replies' lateness at the write (written - due),
// the emulator's own wake precision, not the pty's; and the counts of
// replies written in full and not, the bus's rather than the record's, so
// that they hold beyond the record's capacity too.
void printCadenceSummary(const rtctrl::emu::PtyBus& pty) {
  std::vector<std::int64_t> late;
  std::uint64_t delayed = 0, missed = 0;
  for (const auto& r : pty.cadenceRecords()) {
    if (r.release.delayed) ++delayed;
    missed += r.release.missed;
    if (r.written_ns >= 0) late.push_back(r.written_ns - r.due_ns);
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
      "over, %llu written in full, %zu not at exit; written after due by "
      "p50 %.1f, p99 %.1f, max %.1f us\n",
      static_cast<unsigned long long>(pty.cadenceReplies()),
      pty.cadenceRecords().size(),
      static_cast<unsigned long long>(pty.cadenceRecordsDropped()),
      static_cast<unsigned long long>(delayed),
      static_cast<unsigned long long>(missed),
      static_cast<unsigned long long>(pty.cadenceWritten()),
      pty.cadencePending(), q(0.5), q(0.99), q(1.0));
}

// Why the run stopped, when a held reply was not written in full.
void printCadenceWriteFailure(const rtctrl::emu::CadenceWriteFailure& f,
                              std::size_t pending) {
  const std::string error =
      f.error > 0 ? std::strerror(f.error) : "a write accepted nothing";
  std::fprintf(stderr,
               "dxl_emu: cadence: held reply %llu (counted from 0) was not "
               "written in full: the pty accepted %zu of its %zu bytes (%s); "
               "the cadence cannot be kept, stopping with %zu later replies "
               "unsent\n",
               static_cast<unsigned long long>(f.reply), f.accepted, f.bytes,
               error.c_str(), pending > 0 ? pending - 1 : 0);
}
}  // namespace

int main(int argc, char* argv[]) {
  std::string config_path = "config/crane_x7.toml";
  std::string link_path;
  std::string arrival_log_path;
  int firmware = rtctrl::emu::MotorEmulator::Config{}.firmware_version;
  for (int i = 1; i < argc - 1; ++i) {
    if (std::strcmp(argv[i], "--config") == 0) config_path = argv[i + 1];
    if (std::strcmp(argv[i], "--link") == 0) link_path = argv[i + 1];
    if (std::strcmp(argv[i], "--arrival-log") == 0) {
      arrival_log_path = argv[i + 1];
    }
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
  cad::CadenceArgs cadence;
  {
    std::string why;
    if (!cad::parseCadence(argc, argv, &cadence, &why)) {
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

  std::FILE* arrival_log = nullptr;
  if (!arrival_log_path.empty()) {
    arrival_log = std::fopen(arrival_log_path.c_str(), "wx");
    if (arrival_log == nullptr) {
      std::fprintf(stderr, "dxl_emu: cannot create %s (it must not exist)\n",
                   arrival_log_path.c_str());
      if (cadence_log != nullptr) std::fclose(cadence_log);
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
  const auto write_failure = pty.cadenceWriteFailure();
  if (cadence.on) {
    printCadenceSummary(pty);
    if (write_failure) {
      printCadenceWriteFailure(*write_failure, pty.cadencePending());
    }
    if (cadence_log != nullptr) {
      writeCadenceLog(cadence_log, pty);
      if (std::fclose(cadence_log) != 0) {
        std::fprintf(stderr, "dxl_emu: writing %s failed\n",
                     cadence.log_path.c_str());
        return 1;
      }
    }
  }
  if (arrival_log != nullptr) {
    writeArrivalLog(arrival_log, pty);
    if (std::fclose(arrival_log) != 0) {
      std::fprintf(stderr, "dxl_emu: writing %s failed\n",
                   arrival_log_path.c_str());
      return 1;
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
      if (!write_failure) {  // else said above
        std::fprintf(stderr,
                     "dxl_emu: the pseudo-terminal failed; stopping\n");
      }
      return 1;
  }
  return 1;
}
