// x7_bus_timing: torque-off bus timing probe for the CRANE-X7.
//
// Replays one operating mode's exact per-cycle bus traffic (the grouped
// feedback read and the grouped goal write of the background thread) at a
// fixed rate while every servo is torque-OFF, and records host-side timing
// of each cycle. The arm never moves: the probe never writes Torque
// Enable or Drive Mode. Before any write it refuses unless every servo
// reports torque off AND Drive Mode bit 3 (Torque On by Goal Update)
// clear, since with that bit a goal write would switch torque on; it
// checks both again afterwards. It programs the indirect-address slots
// (the same layout activation programs, torque off) and writes only zero
// goal current, zero goal velocity and the goal position just read.
//
// Two kinds of run:
//  * production traffic (default): exactly the production read and write,
//    for sustained frequency, jitter, host read/write times and failures;
//  * calibration (--confirm-writes and/or --drain): adds an acknowledged
//    read of the last servo's goal after each write, and/or tcdrain(), to
//    study transport timing. Calibration traffic is not production
//    traffic: its cycle feasibility means nothing.
//
// Every stamp is a host observation (see rtctrl/hw/cycle_record.hpp). The
// acknowledged read gives a per-write upper bound on when the write
// finished on the wire: the acknowledging read cannot be sent before the
// write, and its reply cannot arrive before its own serialization, so
//   completion <= ack_return - ack_wire - return_delay(last servo).
//
// Usage:
//   x7_bus_timing [--config F] [--port P] --inventory [--out F.toml]
//   x7_bus_timing [--config F] [--port P] --rate-hz R
//       --profile current|position|cbp --duration S --log F.csv
//       [--summary F.toml] [--confirm-writes] [--drain]
//       [--rt-priority N --rt-cpu C] [--rt-lock-memory]
//       [--fine-timer-slack]
//
// Outputs are created exclusively (an existing file is refused). Exit 0
// means the measurement completed with complete outputs and torque off
// throughout; failures it measured (read failures, overruns) are data,
// not an exit status.

#include <fcntl.h>
#include <sys/utsname.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "common/x7_common.hpp"
#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/dxl/wire_budget.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/cycle_record.hpp"
#include "rtctrl/hw/cycle_timing.hpp"
#include "rtctrl/util/async_writer.hpp"
#include "rtctrl/util/realtime.hpp"
#include "rtctrl/util/spsc_ring.hpp"

namespace dxl = rtctrl::dxl;
namespace hw = rtctrl::hw;
namespace reg = rtctrl::dxl::reg;
namespace util = rtctrl::util;
namespace wire = rtctrl::dxl::wire;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {

enum class Profile { kCurrent, kPosition, kCurrentBasedPosition };

const char* profileName(Profile p) {
  switch (p) {
    case Profile::kCurrent: return "current";
    case Profile::kPosition: return "position";
    case Profile::kCurrentBasedPosition: return "cbp";
  }
  return "?";
}

int writeWidth(Profile p) {
  switch (p) {
    case Profile::kCurrent: return wire::kWriteWidthCurrent;
    case Profile::kPosition: return wire::kWriteWidthPosition;
    case Profile::kCurrentBasedPosition:
      return wire::kWriteWidthCurrentBasedPosition;
  }
  return 0;
}

struct Args {
  bool inventory = false;
  std::string out_path;
  double rate_hz = 0.0;
  double duration_s = 0.0;
  bool have_profile = false;
  Profile profile = Profile::kCurrent;
  std::string log_path;
  std::string summary_path;
  bool confirm_writes = false;
  bool drain = false;
  util::RealtimePolicy realtime;
};

bool parseArgs(const std::vector<const char*>& rest, Args* a) {
  for (std::size_t i = 0; i < rest.size(); ++i) {
    const std::string flag = rest[i];
    const auto value = [&](const char** out) {
      if (i + 1 >= rest.size()) {
        std::fprintf(stderr, "%s requires a value\n", flag.c_str());
        return false;
      }
      *out = rest[++i];
      return true;
    };
    const char* v = nullptr;
    if (flag == "--inventory") {
      a->inventory = true;
    } else if (flag == "--confirm-writes") {
      a->confirm_writes = true;
    } else if (flag == "--drain") {
      a->drain = true;
    } else if (flag == "--rt-lock-memory") {
      a->realtime.lock_memory = true;
    } else if (flag == "--fine-timer-slack") {
      a->realtime.fine_timer_slack = true;
    } else if (flag == "--out" || flag == "--log" || flag == "--summary") {
      if (!value(&v)) return false;
      (flag == "--out" ? a->out_path
                       : flag == "--log" ? a->log_path : a->summary_path) = v;
    } else if (flag == "--rate-hz" || flag == "--duration") {
      if (!value(&v)) return false;
      double d = 0.0;
      if (!x7::parseStrictDouble(v, &d)) {
        std::fprintf(stderr, "%s: not a number: %s\n", flag.c_str(), v);
        return false;
      }
      (flag == "--rate-hz" ? a->rate_hz : a->duration_s) = d;
    } else if (flag == "--rt-priority" || flag == "--rt-cpu") {
      if (!value(&v)) return false;
      long n = 0;
      if (!x7::parseStrictLong(v, &n)) {
        std::fprintf(stderr, "%s: not an integer: %s\n", flag.c_str(), v);
        return false;
      }
      (flag == "--rt-priority" ? a->realtime.priority : a->realtime.cpu) =
          static_cast<int>(n);
    } else if (flag == "--profile") {
      if (!value(&v)) return false;
      const std::string p = v;
      a->have_profile = true;
      if (p == "current") {
        a->profile = Profile::kCurrent;
      } else if (p == "position") {
        a->profile = Profile::kPosition;
      } else if (p == "cbp") {
        a->profile = Profile::kCurrentBasedPosition;
      } else {
        std::fprintf(stderr, "--profile: current, position or cbp\n");
        return false;
      }
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", flag.c_str());
      return false;
    }
  }
  if (a->inventory) return true;
  if (!(a->rate_hz >= 1.0 && a->rate_hz <= 2000.0)) {
    std::fprintf(stderr, "--rate-hz must be within [1, 2000]\n");
    return false;
  }
  if (!(a->duration_s > 0.0 && a->duration_s <= 3600.0)) {
    std::fprintf(stderr, "--duration must be within (0, 3600] s\n");
    return false;
  }
  if (!a->have_profile || a->log_path.empty()) {
    std::fprintf(stderr, "a timing run needs --profile and --log\n");
    return false;
  }
  if (a->summary_path.empty()) a->summary_path = a->log_path + ".summary.toml";
  const std::string why = util::validateRealtimePolicy(a->realtime);
  if (!why.empty()) {
    std::fprintf(stderr, "%s\n", why.c_str());
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Inventory

struct ServoInventory {
  int id = 0;
  std::uint16_t model = 0;
  int firmware = -1;
  int baud_register = -1;
  int return_delay_time = -1;  // 2 us units
  int drive_mode = -1;
  int status_return_level = -1;
  int operating_mode = -1;
  int torque_enable = -1;
  int bus_watchdog = -1;
  int hardware_error_status = -1;
};

struct HostInventory {
  std::string tty;
  std::string latency_timer;
  std::string usb_vendor;
  std::string usb_product_id;
  std::string usb_product;
  std::string usb_manufacturer;
  std::string usb_speed_mbps;
  std::string kernel;
  std::string cpu_governors;
};

std::string readSysfs(const fs::path& path) {
  std::ifstream in(path);
  std::string line;
  if (!in || !std::getline(in, line)) return "";
  while (!line.empty() && (line.back() == '\n' || line.back() == ' ')) {
    line.pop_back();
  }
  return line;
}

HostInventory hostInventory(const std::string& port) {
  HostInventory h;
  std::error_code ec;
  const fs::path dev = fs::canonical(port, ec);
  h.tty = ec ? fs::path(port).filename().string() : dev.filename().string();
  h.latency_timer =
      readSysfs("/sys/bus/usb-serial/devices/" + h.tty + "/latency_timer");
  fs::path usb = fs::canonical("/sys/class/tty/" + h.tty + "/device", ec);
  for (int up = 0; !ec && up < 6 && !usb.empty(); ++up) {
    if (fs::exists(usb / "idVendor")) {
      h.usb_vendor = readSysfs(usb / "idVendor");
      h.usb_product_id = readSysfs(usb / "idProduct");
      h.usb_product = readSysfs(usb / "product");
      h.usb_manufacturer = readSysfs(usb / "manufacturer");
      h.usb_speed_mbps = readSysfs(usb / "speed");
      break;
    }
    usb = usb.parent_path();
  }
  utsname u{};
  if (uname(&u) == 0) h.kernel = std::string(u.sysname) + " " + u.release;
  std::set<std::string> governors;
  for (int cpu = 0; cpu < 1024; ++cpu) {
    const std::string g =
        readSysfs("/sys/devices/system/cpu/cpu" + std::to_string(cpu) +
                  "/cpufreq/scaling_governor");
    if (g.empty()) {
      if (!fs::exists("/sys/devices/system/cpu/cpu" + std::to_string(cpu))) {
        break;
      }
      continue;
    }
    governors.insert(g);
  }
  for (const auto& g : governors) {
    h.cpu_governors += (h.cpu_governors.empty() ? "" : ",") + g;
  }
  return h;
}

bool servoInventory(dxl::PacketIO& io, const hw::Config& config,
                    std::vector<ServoInventory>* out, std::string* error) {
  out->clear();
  for (const auto& joint : config.joints) {
    ServoInventory s;
    s.id = joint.id;
    if (!io.ping(joint.id, &s.model).ok()) {
      *error = "no response from id " + std::to_string(joint.id);
      return false;
    }
    if (s.model != joint.model_number) {
      *error = "id " + std::to_string(joint.id) + " reports model " +
               std::to_string(s.model) + ", config expects " +
               std::to_string(joint.model_number);
      return false;
    }
    const auto read = [&](const dxl::Reg& r, int* value) {
      std::uint8_t b = 0;
      if (!io.read8(joint.id, r.addr, &b).ok()) return false;
      *value = b;
      return true;
    };
    if (!read(reg::kFirmwareVersion, &s.firmware) ||
        !read(reg::kBaudRate, &s.baud_register) ||
        !read(reg::kReturnDelayTime, &s.return_delay_time) ||
        !read(reg::kDriveMode, &s.drive_mode) ||
        !read(reg::kStatusReturnLevel, &s.status_return_level) ||
        !read(reg::kOperatingMode, &s.operating_mode) ||
        !read(reg::kTorqueEnable, &s.torque_enable) ||
        !read(reg::kBusWatchdog, &s.bus_watchdog) ||
        !read(reg::kHardwareErrorStatus, &s.hardware_error_status)) {
      *error = "cannot read the registers of id " + std::to_string(joint.id);
      return false;
    }
    out->push_back(s);
  }
  return true;
}

bool torqueOnByGoalUpdate(int drive_mode) {
  return drive_mode >= 0 &&
         (drive_mode & dxl::kDriveModeTorqueOnByGoalUpdate) != 0;
}

// The torque-off guarantee: every servo reports torque off, and none can
// switch torque on by itself when the probe writes a goal (Drive Mode bit
// 3 clear). The probe changes neither register.
bool torqueStaysOff(dxl::PacketIO& io, const hw::Config& config,
                    std::string* error) {
  for (const auto& joint : config.joints) {
    const std::string id = "id " + std::to_string(joint.id);
    std::uint8_t torque = 1;
    std::uint8_t drive_mode = 0xFF;
    if (!io.read8(joint.id, reg::kTorqueEnable.addr, &torque).ok()) {
      *error = "cannot read Torque Enable of " + id;
      return false;
    }
    if (torque != 0) {
      *error = id +
               " has torque enabled; the probe never changes torque "
               "(disable it with the arm supported, then rerun)";
      return false;
    }
    if (!io.read8(joint.id, reg::kDriveMode.addr, &drive_mode).ok()) {
      *error = "cannot read Drive Mode of " + id;
      return false;
    }
    if (torqueOnByGoalUpdate(drive_mode)) {
      *error = id + " has Drive Mode " + std::to_string(drive_mode) +
               " with bit 3 (Torque On by Goal Update) set: a goal write "
               "would enable torque. The probe never changes Drive Mode "
               "(clear bit 3 with torque off, then rerun)";
      return false;
    }
  }
  return true;
}

std::string tomlString(const std::string& s) {
  std::string out = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out + "\"";
}

void writeInventoryToml(std::FILE* f, const std::vector<ServoInventory>& s,
                        const HostInventory& h) {
  std::fprintf(f, "[host]\n");
  std::fprintf(f, "tty = %s\n", tomlString(h.tty).c_str());
  std::fprintf(f, "latency_timer = %s\n", tomlString(h.latency_timer).c_str());
  std::fprintf(f, "usb_vendor = %s\n", tomlString(h.usb_vendor).c_str());
  std::fprintf(f, "usb_product_id = %s\n",
               tomlString(h.usb_product_id).c_str());
  std::fprintf(f, "usb_product = %s\n", tomlString(h.usb_product).c_str());
  std::fprintf(f, "usb_manufacturer = %s\n",
               tomlString(h.usb_manufacturer).c_str());
  std::fprintf(f, "usb_speed_mbps = %s\n",
               tomlString(h.usb_speed_mbps).c_str());
  std::fprintf(f, "kernel = %s\n", tomlString(h.kernel).c_str());
  std::fprintf(f, "cpu_governors = %s\n", tomlString(h.cpu_governors).c_str());
  for (const auto& v : s) {
    std::fprintf(f, "\n[[servo]]\n");
    std::fprintf(f, "id = %d\nmodel = %u\nfirmware = %d\n", v.id,
                 static_cast<unsigned>(v.model), v.firmware);
    std::fprintf(f, "baud_register = %d\nreturn_delay_time = %d\n",
                 v.baud_register, v.return_delay_time);
    std::fprintf(f, "return_delay_us = %d\n", 2 * v.return_delay_time);
    std::fprintf(f, "drive_mode = %d\ntorque_on_by_goal_update = %s\n",
                 v.drive_mode,
                 torqueOnByGoalUpdate(v.drive_mode) ? "true" : "false");
    std::fprintf(f, "status_return_level = %d\noperating_mode = %d\n",
                 v.status_return_level, v.operating_mode);
    std::fprintf(f, "torque_enable = %d\nbus_watchdog = %d\n",
                 v.torque_enable, v.bus_watchdog);
    std::fprintf(f, "hardware_error_status = %d\n", v.hardware_error_status);
  }
}

void printInventory(const std::vector<ServoInventory>& s,
                    const HostInventory& h) {
  std::printf("host: tty %s, latency_timer %s, usb %s:%s \"%s\" (%s), "
              "%s Mbit/s\n",
              h.tty.c_str(),
              h.latency_timer.empty() ? "n/a" : h.latency_timer.c_str(),
              h.usb_vendor.c_str(), h.usb_product_id.c_str(),
              h.usb_product.c_str(), h.usb_manufacturer.c_str(),
              h.usb_speed_mbps.c_str());
  std::printf("host: %s, cpu governors %s\n", h.kernel.c_str(),
              h.cpu_governors.empty() ? "n/a" : h.cpu_governors.c_str());
  for (const auto& v : s) {
    std::printf("id %d: model %u fw v%d baud_reg %d return_delay %d "
                "(%d us) drive_mode %d status_return %d mode %d torque %d "
                "bus_watchdog %d hw_error %d\n",
                v.id, static_cast<unsigned>(v.model), v.firmware,
                v.baud_register, v.return_delay_time,
                2 * v.return_delay_time, v.drive_mode,
                v.status_return_level, v.operating_mode, v.torque_enable,
                v.bus_watchdog, v.hardware_error_status);
  }
  for (const auto& v : s) {
    if (torqueOnByGoalUpdate(v.drive_mode)) {
      std::printf("warning: id %d has Torque On by Goal Update (Drive Mode "
                  "bit 3): a goal write enables its torque; timing runs "
                  "refuse it\n",
                  v.id);
    }
  }
}

std::FILE* createExclusive(const std::string& path) {
  std::FILE* f = std::fopen(path.c_str(), "wx");
  if (f == nullptr) {
    std::fprintf(stderr, "cannot create %s (it must not exist): %s\n",
                 path.c_str(), std::strerror(errno));
  }
  return f;
}

// ---------------------------------------------------------------------------
// Timing run

struct ProbeRecord {
  hw::CycleRecord cycle;
  std::int64_t drain_return_ns = -1;
  std::int64_t ack_return_ns = -1;
  std::int64_t completion_upper_ns = -1;  // C+: bound on wire completion
  bool ack_ok = false;
};

constexpr const char* kCsvHeader =
    "cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,"
    "first_reply_ns,last_reply_ns,read_end_ns,limit_start_ns,"
    "bus_write_start_ns,write_return_ns,drain_return_ns,ack_return_ns,"
    "completion_upper_ns,cycle_end_ns,read_ok,write_ok,ack_ok,"
    "skipped_periods";

struct RunResult {
  bool started = false;  // the real-time policy was granted
  std::string error;
  std::string scheduler;
  std::uint64_t cycles = 0;
  std::uint64_t skipped_periods = 0;
  std::uint64_t overruns = 0;
  std::uint64_t read_failures = 0;
  std::uint64_t write_failures = 0;
  std::uint64_t ack_mismatches = 0;
  std::uint64_t ring_drops = 0;
  double first_wake_s = 0.0;
  double last_wake_s = 0.0;
  double cpu_s = 0.0;
  double wall_s = 0.0;
  hw::CycleTimingStats timing;
  hw::LatencyHistogram completion_offset;  // C+ - bus_write_start
  hw::LatencyHistogram drain_offset;       // drain_return - bus_write_start
  hw::LatencyHistogram ack_round_trip;     // write_return -> ack_return
};

double threadCpuSeconds() {
  timespec ts{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return static_cast<double>(ts.tv_sec) +
         1e-9 * static_cast<double>(ts.tv_nsec);
}

class Probe {
 public:
  Probe(dxl::PacketIO& io, const hw::Config& config, const Args& args,
        int baudrate, int last_servo_return_delay, int drain_fd)
      : io_(io),
        group_(io, ids(config)),
        args_(args),
        baudrate_(baudrate),
        last_return_delay_us_(2 * last_servo_return_delay),
        drain_fd_(drain_fd),
        ring_(std::max<std::size_t>(
            4096, static_cast<std::size_t>(args.rate_hz * 4.0))) {}

  util::SpscRing<ProbeRecord>& ring() { return ring_; }

  // Runs on the probe thread.
  void run(const std::vector<dxl::Feedback>& initial, RunResult* result,
           std::atomic<bool>* done) {
    std::string why;
    if (!util::applyRealtimePolicy(args_.realtime, &why)) {
      result->error = "real-time policy refused: " + why;
      done->store(true);
      return;
    }
    result->started = true;
    result->scheduler =
        util::describeRealtimeState(util::currentRealtimeState());

    const std::size_t n = initial.size();
    std::vector<dxl::Feedback> fb = initial;
    std::vector<double> zeros(n, 0.0);
    std::vector<double> positions(n, 0.0);
    std::vector<double> currents(n, 0.0);
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / args_.rate_hz));
    const auto cycles = static_cast<std::uint64_t>(
        std::ceil(args_.duration_s * args_.rate_hz));
    const std::uint8_t last_id = group_.ids().back();
    const double byte_s = wire::byteTimeS(baudrate_);
    const int ack_len = args_.profile == Profile::kCurrent ? 2 : 4;
    const std::uint16_t ack_addr = args_.profile == Profile::kCurrent
                                       ? reg::kGoalCurrent.addr
                                       : reg::kGoalPosition.addr;
    const auto ack_wire_ns = static_cast<std::int64_t>(std::llround(
        (wire::readInstructionBytes() + wire::readReplyBytes(ack_len)) *
            byte_s * 1e9 +
        last_return_delay_us_ * 1e3));

    const auto epoch = Clock::now();
    const auto ns = [&](Clock::time_point t) {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(t - epoch)
          .count();
    };
    hw::PeriodicSchedule schedule(epoch, period);
    auto scheduled = epoch;
    std::int64_t prev_wake = -1;
    const double cpu0 = threadCpuSeconds();
    for (std::uint64_t k = 1; k <= cycles; ++k) {
      ProbeRecord r;
      hw::CycleRecord& c = r.cycle;
      c.cycle = k;
      c.command_kind = hw::CommandKind::kPeriodic;
      c.scheduled_wake_ns = ns(scheduled);
      c.wake_ns = ns(Clock::now());
      if (k == 1) result->first_wake_s = c.wake_ns * 1e-9;
      result->last_wake_s = c.wake_ns * 1e-9;

      c.read_start_ns = ns(Clock::now());
      c.read_ok = group_.readAll(fb).ok();
      c.read_end_ns = ns(Clock::now());
      const dxl::SyncReadTiming t = io_.lastSyncReadTiming();
      if (t.sent) c.instr_return_ns = ns(t.instruction_sent);
      if (t.replies > 0) {
        c.first_reply_ns = ns(t.first_reply);
        c.last_reply_ns = ns(t.last_reply);
      }
      if (!c.read_ok) ++result->read_failures;

      // Calibration writes alternate a harmless one-unit offset (torque is
      // off) so the acknowledging read can tell this write from the last.
      const bool odd = args_.confirm_writes && (k % 2 == 1);
      for (std::size_t i = 0; i < n; ++i) {
        positions[i] = fb[i].position +
                       (odd ? 2.0 * M_PI / dxl::kPulsesPerRevolution : 0.0);
        currents[i] = odd ? dxl::kCurrentUnitAmps : 0.0;
      }
      c.limit_start_ns = ns(Clock::now());
      c.bus_write_start_ns = ns(Clock::now());
      dxl::IoResult w;
      switch (args_.profile) {
        case Profile::kCurrent: w = group_.writeGoalCurrents(currents); break;
        case Profile::kPosition:
          w = group_.writeGoalPositions(positions);
          break;
        case Profile::kCurrentBasedPosition:
          w = group_.writeGoals(currents, zeros, positions);
          break;
      }
      c.write_return_ns = ns(Clock::now());
      c.write_ok = w.ok();
      if (!c.write_ok) ++result->write_failures;

      if (args_.drain && drain_fd_ >= 0) {
        tcdrain(drain_fd_);
        r.drain_return_ns = ns(Clock::now());
        result->drain_offset.add((r.drain_return_ns - c.bus_write_start_ns) *
                                 1e-9);
      }
      if (args_.confirm_writes) {
        std::uint8_t buf[4] = {};
        const bool read_ok = io_.read(last_id, ack_addr, buf, ack_len).ok();
        r.ack_return_ns = ns(Clock::now());
        std::uint32_t got = buf[0] | (buf[1] << 8);
        std::uint32_t want =
            static_cast<std::uint16_t>(dxl::ampsToCurrent(currents.back()));
        if (ack_len == 4) {
          got |= (static_cast<std::uint32_t>(buf[2]) << 16) |
                 (static_cast<std::uint32_t>(buf[3]) << 24);
          want = static_cast<std::uint32_t>(dxl::radToPulse(positions.back()));
        }
        r.ack_ok = read_ok && c.write_ok && got == want;
        if (r.ack_ok) {
          r.completion_upper_ns = r.ack_return_ns - ack_wire_ns;
          result->completion_offset.add(
              (r.completion_upper_ns - c.bus_write_start_ns) * 1e-9);
          result->ack_round_trip.add(
              (r.ack_return_ns - c.write_return_ns) * 1e-9);
        } else {
          ++result->ack_mismatches;
        }
      }

      const auto finished = Clock::now();
      c.cycle_end_ns = ns(finished);
      const hw::DeadlineAdvance advance = schedule.closeIteration(finished);
      c.skipped_periods = static_cast<std::uint32_t>(advance.skipped_periods);
      result->skipped_periods += advance.skipped_periods;
      if (advance.skipped_periods > 0) ++result->overruns;
      hw::accumulateCycleTiming(c, prev_wake, &result->timing);
      prev_wake = c.wake_ns;
      if (!ring_.push(r)) ++result->ring_drops;
      ++result->cycles;
      scheduled = advance.wake_time;
      std::this_thread::sleep_until(advance.wake_time);
    }
    result->cpu_s = threadCpuSeconds() - cpu0;
    result->wall_s =
        std::chrono::duration<double>(Clock::now() - epoch).count();
    done->store(true);
  }

 private:
  static std::vector<std::uint8_t> ids(const hw::Config& config) {
    std::vector<std::uint8_t> out;
    for (const auto& j : config.joints) out.push_back(j.id);
    return out;
  }

  dxl::PacketIO& io_;
  dxl::SyncGroup group_;
  const Args& args_;
  int baudrate_;
  int last_return_delay_us_;
  int drain_fd_;
  util::SpscRing<ProbeRecord> ring_;
};

void appendRow(util::AsyncWriter& log, const ProbeRecord& r) {
  const hw::CycleRecord& c = r.cycle;
  log.appendf("%llu,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,"
              "%lld,%lld,%lld,%lld,%d,%d,%d,%u\n",
              static_cast<unsigned long long>(c.cycle),
              static_cast<long long>(c.scheduled_wake_ns),
              static_cast<long long>(c.wake_ns),
              static_cast<long long>(c.read_start_ns),
              static_cast<long long>(c.instr_return_ns),
              static_cast<long long>(c.first_reply_ns),
              static_cast<long long>(c.last_reply_ns),
              static_cast<long long>(c.read_end_ns),
              static_cast<long long>(c.limit_start_ns),
              static_cast<long long>(c.bus_write_start_ns),
              static_cast<long long>(c.write_return_ns),
              static_cast<long long>(r.drain_return_ns),
              static_cast<long long>(r.ack_return_ns),
              static_cast<long long>(r.completion_upper_ns),
              static_cast<long long>(c.cycle_end_ns), c.read_ok ? 1 : 0,
              c.write_ok ? 1 : 0, r.ack_ok ? 1 : 0,
              static_cast<unsigned>(c.skipped_periods));
}

void writeHistogram(std::FILE* f, const char* name,
                    const hw::LatencyHistogram& h) {
  std::fprintf(f, "\n[histogram.%s]\n", name);
  std::fprintf(f, "count = %llu\n", static_cast<unsigned long long>(h.count()));
  // Bin upper edges: never below the empirical quantile, no statistical
  // claim beyond the recorded sample.
  std::fprintf(f, "p50_upper_edge_s = %.9g\n", h.quantileUpperEdge(0.5));
  std::fprintf(f, "p99_upper_edge_s = %.9g\n", h.quantileUpperEdge(0.99));
  std::fprintf(f, "p999_upper_edge_s = %.9g\n", h.quantileUpperEdge(0.999));
  std::fprintf(f, "max_s = %.9g\n", h.max());
}

void printHistogram(const char* name, const hw::LatencyHistogram& h) {
  if (h.count() == 0) return;
  std::printf("  %-18s n=%-8llu p50<=%8.3f p99<=%8.3f p99.9<=%8.3f "
              "max=%8.3f ms\n",
              name, static_cast<unsigned long long>(h.count()),
              1e3 * h.quantileUpperEdge(0.5), 1e3 * h.quantileUpperEdge(0.99),
              1e3 * h.quantileUpperEdge(0.999), 1e3 * h.max());
}

}  // namespace

int main(int argc, char* argv[]) {
  const x7::Cli cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 2;
  Args args;
  if (!parseArgs(cli.rest, &args)) return 2;

  hw::Config config;
  try {
    config = hw::Config::load(cli.config_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "config: %s\n", e.what());
    return 2;
  }
  const std::string port_path =
      cli.port_override.empty() ? config.port : cli.port_override;

  std::unique_ptr<dxl::Port> port;
  try {
    port = std::make_unique<dxl::Port>(port_path, config.baudrate);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }

  std::vector<ServoInventory> servos;
  std::string error;
  if (!servoInventory(*port, config, &servos, &error)) {
    std::fprintf(stderr, "inventory: %s\n", error.c_str());
    return 1;
  }
  const HostInventory host = hostInventory(port_path);
  printInventory(servos, host);

  if (args.inventory) {
    if (args.out_path.empty()) return 0;
    std::FILE* f = createExclusive(args.out_path);
    if (f == nullptr) return 1;
    std::fprintf(f, "format = \"rtctrl-bus-inventory\"\nversion = 1\n");
    std::fprintf(f, "port = %s\nbaudrate = %d\n\n",
                 tomlString(port_path).c_str(), config.baudrate);
    writeInventoryToml(f, servos, host);
    const bool ok = std::fflush(f) == 0 && std::ferror(f) == 0;
    if (std::fclose(f) != 0 || !ok) {
      std::fprintf(stderr, "cannot finish %s\n", args.out_path.c_str());
      return 1;
    }
    return 0;
  }

  if (!torqueStaysOff(*port, config, &error)) {
    std::fprintf(stderr, "refusing: %s\n", error.c_str());
    return 1;
  }
  int drain_fd = -1;
  if (args.drain) {
    drain_fd = open(port_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (drain_fd < 0) {
      std::fprintf(stderr, "cannot open %s for tcdrain: %s\n",
                   port_path.c_str(), std::strerror(errno));
      return 1;
    }
  }
  const auto close_drain = [&] {
    if (drain_fd >= 0) close(drain_fd);
    drain_fd = -1;
  };

  dxl::SyncGroup group(*port, [&] {
    std::vector<std::uint8_t> ids;
    for (const auto& j : config.joints) ids.push_back(j.id);
    return ids;
  }());
  if (!group.setupIndirect().ok()) {
    std::fprintf(stderr, "cannot program the indirect-address slots\n");
    close_drain();
    return 1;
  }
  std::vector<dxl::Feedback> initial;
  if (!group.readAll(initial).ok()) {
    std::fprintf(stderr, "initial feedback read failed\n");
    close_drain();
    return 1;
  }

  std::unique_ptr<util::AsyncWriter> log;
  try {
    util::AsyncWriter::Options log_options;
    log_options.capacity_bytes = std::max<std::size_t>(
        log_options.capacity_bytes,
        static_cast<std::size_t>(30.0 * args.rate_hz * 400.0));
    log = std::make_unique<util::AsyncWriter>(args.log_path, log_options);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    close_drain();
    return 1;
  }
  std::FILE* summary = createExclusive(args.summary_path);
  if (summary == nullptr) {
    close_drain();
    return 1;
  }
  log->append(kCsvHeader);
  log->append("\n");
  log->commitRow();

  const bool calibration = args.confirm_writes || args.drain;
  const auto bytes = wire::cycleBytes(
      static_cast<int>(config.joints.size()), wire::ReadVariant::kSync,
      wire::kFeedbackBytes, writeWidth(args.profile));
  const double wire_s =
      bytes.total(true) * wire::byteTimeS(config.baudrate);
  std::printf("%s run: profile %s, %.1f Hz, %.1f s; analytic wire time "
              "%.3f ms (worst-case stuffing) = %.0f%% of the period\n",
              calibration ? "calibration" : "production-traffic",
              profileName(args.profile), args.rate_hz, args.duration_s,
              1e3 * wire_s, 100.0 * wire_s * args.rate_hz);

  Probe probe(*port, config, args, config.baudrate,
              servos.back().return_delay_time, drain_fd);
  RunResult result;
  std::atomic<bool> done{false};
  std::thread worker([&] { probe.run(initial, &result, &done); });
  // Drain records off the probe thread into the log. `done` is read
  // before draining: every record pushed before it was set is then
  // visible, so the final pass misses nothing.
  ProbeRecord r;
  for (;;) {
    const bool finished = done.load();
    bool any = false;
    while (probe.ring().pop(&r)) {
      appendRow(*log, r);
      log->commitRow();
      any = true;
    }
    if (finished) break;
    if (!any) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  worker.join();
  close_drain();
  const bool log_complete = log->close();

  std::string after_error;
  const bool torque_off_after =
      torqueStaysOff(*port, config, &after_error);
  if (!torque_off_after) {
    std::fprintf(stderr, "TORQUE CHECK FAILED after the run: %s\n",
                 after_error.c_str());
  }

  const double span = result.last_wake_s - result.first_wake_s;
  const double mean_hz =
      result.cycles > 1 && span > 0.0 ? (result.cycles - 1) / span : 0.0;
  std::fprintf(summary, "format = \"rtctrl-bus-timing\"\nversion = 1\n");
  std::fprintf(summary, "kind = \"%s\"\n",
               calibration ? "calibration" : "production-traffic");
  std::fprintf(summary, "profile = \"%s\"\n", profileName(args.profile));
  std::fprintf(summary, "rate_hz = %.9g\nduration_s = %.9g\n", args.rate_hz,
               args.duration_s);
  std::fprintf(summary, "confirm_writes = %s\ndrain = %s\n",
               args.confirm_writes ? "true" : "false",
               args.drain ? "true" : "false");
  std::fprintf(summary, "port = %s\nbaudrate = %d\n",
               tomlString(port_path).c_str(), config.baudrate);
  std::fprintf(summary, "log = %s\nlog_complete = %s\n",
               tomlString(args.log_path).c_str(),
               log_complete ? "true" : "false");
  std::fprintf(summary, "scheduler = %s\n",
               tomlString(result.scheduler).c_str());
  std::fprintf(summary, "\n[result]\n");
  std::fprintf(summary, "started = %s\n", result.started ? "true" : "false");
  std::fprintf(summary, "error = %s\n", tomlString(result.error).c_str());
  std::fprintf(summary, "cycles = %llu\n",
               static_cast<unsigned long long>(result.cycles));
  std::fprintf(summary, "overruns = %llu\nskipped_periods = %llu\n",
               static_cast<unsigned long long>(result.overruns),
               static_cast<unsigned long long>(result.skipped_periods));
  std::fprintf(summary, "read_failures = %llu\nwrite_failures = %llu\n",
               static_cast<unsigned long long>(result.read_failures),
               static_cast<unsigned long long>(result.write_failures));
  std::fprintf(summary, "ack_mismatches = %llu\nring_drops = %llu\n",
               static_cast<unsigned long long>(result.ack_mismatches),
               static_cast<unsigned long long>(result.ring_drops));
  std::fprintf(summary, "mean_frequency_hz = %.9g\n", mean_hz);
  std::fprintf(summary, "cpu_duty = %.6g\n",
               result.wall_s > 0.0 ? result.cpu_s / result.wall_s : 0.0);
  std::fprintf(summary,
               "# torque off and Drive Mode bit 3 (Torque On by Goal "
               "Update) clear on every servo\n");
  std::fprintf(summary, "torque_off_before = true\ntorque_off_after = %s\n",
               torque_off_after ? "true" : "false");
  std::fprintf(summary, "\n[budget]\n");
  std::fprintf(summary,
               "# Analytic wire bytes of the production cycle; excludes USB "
               "and adapter latency,\n# Return Delay Time, servo "
               "processing and scheduling.\n");
  std::fprintf(summary, "read_instruction_bytes = %d\n",
               bytes.read_instruction);
  std::fprintf(summary, "read_reply_bytes = %d\n", bytes.read_reply);
  std::fprintf(summary, "read_stuffing_max_bytes = %d\n",
               bytes.read_stuffing_max);
  std::fprintf(summary, "write_bytes = %d\n", bytes.write);
  std::fprintf(summary, "wire_time_worst_s = %.9g\n", wire_s);
  std::fprintf(summary, "wire_fraction_of_period = %.6g\n",
               wire_s * args.rate_hz);
  const auto& t = result.timing;
  writeHistogram(summary, "wake_lateness", t.wake_lateness);
  writeHistogram(summary, "period", t.period);
  writeHistogram(summary, "host_transaction", t.host_transaction);
  writeHistogram(summary, "reply_span", t.reply_span);
  writeHistogram(summary, "limiter", t.limiter);
  writeHistogram(summary, "host_write", t.host_write);
  writeHistogram(summary, "post_write", t.post_write);
  writeHistogram(summary, "host_work", t.host_work);
  writeHistogram(summary, "completion_offset", result.completion_offset);
  writeHistogram(summary, "drain_offset", result.drain_offset);
  writeHistogram(summary, "ack_round_trip", result.ack_round_trip);
  std::fprintf(summary, "\n");
  writeInventoryToml(summary, servos, host);
  const bool summary_ok = std::fflush(summary) == 0 && !std::ferror(summary);
  const bool summary_closed = std::fclose(summary) == 0;

  std::printf("scheduler: %s\n", result.scheduler.c_str());
  std::printf("cycles %llu, mean %.3f Hz, overruns %llu, skipped %llu, "
              "read failures %llu, write failures %llu",
              static_cast<unsigned long long>(result.cycles), mean_hz,
              static_cast<unsigned long long>(result.overruns),
              static_cast<unsigned long long>(result.skipped_periods),
              static_cast<unsigned long long>(result.read_failures),
              static_cast<unsigned long long>(result.write_failures));
  if (args.confirm_writes) {
    std::printf(", ack mismatches %llu",
                static_cast<unsigned long long>(result.ack_mismatches));
  }
  std::printf("\n");
  printHistogram("wake_lateness", t.wake_lateness);
  printHistogram("period", t.period);
  printHistogram("host_transaction", t.host_transaction);
  printHistogram("host_write", t.host_write);
  printHistogram("host_work", t.host_work);
  printHistogram("completion_offset", result.completion_offset);
  printHistogram("drain_offset", result.drain_offset);

  if (!result.started) {
    std::fprintf(stderr, "%s\n", result.error.c_str());
    return 1;
  }
  if (!log_complete || result.ring_drops > 0) {
    std::fprintf(stderr, "LOG INCOMPLETE: %s\n", args.log_path.c_str());
    return 1;
  }
  if (!summary_ok || !summary_closed) {
    std::fprintf(stderr, "cannot finish %s\n", args.summary_path.c_str());
    return 1;
  }
  return torque_off_after ? 0 : 1;
}
