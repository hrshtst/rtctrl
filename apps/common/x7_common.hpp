// Shared plumbing for the x7_* bring-up apps: strict config/port CLI
// parsing, strict numeric argument helpers, a verified-shutdown guard,
// and a CraneX7 wired to silence the bus on deadman escalation.
#pragma once

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rtctrl/dxl/port.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

namespace x7 {

struct Cli {
  std::string config_path = "config/crane_x7.toml";
  std::string port_override;
  std::vector<const char*> rest;  // app-specific tokens, in argv order
  bool ok = true;                 // false: message already on stderr
};

// Order-independent and strict: --config/--port are consumed wherever
// they appear on the command line, and a flag missing its value is an
// error — never a silent positional (review finding: the old
// front-only parse let `x7_track 0.5 --port X` feed "--port" through
// atof into the scale). A token literally equal to --config/--port is
// always treated as the flag, never as another flag's value: in value
// position it means the value is missing, which is an error.
inline Cli parseCli(int argc, char* argv[]) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const bool is_config = std::strcmp(argv[i], "--config") == 0;
    const bool is_port = std::strcmp(argv[i], "--port") == 0;
    if (is_config || is_port) {
      const bool value_is_flag =
          i + 1 < argc && (std::strcmp(argv[i + 1], "--config") == 0 ||
                           std::strcmp(argv[i + 1], "--port") == 0);
      if (i + 1 >= argc || value_is_flag) {
        std::fprintf(stderr, "%s requires a value\n", argv[i]);
        cli.ok = false;
        return cli;
      }
      (is_config ? cli.config_path : cli.port_override) = argv[++i];
    } else {
      cli.rest.push_back(argv[i]);
    }
  }
  return cli;
}

// Strict numeric parsing for hardware-facing arguments: the whole token
// must parse and the value must be finite — "garbage" silently becoming
// 0 has produced minimum-scale runs and zero-gain writes.
inline bool parseStrictDouble(const char* text, double* out) {
  char* end = nullptr;
  const double v = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(v)) return false;
  *out = v;
  return true;
}

inline bool parseStrictLong(const char* text, long* out) {
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || errno == ERANGE) return false;
  *out = v;
  return true;
}

// Verified shutdown for every torque-enabling app: CraneX7 destruction
// deliberately does NOT torque off, so each post-activation exit must
// deactivate and CHECK the result. On an unclean (or throwing)
// deactivation: requestQuiesce() — bus silence makes the armed servo
// Bus Watchdogs halt any still-torqued joint — plus an operator fault
// message; the app must never print ordinary success over it. run() is
// idempotent and never throws; the destructor is the exception net for
// paths that unwind past an explicit call. Templated so tests can
// substitute a fake hardware type.
template <class Hw>
struct ShutdownGuardT {
  explicit ShutdownGuardT(Hw& h) : hw(h) {}
  ShutdownGuardT(const ShutdownGuardT&) = delete;
  ShutdownGuardT& operator=(const ShutdownGuardT&) = delete;

  Hw& hw;
  bool done = false;
  bool clean = false;

  bool run() {
    if (done) return clean;
    done = true;
    try {
      clean = hw.deactivate();
    } catch (...) {
      clean = false;  // a throw is an unclean shutdown
    }
    if (!clean) {
      hw.requestQuiesce();
      std::fprintf(stderr,
                   "SHUTDOWN FAULT: deactivation incomplete — bus "
                   "silenced, armed servo watchdogs will halt any "
                   "still-torqued joint. Verify the arm is limp "
                   "before approaching; power-cycle before the next "
                   "run.\n");
    }
    return clean;
  }

  ~ShutdownGuardT() {
    if (!done) run();
  }
};

using ShutdownGuard = ShutdownGuardT<rtctrl::hw::CraneX7>;

struct Session {
  rtctrl::hw::Config config;
  std::unique_ptr<rtctrl::dxl::Port> port;
  std::unique_ptr<rtctrl::hw::CraneX7> arm;
};

// operating_mode_override: -1 keeps the config's modes; 0/1/3/5 forces
// that mode on every joint (e.g. x7_float forces current mode).
// customize (optional) mutates the loaded config BEFORE the port
// opens (e.g. x7_gravity_demo writes its kt-derived command scales);
// the mutated config is re-validated, so a customization that breaks
// the reviewed bounds refuses before any bus contact — exactly like
// load() itself.
inline Session openSession(
    const Cli& cli, int operating_mode_override = -1,
    const std::function<void(rtctrl::hw::Config&)>& customize = {}) {
  Session s;
  s.config = rtctrl::hw::Config::load(cli.config_path);
  if (!cli.port_override.empty()) s.config.port = cli.port_override;
  if (operating_mode_override >= 0) {
    for (auto& joint : s.config.joints) {
      joint.operating_mode =
          static_cast<std::uint8_t>(operating_mode_override);
    }
  }
  if (customize) {
    customize(s.config);
    s.config.validate();
  }
  std::printf("bus: %s @ %d baud, %zu joints\n", s.config.port.c_str(),
              s.config.baudrate, s.config.joints.size());
  s.port = std::make_unique<rtctrl::dxl::Port>(s.config.port,
                                               s.config.baudrate);
  s.arm = std::make_unique<rtctrl::hw::CraneX7>(*s.port, s.config);
  // Deadman escalation ends with a silent bus so the servo-side
  // watchdogs are guaranteed to fire.
  rtctrl::dxl::Port* port = s.port.get();
  s.arm->onEscalate([port] {
    std::fprintf(stderr,
                 "DEADMAN: command stream stale — closing the bus; servo "
                 "watchdogs will halt the arm\n");
    port->close();
  });
  return s;
}

}  // namespace x7
