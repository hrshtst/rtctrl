#pragma once

// dxl_emu's opt-in cadence options (--cadence and --cadence-*, listed in
// dxl_emu.cpp), parsed into an emu::CadenceConfig. Header-only so that its
// refusals are unit-tested (tests/unit/dxl_emu_cadence_test.cpp).
//
// Every number is checked before it is used:
// - a real (strtod's syntax, the whole argument) is refused when it is not
//   a number (nan included), or out of range: infinite, or beyond a double
//   (ERANGE, on overflow or underflow);
// - a time is refused as out of range unless its magnitude is at most
//   kMaxNs nanoseconds, checked on the value in its own unit before it is
//   scaled; only then is it scaled and rounded to the nearest nanosecond,
//   and a nonzero time that would round to 0 is refused (finer than one
//   nanosecond);
// - a count (decimal digits only: no sign, no space) is refused as out of
//   range when it exceeds 64 bits (ERANGE).
// The supported ranges are then the model's (emu::cadenceConfigError), on
// the nanoseconds it will run with.

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/emu/completion_cadence.hpp"

namespace rtctrl::apps::dxl_emu_cadence {

// The largest time magnitude an option may give: 1e18 ns (about 31.7
// years), well inside int64 nanoseconds and beyond every supported range.
inline constexpr double kMaxNs = 1e18;

enum class Parsed : std::uint8_t {
  kOk,
  kNotNumber,    // a real that is not one
  kNotCount,     // a count that is not decimal digits
  kOutOfRange,   // not representable as given (see above)
  kFinerThanNs,  // a nonzero time that rounds to 0 ns
};

// A finite real, the whole of text.
inline Parsed parseReal(const char* text, double* out) {
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(text, &end);
  if (end == text || *end != '\0' || std::isnan(v)) return Parsed::kNotNumber;
  if (errno == ERANGE || std::isinf(v)) return Parsed::kOutOfRange;
  *out = v;
  return Parsed::kOk;
}

// A time given in units of `scale` nanoseconds (1: ns, 1e3: us, 1e9: s), as
// nanoseconds; its magnitude is bounded before it is scaled.
inline Parsed parseNs(const char* text, double scale, std::int64_t* out) {
  double v = 0.0;
  const Parsed p = parseReal(text, &v);
  if (p != Parsed::kOk) return p;
  if (!(std::fabs(v) <= kMaxNs / scale)) return Parsed::kOutOfRange;
  const std::int64_t ns = std::llround(v * scale);  // |v * scale| <= 1e18
  if (ns == 0 && v != 0.0) return Parsed::kFinerThanNs;
  *out = ns;
  return Parsed::kOk;
}

// A count: decimal digits, at most 64 bits.
inline Parsed parseCount(const char* text, std::uint64_t* out) {
  static_assert(sizeof(unsigned long long) == sizeof(std::uint64_t), "strtoull is 64-bit");
  if (!std::isdigit(static_cast<unsigned char>(*text))) return Parsed::kNotCount;
  char* end = nullptr;
  errno = 0;
  const unsigned long long v = std::strtoull(text, &end, 10);
  if (*end != '\0') return Parsed::kNotCount;
  if (errno == ERANGE) return Parsed::kOutOfRange;
  *out = v;
  return Parsed::kOk;
}

inline std::string refusal(const std::string& flag, Parsed p, const std::string& value) {
  switch (p) {
    case Parsed::kNotNumber:
      return flag + ": not a number: " + value;
    case Parsed::kNotCount:
      return flag + ": not a count (decimal digits): " + value;
    case Parsed::kOutOfRange:
      return flag + ": out of range: " + value;
    case Parsed::kFinerThanNs:
      return flag + ": finer than one nanosecond: " + value;
    case Parsed::kOk:
      break;
  }
  return flag + ": " + value;
}

struct CadenceArgs {
  bool on = false;  // --cadence given
  emu::CadenceConfig config;
  std::string log_path;
};

// The cadence options among argv (others are skipped). False, with *why
// set, if one is malformed, out of range or misplaced (a --cadence-*
// option without --cadence), or if the configuration is not supported.
inline bool parseCadence(int argc, const char* const argv[], CadenceArgs* c, std::string* why) {
  constexpr const char* kStepSyntax = "--cadence-step: T:S (seconds after the start, a shift in us)";
  bool any_option = false;
  emu::CadenceConfig& k = c->config;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag.rfind("--cadence", 0) != 0) continue;
    if (i + 1 >= argc) {
      *why = flag + ": needs a value";
      return false;
    }
    const char* value = argv[++i];
    if (flag == "--cadence") {
      if (std::strcmp(value, "independent") == 0) {
        k.mode = emu::CadenceMode::kIndependent;
      } else if (std::strcmp(value, "request-tied") == 0) {
        k.mode = emu::CadenceMode::kRequestTied;
      } else {
        *why = "--cadence: independent or request-tied";
        return false;
      }
      c->on = true;
      continue;
    }
    any_option = true;
    Parsed p = Parsed::kOk;
    if (flag == "--cadence-log") {
      c->log_path = value;
    } else if (flag == "--cadence-ppm") {
      p = parseReal(value, &k.ppm);
    } else if (flag == "--cadence-phase-us") {
      p = parseNs(value, 1e3, &k.phase_ns);
    } else if (flag == "--cadence-ready-us") {
      p = parseNs(value, 1e3, &k.ready_ns);
    } else if (flag == "--cadence-byte-ns") {
      p = parseNs(value, 1.0, &k.byte_ns);
    } else if (flag == "--cadence-delay-every") {
      p = parseCount(value, &k.delay_every);
    } else if (flag == "--cadence-delay-us") {
      p = parseNs(value, 1e3, &k.delay_ns);
    } else if (flag == "--cadence-miss-every") {
      p = parseCount(value, &k.miss_every);
    } else if (flag == "--cadence-impair-from-s") {
      p = parseNs(value, 1e9, &k.impair_from_ns);
    } else if (flag == "--cadence-tie-us") {
      p = parseNs(value, 1e3, &k.tie_ns);
    } else if (flag == "--cadence-step") {
      const std::string v = value;
      const auto colon = v.find(':');
      if (colon == std::string::npos) {
        *why = kStepSyntax;
        return false;
      }
      if (k.step_count >= emu::CadenceConfig::kMaxSteps) {
        *why = "--cadence-step: at most 16";
        return false;
      }
      emu::PhaseStep s;
      p = parseNs(v.substr(0, colon).c_str(), 1e9, &s.at_ns);
      if (p == Parsed::kOk) p = parseNs(v.substr(colon + 1).c_str(), 1e3, &s.shift_ns);
      if (p == Parsed::kNotNumber) {
        *why = kStepSyntax;
        return false;
      }
      if (p == Parsed::kOk) k.steps[k.step_count++] = s;
    } else {
      *why = flag + ": unknown option";
      return false;
    }
    if (p != Parsed::kOk) {
      *why = refusal(flag, p, value);
      return false;
    }
  }
  if (any_option && !c->on) {
    *why = "the --cadence-* options need --cadence";
    return false;
  }
  if (c->on) {
    const std::string e = emu::cadenceConfigError(k);
    if (!e.empty()) {
      *why = "--cadence: " + e;
      return false;
    }
  }
  return true;
}

}  // namespace rtctrl::apps::dxl_emu_cadence
