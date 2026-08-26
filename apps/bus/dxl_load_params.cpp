// Transactional loader for versioned dxl_inspect parameter dumps.
//
// Usage:
//   dxl_load_params --port <dev> [--baud 3000000] <file> [id ...]

// Only keys present in each [motor.parameters] table are considered. Every
// selected motor must match its recorded model and report torque disabled.
// All reads/preflight checks complete before the first write; each write is
// read back exactly, and a failure triggers best-effort rollback.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "bus/dxl_parameters.hpp"
#include "rtctrl/dxl/port.hpp"

namespace dxl = rtctrl::dxl;
namespace params = rtctrl::apps::dxl_parameters;

namespace {

std::uint8_t parseId(const char* text) {
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || value < 0 || value > 252) {
    throw std::runtime_error("invalid motor id '" + std::string(text) +
                             "' (expected 0..252)");
  }
  return static_cast<std::uint8_t>(value);
}

params::ParameterDump selectMotors(const params::ParameterDump& source,
                                   const std::set<std::uint8_t>& selected) {
  if (selected.empty()) return source;
  params::ParameterDump filtered;
  for (const auto id : selected) {
    const auto motor = std::find_if(
        source.motors.begin(), source.motors.end(),
        [id](const params::MotorRecord& record) { return record.id == id; });
    if (motor == source.motors.end()) {
      throw std::runtime_error("motor id " + std::to_string(id) +
                               " is not present in the dump");
    }
    filtered.motors.push_back(*motor);
  }
  return filtered;
}

void usage() {
  std::fprintf(stderr,
               "usage: dxl_load_params --port <dev> [--baud N] "
               "<file> [id ...]\n");
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string device;
  int baud = 3000000;
  int argi = 1;
  while (argi < argc) {
    if (std::strcmp(argv[argi], "--port") == 0) {
      if (++argi >= argc) {
        std::fprintf(stderr, "error: --port requires a value\n");
        return 2;
      }
      device = argv[argi++];
    } else if (std::strcmp(argv[argi], "--baud") == 0) {
      if (++argi >= argc) {
        std::fprintf(stderr, "error: --baud requires a value\n");
        return 2;
      }
      char* end = nullptr;
      const long parsed = std::strtol(argv[argi], &end, 10);
      if (end == argv[argi] || *end != '\0' || parsed <= 0 ||
          parsed > std::numeric_limits<int>::max()) {
        std::fprintf(stderr, "error: invalid baud rate '%s'\n", argv[argi]);
        return 2;
      }
      baud = static_cast<int>(parsed);
      ++argi;
    } else {
      break;
    }
  }
  if (device.empty() || argi >= argc) {
    usage();
    return 2;
  }

  try {
    const std::string path = argv[argi++];
    const auto parsed = params::parseFile(path);  // before bus contact
    std::set<std::uint8_t> selected;
    while (argi < argc) {
      const auto id = parseId(argv[argi++]);
      if (!selected.insert(id).second) {
        throw std::runtime_error("duplicate motor id " + std::to_string(id));
      }
    }
    const auto dump = selectMotors(parsed, selected);

    std::printf("loading %zu motor(s) from %s\n", dump.motors.size(),
                path.c_str());
    dxl::Port port(device, baud);
    const auto result = params::apply(port, dump);
    if (!result.ok) {
      std::fprintf(stderr, "parameter update failed: %s\n",
                   result.error.c_str());
      if (result.rollback_attempted) {
        std::fprintf(stderr, "rollback %s\n",
                     result.rollback_ok ? "verified" : "INCOMPLETE");
      }
      return 1;
    }
    std::printf("parameter update verified: %zu write(s), %zu unchanged\n",
                result.changed, result.unchanged);
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "error: %s\n", error.what());
    return 1;
  }
}
