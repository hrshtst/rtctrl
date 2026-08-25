// Bring-up step 4: read or write servo parameters on all configured
// joints (gains, profiles). Values print before and after.
//
// Usage: x7_set_param [--config path] [--port dev]
//                     [--p-gain N] [--profile-vel N] [--profile-acc N]
//                     [--profile-vel-si RAD_PER_S]
//                     [--profile-acc-si RAD_PER_S2]
//   The -si variants convert like the vendor API: truncating, clamped
//   to [1, 32767]. A request of 0 selects the SLOWEST profile —
//   register 0 means MAXIMUM on the X series and stays reachable only
//   through the raw flags. Raw and SI forms of the same register are
//   mutually exclusive.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "rtctrl/dxl/control_table.hpp"
#include "bringup/set_param_common.hpp"
#include "common/x7_common.hpp"

namespace reg = rtctrl::dxl::reg;

namespace {

void dumpParams(x7::Session& session) {
  std::printf("%-6s %-8s %-12s %-12s\n", "id", "p_gain", "profile_vel",
              "profile_acc");
  for (const auto& joint : session.config.joints) {
    std::uint16_t p = 0;
    std::uint32_t pv = 0, pa = 0;
    session.port->read16(joint.id, reg::kPositionPGain.addr, &p);
    session.port->read32(joint.id, reg::kProfileVelocity.addr, &pv);
    session.port->read32(joint.id, reg::kProfileAcceleration.addr, &pa);
    std::printf("%-6u %-8u %-12u %-12u\n", joint.id, p, pv, pa);
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  // strict parsing with register-range validation BEFORE narrowing
  // (review finding: atol garbage wrote P-gain 0 to the servos, and
  // out-of-range values narrowed silently)
  long p_gain = -1, profile_vel = -1, profile_acc = -1;
  double profile_vel_si = 0.0, profile_acc_si = 0.0;
  bool have_vel_si = false, have_acc_si = false;
  for (std::size_t i = 0; i < cli.rest.size(); ++i) {
    // SI profile flags: strict double, nonnegative (a negative rate
    // has no meaning; the conversion itself handles the 0 case)
    double* si_dst = nullptr;
    bool* si_have = nullptr;
    if (std::strcmp(cli.rest[i], "--profile-vel-si") == 0) {
      si_dst = &profile_vel_si;
      si_have = &have_vel_si;
    } else if (std::strcmp(cli.rest[i], "--profile-acc-si") == 0) {
      si_dst = &profile_acc_si;
      si_have = &have_acc_si;
    }
    if (si_dst != nullptr) {
      if (i + 1 >= cli.rest.size()) {
        std::fprintf(stderr, "%s requires a value\n", cli.rest[i]);
        return 1;
      }
      if (!x7::parseStrictDouble(cli.rest[++i], si_dst) ||
          *si_dst < 0.0) {
        std::fprintf(stderr, "%s: invalid value %s (nonnegative)\n",
                     cli.rest[i - 1], cli.rest[i]);
        return 1;
      }
      *si_have = true;
      continue;
    }
    long* dst = nullptr;
    long max = 0;
    if (std::strcmp(cli.rest[i], "--p-gain") == 0) {
      dst = &p_gain;
      max = UINT16_MAX;
    } else if (std::strcmp(cli.rest[i], "--profile-vel") == 0) {
      dst = &profile_vel;
      max = UINT32_MAX;
    } else if (std::strcmp(cli.rest[i], "--profile-acc") == 0) {
      dst = &profile_acc;
      max = UINT32_MAX;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", cli.rest[i]);
      return 1;
    }
    if (i + 1 >= cli.rest.size()) {
      std::fprintf(stderr, "%s requires a value\n", cli.rest[i]);
      return 1;
    }
    if (!x7::parseStrictLong(cli.rest[++i], dst) || *dst < 0 ||
        *dst > max) {
      std::fprintf(stderr, "%s: invalid value %s (range 0..%ld)\n",
                   cli.rest[i - 1], cli.rest[i], max);
      return 1;
    }
  }
  if ((profile_vel >= 0 && have_vel_si) ||
      (profile_acc >= 0 && have_acc_si)) {
    std::fprintf(stderr, "choose either the raw or the -si form of a "
                         "profile flag, not both\n");
    return 1;
  }

  try {
    auto session = x7::openSession(cli);
    const bool wants_write = p_gain >= 0 || profile_vel >= 0 ||
                             profile_acc >= 0 || have_vel_si || have_acc_si;
    if (wants_write) {
      const auto torque =
          x7::checkAllTorqueOff(*session.port, session.config);
      if (torque.status == x7::TorqueCheckStatus::kReadFailed) {
        std::fprintf(stderr,
                     "refusing parameter changes: cannot verify torque "
                     "state on id %u\n",
                     torque.id);
        return 1;
      }
      if (torque.status == x7::TorqueCheckStatus::kEnabled) {
        std::fprintf(stderr,
                     "refusing parameter changes: torque is enabled on "
                     "id %u\n",
                     torque.id);
        return 1;
      }
    }
    std::printf("-- before --\n");
    dumpParams(session);
    // every REQUESTED write must succeed — a failed parameter write
    // exiting 0 hid real bus problems (review finding)
    bool wrote = false;
    bool all_ok = true;
    if (p_gain >= 0) {
      wrote = true;
      if (!session.arm->writePositionPGain(
              static_cast<std::uint16_t>(p_gain))) {
        std::fprintf(stderr, "--p-gain write FAILED\n");
        all_ok = false;
      }
    }
    if (profile_vel >= 0) {
      wrote = true;
      if (!session.arm->writeProfileVelocity(
              static_cast<std::uint32_t>(profile_vel))) {
        std::fprintf(stderr, "--profile-vel write FAILED\n");
        all_ok = false;
      }
    }
    if (profile_acc >= 0) {
      wrote = true;
      if (!session.arm->writeProfileAcceleration(
              static_cast<std::uint32_t>(profile_acc))) {
        std::fprintf(stderr, "--profile-acc write FAILED\n");
        all_ok = false;
      }
    }
    if (have_vel_si) {
      wrote = true;
      if (!session.arm->writeProfileVelocityRadPerSec(profile_vel_si)) {
        std::fprintf(stderr, "--profile-vel-si write FAILED\n");
        all_ok = false;
      }
    }
    if (have_acc_si) {
      wrote = true;
      if (!session.arm->writeProfileAccelerationRadPerSec2(
              profile_acc_si)) {
        std::fprintf(stderr, "--profile-acc-si write FAILED\n");
        all_ok = false;
      }
    }
    if (wrote) {
      std::printf("-- after --\n");
      dumpParams(session);
    }
    return all_ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
