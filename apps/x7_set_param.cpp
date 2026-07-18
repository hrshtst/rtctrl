// Bring-up step 3: read or write servo parameters on all configured
// joints (gains, profiles). Values print before and after.
//
// Usage: x7_set_param [--config path] [--port dev]
//                     [--p-gain N] [--profile-vel N] [--profile-acc N]

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "rtctrl/dxl/control_table.hpp"
#include "x7_common.hpp"

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
  long p_gain = -1, profile_vel = -1, profile_acc = -1;
  for (int i = cli.argi; i < argc - 1; ++i) {
    if (std::strcmp(argv[i], "--p-gain") == 0) p_gain = std::atol(argv[i + 1]);
    if (std::strcmp(argv[i], "--profile-vel") == 0)
      profile_vel = std::atol(argv[i + 1]);
    if (std::strcmp(argv[i], "--profile-acc") == 0)
      profile_acc = std::atol(argv[i + 1]);
  }

  try {
    auto session = x7::openSession(cli);
    std::printf("-- before --\n");
    dumpParams(session);
    bool wrote = false;
    if (p_gain >= 0) {
      wrote |= session.arm->writePositionPGain(
          static_cast<std::uint16_t>(p_gain));
    }
    if (profile_vel >= 0) {
      wrote |= session.arm->writeProfileVelocity(
          static_cast<std::uint32_t>(profile_vel));
    }
    if (profile_acc >= 0) {
      wrote |= session.arm->writeProfileAcceleration(
          static_cast<std::uint32_t>(profile_acc));
    }
    if (wrote) {
      std::printf("-- after --\n");
      dumpParams(session);
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
