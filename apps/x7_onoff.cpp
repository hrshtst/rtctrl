// Bring-up step 1: activate (torque on, safety armed, no motion), hold
// for a few seconds with a live command stream, then deactivate gently.
//
// Usage: x7_onoff [--config path] [--port dev] [hold_seconds]

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "x7_common.hpp"

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  const double hold_s =
      cli.argi < argc ? std::atof(argv[cli.argi]) : 3.0;

  try {
    auto session = x7::openSession(cli);
    auto& arm = *session.arm;

    std::printf("activating (no motion expected)...\n");
    if (!arm.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   arm.lastError().c_str());
      return 1;
    }
    std::printf("active; holding for %.1f s\n", hold_s);

    std::vector<rtctrl::dxl::Feedback> fb;
    std::vector<double> hold;
    constexpr int kCycleUs = 10000;  // 100 Hz
    for (double t = 0.0; t < hold_s; t += 1e-6 * kCycleUs) {
      if (!arm.readAll(fb)) {
        std::fprintf(stderr, "read failed\n");
        break;
      }
      if (hold.empty()) {
        for (const auto& f : fb) hold.push_back(f.position);
      }
      arm.writePositions(hold);
      if (!arm.checkDeadman()) return 1;
      usleep(kCycleUs);
    }

    std::printf("deactivating (arm goes limp gently)...\n");
    arm.deactivate();
    std::printf("done\n");
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
