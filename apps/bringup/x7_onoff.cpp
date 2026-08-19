// Bring-up step 1: activate (torque on, safety armed, no motion), hold
// for a few seconds with a live command stream, then deactivate gently.
//
// Usage: x7_onoff [--config path] [--port dev] [hold_seconds]

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "common/x7_common.hpp"

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  double hold_s = 3.0;
  if (cli.rest.size() > 1) {
    std::fprintf(stderr, "usage: x7_onoff [--config path] [--port dev] "
                         "[hold_seconds]\n");
    return 1;
  }
  if (!cli.rest.empty() && !x7::parseStrictDouble(cli.rest[0], &hold_s)) {
    std::fprintf(stderr, "invalid hold duration: %s\n", cli.rest[0]);
    return 1;
  }

  try {
    auto session = x7::openSession(cli);
    auto& arm = *session.arm;

    std::printf("activating (no motion expected)...\n");
    if (!arm.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   arm.lastError().c_str());
      return 1;
    }
    x7::ShutdownGuard shutdown{arm};
    std::printf("active; holding for %.1f s\n", hold_s);

    bool ok = true;
    std::vector<rtctrl::dxl::Feedback> fb;
    std::vector<double> hold;
    constexpr int kCycleUs = 10000;  // 100 Hz
    for (double t = 0.0; t < hold_s; t += 1e-6 * kCycleUs) {
      if (!arm.readAll(fb)) {
        std::fprintf(stderr, "read failed\n");
        ok = false;
        break;
      }
      if (hold.empty()) {
        for (const auto& f : fb) hold.push_back(f.position);
      }
      arm.writePositions(hold);
      if (!arm.checkDeadman()) {
        shutdown.run();
        return 1;
      }
      usleep(kCycleUs);
    }

    std::printf("deactivating (arm goes limp gently)...\n");
    const bool clean = shutdown.run();
    if (!clean) {
      std::printf("SHUTDOWN FAULT (hold %s)\n", ok ? "done" : "ABORTED");
      return 1;
    }
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
