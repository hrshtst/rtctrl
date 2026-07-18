// Bring-up step 2: stream all present values. Torque stays OFF — safe
// to run at any time, the arm can be moved by hand while watching.
//
// Usage: x7_read [--config path] [--port dev] [seconds]

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "x7_common.hpp"

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  const double duration_s =
      cli.argi < argc ? std::atof(argv[cli.argi]) : 10.0;

  try {
    auto session = x7::openSession(cli);
    auto& arm = *session.arm;
    rtctrl::dxl::SyncGroup group(*session.port, [&] {
      std::vector<std::uint8_t> ids;
      for (const auto& j : session.config.joints) ids.push_back(j.id);
      return ids;
    }());
    if (!group.setupIndirect().ok()) {
      std::fprintf(stderr,
                   "indirect setup failed (is torque on? run after a "
                   "clean power-up)\n");
      return 1;
    }

    std::vector<rtctrl::dxl::Feedback> fb;
    for (double t = 0.0; t < duration_s; t += 0.1) {
      if (!group.readAll(fb).ok()) {
        std::fprintf(stderr, "read failed\n");
        return 1;
      }
      std::printf("\rt=%5.1f  q[rad]:", t);
      for (const auto& f : fb) std::printf(" %+7.3f", f.position);
      std::printf("  %4.1fV %2.0fC", fb[0].voltage, fb[0].temperature);
      std::fflush(stdout);
      usleep(100000);
    }
    std::printf("\n");
    (void)arm;
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
