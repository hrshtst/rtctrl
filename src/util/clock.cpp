#include "rtctrl/util/clock.hpp"

#include <time.h>
#include <unistd.h>

namespace rtctrl::util {

std::int64_t clockMonotonicNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000 +
         static_cast<std::int64_t>(ts.tv_nsec);
}

std::int64_t monotonicNs(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             t.time_since_epoch())
      .count();
}

long currentThreadId() { return static_cast<long>(::gettid()); }

}  // namespace rtctrl::util
