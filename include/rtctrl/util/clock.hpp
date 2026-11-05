#pragma once

#include <chrono>
#include <cstdint>

namespace rtctrl::util {

// Absolute values that place a log's relative times on the system's
// clocks and name the thread that took them, so a run can be correlated
// with scheduler and USB traces.

// CLOCK_MONOTONIC now, read by clock_gettime(2), in nanoseconds.
std::int64_t clockMonotonicNs();

// The CLOCK_MONOTONIC time of a steady_clock time point, in nanoseconds.
// On Linux, std::chrono::steady_clock reads CLOCK_MONOTONIC, so this is
// the time point's count since the clock's epoch; the unit tests check it
// against clockMonotonicNs().
std::int64_t monotonicNs(std::chrono::steady_clock::time_point t);

// The calling thread's kernel thread ID (gettid(2)): the ID that scheduler
// traces give the thread. The process's main thread has the process ID.
long currentThreadId();

}  // namespace rtctrl::util
