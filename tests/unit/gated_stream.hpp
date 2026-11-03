#pragma once

// Test support shared by the async writers' tests.

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <string>

#include <sys/types.h>

namespace rtctrl_test {

// An in-memory stream whose writes block while the gate is shut, standing
// in for a file write stalled in the kernel.
struct GatedSink {
  std::mutex mutex;
  std::condition_variable changed;
  bool open = true;
  bool entered = false;  // a write is in progress or has happened
  bool closed = false;
  bool fail = false;  // writes fail once the gate opens
  std::string data;

  void shut() {
    std::lock_guard<std::mutex> lock(mutex);
    open = false;
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex);
    open = true;
    changed.notify_all();
  }
  bool waitEntered() {
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock, std::chrono::seconds(5),
                            [this] { return entered; });
  }
  std::string contents() {
    std::lock_guard<std::mutex> lock(mutex);
    return data;
  }
};

inline ssize_t gatedWrite(void* cookie, const char* buffer, size_t size) {
  auto* sink = static_cast<GatedSink*>(cookie);
  std::unique_lock<std::mutex> lock(sink->mutex);
  sink->entered = true;
  sink->changed.notify_all();
  sink->changed.wait(lock, [sink] { return sink->open; });
  if (sink->fail) return -1;
  sink->data.append(buffer, size);
  return static_cast<ssize_t>(size);
}

inline int gatedClose(void* cookie) {
  auto* sink = static_cast<GatedSink*>(cookie);
  std::lock_guard<std::mutex> lock(sink->mutex);
  sink->closed = true;
  return 0;
}

inline std::FILE* openGated(GatedSink* sink) {
  cookie_io_functions_t io{};
  io.write = gatedWrite;
  io.close = gatedClose;
  std::FILE* stream = fopencookie(sink, "w", io);
  REQUIRE(stream != nullptr);
  return stream;
}

}  // namespace rtctrl_test
