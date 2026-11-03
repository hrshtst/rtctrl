// Integration: the bounded read accepts nothing at or after its deadline,
// even when a receiver paused across it then holds a complete reply.
//
// The pause has to come before a receive that then returns the late bytes:
// the paused receiver of a preempted thread. A signal cannot place it
// there, since it is handled when the non-blocking read() returns, before
// the deadline is checked. So this executable defines read() itself:
// every call passes through to libc's, except one armed call on the
// emulator's port, which first sleeps. The SDK is a shared library, so
// its read() binds to this definition; the override stays in this
// executable, apart from the other integration tests.
#include <catch2/catch_test_macros.hpp>
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "fault_bus.hpp"

namespace {

std::atomic<bool> g_pause_armed{false};
char g_pause_path[128] = {};
std::chrono::nanoseconds g_pause{0};

// The next read() of the file at `path` sleeps for `pause` first.
void armReadPause(const std::string& path, std::chrono::nanoseconds pause) {
  std::snprintf(g_pause_path, sizeof(g_pause_path), "%s", path.c_str());
  g_pause = pause;
  g_pause_armed.store(true);
}

}  // namespace

extern "C" ssize_t read(int fd, void* buf, size_t count) {
  using Read = ssize_t (*)(int, void*, size_t);
  static const auto real = reinterpret_cast<Read>(dlsym(RTLD_NEXT, "read"));
  if (g_pause_armed.load()) {
    char link[64];
    char path[128];
    std::snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
    const ssize_t len = readlink(link, path, sizeof(path) - 1);
    if (len >= 0) {
      path[len] = '\0';
      if (std::strcmp(path, g_pause_path) == 0 &&
          g_pause_armed.exchange(false)) {
        const auto ns = g_pause.count();
        const timespec ts{static_cast<time_t>(ns / 1'000'000'000),
                          static_cast<long>(ns % 1'000'000'000)};
        nanosleep(&ts, nullptr);
      }
    }
  }
  return real(fd, buf, count);
}

using namespace fault_bus;  // NOLINT(build/namespaces)

TEST_CASE("bytes held only after the deadline are not accepted",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(positionRead());  // 30 ms budget

  // The receiver's first read() of the port sleeps 80 ms before reading,
  // then returns every byte that arrived meanwhile: a complete reply.
  SECTION("replies that arrive after the deadline") {
    bus.pty().scriptSyncReadFaults(
        {{emu::ReplyFault::Kind::kDelay, milliseconds(45), 0, {}}});
  }
  SECTION("replies that arrive in time but are read after the deadline") {
    // When the bytes reached the host cannot be known; only when they were
    // read can.
    bus.pty().scriptSyncReadFaults(
        {{emu::ReplyFault::Kind::kDelay, milliseconds(10), 0, {}}});
  }
  std::vector<std::uint8_t> out;
  armReadPause(bus.path(), milliseconds(80));
  const auto start = Clock::now();
  const dxl::IoResult r = readPositions(port, out);
  const auto elapsed = Clock::now() - start;

  REQUIRE_FALSE(g_pause_armed.load());  // the pause fell inside the read
  CHECK(elapsed >= milliseconds(80));
  CHECK_FALSE(r.ok());
  CHECK(port.boundedReadStats().timeouts == 1);
  CHECK(port.inRecovery());
  CHECK(port.quietGuardPending());

  // The recovery read after the guard is clean.
  REQUIRE(readPositions(port, out).ok());
  CHECK(positions(out) == expected(2000));
}
