#include "rtctrl/emu/standalone_loop.hpp"

#include <poll.h>

#include <algorithm>
#include <cerrno>
#include <ctime>
#include <stdexcept>

#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"

namespace rtctrl::emu {

namespace {

timespec until(StandaloneLoop::Clock::time_point deadline) {
  const auto left = std::max(StandaloneLoop::Clock::duration::zero(),
                             deadline - StandaloneLoop::Clock::now());
  const auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(left).count();
  return timespec{static_cast<time_t>(ns / 1'000'000'000),
                  static_cast<long>(ns % 1'000'000'000)};
}

}  // namespace

StandaloneLoop::StandaloneLoop(PtyBus& pty, MotorBus& bus,
                               const Config& config, Clock::time_point start)
    : pty_(pty),
      bus_(bus),
      config_(config),
      dt_(std::chrono::duration<double>(config.tick).count()),
      next_tick_(start + config.tick) {
  if (config.tick <= std::chrono::nanoseconds::zero() ||
      config.max_catch_up_ticks == 0 || config.max_read_bytes == 0) {
    throw std::invalid_argument(
        "StandaloneLoop: tick, catch-up cap and read bound must be positive");
  }
}

StandaloneLoop::Status StandaloneLoop::step(Clock::time_point now) {
  ++stats_.steps;
  if (now >= next_tick_) {
    const auto due =
        static_cast<std::uint64_t>((now - next_tick_) / config_.tick) + 1;
    if (due > config_.max_catch_up_ticks) {
      stats_.overrun_due = due;
      return Status::kOverrun;
    }
    for (std::uint64_t i = 0; i < due; ++i) bus_.tick(dt_);
    next_tick_ += config_.tick * static_cast<std::int64_t>(due);
    stats_.ticks += due;
    stats_.max_ticks_in_step = std::max(stats_.max_ticks_in_step, due);
    if (due > 1) ++stats_.late_steps;
  }
  const PtyBus::Serviced io = pty_.serviceIo(config_.max_read_bytes);
  if (!io.ok) return Status::kIoError;
  if (io.limited) ++stats_.limited_reads;
  return Status::kOk;
}

StandaloneLoop::Clock::time_point StandaloneLoop::deadline() const {
  const auto reply = pty_.nextReplyDue();
  return reply ? std::min(next_tick_, *reply) : next_tick_;
}

bool StandaloneLoop::wait(Clock::time_point deadline) {
  pollfd pfd{pty_.masterFd(), POLLIN, 0};
  timespec ts = until(deadline);
  const int n = ::ppoll(&pfd, 1, &ts, nullptr);
  if (n < 0) return errno == EINTR;  // a signal: the caller checks its stop
  if (n == 0) return true;           // the deadline
  if ((pfd.revents & POLLNVAL) != 0) return false;
  if ((pfd.revents & POLLIN) != 0) return true;
  if ((pfd.revents & (POLLHUP | POLLERR)) != 0) {
    // No client: the hangup is reported at once, so wait by time alone.
    ++stats_.hangup_waits;
    ts = until(deadline);
    if (::ppoll(nullptr, 0, &ts, nullptr) < 0 && errno != EINTR) return false;
  }
  return true;
}

StandaloneLoop::Status StandaloneLoop::run(const std::function<bool()>& stop) {
  while (!stop()) {
    const Status status = step(Clock::now());
    if (status != Status::kOk) return status;
    if (stop()) break;
    if (!wait(deadline())) return Status::kIoError;
  }
  return Status::kOk;
}

}  // namespace rtctrl::emu
