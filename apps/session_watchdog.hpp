// Independent session deadline watchdog for hardware runs whose
// activation-to-deactivation time is safety-bounded.
#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace x7 {

// A dedicated thread that fires an injected callable EXACTLY ONCE when
// a host-steady-clock deadline expires, and never after disarm(). The
// deadline is deliberately independent of the feedback clock: it must
// fire even when feedback is dead or the arming thread is blocked in a
// synchronous shutdown.
//
// The expiry callable runs on the watchdog thread, so it must be safe
// to invoke from any thread. On hardware that means
// CraneX7::requestQuiesce() plus a report — NEVER deactivate() or
// Port::close(), neither of which is concurrency-safe against a
// blocked shutdown.
class SessionWatchdog {
 public:
  SessionWatchdog(std::chrono::steady_clock::duration deadline,
                  std::function<void()> on_expiry)
      : deadline_(std::chrono::steady_clock::now() + deadline),
        on_expiry_(std::move(on_expiry)),
        thread_([this] { run(); }) {}

  ~SessionWatchdog() { disarm(); }

  SessionWatchdog(const SessionWatchdog&) = delete;
  SessionWatchdog& operator=(const SessionWatchdog&) = delete;

  // Idempotent; blocks until the watchdog thread has exited, so after
  // disarm() returns the callable is guaranteed not to fire.
  void disarm() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      disarmed_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
  }

  bool fired() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fired_;
  }

 private:
  void run() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_until(lock, deadline_, [this] { return disarmed_; });
    if (disarmed_) return;
    fired_ = true;
    lock.unlock();
    on_expiry_();
  }

  const std::chrono::steady_clock::time_point deadline_;
  std::function<void()> on_expiry_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool disarmed_ = false;
  bool fired_ = false;
  std::thread thread_;  // last member: started after everything above
};

}  // namespace x7
