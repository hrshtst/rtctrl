// The pty bus emulator with scripted reply faults, shared by the bounded
// read's integration tests (bounded_read_pty_test.cpp and, in its own
// executable, bounded_read_deadline_test.cpp).
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"

namespace fault_bus {

namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace reg = rtctrl::dxl::reg;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;


inline constexpr std::uint8_t kSyncRead = 0x82;
inline constexpr std::uint8_t kSyncWrite = 0x83;
inline constexpr std::uint8_t kRead = 0x02;
inline constexpr std::uint8_t kWrite = 0x03;
inline constexpr std::uint8_t kPing = 0x01;

// A CRANE-X7-shaped bus served on a background thread. The poll loop and
// withBus() share a mutex, so a test can change the emulated servos
// between transactions.
class FaultBus {
 public:
  FaultBus() {
    for (std::uint8_t id = 2; id <= 9; ++id) {
      emu::MotorEmulator::Config config;
      config.id = id;
      config.model_number =
          (id == 3) ? dxl::kModelXm540W270 : dxl::kModelXm430W350;
      bus_.add(config);
    }
    pty_ = std::make_unique<emu::PtyBus>(bus_);
    thread_ = std::thread([this] {
      while (!stop_.load()) {
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          pty_->poll(0.0001);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });
  }

  ~FaultBus() {
    stop_.store(true);
    thread_.join();
  }

  const std::string& path() const { return pty_->slavePath(); }
  emu::PtyBus& pty() { return *pty_; }

  template <class F>
  void withBus(F f) {
    const std::lock_guard<std::mutex> lock(mutex_);
    f(bus_);
  }

  void setPositions(std::uint32_t base) {
    withBus([base](emu::MotorBus& bus) {
      for (auto& motor : bus.motors()) {
        motor.poke(reg::kPresentPosition, base + motor.id());
      }
    });
  }

  // Waits until the emulator has received n instructions.
  std::vector<emu::Arrival> waitArrivals(std::size_t n) {
    const auto give_up = Clock::now() + milliseconds(500);
    std::vector<emu::Arrival> a = pty_->arrivals();
    while (a.size() < n && Clock::now() < give_up) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      a = pty_->arrivals();
    }
    return a;
  }

 private:
  emu::MotorBus bus_;
  std::unique_ptr<emu::PtyBus> pty_;
  std::thread thread_;
  std::mutex mutex_;
  std::atomic<bool> stop_{false};
};

inline const std::vector<std::uint8_t> kIds{2, 3, 4, 5, 6, 7, 8, 9};
inline constexpr auto kBudget = milliseconds(30);
inline constexpr auto kGuard = milliseconds(30);

inline dxl::BoundedReadConfig positionRead() {
  dxl::BoundedReadConfig config;
  config.addr = reg::kPresentPosition.addr;
  config.len = reg::kPresentPosition.len;
  config.ids = kIds;
  config.budget = kBudget;
  config.guard = kGuard;
  return config;
}

inline std::vector<std::uint32_t> positions(const std::vector<std::uint8_t>& out) {
  std::vector<std::uint32_t> p;
  for (std::size_t i = 0; i + 3 < out.size(); i += 4) {
    p.push_back(static_cast<std::uint32_t>(out[i]) |
                (static_cast<std::uint32_t>(out[i + 1]) << 8) |
                (static_cast<std::uint32_t>(out[i + 2]) << 16) |
                (static_cast<std::uint32_t>(out[i + 3]) << 24));
  }
  return p;
}

inline std::vector<std::uint32_t> expected(std::uint32_t base) {
  std::vector<std::uint32_t> p;
  for (const std::uint8_t id : kIds) p.push_back(base + id);
  return p;
}

inline dxl::IoResult readPositions(dxl::Port& port, std::vector<std::uint8_t>& out) {
  return port.syncRead(reg::kPresentPosition.addr, reg::kPresentPosition.len,
                       kIds, out);
}

inline std::vector<std::uint16_t> syncReadLengths(
    const std::vector<emu::Arrival>& arrivals) {
  std::vector<std::uint16_t> lengths;
  for (const auto& a : arrivals) {
    if (a.instruction == kSyncRead) lengths.push_back(a.data_length);
  }
  return lengths;
}

}  // namespace fault_bus
