#pragma once

#include "rtctrl/dxl/packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"

namespace rtctrl::emu {

// In-process PacketIO over a MotorBus — the fast transport for unit
// tests. Every call touches the addressed motors (feeding their Bus
// Watchdogs), mirroring how real instruction packets behave; simulated
// time advances only through MotorBus::tick, which the test drives.
class FakePacketIO : public dxl::PacketIO {
 public:
  explicit FakePacketIO(MotorBus& bus) : bus_(bus) {}

  dxl::IoResult ping(std::uint8_t id,
                     std::uint16_t* model_number = nullptr) override;
  dxl::IoResult read(std::uint8_t id, std::uint16_t addr, std::uint8_t* data,
                     std::uint16_t len) override;
  dxl::IoResult write(std::uint8_t id, std::uint16_t addr,
                      const std::uint8_t* data, std::uint16_t len) override;
  dxl::IoResult syncRead(std::uint16_t addr, std::uint16_t len,
                         const std::vector<std::uint8_t>& ids,
                         std::vector<std::uint8_t>& out) override;
  dxl::IoResult syncWrite(std::uint16_t addr, std::uint16_t len,
                          const std::vector<std::uint8_t>& ids,
                          const std::vector<std::uint8_t>& data) override;

  // Fault injection: transactions fail with this comm code until reset.
  void setCommFailure(int comm_code) { forced_comm_ = comm_code; }

 private:
  MotorBus& bus_;
  int forced_comm_ = 0;
};

}  // namespace rtctrl::emu
