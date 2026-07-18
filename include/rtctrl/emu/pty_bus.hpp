#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rtctrl/emu/motor_emulator.hpp"

namespace rtctrl::emu {

// Serves a MotorBus over a pseudo-terminal speaking Dynamixel Protocol
// 2.0 (header/CRC-16/byte stuffing), so the UNMODIFIED DynamixelSDK
// PortHandler connects to the slave path like a real serial adapter.
// Handles Ping (incl. broadcast), Read, Write, SyncRead, SyncWrite.
//
// Single-threaded ownership: whoever calls poll() owns the MotorBus;
// clients on the other end of the pty interact only through the fd.
class PtyBus {
 public:
  explicit PtyBus(MotorBus& bus);
  ~PtyBus();

  PtyBus(const PtyBus&) = delete;
  PtyBus& operator=(const PtyBus&) = delete;

  const std::string& slavePath() const { return slave_path_; }

  // Service pending frames, then advance simulated time by dt.
  // Returns false on an unrecoverable pty error.
  bool poll(double dt);

 private:
  void handleFrame(std::uint8_t id, std::uint8_t instruction,
                   const std::vector<std::uint8_t>& params);
  void sendStatus(std::uint8_t id, std::uint8_t error,
                  const std::vector<std::uint8_t>& params);
  void extractFrames();

  MotorBus& bus_;
  int master_fd_ = -1;
  std::string slave_path_;
  std::vector<std::uint8_t> rx_;
};

// Protocol 2.0 helpers, exposed for tests.
std::uint16_t dxlCrc16(const std::uint8_t* data, std::size_t size);
void dxlAddStuffing(std::vector<std::uint8_t>& payload);
void dxlRemoveStuffing(std::vector<std::uint8_t>& payload);

}  // namespace rtctrl::emu
