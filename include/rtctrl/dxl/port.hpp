#pragma once

#include <memory>
#include <string>

#include "rtctrl/dxl/packet_io.hpp"

namespace dynamixel {
class PortHandler;
class PacketHandler;
}  // namespace dynamixel

namespace rtctrl::dxl {

// PacketIO over DynamixelSDK (Protocol 2.0): the real serial bus.
class Port : public PacketIO {
 public:
  Port(const std::string& device, int baudrate);
  ~Port() override;

  Port(const Port&) = delete;
  Port& operator=(const Port&) = delete;

  IoResult ping(std::uint8_t id, std::uint16_t* model_number) override;
  IoResult read(std::uint8_t id, std::uint16_t addr, std::uint8_t* data,
                std::uint16_t len) override;
  IoResult write(std::uint8_t id, std::uint16_t addr,
                 const std::uint8_t* data, std::uint16_t len) override;
  IoResult syncRead(std::uint16_t addr, std::uint16_t len,
                    const std::vector<std::uint8_t>& ids,
                    std::vector<std::uint8_t>& out) override;
  IoResult syncWrite(std::uint16_t addr, std::uint16_t len,
                     const std::vector<std::uint8_t>& ids,
                     const std::vector<std::uint8_t>& data) override;

  // Stop all bus traffic: the loss-of-comms escalation path — after
  // this the servo-side Bus Watchdogs necessarily fire.
  void close();
  bool isOpen() const { return open_; }

 private:
  dynamixel::PortHandler* port_;
  dynamixel::PacketHandler* packet_;
  bool open_ = false;
};

}  // namespace rtctrl::dxl
