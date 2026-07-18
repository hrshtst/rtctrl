#pragma once

#include <cstdint>

namespace rtctrl::dxl {

// Status-packet error codes (Protocol 2.0).
inline constexpr std::uint8_t kErrNone = 0x00;
inline constexpr std::uint8_t kErrResultFail = 0x01;
inline constexpr std::uint8_t kErrInstruction = 0x02;
inline constexpr std::uint8_t kErrCrc = 0x03;
inline constexpr std::uint8_t kErrDataRange = 0x04;
inline constexpr std::uint8_t kErrDataLength = 0x05;
inline constexpr std::uint8_t kErrDataLimit = 0x06;
inline constexpr std::uint8_t kErrAccess = 0x07;

// Outcome of one bus transaction: `comm` is the SDK COMM_* code (0 =
// COMM_SUCCESS) and `error` the status-packet error field.
struct IoResult {
  int comm = 0;
  std::uint8_t error = kErrNone;
  bool ok() const { return comm == 0 && error == kErrNone; }
};

}  // namespace rtctrl::dxl
