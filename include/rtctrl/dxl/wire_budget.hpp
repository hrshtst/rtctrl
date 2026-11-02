#pragma once

namespace rtctrl::dxl::wire {

// Serial-line byte counts of Dynamixel Protocol 2.0 packets, for budgeting
// a control cycle. A packet is the 4-byte header, ID, 2-byte length,
// instruction, parameters and 2-byte CRC; a status packet adds an error
// byte. These are wire bytes only: USB and adapter latency, Return Delay
// Time, servo processing and host scheduling all come on top.
//
// Byte stuffing inserts 0xFD after any FF FF FD in a packet body. In a
// sync-read reply, negative current or velocity values can form that
// pattern, so the worst case is one stuffed byte per three data bytes.
// Fast Sync Read replies are never stuffed. rtctrl's sync writes cannot
// form it (IDs 2..9 separate the fields; goal positions are 0..4095).

constexpr int kStatusOverhead = 11;  // header..error + CRC around data

// A read of len bytes from one servo.
constexpr int readInstructionBytes() { return 14; }
constexpr int readReplyBytes(int len) { return kStatusOverhead + len; }

// One sync read of len bytes from n servos.
constexpr int syncReadInstructionBytes(int n) { return 14 + n; }
constexpr int syncReadReplyBytes(int n, int len) {
  return n * (kStatusOverhead + len);
}
constexpr int syncReadStuffingMaxBytes(int n, int len) { return n * (len / 3); }

// One Fast Sync Read reply: a single status packet for all n servos.
constexpr int fastSyncReadReplyBytes(int n, int len) {
  return 8 + n * (len + 4);
}

// One sync write of width bytes to n servos.
constexpr int syncWriteBytes(int n, int width) {
  return 14 + n * (1 + width);
}

// rtctrl's current indirect layout: the feedback window and the goal bytes
// each operating mode writes per servo.
constexpr int kFeedbackBytes = 13;
constexpr int kWriteWidthCurrent = 2;
constexpr int kWriteWidthVelocity = 4;
constexpr int kWriteWidthPosition = 4;
constexpr int kWriteWidthCurrentBasedPosition = 10;

enum class ReadVariant { kSync, kFastSync };

// Bytes on the line in one read-then-write cycle.
struct CycleBytes {
  int read_instruction = 0;
  int read_reply = 0;
  int read_stuffing_max = 0;
  int write = 0;
  constexpr int total(bool worst_case_stuffing) const {
    return read_instruction + read_reply + write +
           (worst_case_stuffing ? read_stuffing_max : 0);
  }
};

constexpr CycleBytes cycleBytes(int n, ReadVariant variant, int read_len,
                                int write_width) {
  CycleBytes b;
  b.read_instruction = syncReadInstructionBytes(n);
  if (variant == ReadVariant::kSync) {
    b.read_reply = syncReadReplyBytes(n, read_len);
    b.read_stuffing_max = syncReadStuffingMaxBytes(n, read_len);
  } else {
    b.read_reply = fastSyncReadReplyBytes(n, read_len);
  }
  b.write = syncWriteBytes(n, write_width);
  return b;
}

// Seconds per byte: start bit, 8 data bits, stop bit.
constexpr double byteTimeS(int baudrate) { return 10.0 / baudrate; }

}  // namespace rtctrl::dxl::wire
