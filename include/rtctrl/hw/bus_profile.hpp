#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rtctrl/dxl/packet_io.hpp"

namespace rtctrl::hw {

// The bus a bounded grouped read relies on (control-rate plan, contract
// C-4a): the assumptions its budget makes about the servos and the
// adapter.
struct BusProfile {
  // Every servo's Return Delay Time (register units, 2 us) at most this.
  // Checkpoint 1 records the arm's 0.
  int max_return_delay_time = 0;
  // Every servo replies to reads at least at this level.
  int min_status_return_level = 1;
  // Fast Sync Read (Phase B) needs firmware >= v45 on every servo.
  bool fast_sync_read = false;
  // A latency timer the adapter exposes must read this; one it does not
  // expose is reported (none) and left unverified.
  int required_latency_timer_ms = 1;
};

// What checkBusProfile read.
struct BusProfileReport {
  struct Servo {
    std::uint8_t id = 0;
    int firmware = -1;
    int return_delay_time = -1;
    int status_return_level = -1;
    int baud_register = -1;
  };
  std::vector<Servo> servos;
  std::optional<int> latency_timer_ms;
};

// Reads, and never writes, every listed servo's firmware, Return Delay
// Time, Status Return Level and Baud Rate register, and the adapter's
// latency timer, and checks them against `profile` for a bus running at
// `baudrate`. False, with *error, at the first failed read or mismatch;
// *report then holds what was read so far. Shared by a powered session's
// activation (CraneX7) and the torque-off timing probe; each adds its own
// policy (the session, its Bus Watchdog timing).
bool checkBusProfile(dxl::PacketIO& io, const std::vector<std::uint8_t>& ids,
                     int baudrate, const BusProfile& profile,
                     BusProfileReport* report, std::string* error);

}  // namespace rtctrl::hw
