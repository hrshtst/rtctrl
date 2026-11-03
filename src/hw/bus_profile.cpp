#include "rtctrl/hw/bus_profile.hpp"

#include "rtctrl/dxl/control_table.hpp"

namespace rtctrl::hw {

namespace reg = rtctrl::dxl::reg;

bool checkBusProfile(dxl::PacketIO& io, const std::vector<std::uint8_t>& ids,
                     int baudrate, const BusProfile& profile,
                     BusProfileReport* report, std::string* error) {
  *report = BusProfileReport{};
  const std::optional<std::uint8_t> baud = dxl::baudRegisterFor(baudrate);
  if (!baud) {
    *error = "no Baud Rate register value for " + std::to_string(baudrate) +
             " bps";
    return false;
  }
  for (const std::uint8_t id : ids) {
    const std::string name = std::to_string(id);
    std::uint8_t firmware = 0;
    std::uint8_t return_delay = 0;
    std::uint8_t status_return = 0;
    std::uint8_t baud_register = 0;
    if (!io.read8(id, reg::kFirmwareVersion.addr, &firmware).ok() ||
        !io.read8(id, reg::kReturnDelayTime.addr, &return_delay).ok() ||
        !io.read8(id, reg::kStatusReturnLevel.addr, &status_return).ok() ||
        !io.read8(id, reg::kBaudRate.addr, &baud_register).ok()) {
      *error = "cannot read the bus profile of id " + name;
      return false;
    }
    BusProfileReport::Servo servo;
    servo.id = id;
    servo.firmware = firmware;
    servo.return_delay_time = return_delay;
    servo.status_return_level = status_return;
    servo.baud_register = baud_register;
    report->servos.push_back(servo);
    if (return_delay > profile.max_return_delay_time) {
      *error = "id " + name + " has Return Delay Time " +
               std::to_string(return_delay) + " (" +
               std::to_string(2 * return_delay) + " us), above the " +
               std::to_string(profile.max_return_delay_time) +
               " the bounded read's budget assumes";
      return false;
    }
    if (status_return < profile.min_status_return_level) {
      *error = "id " + name + " has Status Return Level " +
               std::to_string(status_return) + ", below " +
               std::to_string(profile.min_status_return_level);
      return false;
    }
    if (baud_register != *baud) {
      *error = "id " + name + " has Baud Rate register " +
               std::to_string(baud_register) + ", not the " +
               std::to_string(*baud) + " of " + std::to_string(baudrate) +
               " bps";
      return false;
    }
    if (profile.fast_sync_read && firmware < dxl::kMinFirmwareFastSyncRead) {
      *error = "id " + name + " firmware v" + std::to_string(firmware) +
               " lacks Fast Sync Read (needs >= v45)";
      return false;
    }
  }
  report->latency_timer_ms = io.latencyTimerMs();
  if (report->latency_timer_ms &&
      *report->latency_timer_ms != profile.required_latency_timer_ms) {
    *error = "the adapter's latency timer is " +
             std::to_string(*report->latency_timer_ms) + " ms, not " +
             std::to_string(profile.required_latency_timer_ms);
    return false;
  }
  return true;
}

}  // namespace rtctrl::hw
