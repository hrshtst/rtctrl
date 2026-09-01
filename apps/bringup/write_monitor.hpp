#pragma once

#include <cstdio>

namespace x7 {

// A transient failed write may recover before the host deadman expires.
// Keep the stream alive, but retain the failure so a motion app never
// reports ordinary success after dropping a command.
class PositionWriteMonitor {
 public:
  explicit PositionWriteMonitor(std::FILE* output = stderr,
                                const char* command_name = "position")
      : output_(output), command_name_(command_name) {}

  bool record(bool succeeded, const char* phase) {
    if (succeeded) return true;
    ++failures_;
    if (failures_ == 1 && output_ != nullptr) {
      std::fprintf(output_,
                   "%s: %s command write failed; command stream "
                   "remains under deadman protection\n",
                   phase, command_name_);
    }
    return false;
  }

  bool ok() const { return failures_ == 0; }
  int failures() const { return failures_; }

  void reportSummary(const char* phase) const {
    if (failures_ > 0 && output_ != nullptr) {
      std::fprintf(output_, "%s: %d %s command write(s) failed\n",
                   phase, failures_, command_name_);
    }
  }

 private:
  std::FILE* output_;
  const char* command_name_;
  int failures_ = 0;
};

}  // namespace x7
