#pragma once

#include <cstdio>
#include <string>

#include "rtctrl/arm/link_force_interval.hpp"

namespace rtctrl::arm {

// Schema-versioned CSV of LinkForceInterval rows, one per SimArm::step()
// attempt (attach with SimArm::linkForceLogTo). Join rows to a run's main
// log by seq, never by timestamp: the main row with feedback seq s joins
// the complete row with end_seq == s (the interval that produced it) or
// the row with start_seq == s (the interval that followed it). By design
// the initial main row has no preceding interval, the terminal step has
// no later main row, a rejected step (and every FaultLatched row after
// it) has no later main row, and zero-substep rows repeat their start
// time and seq. Keep the unmatched rows.
class LinkForceCsvLog {
 public:
  struct AdoptStream {};

  // Creates path exclusively ("wx"); throws std::runtime_error if it
  // exists or cannot be created.
  explicit LinkForceCsvLog(const std::string& path);
  // Takes ownership of an already-open stream (e.g. an injected stream
  // in tests); the header is written but not flushed yet.
  LinkForceCsvLog(std::FILE* stream, AdoptStream);
  ~LinkForceCsvLog();

  LinkForceCsvLog(const LinkForceCsvLog&) = delete;
  LinkForceCsvLog& operator=(const LinkForceCsvLog&) = delete;

  // Writes and flushes one row. Returns false on any fprintf or fflush
  // failure; a failed log stays failed and accepts no further rows.
  bool write(const LinkForceInterval& interval);
  // Closes the stream; false if closing failed or the log had failed.
  // Idempotent.
  bool close();
  bool failed() const { return failed_; }

 private:
  void writeHeader();

  std::FILE* file_ = nullptr;
  bool failed_ = false;
  bool closed_ = false;
  bool close_ok_ = true;
};

}  // namespace rtctrl::arm
