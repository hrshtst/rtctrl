#pragma once

#include <zm/zm.h>

#include <cstdio>
#include <string>

#include "rtctrl/util/async_writer.hpp"

namespace rtctrl::model {

// The .zvs format of ZvsWriter, byte for byte, with the file I/O on a
// writer thread (util::AsyncWriter): frame() formats one line into
// memory and queues it, so the calling thread never blocks in a write.
// For a running control loop's per-cycle export (x7_wave --zvs); offline
// tools keep ZvsWriter.
//
// A frame that does not fit the queue, or a failed write, latches
// failed(): that frame and every later one return false and are not
// counted, and close() reports it. A write's failure is known only once
// the write returns, so it surfaces at a later frame() or at close().
// Only close() returning true means every counted frame is in the file.
class AsyncZvsWriter {
 public:
  // Creates or truncates path, as ZvsWriter does; throws
  // std::runtime_error if it cannot be opened.
  explicit AsyncZvsWriter(const std::string& path);
  // Takes ownership of an open stream (an injected stream in tests).
  AsyncZvsWriter(std::FILE* stream, util::AsyncWriter::Options options);

  AsyncZvsWriter(const AsyncZvsWriter&) = delete;
  AsyncZvsWriter& operator=(const AsyncZvsWriter&) = delete;

  // Formats and queues one frame; no file I/O. False (the frame not
  // written or counted) once failed or closed.
  bool frame(double dt, const zVec dis);
  // Writes every queued frame, flushes and closes; blocks on file I/O,
  // so call it after the control loop. Idempotent.
  bool close();
  bool failed() const { return failed_ || !writer_.ok(); }
  int frames() const { return frames_; }

 private:
  util::AsyncWriter writer_;
  int frames_ = 0;
  bool failed_ = false;
  bool closed_ = false;
};

}  // namespace rtctrl::model
