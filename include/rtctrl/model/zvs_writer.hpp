#pragma once

#include <zm/zm.h>

#include <cstdio>
#include <string>

namespace rtctrl::model {

// Writes a joint-displacement sequence in the .zvs format consumed by
// rk_anim and zSeqScanFile: one line per frame, "<dt> <zVec>".
//
// Frames are buffered, so most write errors surface only when the buffer
// is flushed. A failed write latches failed(): that frame and every later
// one return false and are not written or counted, and close() reports
// it. Only close() returning true means every counted frame is in the file.
class ZvsWriter {
 public:
  explicit ZvsWriter(const std::string& path);
  ~ZvsWriter();  // closes if close() was not called

  ZvsWriter(const ZvsWriter&) = delete;
  ZvsWriter& operator=(const ZvsWriter&) = delete;

  bool frame(double dt, const zVec dis);
  // Flushes and closes the file; false if a write or the close failed.
  // Idempotent.
  bool close();
  bool failed() const { return failed_; }
  int frames() const { return frames_; }

 private:
  std::FILE* fp_;
  int frames_ = 0;
  bool failed_ = false;
};

}  // namespace rtctrl::model
