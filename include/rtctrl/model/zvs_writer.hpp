#pragma once

#include <zm/zm.h>

#include <cstdio>
#include <string>

namespace rtctrl::model {

// Writes a joint-displacement sequence in the .zvs format consumed by
// rk_anim and zSeqScanFile: one line per frame, "<dt> <zVec>".
class ZvsWriter {
 public:
  explicit ZvsWriter(const std::string& path);
  ~ZvsWriter();

  ZvsWriter(const ZvsWriter&) = delete;
  ZvsWriter& operator=(const ZvsWriter&) = delete;

  void frame(double dt, const zVec dis);
  int frames() const { return frames_; }

 private:
  std::FILE* fp_;
  int frames_ = 0;
};

}  // namespace rtctrl::model
