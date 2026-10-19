#include "rtctrl/model/zvs_writer.hpp"

#include <stdexcept>

namespace rtctrl::model {

ZvsWriter::ZvsWriter(const std::string& path)
    : fp_(std::fopen(path.c_str(), "w")) {
  if (fp_ == nullptr) {
    throw std::runtime_error("ZvsWriter: cannot open '" + path + "'");
  }
}

ZvsWriter::~ZvsWriter() { close(); }

bool ZvsWriter::frame(double dt, const zVec dis) {
  if (failed_ || fp_ == nullptr) return false;
  // zVecFPrint reports nothing, so the stream's error flag is the check.
  if (std::fprintf(fp_, "%.15g ", dt) < 0) {
    failed_ = true;
  } else {
    zVecFPrint(fp_, dis);
    failed_ = std::ferror(fp_) != 0;
  }
  if (failed_) return false;
  ++frames_;
  return true;
}

bool ZvsWriter::close() {
  if (fp_ != nullptr) {
    if (std::ferror(fp_) != 0) failed_ = true;
    if (std::fclose(fp_) != 0) failed_ = true;
    fp_ = nullptr;
  }
  return !failed_;
}

}  // namespace rtctrl::model
