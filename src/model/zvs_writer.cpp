#include "rtctrl/model/zvs_writer.hpp"

#include <stdexcept>

namespace rtctrl::model {

ZvsWriter::ZvsWriter(const std::string& path)
    : fp_(std::fopen(path.c_str(), "w")) {
  if (fp_ == nullptr) {
    throw std::runtime_error("ZvsWriter: cannot open '" + path + "'");
  }
}

ZvsWriter::~ZvsWriter() {
  if (fp_ != nullptr) std::fclose(fp_);
}

void ZvsWriter::frame(double dt, const zVec dis) {
  std::fprintf(fp_, "%.15g ", dt);
  zVecFPrint(fp_, dis);
  ++frames_;
}

}  // namespace rtctrl::model
