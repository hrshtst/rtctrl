#include "rtctrl/model/async_zvs_writer.hpp"

#include <stdexcept>

namespace rtctrl::model {

namespace {

std::FILE* openOrThrow(const std::string& path) {
  std::FILE* stream = std::fopen(path.c_str(), "w");
  if (stream == nullptr) {
    throw std::runtime_error("AsyncZvsWriter: cannot open '" + path + "'");
  }
  return stream;
}

}  // namespace

AsyncZvsWriter::AsyncZvsWriter(const std::string& path)
    : writer_(openOrThrow(path), util::AsyncWriter::AdoptStream{}) {}

AsyncZvsWriter::AsyncZvsWriter(std::FILE* stream,
                               util::AsyncWriter::Options options)
    : writer_(stream, util::AsyncWriter::AdoptStream{}, options) {}

bool AsyncZvsWriter::frame(double dt, const zVec dis) {
  if (failed_ || closed_) return false;
  // ZvsWriter's line: "%.15g " then zVecFPrint, whose layout
  // (zRawVecFPrint) is "size[/capacity] (v v ... )\n" with "%.10g ".
  writer_.appendf("%.15g ", dt);
  if (dis == nullptr) {
    writer_.append("(null vector)\n");
  } else {
    const int size = zVecSizeNC(dis);
    const int capacity = zVecCapacity(dis);
    writer_.appendf("%d", size);
    if (capacity != size) writer_.appendf("/%d", capacity);
    writer_.append(" (");
    for (int i = 0; i < size; ++i) {
      writer_.appendf("%.10g ", zVecElemNC(dis, i));
    }
    writer_.append(")\n");
  }
  if (!writer_.commitRow()) {
    failed_ = true;
    return false;
  }
  ++frames_;
  return true;
}

bool AsyncZvsWriter::close() {
  if (!closed_) {
    closed_ = true;
    if (!writer_.close()) failed_ = true;
  }
  return !failed_;
}

}  // namespace rtctrl::model
