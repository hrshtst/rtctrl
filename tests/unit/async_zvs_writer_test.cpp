// AsyncZvsWriter: ZvsWriter's .zvs format byte for byte, with the file
// I/O off the calling thread.
#include "rtctrl/model/async_zvs_writer.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gated_stream.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"

using rtctrl::model::AsyncZvsWriter;
using rtctrl::model::ZVector;
using rtctrl::model::ZvsWriter;
using rtctrl_test::GatedSink;
using rtctrl_test::openGated;

namespace {

namespace fs = std::filesystem;

std::string tempPath(const std::string& name) {
  fs::create_directories("build/async_zvs_writer_test");
  const std::string path = "build/async_zvs_writer_test/" + name;
  std::remove(path.c_str());
  return path;
}

std::string slurp(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream text;
  text << in.rdbuf();
  return text.str();
}

// Frames with values that exercise the number formats: zeros, signs,
// tiny and large magnitudes, and digits beyond %.10g.
std::vector<ZVector> sampleFrames() {
  const std::vector<std::vector<double>> rows = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0.1, -0.25, 3.141592653589793, -1e-12, 1e9, 2.5e-300, -0.0,
       123456.7890123, 1.0 / 3.0},
      {-2.0, 1.5, 0.30000000000000004, 7, -7, 1e-5, 0.15, 0.25, 0.2},
  };
  std::vector<ZVector> frames;
  for (const auto& row : rows) {
    ZVector v(static_cast<int>(row.size()));
    for (std::size_t i = 0; i < row.size(); ++i) {
      zVecElemNC(v.get(), static_cast<int>(i)) = row[i];
    }
    frames.push_back(std::move(v));
  }
  return frames;
}

// The reference bytes: the same frames through ZvsWriter.
std::string referenceBytes(const std::vector<ZVector>& frames,
                           const std::vector<double>& dts) {
  const std::string path = tempPath("reference.zvs");
  {
    ZvsWriter writer(path);
    for (std::size_t k = 0; k < frames.size(); ++k) {
      REQUIRE(writer.frame(dts[k], frames[k].get()));
    }
    REQUIRE(writer.close());
  }
  return slurp(path);
}

rtctrl::util::AsyncWriter::Options fast() {
  rtctrl::util::AsyncWriter::Options options;
  options.flush_interval_s = 0.001;
  return options;
}

}  // namespace

TEST_CASE("async zvs writer writes ZvsWriter's format byte for byte",
          "[model][zvs]") {
  const auto frames = sampleFrames();
  const std::vector<double> dts = {0.01, 0.0100000000000001, 1.0 / 3.0};
  const std::string expected = referenceBytes(frames, dts);
  REQUIRE_FALSE(expected.empty());

  const std::string path = tempPath("async.zvs");
  AsyncZvsWriter writer(path);
  for (std::size_t k = 0; k < frames.size(); ++k) {
    REQUIRE(writer.frame(dts[k], frames[k].get()));
  }
  CHECK(writer.frames() == 3);
  REQUIRE(writer.close());
  CHECK_FALSE(writer.failed());
  CHECK(slurp(path) == expected);
}

TEST_CASE("async zvs writer matches a vector smaller than its capacity and "
          "a null vector",
          "[model][zvs]") {
  ZVector v(9);
  for (int i = 0; i < 9; ++i) zVecElemNC(v.get(), i) = 0.5 * i;
  zVecSetSizeNC(v.get(), 4);  // prints as "4/9 (...)"
  const std::string reference = tempPath("reference_small.zvs");
  {
    ZvsWriter writer(reference);
    REQUIRE(writer.frame(0.01, v.get()));
    REQUIRE(writer.frame(0.01, nullptr));
    REQUIRE(writer.close());
  }
  const std::string path = tempPath("async_small.zvs");
  AsyncZvsWriter writer(path);
  REQUIRE(writer.frame(0.01, v.get()));
  REQUIRE(writer.frame(0.01, nullptr));
  REQUIRE(writer.close());
  zVecSetSizeNC(v.get(), 9);
  CHECK(slurp(path) == slurp(reference));
  CHECK(slurp(path).find("4/9 (") != std::string::npos);
}

TEST_CASE("async zvs writer frames never wait for a blocked write",
          "[model][zvs]") {
  const auto frames = sampleFrames();
  std::vector<ZVector> many;
  std::vector<double> dts;
  for (int k = 0; k < 300; ++k) {
    many.emplace_back(frames[k % frames.size()]);
    dts.push_back(0.01);
  }
  const std::string expected = referenceBytes(many, dts);

  GatedSink sink;
  sink.shut();
  AsyncZvsWriter writer(openGated(&sink), fast());
  REQUIRE(writer.frame(dts[0], many[0].get()));
  // The writer thread is now inside a write that cannot finish. A frame
  // that did file I/O on this thread would block here instead (300
  // frames overflow any stdio buffer).
  REQUIRE(sink.waitEntered());
  for (std::size_t k = 1; k < many.size(); ++k) {
    REQUIRE(writer.frame(dts[k], many[k].get()));
  }
  CHECK_FALSE(writer.failed());
  sink.release();
  REQUIRE(writer.close());
  CHECK(sink.contents() == expected);
  CHECK(sink.closed);
  CHECK(writer.frames() == 300);
}

TEST_CASE("async zvs writer latches a failed write and close reports it",
          "[model][zvs]") {
  const auto frames = sampleFrames();
  GatedSink sink;
  sink.fail = true;
  AsyncZvsWriter writer(openGated(&sink), fast());
  REQUIRE(writer.frame(0.01, frames[0].get()));
  REQUIRE(sink.waitEntered());
  CHECK_FALSE(writer.close());
  CHECK(writer.failed());
  CHECK_FALSE(writer.frame(0.01, frames[1].get()));
  CHECK(writer.frames() == 1);
}

TEST_CASE("async zvs writer drops a frame that overflows the queue",
          "[model][zvs]") {
  const auto frames = sampleFrames();
  GatedSink sink;
  sink.shut();
  auto options = fast();
  options.capacity_bytes = 256;  // about two frames
  AsyncZvsWriter writer(openGated(&sink), options);
  REQUIRE(writer.frame(0.01, frames[1].get()));
  REQUIRE(sink.waitEntered());
  int accepted = 0;
  while (accepted < 100 && writer.frame(0.01, frames[1].get())) ++accepted;
  CHECK(accepted < 100);
  CHECK(writer.failed());
  // Latched: later frames are refused too, and close reports it.
  CHECK_FALSE(writer.frame(0.01, frames[0].get()));
  sink.release();
  CHECK_FALSE(writer.close());
  CHECK(writer.frames() == accepted + 1);
}

TEST_CASE("async zvs writer truncates an existing file, as ZvsWriter does",
          "[model][zvs]") {
  const std::string path = tempPath("existing.zvs");
  {
    std::ofstream out(path);
    out << "old contents that must not survive\n";
  }
  const auto frames = sampleFrames();
  AsyncZvsWriter writer(path);
  REQUIRE(writer.frame(0.01, frames[0].get()));
  REQUIRE(writer.close());
  CHECK(slurp(path) == "0.01 9 (0 0 0 0 0 0 0 0 0 )\n");
}

TEST_CASE("async zvs writer refuses an unopenable path and frames after "
          "close",
          "[model][zvs]") {
  CHECK_THROWS_AS(AsyncZvsWriter("build/no_such_directory/x/out.zvs"),
                  std::runtime_error);
  const auto frames = sampleFrames();
  AsyncZvsWriter writer(tempPath("closed.zvs"));
  REQUIRE(writer.frame(0.01, frames[0].get()));
  REQUIRE(writer.close());
  CHECK(writer.close());  // idempotent
  CHECK_FALSE(writer.frame(0.01, frames[0].get()));
  CHECK_FALSE(writer.failed());  // closing cleanly is not a failure
  CHECK(writer.frames() == 1);
}
