#include "rtctrl/util/async_writer.hpp"

#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

using rtctrl::util::AsyncWriter;

namespace {

namespace fs = std::filesystem;

class TempPath {
 public:
  explicit TempPath(const std::string& name) {
    fs::create_directories("build/async_writer_test");
    path_ = "build/async_writer_test/" + name;
    std::remove(path_.c_str());
  }
  ~TempPath() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

std::string slurp(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream text;
  text << in.rdbuf();
  return text.str();
}

// An in-memory stream whose writes block while the gate is shut, standing
// in for a file write stalled in the kernel.
struct GatedSink {
  std::mutex mutex;
  std::condition_variable changed;
  bool open = true;
  bool entered = false;  // a write is in progress or has happened
  bool closed = false;
  std::string data;

  void shut() {
    std::lock_guard<std::mutex> lock(mutex);
    open = false;
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex);
    open = true;
    changed.notify_all();
  }
  bool waitEntered() {
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock, std::chrono::seconds(5),
                            [this] { return entered; });
  }
  std::string contents() {
    std::lock_guard<std::mutex> lock(mutex);
    return data;
  }
};

ssize_t gatedWrite(void* cookie, const char* buffer, size_t size) {
  auto* sink = static_cast<GatedSink*>(cookie);
  std::unique_lock<std::mutex> lock(sink->mutex);
  sink->entered = true;
  sink->changed.notify_all();
  sink->changed.wait(lock, [sink] { return sink->open; });
  sink->data.append(buffer, size);
  return static_cast<ssize_t>(size);
}

int gatedClose(void* cookie) {
  auto* sink = static_cast<GatedSink*>(cookie);
  std::lock_guard<std::mutex> lock(sink->mutex);
  sink->closed = true;
  return 0;
}

std::FILE* openGated(GatedSink* sink) {
  cookie_io_functions_t io{};
  io.write = gatedWrite;
  io.close = gatedClose;
  std::FILE* stream = fopencookie(sink, "w", io);
  REQUIRE(stream != nullptr);
  return stream;
}

AsyncWriter::Options fast(std::size_t capacity = std::size_t{1} << 20) {
  AsyncWriter::Options options;
  options.capacity_bytes = capacity;
  options.flush_interval_s = 0.001;
  return options;
}

}  // namespace

TEST_CASE("async writer rows reach the file whole and in commit order",
          "[util][async_writer]") {
  TempPath file("ordered.csv");
  std::string expected = "# header\nindex,value\n";
  {
    AsyncWriter writer(file.path());
    writer.append("# header\n");
    REQUIRE(writer.commitRow());
    writer.append("index,value");
    writer.append("\n");
    REQUIRE(writer.commitRow());
    for (int i = 0; i < 1000; ++i) {
      writer.appendf("%d", i);
      writer.appendf(",%.9f\n", 0.001 * i);
      REQUIRE(writer.commitRow());
      char row[64];
      std::snprintf(row, sizeof row, "%d,%.9f\n", i, 0.001 * i);
      expected += row;
    }
    CHECK(writer.ok());
    REQUIRE(writer.close());
    const auto stats = writer.stats();
    CHECK(stats.rows_committed == 1002);
    CHECK(stats.rows_dropped == 0);
    CHECK(stats.bytes_written == expected.size());
  }
  CHECK(slurp(file.path()) == expected);
}

TEST_CASE("async writer refuses to overwrite an existing file",
          "[util][async_writer]") {
  TempPath file("existing.csv");
  { std::ofstream(file.path()) << "keep\n"; }
  CHECK_THROWS_AS(AsyncWriter(file.path()), std::runtime_error);
  CHECK(slurp(file.path()) == "keep\n");
}

TEST_CASE("async writer commits never wait for a blocked write",
          "[util][async_writer]") {
  GatedSink sink;
  sink.shut();
  AsyncWriter writer(openGated(&sink), AsyncWriter::AdoptStream{}, fast());
  std::string expected = "first\n";
  writer.append("first\n");
  REQUIRE(writer.commitRow());
  // The writer thread is now inside a write that cannot finish. If a
  // commit waited for it, this test would deadlock instead of passing.
  REQUIRE(sink.waitEntered());
  for (int i = 0; i < 200; ++i) {
    writer.appendf("row %d\n", i);
    REQUIRE(writer.commitRow());
    expected += "row " + std::to_string(i) + "\n";
  }
  CHECK(writer.ok());
  CHECK(writer.stats().max_queued_bytes > 0);
  sink.release();
  REQUIRE(writer.close());
  CHECK(sink.contents() == expected);
  CHECK(sink.closed);
}

TEST_CASE("async writer drops a row that overflows the queue and latches",
          "[util][async_writer]") {
  GatedSink sink;
  sink.shut();
  AsyncWriter writer(openGated(&sink), AsyncWriter::AdoptStream{}, fast(64));
  writer.append("row 0\n");
  REQUIRE(writer.commitRow());
  REQUIRE(sink.waitEntered());  // "row 0" is out of the queue, blocked
  std::string expected = "row 0\n";
  for (int i = 1; i <= 6; ++i) {  // 6 x 10 bytes fit in 64
    writer.appendf("row %05d\n", i);
    REQUIRE(writer.commitRow());
    expected += "row " + std::string(5 - std::to_string(i).size(), '0') +
                std::to_string(i) + "\n";
  }
  writer.append("row 00007\n");
  CHECK_FALSE(writer.commitRow());  // 70 bytes > 64: dropped
  CHECK_FALSE(writer.ok());
  sink.release();
  // Latched: later rows are refused even with room in the queue.
  writer.append("x\n");
  CHECK_FALSE(writer.commitRow());
  CHECK_FALSE(writer.close());
  const auto stats = writer.stats();
  CHECK(stats.rows_committed == 7);
  CHECK(stats.rows_dropped == 2);
  CHECK(sink.contents() == expected);  // whole rows only, no gap filled
}

TEST_CASE("async writer reports a failed write at a later commit and at close",
          "[util][async_writer]") {
  std::FILE* full = std::fopen("/dev/full", "w");
  REQUIRE(full != nullptr);
  AsyncWriter writer(full, AsyncWriter::AdoptStream{}, fast());
  writer.append("lost\n");
  REQUIRE(writer.commitRow());
  bool refused = false;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!refused && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    writer.append("next\n");
    refused = !writer.commitRow();
  }
  CHECK(refused);
  CHECK_FALSE(writer.ok());
  CHECK_FALSE(writer.close());
  CHECK_FALSE(writer.close());  // idempotent
}

TEST_CASE("async writer close discards an uncommitted row",
          "[util][async_writer]") {
  TempPath file("partial.csv");
  AsyncWriter writer(file.path());
  writer.append("whole\n");
  REQUIRE(writer.commitRow());
  writer.appendf("half,%d", 1);
  REQUIRE(writer.close());
  CHECK(writer.close());  // idempotent
  writer.append("late\n");
  CHECK_FALSE(writer.commitRow());
  CHECK(slurp(file.path()) == "whole\n");
}

TEST_CASE("async writer destructor drains committed rows",
          "[util][async_writer]") {
  TempPath file("drained.csv");
  std::string expected;
  {
    AsyncWriter writer(file.path(), fast());
    for (int i = 0; i < 50; ++i) {
      writer.appendf("%d\n", i);
      REQUIRE(writer.commitRow());
      expected += std::to_string(i) + "\n";
    }
  }
  CHECK(slurp(file.path()) == expected);
}

TEST_CASE("async writer grows the row buffer for long rows",
          "[util][async_writer]") {
  TempPath file("long.csv");
  const std::string wide(20000, 'w');
  const std::string tail(9000, 't');
  {
    AsyncWriter writer(file.path());
    writer.appendf("%s|", wide.c_str());
    writer.append(tail.c_str());
    writer.appendf("|%d\n", 42);
    REQUIRE(writer.commitRow());
    REQUIRE(writer.close());
  }
  CHECK(slurp(file.path()) == wide + "|" + tail + "|42\n");
}

TEST_CASE("async writer rejects invalid options and closes the stream",
          "[util][async_writer]") {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::pair<std::size_t, double> invalid[] = {
      {0, 0.02}, {1024, 0.0}, {1024, nan}};
  for (const auto& [capacity, interval] : invalid) {
    GatedSink sink;
    AsyncWriter::Options options;
    options.capacity_bytes = capacity;
    options.flush_interval_s = interval;
    CHECK_THROWS_AS(
        AsyncWriter(openGated(&sink), AsyncWriter::AdoptStream{}, options),
        std::invalid_argument);
    CHECK(sink.closed);
  }
  CHECK_THROWS_AS(AsyncWriter(nullptr, AsyncWriter::AdoptStream{}),
                  std::invalid_argument);
}
