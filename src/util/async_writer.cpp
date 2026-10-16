#include "rtctrl/util/async_writer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstring>
#include <stdexcept>

namespace rtctrl::util {

namespace {

constexpr std::size_t kInitialRowBytes = 4096;

}  // namespace

AsyncWriter::AsyncWriter(const std::string& path)
    : AsyncWriter(path, Options{}) {}

AsyncWriter::AsyncWriter(const std::string& path, Options options) {
  std::FILE* stream = std::fopen(path.c_str(), "wx");
  if (stream == nullptr) {
    throw std::runtime_error("cannot create '" + path +
                             "' (it may already exist)");
  }
  start(stream, options);
}

AsyncWriter::AsyncWriter(std::FILE* stream, AdoptStream)
    : AsyncWriter(stream, AdoptStream{}, Options{}) {}

AsyncWriter::AsyncWriter(std::FILE* stream, AdoptStream, Options options) {
  if (stream == nullptr) {
    throw std::invalid_argument("AsyncWriter: null stream");
  }
  start(stream, options);
}

void AsyncWriter::start(std::FILE* stream, Options options) {
  // The stream is owned from here on: close it on every failure, since a
  // throwing constructor never runs the destructor.
  try {
    if (options.capacity_bytes == 0 ||
        !std::isfinite(options.flush_interval_s) ||
        options.flush_interval_s <= 0.0) {
      throw std::invalid_argument(
          "AsyncWriter: capacity_bytes must be positive and "
          "flush_interval_s finite and positive");
    }
    options_ = options;
    // At least one byte: a zero threshold would keep the writer's wait
    // predicate true on an empty queue, so it would never release the lock.
    wake_bytes_ = std::max<std::size_t>(1, options_.capacity_bytes / 2);
    row_.resize(kInitialRowBytes);
    // Both buffers hold a full queue, so neither commitRow nor the swap
    // ever allocates.
    queue_.reserve(options_.capacity_bytes);
    writing_.reserve(options_.capacity_bytes);
    file_ = stream;
    writer_ = std::thread(&AsyncWriter::writerLoop, this);
  } catch (...) {
    std::fclose(stream);
    file_ = nullptr;
    throw;
  }
}

AsyncWriter::~AsyncWriter() { close(); }

void AsyncWriter::appendf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  va_list retry;
  va_copy(retry, args);
  const std::size_t room = row_.size() - row_size_;
  const int n = std::vsnprintf(row_.data() + row_size_, room, format, args);
  va_end(args);
  if (n < 0) {
    row_failed_ = true;  // an encoding error: the row cannot be trusted
  } else {
    const auto needed = static_cast<std::size_t>(n);
    if (needed >= room) {
      // A row longer than any seen so far: a one-off growth.
      row_.resize(std::max(row_.size() * 2, row_size_ + needed + 1));
      std::vsnprintf(row_.data() + row_size_, row_.size() - row_size_,
                     format, retry);
    }
    row_size_ += needed;
  }
  va_end(retry);
}

void AsyncWriter::append(const char* text) {
  const std::size_t length = std::strlen(text);
  if (row_size_ + length > row_.size()) {
    row_.resize(std::max(row_.size() * 2, row_size_ + length));
  }
  std::memcpy(row_.data() + row_size_, text, length);
  row_size_ += length;
}

bool AsyncWriter::commitRow() {
  const std::size_t size = row_size_;
  const bool row_ok = !row_failed_;
  row_size_ = 0;
  row_failed_ = false;
  bool wake = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_) return false;
    if (failed_ || !row_ok || queue_.size() + size > options_.capacity_bytes) {
      failed_ = true;
      ++stats_.rows_dropped;
      return false;
    }
    queue_.append(row_.data(), size);
    ++stats_.rows_committed;
    stats_.max_queued_bytes = std::max(stats_.max_queued_bytes, queue_.size());
    wake = queue_.size() >= wake_bytes_;
  }
  if (wake) wake_.notify_one();
  return true;
}

bool AsyncWriter::ok() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !failed_;
}

AsyncWriter::Stats AsyncWriter::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void AsyncWriter::writerLoop() {
  const auto interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(options_.flush_interval_s));
  bool io_failed = false;  // after a failed write the stream is untrusted
  std::unique_lock<std::mutex> lock(mutex_);
  for (;;) {
    wake_.wait_for(lock, interval, [this] {
      return stop_ || queue_.size() >= wake_bytes_;
    });
    if (!queue_.empty()) {
      queue_.swap(writing_);
      lock.unlock();
      // The only file I/O, done without the lock: the producer keeps
      // committing into the (swapped-in, empty) queue meanwhile.
      const auto started = std::chrono::steady_clock::now();
      bool ok = !io_failed;
      if (ok) {
        ok = std::fwrite(writing_.data(), 1, writing_.size(), file_) ==
                 writing_.size() &&
             std::fflush(file_) == 0;
      }
      const double took = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - started)
                              .count();
      const std::size_t bytes = writing_.size();
      writing_.clear();  // keeps the reserved capacity
      lock.lock();
      stats_.max_write_s = std::max(stats_.max_write_s, took);
      if (ok) {
        stats_.bytes_written += bytes;
      } else {
        io_failed = true;
        failed_ = true;
      }
    }
    if (stop_ && queue_.empty()) break;
  }
}

bool AsyncWriter::close() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) return close_ok_;
    stop_ = true;
  }
  wake_.notify_one();
  if (writer_.joinable()) writer_.join();
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ok = !failed_;
  }
  if (file_ != nullptr) {
    if (std::fclose(file_) != 0) ok = false;
    file_ = nullptr;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  closed_ = true;
  close_ok_ = ok;
  return ok;
}

}  // namespace rtctrl::util
