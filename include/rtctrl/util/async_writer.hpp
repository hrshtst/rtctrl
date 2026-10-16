#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rtctrl::util {

// Row-oriented file writer that keeps file I/O off a control thread.
//
// A buffered write to a regular file can block in the kernel for tens to
// hundreds of milliseconds (journal commits, device power states, other
// processes' fsync calls; measured on the development workstation: a
// 4 KiB append took up to 162 ms on ext4). A control loop that writes its
// telemetry directly turns such a stall into a missed command window, a
// feedback gap abort, or the 250 ms command deadman.
//
// The producer (one control thread) formats a row into memory with
// appendf/append and queues it with commitRow, which only appends to a
// preallocated buffer under a mutex that the writer thread never holds
// during I/O. The writer thread wakes every flush_interval_s (nominally;
// or early when the queue is half full), swaps the queue out, and writes
// and flushes it. Rows reach the file whole and in commit order.
//
// The queue is bounded: a row that does not fit is dropped and the writer
// latches failed, as does any failed write or flush. Once failed, every
// later commitRow drops its row and returns false, so the log is either
// complete or reported incomplete; it never silently has gaps.
//
// Failure detection has no time bound. A write's failure is known only
// when the write returns: while it is blocked in the kernel, commits keep
// succeeding (rows queue behind it) and ok() stays true. Afterwards the
// failure surfaces at the next commitRow, and close() always reports it.
// A queue overflow, by contrast, is reported by the commit that hits it.
//
// Durability: committed rows are in memory until a write of them returns.
// A process that is killed loses the unwritten part of the batch being
// written (up to capacity_bytes) and every row queued behind it (up to
// capacity_bytes more), so up to twice capacity_bytes in all. Normally
// that is the last flush interval, but after a blocked write it can be
// much more. Normal exits, exceptions and the destructor drain the queue.
class AsyncWriter {
 public:
  struct Options {
    // Bound on committed bytes not yet handed to the writer thread. The
    // default absorbs about 30 s of a 100 Hz, 1.2 kB-per-row log.
    std::size_t capacity_bytes = std::size_t{4} << 20;
    double flush_interval_s = 0.02;  // nominal writer wake period
  };
  struct AdoptStream {};
  struct Stats {
    std::uint64_t rows_committed = 0;
    std::uint64_t rows_dropped = 0;
    std::uint64_t bytes_written = 0;
    std::size_t max_queued_bytes = 0;  // high-water mark of the queue
    double max_write_s = 0.0;          // slowest write + flush of a batch
  };

  // Creates path exclusively ("wx"); throws std::runtime_error if it
  // exists or cannot be created. Starts the writer thread.
  explicit AsyncWriter(const std::string& path);
  AsyncWriter(const std::string& path, Options options);
  // Takes ownership of an already-open stream (a caller that reports its
  // own open errors, or an injected stream in tests).
  AsyncWriter(std::FILE* stream, AdoptStream);
  AsyncWriter(std::FILE* stream, AdoptStream, Options options);
  // Closes (draining the queue) if close() was not called.
  ~AsyncWriter();

  AsyncWriter(const AsyncWriter&) = delete;
  AsyncWriter& operator=(const AsyncWriter&) = delete;

  // Producer side: a single thread. appendf/append extend the pending row
  // in producer-owned memory (no I/O, no lock); commitRow queues it
  // whole. commitRow returns false, dropping the row, if the queue is
  // full, a write failed, or the writer is closed.
  void appendf(const char* format, ...) __attribute__((format(printf, 2, 3)));
  void append(const char* text);
  bool commitRow();

  // False once a row was dropped or a write failed.
  bool ok() const;
  Stats stats() const;

  // Writes everything committed, flushes and closes the file. Returns
  // true only if every committed row reached the file, no row was
  // dropped, and closing succeeded. An uncommitted pending row is
  // discarded. Blocks on file I/O: call it after the control loop, from
  // the producer thread or once it has stopped committing. Idempotent.
  bool close();

 private:
  void start(std::FILE* stream, Options options);
  void writerLoop();

  std::FILE* file_ = nullptr;
  Options options_;
  std::size_t wake_bytes_ = 1;  // queue size that wakes the writer early
  std::vector<char> row_;  // producer-owned pending row
  std::size_t row_size_ = 0;
  bool row_failed_ = false;  // a formatting error in the pending row

  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::string queue_;    // committed, not yet taken by the writer
  std::string writing_;  // writer-owned batch being written
  bool stop_ = false;
  bool failed_ = false;
  bool closed_ = false;
  bool close_ok_ = false;
  Stats stats_;
  std::thread writer_;
};

}  // namespace rtctrl::util
