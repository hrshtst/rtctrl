#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

namespace rtctrl::util {

// Bounded single-producer single-consumer ring of trivially copyable
// records. All storage is allocated by the constructor; push and pop
// never allocate, lock, or block, so a control thread can publish
// records that a lower-priority thread drains. A full ring refuses the
// newest record (push returns false) and the producer decides how to
// account for it.
template <typename T>
class SpscRing {
 public:
  // Capacity is rounded up to a power of two (at least 2).
  explicit SpscRing(std::size_t capacity) : slots_(roundUp(capacity)) {
    mask_ = slots_.size() - 1;
  }

  SpscRing(const SpscRing&) = delete;
  SpscRing& operator=(const SpscRing&) = delete;

  std::size_t capacity() const { return slots_.size(); }

  // Producer thread only.
  bool push(const T& value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) == slots_.size()) {
      return false;
    }
    slots_[head & mask_] = value;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  // Consumer thread only.
  bool pop(T* out) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    *out = slots_[tail & mask_];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

 private:
  static std::size_t roundUp(std::size_t n) {
    std::size_t c = 2;
    while (c < n) c <<= 1;
    return c;
  }

  std::vector<T> slots_;
  std::size_t mask_ = 0;
  alignas(64) std::atomic<std::size_t> head_{0};  // next slot to write
  alignas(64) std::atomic<std::size_t> tail_{0};  // next slot to read
};

}  // namespace rtctrl::util
