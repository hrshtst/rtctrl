#pragma once

#include <zm/zm.h>

#include <new>
#include <utility>

namespace rtctrl::model {

// RAII owner of a zm vector, implicitly convertible to zVec for direct
// use with the mi-lib C API.
class ZVector {
 public:
  explicit ZVector(int size) : vec_(zVecAlloc(size)) {
    if (!vec_) throw std::bad_alloc();
    zVecZero(vec_);
  }
  ZVector(const ZVector& other) : vec_(zVecClone(other.vec_)) {
    if (!vec_) throw std::bad_alloc();
  }
  ZVector(ZVector&& other) noexcept
      : vec_(std::exchange(other.vec_, nullptr)) {}
  ZVector& operator=(const ZVector&) = delete;
  ZVector& operator=(ZVector&&) = delete;
  ~ZVector() {
    if (vec_) zVecFree(vec_);
  }

  operator zVec() const { return vec_; }
  zVec get() const { return vec_; }
  int size() const { return zVecSizeNC(vec_); }
  double& operator[](int i) { return zVecElemNC(vec_, i); }
  double operator[](int i) const { return zVecElemNC(vec_, i); }

 private:
  zVec vec_;
};

}  // namespace rtctrl::model
