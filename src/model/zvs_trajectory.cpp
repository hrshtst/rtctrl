#include "rtctrl/model/zvs_trajectory.hpp"

#include <zm/zm.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rtctrl::model {

namespace {

void requireOutput(zVec value, const char* name) {
  if (value == nullptr || zVecSizeNC(value) != kCanonicalDof) {
    throw std::invalid_argument(std::string("ZvsTrajectory::sample: ") + name +
                                " must have canonical width");
  }
}

std::vector<double> solve(std::vector<std::vector<double>> a,
                          std::vector<double> b) {
  const int n = static_cast<int>(b.size());
  for (int col = 0; col < n; ++col) {
    int pivot = col;
    for (int row = col + 1; row < n; ++row) {
      if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) pivot = row;
    }
    if (std::fabs(a[pivot][col]) < 1e-14) {
      throw std::runtime_error("ZVS filter: singular polynomial fit");
    }
    std::swap(a[pivot], a[col]);
    std::swap(b[pivot], b[col]);
    const double diagonal = a[col][col];
    for (int j = col; j < n; ++j) a[col][j] /= diagonal;
    b[col] /= diagonal;
    for (int row = 0; row < n; ++row) {
      if (row == col) continue;
      const double factor = a[row][col];
      for (int j = col; j < n; ++j) a[row][j] -= factor * a[col][j];
      b[row] -= factor * b[col];
    }
  }
  return b;
}

}  // namespace

ZvsTrajectory::ZvsTrajectory(const std::string& path, const JointMap& map,
                             ZvsTrajectoryOptions options)
    : interpolation_(options.interpolation) {
  if (!std::isfinite(options.mimic_tolerance) ||
      options.mimic_tolerance < 0.0) {
    throw std::invalid_argument("ZVS reference: invalid mimic tolerance");
  }
  zSeq sequence;
  if (!zSeqScanFile(&sequence, const_cast<char*>(path.c_str()))) {
    throw std::runtime_error("ZVS reference: cannot read '" + path + "'");
  }
  try {
    zSeqCell* cell;
    zListForEach(&sequence, cell) {
      const double dt = cell->data.dt;
      const zVec input = cell->data.v;
      if (!std::isfinite(dt) || dt <= 0.0 || input == nullptr) {
        throw std::runtime_error(
            "ZVS reference: every frame needs a finite positive dt");
      }
      const int width = zVecSizeNC(input);
      if (width != kCanonicalDof && width != kModelDof) {
        throw std::runtime_error("ZVS reference: expected 8 or 9 values, got " +
                                 std::to_string(width));
      }
      ZVector canonical(kCanonicalDof);
      if (width == kCanonicalDof) {
        zVecCopyNC(input, canonical.get());
      } else {
        const double finger_a =
            zVecElemNC(input, map.rokiOffset(kCanonicalDof - 1));
        const double finger_b = zVecElemNC(input, map.rokiOffsetFingerB());
        if (std::fabs(finger_a - finger_b) > options.mimic_tolerance) {
          throw std::runtime_error(
              "ZVS reference: finger mimic coordinates disagree");
        }
        map.reduce(input, canonical.get());
      }
      std::vector<double> q(kCanonicalDof);
      for (int i = 0; i < kCanonicalDof; ++i) {
        q[i] = canonical[i];
        if (!std::isfinite(q[i])) {
          throw std::runtime_error("ZVS reference: non-finite displacement");
        }
      }
      durations_.push_back(dt);
      positions_.push_back(std::move(q));
    }
    if (positions_.empty()) {
      throw std::runtime_error("ZVS reference: sequence is empty");
    }
    // zSeq stores scanned cells newest-first. Restore file/playback order.
    std::reverse(durations_.begin(), durations_.end());
    std::reverse(positions_.begin(), positions_.end());
    double time = 0.0;
    for (const double dt : durations_) {
      times_.push_back(time);
      time += dt;
      if (!std::isfinite(time)) {
        throw std::runtime_error("ZVS reference: duration overflow");
      }
    }
    duration_ = time;
    filterPositions(options);
    computeSlopes();
  } catch (...) {
    zSeqFree(&sequence);
    throw;
  }
  zSeqFree(&sequence);
}

void ZvsTrajectory::filterPositions(const ZvsTrajectoryOptions& options) {
  if (options.filter == ReferenceFilter::None || positions_.size() < 3) return;
  const auto original_first = positions_.front();
  const auto original_last = positions_.back();
  auto filtered = positions_;
  const int count = static_cast<int>(positions_.size());

  if (options.filter == ReferenceFilter::LowPass) {
    if (!std::isfinite(options.low_pass_cutoff_hz) ||
        options.low_pass_cutoff_hz <= 0.0) {
      throw std::invalid_argument("ZVS low-pass cutoff must be positive");
    }
    const double tau = 1.0 / (2.0 * M_PI * options.low_pass_cutoff_hz);
    for (int joint = 0; joint < kCanonicalDof; ++joint) {
      for (int i = 1; i < count; ++i) {
        const double alpha = durations_[i - 1] / (tau + durations_[i - 1]);
        filtered[i][joint] = filtered[i - 1][joint] +
                             alpha * (positions_[i][joint] -
                                      filtered[i - 1][joint]);
      }
      for (int i = count - 2; i >= 0; --i) {
        const double alpha = durations_[i] / (tau + durations_[i]);
        filtered[i][joint] = filtered[i + 1][joint] +
                             alpha * (filtered[i][joint] -
                                      filtered[i + 1][joint]);
      }
    }
  } else {
    if (options.filter_window < 3 || options.filter_window % 2 == 0 ||
        options.filter_window > count) {
      throw std::invalid_argument(
          "ZVS filter window must be odd, at least 3, and no larger than the sequence");
    }
    const int half = options.filter_window / 2;
    if (options.filter == ReferenceFilter::MovingAverage) {
      for (int i = 0; i < count; ++i) {
        const int lo = std::max(0, i - half);
        const int hi = std::min(count - 1, i + half);
        for (int joint = 0; joint < kCanonicalDof; ++joint) {
          double sum = 0.0;
          double weight = 0.0;
          for (int j = lo; j <= hi; ++j) {
            sum += durations_[j] * positions_[j][joint];
            weight += durations_[j];
          }
          filtered[i][joint] = sum / weight;
        }
      }
    } else {
      if (options.savitzky_golay_order < 1 ||
          options.savitzky_golay_order >= options.filter_window) {
        throw std::invalid_argument(
            "ZVS Savitzky-Golay order must be positive and below the window");
      }
      const int degree = options.savitzky_golay_order;
      for (int i = 0; i < count; ++i) {
        int lo = std::max(0, i - half);
        int hi = std::min(count - 1, lo + options.filter_window - 1);
        lo = std::max(0, hi - options.filter_window + 1);
        for (int joint = 0; joint < kCanonicalDof; ++joint) {
          std::vector<std::vector<double>> normal(
              degree + 1, std::vector<double>(degree + 1));
          std::vector<double> rhs(degree + 1);
          for (int row = lo; row <= hi; ++row) {
            const double x = times_[row] - times_[i];
            std::vector<double> power(2 * degree + 1, 1.0);
            for (int p = 1; p <= 2 * degree; ++p) power[p] = power[p - 1] * x;
            for (int r = 0; r <= degree; ++r) {
              rhs[r] += power[r] * positions_[row][joint];
              for (int c = 0; c <= degree; ++c) normal[r][c] += power[r + c];
            }
          }
          filtered[i][joint] = solve(std::move(normal), std::move(rhs))[0];
        }
      }
    }
  }
  filtered.front() = original_first;
  filtered.back() = original_last;
  positions_ = std::move(filtered);
}

void ZvsTrajectory::computeSlopes() {
  slopes_.assign(positions_.size(), std::vector<double>(kCanonicalDof, 0.0));
  const int n = static_cast<int>(positions_.size());
  if (n < 2 || interpolation_ == ReferenceInterpolation::Linear) return;
  for (int joint = 0; joint < kCanonicalDof; ++joint) {
    std::vector<double> delta(n - 1);
    for (int i = 0; i + 1 < n; ++i) {
      delta[i] = (positions_[i + 1][joint] - positions_[i][joint]) /
                 (times_[i + 1] - times_[i]);
    }
    slopes_[0][joint] = delta[0];
    slopes_[n - 1][joint] = delta[n - 2];
    for (int i = 1; i + 1 < n; ++i) {
      if (delta[i - 1] * delta[i] <= 0.0) {
        slopes_[i][joint] = 0.0;
      } else {
        const double h0 = times_[i] - times_[i - 1];
        const double h1 = times_[i + 1] - times_[i];
        const double w1 = 2.0 * h1 + h0;
        const double w2 = h1 + 2.0 * h0;
        slopes_[i][joint] =
            (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
      }
    }
  }
}

void ZvsTrajectory::sample(double t, zVec q, zVec dq, zVec ddq) const {
  requireOutput(q, "q");
  if (dq != nullptr) requireOutput(dq, "dq");
  if (ddq != nullptr) requireOutput(ddq, "ddq");
  const double clamped = std::clamp(t, 0.0, duration_);
  if (positions_.size() == 1 || clamped >= times_.back()) {
    for (int joint = 0; joint < kCanonicalDof; ++joint) {
      zVecElemNC(q, joint) = positions_.back()[joint];
      if (dq != nullptr) zVecElemNC(dq, joint) = 0.0;
      if (ddq != nullptr) zVecElemNC(ddq, joint) = 0.0;
    }
    return;
  }
  const auto upper = std::upper_bound(times_.begin(), times_.end(), clamped);
  const int i = std::max(0, static_cast<int>(upper - times_.begin()) - 1);
  const double h = times_[i + 1] - times_[i];
  const double u = (clamped - times_[i]) / h;
  for (int joint = 0; joint < kCanonicalDof; ++joint) {
    const double y0 = positions_[i][joint];
    const double y1 = positions_[i + 1][joint];
    if (interpolation_ == ReferenceInterpolation::Linear) {
      zVecElemNC(q, joint) = y0 + u * (y1 - y0);
      if (dq != nullptr) zVecElemNC(dq, joint) = (y1 - y0) / h;
      if (ddq != nullptr) zVecElemNC(ddq, joint) = 0.0;
      continue;
    }
    const double m0 = slopes_[i][joint];
    const double m1 = slopes_[i + 1][joint];
    const double u2 = u * u;
    const double u3 = u2 * u;
    zVecElemNC(q, joint) = (2 * u3 - 3 * u2 + 1) * y0 +
                           (u3 - 2 * u2 + u) * h * m0 +
                           (-2 * u3 + 3 * u2) * y1 +
                           (u3 - u2) * h * m1;
    if (dq != nullptr) {
      zVecElemNC(dq, joint) =
          ((6 * u2 - 6 * u) * y0 + (-6 * u2 + 6 * u) * y1) / h +
          (3 * u2 - 4 * u + 1) * m0 + (3 * u2 - 2 * u) * m1;
    }
    if (ddq != nullptr) {
      zVecElemNC(ddq, joint) =
          ((12 * u - 6) * y0 + (-12 * u + 6) * y1) / (h * h) +
          ((6 * u - 4) * m0 + (6 * u - 2) * m1) / h;
    }
  }
}

}  // namespace rtctrl::model
