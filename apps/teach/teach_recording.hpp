#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/util/async_writer.hpp"
#include "teach/teach_config.hpp"

namespace x7::teach {

namespace model = rtctrl::model;

using JointArray = std::array<double, model::kCanonicalDof>;

struct TimedPosition {
  double time_s = 0.0;
  JointArray q{};
};

inline std::vector<TimedPosition> uniformResample(
    const std::vector<TimedPosition>& input, double sample_rate_hz) {
  if (!std::isfinite(sample_rate_hz) || sample_rate_hz < 1.0 ||
      sample_rate_hz > kHardwareRateHz) {
    throw std::invalid_argument("teach resampler: invalid sample rate");
  }
  if (input.size() < 2) {
    throw std::invalid_argument("teach resampler: at least two samples needed");
  }
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (!std::isfinite(input[i].time_s) || input[i].time_s < 0.0 ||
        (i > 0 && input[i].time_s <= input[i - 1].time_s)) {
      throw std::invalid_argument(
          "teach resampler: sample times must increase strictly");
    }
    for (const double value : input[i].q) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("teach resampler: non-finite position");
      }
    }
  }
  const double period = 1.0 / sample_rate_hz;
  const long intervals =
      static_cast<long>(std::floor((input.back().time_s + 1e-12) / period));
  if (intervals < 1) {
    throw std::invalid_argument(
        "teach resampler: recording is shorter than one sample interval");
  }

  std::vector<TimedPosition> output;
  output.reserve(static_cast<std::size_t>(intervals) + 1);
  std::size_t upper = 1;
  for (long sample = 0; sample <= intervals; ++sample) {
    const double time = sample * period;
    while (upper + 1 < input.size() && input[upper].time_s < time) ++upper;
    const auto& before = input[upper - 1];
    const auto& after = input[upper];
    const double fraction =
        (time - before.time_s) / (after.time_s - before.time_s);
    TimedPosition value;
    value.time_s = time;
    for (int joint = 0; joint < model::kCanonicalDof; ++joint) {
      value.q[joint] =
          before.q[joint] + fraction * (after.q[joint] - before.q[joint]);
    }
    output.push_back(value);
  }
  return output;
}

class MotionRecorder {
 public:
  void append(double time_s, const JointArray& q) {
    if (!samples_.empty() && time_s <= samples_.back().time_s) {
      if (std::fabs(time_s - samples_.back().time_s) <= 1e-12) {
        samples_.back().q = q;
        return;
      }
      throw std::runtime_error("teach recording time moved backward");
    }
    samples_.push_back({time_s, q});
  }

  const std::vector<TimedPosition>& samples() const { return samples_; }

 private:
  std::vector<TimedPosition> samples_;
};

inline void requireNewOutput(const fs::path& path, const char* label) {
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (!error && status.type() != fs::file_type::not_found) {
    throw std::runtime_error(std::string(label) + " already exists: " +
                             path.string());
  }
  if (error && error != std::errc::no_such_file_or_directory) {
    throw std::runtime_error("cannot inspect " + path.string() + ": " +
                             error.message());
  }
  if (!path.parent_path().empty()) {
    fs::create_directories(path.parent_path());
  }
}

class ExclusiveZvsOutput {
 public:
  explicit ExclusiveZvsOutput(fs::path path) : path_(std::move(path)) {
    file_ = std::fopen(path_.string().c_str(), "wx");
    if (file_ == nullptr) {
      throw std::runtime_error("cannot create motion output " +
                               path_.string());
    }
  }

  ExclusiveZvsOutput(const ExclusiveZvsOutput&) = delete;
  ExclusiveZvsOutput& operator=(const ExclusiveZvsOutput&) = delete;

  ~ExclusiveZvsOutput() {
    if (file_ != nullptr) std::fclose(file_);
    if (!committed_) {
      std::error_code ignored;
      fs::remove(path_, ignored);
    }
  }

  int write(const std::vector<TimedPosition>& samples, double sample_rate_hz,
            const model::JointMap& map) {
    if (committed_ || file_ == nullptr) {
      throw std::runtime_error("motion output is already finished");
    }
    const double period = 1.0 / sample_rate_hz;
    model::ZVector canonical(model::kCanonicalDof);
    model::ZVector expanded(model::kModelDof);
    for (const auto& sample : samples) {
      for (int joint = 0; joint < model::kCanonicalDof; ++joint) {
        canonical[joint] = sample.q[joint];
      }
      map.expand(canonical.get(), expanded.get());
      std::fprintf(file_, "%.15g ", period);
      zVecFPrint(file_, expanded.get());
    }
    if (std::fflush(file_) != 0 || std::fclose(file_) != 0) {
      file_ = nullptr;
      throw std::runtime_error("cannot finish motion output " +
                               path_.string());
    }
    file_ = nullptr;
    committed_ = true;
    return static_cast<int>(samples.size());
  }

 private:
  fs::path path_;
  std::FILE* file_ = nullptr;
  bool committed_ = false;
};

enum class Phase { AwaitStart, Recording, AwaitSupport };
enum class Event { None, Start, Stop, DurationStop, Support };

inline const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::AwaitStart: return "await_start";
    case Phase::Recording: return "recording";
    case Phase::AwaitSupport: return "await_support";
  }
  return "unknown";
}

inline const char* eventName(Event event) {
  switch (event) {
    case Event::None: return "none";
    case Event::Start: return "start";
    case Event::Stop: return "stop";
    case Event::DurationStop: return "duration_stop";
    case Event::Support: return "support";
  }
  return "none";
}

struct LogRow {
  double session_time_s = 0.0;
  double recording_time_s = -1.0;
  double feedback_time_s = 0.0;
  std::uint64_t feedback_seq = 0;
  Phase phase = Phase::AwaitStart;
  Event event = Event::None;
  bool command_valid = false;
  std::uint64_t submitted_seq = 0;
  double submission_time_s = 0.0;
  bool receipt_accepted = false;
  bool applied_valid = false;
  std::uint64_t applied_seq = 0;
  double latest_apply_time_s = 0.0;
  JointArray q{};
  JointArray dq{};
  JointArray tau{};
  JointArray tau_request{};
  JointArray tau_applied{};
  std::array<int, model::kCanonicalDof> clamped{};
  std::array<int, model::kCanonicalDof> gated{};
};

// Rows are queued through an AsyncWriter, so the acquisition loop never
// blocks on file I/O. A dropped row or failed write latches in the writer
// and is reported by finish(); the session itself is not aborted for it.
class TeachCsvLog {
 public:
  TeachCsvLog(const fs::path& path, Mode mode, double sample_rate_hz)
      : writer_(openExclusive(path), rtctrl::util::AsyncWriter::AdoptStream{}) {
    writer_.appendf("# format: rtctrl-x7-teach-log\n# version: 1\n");
    writer_.appendf("# mode: %s\n# acquisition_rate_hz: 100\n", modeName(mode));
    writer_.appendf("# output_sample_rate_hz: %.17g\n", sample_rate_hz);
    writer_.appendf(
        "schema_version,session_time_s,recording_time_s,phase,event,"
        "mode,feedback_time_s,feedback_seq,command_valid,"
        "submitted_seq,submission_time_s,receipt_accepted,"
        "applied_valid,applied_seq,latest_apply_time_s");
    for (int joint = 0; joint < model::kCanonicalDof; ++joint) {
      writer_.appendf(
          ",q%d_rad,dq%d_rad_s,tau%d_nm,tau_request%d_nm,"
          "tau_applied%d_nm,clamped%d,gated%d",
          joint, joint, joint, joint, joint, joint, joint);
    }
    writer_.appendf("\n");
    (void)writer_.commitRow();  // the header block as one queued row
  }

  TeachCsvLog(const TeachCsvLog&) = delete;
  TeachCsvLog& operator=(const TeachCsvLog&) = delete;

  void row(const LogRow& row, Mode mode) {
    writer_.appendf(
        "1,%.9f,%.9f,%s,%s,%s,%.9f,%llu,%d,%llu,%.9f,%d,%d,"
        "%llu,%.9f",
        row.session_time_s, row.recording_time_s, phaseName(row.phase),
        eventName(row.event), modeName(mode), row.feedback_time_s,
        static_cast<unsigned long long>(row.feedback_seq),
        row.command_valid ? 1 : 0,
        static_cast<unsigned long long>(row.submitted_seq),
        row.submission_time_s, row.receipt_accepted ? 1 : 0,
        row.applied_valid ? 1 : 0,
        static_cast<unsigned long long>(row.applied_seq),
        row.latest_apply_time_s);
    for (int joint = 0; joint < model::kCanonicalDof; ++joint) {
      writer_.appendf(",%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d", row.q[joint],
                      row.dq[joint], row.tau[joint], row.tau_request[joint],
                      row.tau_applied[joint], row.clamped[joint],
                      row.gated[joint]);
    }
    writer_.appendf("\n");
    (void)writer_.commitRow();
  }

  // Appends the status, then writes out every queued row and closes the
  // file (blocks on I/O: call it after the session). Throws if any row
  // was dropped or any write failed.
  void finish(const std::string& status) {
    writer_.appendf("# status: %s\n", status.c_str());
    (void)writer_.commitRow();
    if (!writer_.close()) {
      throw std::runtime_error("cannot finish teach log");
    }
  }

 private:
  static std::FILE* openExclusive(const fs::path& path) {
    std::FILE* file = std::fopen(path.string().c_str(), "wx");
    if (file == nullptr) {
      throw std::runtime_error("cannot create teach log " + path.string());
    }
    return file;
  }

  rtctrl::util::AsyncWriter writer_;
};

}  // namespace x7::teach
