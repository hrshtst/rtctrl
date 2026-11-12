// Slotine and Li's adaptive controller in position form: a SIMULATION-ONLY
// study (the controllers: study/adaptive_position.hpp; the theory:
// docs/theory/learning-control.md, "Slotine and Li adaptive control in
// position form"). Repeated passes of one follow reference on the simulated
// arm of x7_follow_sim (follow/follow_sim_plant.hpp: the same SimArm, in
// LaggedArm for the servo-like keys). Each pass, on a fresh simulated arm
// at the configured initial posture:
//
//   home     the plain position command: the follower's minimum-jerk (the
//            config's home profile) move to q_d(0) and its settle, without
//            the follower's home corrections (FollowRun's homing is private
//            to its run);
//   ramp     the follower's offset ramp: from q_d(0) to q_d(0) + delta_0 at
//            learning.max_offset_rate_rad_s, then home.settle_time_s there,
//            with delta_0 the carried weights on the desired regressor row
//            of the first sample (both forms); --no-offset-ramp skips it;
//   tracking q_d + delta with the in-loop adaptive offset;
//   hold     1 s at q_d(T) + delta, the controller still in the loop with
//            the reference held at its last sample.
//
// The weights adapt on the cycles whose paired regressor row (Delta ago)
// is a tracking row: tracking cycles from Delta on and the first Delta of
// the hold, the window of the trial-wise batch rule. They carry over
// between passes; the regressor history does not.
//
// There is no bus code here and no way to reach the real arm: the follow
// config's hardware file is read for the simulated effort limits only.
//
// Usage: x7_adaptive_follow_study --config FOLLOW_SIM.toml
//          --form desired|proper --gamma G --out DIR [--reference FILE]
//          [--lambda L] [--lead T_S] [--advance DELTA] [--passes P]
//          [--command-delay-cycles N] [--no-offset-ramp]
//          [--initial-weights WEIGHTS.csv]
//
// --initial-weights starts from the weights of the last pass in a
// weights.csv of an earlier run (with --gamma 0 they stay frozen).
//
// Writes DIR/summary.csv (one row per pass), DIR/weights.csv (the weights
// after each pass), DIR/regressor_scale.csv (per pass, the mean square of
// each regressor column as used in the loop over its desired-trajectory
// mean square: the factor by which the effective adaptation rate of that
// weight differs from gamma), DIR/last_pass.csv (every cycle of the last
// pass run) and DIR/settings.txt, overwriting them.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "follow/follow_config.hpp"
#include "follow/follow_sim_plant.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/learn/command_offset.hpp"
#include "rtctrl/learn/regressor_feedforward.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/ptp_planner.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "study/adaptive_position.hpp"

namespace x7::gitrev {
extern const char* const kBuildSha;
extern const bool kBuildDirty;
}  // namespace x7::gitrev

namespace {

namespace arm = rtctrl::arm;
namespace follow = x7::follow;
namespace fs = std::filesystem;
namespace learn = rtctrl::learn;
namespace model = rtctrl::model;
namespace study = x7::study;
using learn::JointArray;
using learn::RegressorRow;

constexpr int kDof = model::kCanonicalDof;
constexpr int kArm = study::kAdaptiveArmJoints;
constexpr double kHoldS = 1.0;            // the hold after tracking
constexpr double kSplitHz = 3.0;          // follow_learnability's band split
constexpr double kDivergenceRatio = 3.0;  // pass RMS against pass 0's

enum class Form { Desired, Proper };

const char* formName(Form form) {
  return form == Form::Desired ? "desired" : "proper";
}

struct Options {
  fs::path config_path;
  std::optional<fs::path> reference_path;
  std::optional<Form> form;
  std::optional<double> gamma;
  std::optional<double> lambda;
  double lead_s = 0.0;
  double advance_s = 0.05;
  int passes = 8;
  std::optional<int> command_delay_cycles;
  bool offset_ramp = true;
  std::optional<fs::path> initial_weights;
  fs::path out;
  bool help = false;
};

void usage() {
  std::printf(
      "usage: x7_adaptive_follow_study --config FOLLOW_SIM.toml\n"
      "         --form desired|proper --gamma G --out DIR [options]\n"
      "  simulation only: there is no bus and no --port\n"
      "  --reference FILE           override the config's reference\n"
      "  --lambda L                 proper form: dq_r = dq_d + L e [1/s]\n"
      "                             (default 0)\n"
      "  --lead T_S                 error signal e + T_S edot [s] "
      "(default 0)\n"
      "  --advance DELTA            pair phi(t - DELTA) with s(t) [s] "
      "(default 0.05)\n"
      "  --passes P                 repeated passes (default 8)\n"
      "  --command-delay-cycles N   override "
      "simulation.command_delay_cycles\n"
      "  --no-offset-ramp           start tracking without the ramp to "
      "q_d(0) + delta_0\n"
      "  --initial-weights FILE     start from the last pass of a "
      "weights.csv\n");
}

double parseNumber(const std::string& option, const char* text) {
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(value)) {
    throw std::runtime_error(option + " requires a finite number");
  }
  return value;
}

int parseInteger(const std::string& option, const char* text) {
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || value < 0 || value > 1000000) {
    throw std::runtime_error(option + " requires a non-negative integer");
  }
  return static_cast<int>(value);
}

Options parseOptions(int argc, char* argv[]) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      options.help = true;
      continue;
    }
    if (arg == "--no-offset-ramp") {
      options.offset_ramp = false;
      continue;
    }
    const bool valued = arg == "--config" || arg == "--reference" ||
                        arg == "--form" || arg == "--gamma" ||
                        arg == "--lambda" || arg == "--lead" ||
                        arg == "--advance" || arg == "--passes" ||
                        arg == "--command-delay-cycles" || arg == "--out" ||
                        arg == "--initial-weights";
    if (!valued) {
      throw std::runtime_error("unknown option: " + arg +
                               " (this study is simulation only)");
    }
    if (i + 1 >= argc) throw std::runtime_error(arg + " requires a value");
    const char* value = argv[++i];
    if (arg == "--config") {
      options.config_path = value;
    } else if (arg == "--reference") {
      options.reference_path = fs::absolute(value).lexically_normal();
    } else if (arg == "--form") {
      const std::string form = value;
      if (form == "desired") {
        options.form = Form::Desired;
      } else if (form == "proper") {
        options.form = Form::Proper;
      } else {
        throw std::runtime_error("--form must be desired or proper");
      }
    } else if (arg == "--gamma") {
      options.gamma = parseNumber(arg, value);
    } else if (arg == "--lambda") {
      options.lambda = parseNumber(arg, value);
    } else if (arg == "--lead") {
      options.lead_s = parseNumber(arg, value);
    } else if (arg == "--advance") {
      options.advance_s = parseNumber(arg, value);
    } else if (arg == "--passes") {
      options.passes = parseInteger(arg, value);
    } else if (arg == "--command-delay-cycles") {
      options.command_delay_cycles = parseInteger(arg, value);
    } else if (arg == "--initial-weights") {
      options.initial_weights = value;
    } else {
      options.out = value;
    }
  }
  if (options.help) return options;
  if (options.config_path.empty())
    throw std::runtime_error("--config is required");
  if (!options.form) throw std::runtime_error("--form is required");
  if (!options.gamma) throw std::runtime_error("--gamma is required");
  if (options.out.empty()) throw std::runtime_error("--out is required");
  if (*options.gamma < 0.0) throw std::runtime_error("--gamma must be >= 0");
  if (options.lambda && *options.form != Form::Proper) {
    throw std::runtime_error("--lambda applies to the proper form only");
  }
  if (options.lambda && *options.lambda < 0.0) {
    throw std::runtime_error("--lambda must be >= 0");
  }
  if (options.lead_s < 0.0) throw std::runtime_error("--lead must be >= 0");
  if (options.advance_s < 0.0) {
    throw std::runtime_error("--advance must be >= 0");
  }
  if (options.passes < 1) throw std::runtime_error("--passes must be >= 1");
  return options;
}

bool finite(const JointArray& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double v) { return std::isfinite(v); });
}

bool finite(const study::RegressorMatrix& w) {
  for (const auto& row : w) {
    for (const double v : row) {
      if (!std::isfinite(v)) return false;
    }
  }
  return true;
}

// One logged cycle of a pass.
struct LogRow {
  const char* phase;
  double time_s;
  JointArray qref, q, command, delta;
};

struct PassResult {
  bool diverged = false;
  std::string reason;
  JointArray rms{};
  double rms_all = 0.0;
  double rms_arm = 0.0;
  JointArray above{};  // RMS above kSplitHz per joint
  double above_all = 0.0;
  double above_arm = 0.0;
  double worst_abs_error = 0.0;
  int worst_joint = -1;
  double max_abs_offset = 0.0;
  double max_offset_rate = 0.0;
  long clamped_cycles = 0;
  double hold_rms_all = 0.0;
  study::RegressorMatrix weights{};
  // Sum over the adaptive cycles of each column's square as used, and the
  // number of those cycles.
  study::RegressorMatrix phi_square_sum{};
  long adaptive_cycles = 0;
  std::vector<LogRow> log;
};

// Everything one pass needs that does not change between passes.
struct Study {
  Options options;
  Form form = Form::Desired;
  double lambda = 0.0;
  follow::Config config;
  arm::SimArm::Options sim_options;
  std::optional<x7::LaggedArmOptions> lag;
  double dt = 0.01;
  learn::TrackingGrid grid;
  std::vector<JointArray> qref;     // the reference on the tracking grid
  learn::SmoothedReference smooth;  // its smoothed q, dq, ddq
  std::vector<RegressorRow> desired;
  learn::RegressorOptions regressors;
  std::size_t advance_cycles = 0;
  double max_offset = 0.15;
};

class PassRunner {
 public:
  PassRunner(const Study& study, study::AdaptiveOffset* controller,
             study::ModelProducts* products)
      : s_(study),
        controller_(controller),
        products_(products),
        robot_(study.sim_options) {
    if (!robot_.setMode(study.config.mode) || !robot_.activate()) {
      throw std::runtime_error("simulation arm activation failed");
    }
    if (study.lag) {
      lagged_ = std::make_unique<x7::LaggedArm>(
          robot_, follow::kServoVelocityLagS, *study.lag);
    }
    plant_ = lagged_ ? static_cast<arm::Arm*>(lagged_.get()) : &robot_;
  }

  PassResult run() {
    PassResult result;
    controller_->startPass();
    arm::JointState initial;
    if (!plant_->readState(initial))
      throw std::runtime_error("readState failed");
    origin_ = initial.t;
    JointArray start{};
    for (int j = 0; j < kDof; ++j) start[j] = initial.q[j];
    const JointArray& home = s_.qref.front();
    homeMove(start, home);
    JointArray ramp_delta{};
    if (s_.options.offset_ramp) {
      ramp_delta = controller_->offset(s_.desired.front()).delta;
      ramp(home, ramp_delta);
    }
    previous_delta_ = ramp_delta;
    const std::size_t n = s_.grid.samples();
    std::vector<std::vector<double>> error(kDof, std::vector<double>(n));
    for (std::size_t i = 0; i < n && !result.diverged; ++i) {
      JointArray e{};
      if (!adaptiveCycle("tracking", i, true, &e, &result)) break;
      for (int j = 0; j < kDof; ++j) {
        error[j][i] = e[j];
        if (std::fabs(e[j]) > result.worst_abs_error) {
          result.worst_abs_error = std::fabs(e[j]);
          result.worst_joint = j;
        }
      }
    }
    double hold_sum = 0.0;
    const long hold = static_cast<long>(std::ceil(kHoldS / s_.dt));
    for (long k = 0; k < hold && !result.diverged; ++k) {
      JointArray e{};
      if (!adaptiveCycle("hold", n - 1, false, &e, &result)) break;
      for (int j = 0; j < kDof; ++j) hold_sum += e[j] * e[j];
    }
    robot_.deactivate();
    result.weights = controller_->weights();
    result.log = std::move(log_);
    if (result.diverged) return result;
    for (int j = 0; j < kDof; ++j) {
      result.rms[j] = study::seriesRms(error[j]);
      result.above[j] =
          study::seriesRms(study::aboveBand(error[j], s_.dt, kSplitHz));
    }
    result.rms_all = study::aggregateRms(result.rms, kDof);
    result.rms_arm = study::aggregateRms(result.rms, kArm);
    result.above_all = study::aggregateRms(result.above, kDof);
    result.above_arm = study::aggregateRms(result.above, kArm);
    result.hold_rms_all =
        std::sqrt(hold_sum / static_cast<double>(hold * kDof));
    return result;
  }

 private:
  struct Feedback {
    JointArray q{}, dq{};
    double t = 0.0;
  };

  Feedback read() {
    arm::JointState state;
    if (!plant_->readState(state)) throw std::runtime_error("readState failed");
    Feedback feedback;
    for (int j = 0; j < kDof; ++j) {
      feedback.q[j] = state.q[j];
      feedback.dq[j] = state.dq[j];
    }
    feedback.t = state.t - origin_;
    return feedback;
  }

  void send(const char* phase, const Feedback& feedback, const JointArray& qref,
            const JointArray& command, const JointArray& delta) {
    arm::JointCommand cmd;
    cmd.mode = s_.config.mode;
    for (int j = 0; j < kDof; ++j) cmd.q[j] = command[j];
    if (!plant_->writeCommand(cmd) || !plant_->step()) {
      throw std::runtime_error("simulation step failed");
    }
    log_.push_back({phase, feedback.t, qref, feedback.q, command, delta});
  }

  void plainCycle(const char* phase, const JointArray& qref,
                  const JointArray& command) {
    const Feedback feedback = read();
    JointArray delta{};
    for (int j = 0; j < kDof; ++j) delta[j] = command[j] - qref[j];
    send(phase, feedback, qref, command, delta);
  }

  // FollowRun's homeDuration, runPtp and settleHome, without the
  // correction retries.
  void homeMove(const JointArray& start, const JointArray& home) {
    const auto& h = s_.config.home;
    double widest = 0.0;
    for (int j = 0; j < kDof; ++j) {
      widest = std::max(widest, std::fabs(home[j] - start[j]));
    }
    const double duration =
        std::max({s_.dt,
                  model::ptpPeakSpeedFactor(h.profile,
                                            h.trapezoid_acceleration_fraction) *
                      widest / h.velocity_limit,
                  h.motion_time.value_or(0.0)});
    const long intervals =
        std::max(1L, static_cast<long>(std::ceil(duration / s_.dt)));
    const double step_dt = duration / static_cast<double>(intervals);
    JointArray q{};
    for (long step = 0; step <= intervals; ++step) {
      const double t = std::min(duration, static_cast<double>(step) * step_dt);
      const auto progress = model::ptpProgress(
          h.profile, t / duration, h.trapezoid_acceleration_fraction);
      for (int j = 0; j < kDof; ++j) {
        q[j] = start[j] + (home[j] - start[j]) * progress.position;
      }
      plainCycle("home", q, q);
    }
    const long settle = static_cast<long>(std::ceil(h.settle_time_s / s_.dt));
    for (long k = 0; k < settle; ++k) plainCycle("home", home, home);
  }

  // FollowRun's rampOffset from the homing goal.
  void ramp(const JointArray& home, const JointArray& delta) {
    JointArray target{};
    double widest = 0.0;
    for (int j = 0; j < kDof; ++j) {
      target[j] = home[j] + delta[j];
      widest = std::max(widest, std::fabs(delta[j]));
    }
    const long steps = static_cast<long>(
        std::ceil(widest / s_.config.learning.max_offset_rate_rad_s / s_.dt));
    JointArray q{};
    for (long k = 1; k <= steps; ++k) {
      if (k == steps) {
        q = target;
      } else {
        const double fraction =
            static_cast<double>(k) / static_cast<double>(steps);
        for (int j = 0; j < kDof; ++j) q[j] = home[j] + delta[j] * fraction;
      }
      plainCycle("offset_ramp", home, q);
    }
    const long hold =
        static_cast<long>(std::ceil(s_.config.home.settle_time_s / s_.dt));
    for (long k = 0; k < hold; ++k) plainCycle("offset_ramp", home, target);
  }

  // One cycle with the adaptive offset at grid sample i (the hold uses the
  // last sample). False (result marked diverged) on a non-finite value.
  bool adaptiveCycle(const char* phase, std::size_t i, bool adaptable,
                     JointArray* e, PassResult* result) {
    const Feedback feedback = read();
    if (!finite(feedback.q) || !finite(feedback.dq)) {
      result->diverged = true;
      result->reason = "non-finite feedback";
      return false;
    }
    const JointArray& qd = s_.qref[i];
    const JointArray& dqd = s_.smooth.dq[i];
    const JointArray& ddqd = s_.smooth.ddq[i];
    JointArray signal{};
    for (int j = 0; j < kDof; ++j) {
      (*e)[j] = qd[j] - feedback.q[j];
      signal[j] = (*e)[j] + s_.options.lead_s * (dqd[j] - feedback.dq[j]);
    }
    const RegressorRow phi =
        s_.form == Form::Desired
            ? s_.desired[i]
            : study::properRegressorRow(*products_, feedback.q, feedback.dq, qd,
                                        dqd, ddqd, s_.lambda,
                                        s_.regressors.coulomb_v0_rad_s);
    for (int j = 0; j < kArm; ++j) {
      for (int l = 0; l < learn::kRegressorCount; ++l) {
        result->phi_square_sum[j][l] += phi[j][l] * phi[j][l];
      }
    }
    ++result->adaptive_cycles;
    const auto cycle = controller_->cycle(phi, signal, adaptable);
    if (!finite(cycle.delta) || !finite(controller_->weights())) {
      result->diverged = true;
      result->reason = "non-finite offset or weights";
      return false;
    }
    JointArray command{};
    for (int j = 0; j < kDof; ++j) command[j] = qd[j] + cycle.delta[j];
    for (int j = 0; j < kArm; ++j) {
      result->max_abs_offset =
          std::max(result->max_abs_offset, std::fabs(cycle.delta[j]));
      result->max_offset_rate =
          std::max(result->max_offset_rate,
                   std::fabs(cycle.delta[j] - previous_delta_[j]) / s_.dt);
    }
    previous_delta_ = cycle.delta;
    if (cycle.any_clamped) ++result->clamped_cycles;
    send(phase, feedback, qd, command, cycle.delta);
    return true;
  }

  const Study& s_;
  study::AdaptiveOffset* controller_;
  study::ModelProducts* products_;
  arm::SimArm robot_;
  std::unique_ptr<x7::LaggedArm> lagged_;
  arm::Arm* plant_ = nullptr;
  double origin_ = 0.0;
  JointArray previous_delta_{};
  std::vector<LogRow> log_;
};

std::FILE* create(const fs::path& path) {
  std::FILE* file = std::fopen(path.string().c_str(), "w");
  if (file == nullptr) {
    throw std::runtime_error("cannot write " + path.string());
  }
  return file;
}

void close(std::FILE* file, const fs::path& path) {
  if (std::ferror(file) != 0 || std::fclose(file) != 0) {
    throw std::runtime_error("writing " + path.string() + " failed");
  }
}

void writeSummaryHeader(std::FILE* f) {
  std::fprintf(f, "pass,status,rms_rad,rms_arm_rad");
  for (int j = 0; j < kDof; ++j) std::fprintf(f, ",rms%d_rad", j);
  std::fprintf(f, ",above3hz_rms_rad,above3hz_rms_arm_rad");
  for (int j = 0; j < kDof; ++j) std::fprintf(f, ",above3hz_rms%d_rad", j);
  std::fprintf(f,
               ",worst_abs_error_rad,worst_joint,max_abs_offset_rad,"
               "max_offset_rate_rad_s,clamped_cycles,hold_rms_rad\n");
}

void writeSummaryRow(std::FILE* f, int pass, const PassResult& r) {
  std::fprintf(f, "%d,%s,%.9g,%.9g", pass, r.diverged ? "diverged" : "ok",
               r.rms_all, r.rms_arm);
  for (const double v : r.rms) std::fprintf(f, ",%.9g", v);
  std::fprintf(f, ",%.9g,%.9g", r.above_all, r.above_arm);
  for (const double v : r.above) std::fprintf(f, ",%.9g", v);
  std::fprintf(f, ",%.9g,%d,%.9g,%.9g,%ld,%.9g\n", r.worst_abs_error,
               r.worst_joint, r.max_abs_offset, r.max_offset_rate,
               r.clamped_cycles, r.hold_rms_all);
}

void writeWeights(std::FILE* f, int pass, const study::RegressorMatrix& w) {
  for (int j = 0; j < kDof; ++j) {
    std::fprintf(f, "%d,%d", pass, j);
    for (const double v : w[j]) std::fprintf(f, ",%.17g", v);
    std::fprintf(f, "\n");
  }
}

// mean(phi^2) as used over mean(phi^2) on the desired trajectory; empty
// for a column that is not adapted.
void writeRegressorScale(std::FILE* f, int pass, const PassResult& r,
                         const study::AdaptationGains& gains) {
  for (int j = 0; j < kArm; ++j) {
    std::fprintf(f, "%d,%d", pass, j);
    for (int l = 0; l < learn::kRegressorCount; ++l) {
      if (!gains.adapted[j][l] || r.adaptive_cycles == 0) {
        std::fprintf(f, ",");
        continue;
      }
      const double used =
          r.phi_square_sum[j][l] / static_cast<double>(r.adaptive_cycles);
      std::fprintf(f, ",%.6g", used / (gains.rms[j][l] * gains.rms[j][l]));
    }
    std::fprintf(f, "\n");
  }
}

// The weights of the last pass in a weights.csv this study wrote.
study::RegressorMatrix readWeights(const fs::path& path) {
  std::FILE* f = std::fopen(path.string().c_str(), "r");
  if (f == nullptr) throw std::runtime_error("cannot read " + path.string());
  char line[4096];
  if (std::fgets(line, sizeof line, f) == nullptr ||
      std::strncmp(line, "pass,joint,w_g,", 15) != 0) {
    std::fclose(f);
    throw std::runtime_error(path.string() + ": not a study weights.csv");
  }
  study::RegressorMatrix w{};
  int last_pass = -1;
  std::array<bool, kDof> seen{};
  while (std::fgets(line, sizeof line, f) != nullptr) {
    int pass = 0;
    int joint = 0;
    double v[learn::kRegressorCount];
    if (std::sscanf(line, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf", &pass, &joint, &v[0],
                    &v[1], &v[2], &v[3], &v[4], &v[5]) != 8 ||
        joint < 0 || joint >= kDof || pass < 0) {
      std::fclose(f);
      throw std::runtime_error(path.string() + ": malformed row");
    }
    if (pass > last_pass) {
      last_pass = pass;
      seen.fill(false);
    }
    if (pass < last_pass) continue;
    for (int l = 0; l < learn::kRegressorCount; ++l) w[joint][l] = v[l];
    seen[joint] = true;
  }
  std::fclose(f);
  if (last_pass < 0 ||
      !std::all_of(seen.begin(), seen.end(), [](bool b) { return b; })) {
    throw std::runtime_error(path.string() + ": incomplete last pass");
  }
  return w;
}

void writeLog(const fs::path& path, const std::vector<LogRow>& log) {
  std::FILE* f = create(path);
  std::fprintf(f, "time_s,phase");
  for (int j = 0; j < kDof; ++j) {
    std::fprintf(f, ",qref%d_rad,q%d_rad,qcmd%d_rad,offset%d_rad", j, j, j, j);
  }
  std::fprintf(f, "\n");
  for (const auto& row : log) {
    std::fprintf(f, "%.6f,%s", row.time_s, row.phase);
    for (int j = 0; j < kDof; ++j) {
      std::fprintf(f, ",%.9g,%.9g,%.9g,%.9g", row.qref[j], row.q[j],
                   row.command[j], row.delta[j]);
    }
    std::fprintf(f, "\n");
  }
  close(f, path);
}

void writeSettings(const fs::path& path, const Study& s,
                   const study::AdaptationGains& gains, int argc,
                   char* argv[]) {
  std::FILE* f = create(path);
  std::fprintf(f, "invocation =");
  for (int i = 0; i < argc; ++i) std::fprintf(f, " %s", argv[i]);
  std::fprintf(f, "\nbuild_sha = %s%s\n", x7::gitrev::kBuildSha,
               x7::gitrev::kBuildDirty ? " (dirty)" : "");
  std::fprintf(f, "config = %s\nreference = %s\nmodel = %s\n",
               s.config.source_path.string().c_str(),
               s.config.reference_path.string().c_str(),
               s.config.model_path.string().c_str());
  std::fprintf(f, "form = %s\ngamma_per_s = %.17g\nlambda_per_s = %.17g\n",
               formName(s.form), *s.options.gamma, s.lambda);
  std::fprintf(f, "lead_s = %.17g\nadvance_s = %.17g\nadvance_cycles = %zu\n",
               s.options.lead_s, s.options.advance_s, s.advance_cycles);
  std::fprintf(f, "passes = %d\noffset_ramp = %s\nhold_s = %g\n",
               s.options.passes, s.options.offset_ramp ? "yes" : "no", kHoldS);
  std::fprintf(f, "initial_weights = %s\n",
               s.options.initial_weights
                   ? s.options.initial_weights->string().c_str()
                   : "zero");
  std::fprintf(f, "max_offset_rad = %.17g\ncontrol_dt_s = %.17g\n",
               s.max_offset, s.dt);
  std::fprintf(f, "grid_samples = %zu\ngrid_dt_s = %.17g\n", s.grid.samples(),
               s.grid.dt_s);
  std::fprintf(f, "command_delay_cycles = %d\nquantize_encoder = %s\n",
               s.config.simulation.command_delay_cycles,
               s.config.simulation.quantize_encoder ? "yes" : "no");
  std::fprintf(f,
               "regressor_smooth_sigma_s = %.17g\ncoulomb_v0_rad_s = %.17g\n",
               s.regressors.smooth_sigma_s, s.regressors.coulomb_v0_rad_s);
  std::fprintf(f, "kp8 =");
  for (const double v : s.sim_options.kp8) std::fprintf(f, " %g", v);
  std::fprintf(f, "\nkd8 =");
  for (const double v : s.sim_options.kd8) std::fprintf(f, " %g", v);
  std::fprintf(f, "\n# desired-regressor column RMS (adapted: *), per joint:");
  for (const auto name : learn::kRegressorNames) {
    std::fprintf(f, " %.*s", static_cast<int>(name.size()), name.data());
  }
  std::fprintf(f, "\n");
  for (int j = 0; j < kDof; ++j) {
    std::fprintf(f, "rms_joint%d =", j);
    for (int l = 0; l < learn::kRegressorCount; ++l) {
      std::fprintf(f, " %.4g%s", gains.rms[j][l],
                   gains.adapted[j][l] ? "*" : "");
    }
    std::fprintf(f, "\n");
  }
  close(f, path);
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const Options options = parseOptions(argc, argv);
    if (options.help) {
      usage();
      return 0;
    }
    Study s;
    s.options = options;
    s.form = *options.form;
    s.lambda = options.lambda.value_or(0.0);
    s.config = follow::loadConfig(options.config_path);
    if (options.reference_path)
      s.config.reference_path = *options.reference_path;
    if (options.command_delay_cycles) {
      s.config.simulation.command_delay_cycles = *options.command_delay_cycles;
    }
    if (s.config.mode != arm::ControlMode::Position) {
      throw std::runtime_error("the study runs in position mode only");
    }
    model::ChainModel chain(s.config.model_path.string());
    model::JointMap map(chain);
    const auto hardware =
        rtctrl::hw::Config::load(s.config.hardware_config_path.string());
    // Before any dynamics evaluation on the chain (see simArmOptions).
    s.sim_options = follow::simArmOptions(s.config, hardware, chain, map);
    s.lag = follow::laggedArmOptions(s.config);
    s.dt = 1.0 / s.config.control_rate_hz;
    s.max_offset = s.config.learning.max_offset_rad;
    model::ZvsTrajectory reference(s.config.reference_path.string(), map,
                                   s.config.reference);
    s.grid = learn::trackingGrid(reference.duration(), s.dt);
    model::ZVector q(kDof);
    s.qref.resize(s.grid.samples());
    for (std::size_t i = 0; i < s.grid.samples(); ++i) {
      const double t =
          std::min(reference.duration(), static_cast<double>(i) * s.grid.dt_s);
      reference.sample(t, q.get());
      for (int j = 0; j < kDof; ++j) s.qref[i][j] = q[j];
    }
    s.desired =
        learn::buildRegressors(chain, map, s.qref, s.grid.dt_s, s.regressors);
    s.smooth = learn::smoothReference(s.qref, s.grid.dt_s,
                                      s.regressors.smooth_sigma_s);
    s.advance_cycles = study::advanceCycles(options.advance_s, s.dt);
    const auto gains = study::normalisedGains(s.desired, *options.gamma);
    study::AdaptiveOffset controller(gains, s.dt, s.advance_cycles,
                                     s.max_offset);
    if (options.initial_weights) {
      controller.setWeights(readWeights(*options.initial_weights));
    }
    study::ModelProducts products(chain, map);

    std::error_code ec;
    fs::create_directories(options.out, ec);
    if (ec) {
      throw std::runtime_error("cannot create " + options.out.string());
    }
    writeSettings(options.out / "settings.txt", s, gains, argc, argv);
    const fs::path summary_path = options.out / "summary.csv";
    const fs::path weights_path = options.out / "weights.csv";
    const fs::path scale_path = options.out / "regressor_scale.csv";
    std::FILE* summary = create(summary_path);
    std::FILE* weights = create(weights_path);
    std::FILE* scale = create(scale_path);
    writeSummaryHeader(summary);
    std::fprintf(weights, "pass,joint");
    for (const auto name : learn::kRegressorNames) {
      std::fprintf(weights, ",w_%.*s", static_cast<int>(name.size()),
                   name.data());
    }
    std::fprintf(weights, "\n");
    std::fprintf(scale, "pass,joint");
    for (const auto name : learn::kRegressorNames) {
      std::fprintf(scale, ",%.*s", static_cast<int>(name.size()), name.data());
    }
    std::fprintf(scale, "\n");

    std::printf(
        "adaptive follow study (simulation only): form %s, gamma %g 1/s, "
        "lambda %g 1/s, lead %g s, advance %zu cycles, delay %d cycles, "
        "offset ramp %s, %zu tracking samples\n",
        formName(s.form), *options.gamma, s.lambda, options.lead_s,
        s.advance_cycles, s.config.simulation.command_delay_cycles,
        options.offset_ramp ? "on" : "off", s.grid.samples());
    double rms0 = 0.0;
    int passes_run = 0;
    bool diverged = false;
    std::vector<LogRow> last_log;
    for (int pass = 0; pass < options.passes; ++pass) {
      PassRunner runner(s, &controller, &products);
      PassResult result = runner.run();
      ++passes_run;
      if (!result.diverged) {
        if (pass == 0) rms0 = result.rms_all;
        if (pass > 0 && result.rms_all > kDivergenceRatio * rms0) {
          result.diverged = true;
          result.reason = "RMS above 3 x pass 0's";
        }
      }
      writeSummaryRow(summary, pass, result);
      writeWeights(weights, pass, result.weights);
      writeRegressorScale(scale, pass, result, gains);
      std::printf(
          "pass %d: rms %.3g mrad (arm %.3g), >3Hz %.3g mrad, worst %.3g "
          "mrad (joint %d), max|delta| %.3g mrad, rate %.3g rad/s, "
          "clamped %ld, hold %.3g mrad%s%s\n",
          pass, result.rms_all * 1e3, result.rms_arm * 1e3,
          result.above_all * 1e3, result.worst_abs_error * 1e3,
          result.worst_joint, result.max_abs_offset * 1e3,
          result.max_offset_rate, result.clamped_cycles,
          result.hold_rms_all * 1e3, result.diverged ? ", DIVERGED: " : "",
          result.reason.c_str());
      last_log = std::move(result.log);
      if (result.diverged) {
        diverged = true;
        break;
      }
    }
    close(summary, summary_path);
    close(weights, weights_path);
    close(scale, scale_path);
    writeLog(options.out / "last_pass.csv", last_log);
    std::printf("adaptive follow study: %d passes, %s; outputs in %s\n",
                passes_run, diverged ? "diverged" : "completed",
                options.out.string().c_str());
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_adaptive_follow_study: %s\n", error.what());
    return 1;
  }
}
