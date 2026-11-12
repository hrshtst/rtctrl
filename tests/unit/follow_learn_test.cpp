// x7_follow_learn's pure logic (apps/learn/follow_learn.hpp): the learn
// config, the follow log, the checks that a log, an offset, weights and a
// reference belong together, the lineage of every output, and two closed
// loops through the tool's own functions and files.
#include "learn/follow_learn.hpp"

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>
#include <utility>
#include <vector>

#include "follow/follow_run.hpp"  // FollowCsvLog, the follower's log writer
#include "plan/ptp_bundle.hpp"
#include "rtctrl/learn/command_offset.hpp"
#include "rtctrl/learn/regressor_feedforward.hpp"

using Catch::Matchers::ContainsSubstring;
using rtctrl::learn::JointArray;
namespace fl = x7::learn;
namespace rl = rtctrl::learn;
namespace fs = std::filesystem;

namespace {

constexpr int kDof = 8;
constexpr int kArm = 7;
constexpr int kGripper = 7;
constexpr int kColumns = rl::kRegressorCount;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.01;  // 100 Hz, the follow configs' control rate

using Row = std::array<double, kColumns>;
using Weights = std::array<Row, kDof>;

std::string sha(const fs::path& path) { return x7::ptp::sha256File(path); }

fs::path scratchDir() {
  const fs::path dir = "build/follow_learn_test";
  fs::create_directories(dir);
  return dir;
}

// The files one test case writes, named with the case's own prefix (ctest
// runs the cases in parallel). A leftover of an earlier run of the same
// case is removed when its name is first used; every file is removed at the
// end. Nothing else is ever deleted.
class Files {
 public:
  explicit Files(std::string prefix)
      : prefix_(std::move(prefix)), dir_(scratchDir()) {}
  Files(const Files&) = delete;
  Files& operator=(const Files&) = delete;
  ~Files() {
    for (const auto& path : used_) {
      std::error_code ignored;
      fs::remove(path, ignored);
    }
  }
  fs::path operator()(const std::string& name) {
    const fs::path path = dir_ / (prefix_ + "_" + name);
    if (std::find(used_.begin(), used_.end(), path) == used_.end()) {
      std::error_code ignored;
      fs::remove(path, ignored);
      used_.push_back(path);
    }
    return path;
  }

 private:
  std::string prefix_;
  fs::path dir_;
  std::vector<fs::path> used_;
};

std::string fileText(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input), {});
}

void writeText(const fs::path& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary);
  out << text;
  REQUIRE(out.good());
}

// The refusal's message; fails the test when the call succeeds.
template <class F>
std::string refusal(F&& call) {
  try {
    call();
  } catch (const std::exception& error) {
    return error.what();
  }
  FAIL("the call succeeded, expected a refusal");
  return {};
}

double minimumJerk(double s) {
  if (s <= 0.0) return 0.0;
  if (s >= 1.0) return 1.0;
  return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

// --------------------------------------------------------- references

// A rest-to-rest minimum-jerk move per arm joint about a posture that
// loads the arm; the gripper holds still.
struct Move {
  double start_rad;
  double amplitude_rad;
  double begin_s;
  double duration_s;
};
constexpr std::array<Move, kDof> kMoves = {{
    {0.3, 0.5, 0.25, 2.0},
    {-0.6, 0.4, 0.30, 2.2},
    {0.2, -0.45, 0.25, 1.9},
    {-1.1, 0.5, 0.35, 2.1},
    {0.4, -0.4, 0.30, 2.0},
    {0.7, 0.35, 0.25, 2.3},
    {-0.5, 0.5, 0.30, 1.8},
    {0.2, 0.0, 0.0, 1.0},
}};

// A ZVS reference of the given length (a multiple of 1/32 s, so the frame
// durations sum exactly) with a frame every 1/32 s; the moves are scaled in
// amplitude and in time by `scale` and `time_scale`.
void writeReference(const fs::path& path, double seconds, double scale = 1.0,
                    double time_scale = 1.0) {
  std::ofstream out(path);
  const long frames = std::lround(seconds * 32.0);
  REQUIRE(static_cast<double>(frames) == seconds * 32.0);
  char buffer[64];
  for (long k = 0; k < frames; ++k) {
    const double t = static_cast<double>(k) / 32.0;
    out << "0.03125 8 (";
    for (int j = 0; j < kDof; ++j) {
      const Move& m = kMoves[j];
      const double q =
          m.start_rad + scale * m.amplitude_rad *
                            minimumJerk((t - m.begin_s * time_scale) /
                                        (m.duration_s * time_scale));
      std::snprintf(buffer, sizeof buffer, "%s%.17g", j == 0 ? "" : " ", q);
      out << buffer;
    }
    out << " )\n";
  }
  REQUIRE(out.good());
}

fs::path writeFollowConfig(Files& files, const std::string& name,
                           const fs::path& reference) {
  const fs::path path = files(name);
  std::ofstream out(path);
  // build/follow_learn_test/ is two levels below the repo root.
  out << "format = \"rtctrl-x7-follow\"\nversion = 1\n"
      << "model = \"../../models/crane_x7/crane_x7.ztk\"\n"
      << "reference = \"" << reference.filename().string() << "\"\n"
      << "hardware_config = \"../../config/crane_x7.toml\"\n"
      << "[control]\nrate_hz = 100.0\n";
  REQUIRE(out.good());
  return path;
}

fs::path writeLearnConfig(Files& files, const std::string& name,
                          const std::string& body) {
  const fs::path path = files(name);
  writeText(path, "format = \"rtctrl-x7-follow-learn\"\nversion = 1\n" + body);
  return path;
}

// A reference, its follow config and a learn config in one place.
struct Setup {
  Files files;
  fs::path reference;
  fs::path follow;
  fs::path config;
  fs::path zero;  // the trial-0 offset
  fl::FollowReference ref;

  explicit Setup(const std::string& prefix, double seconds = 1.0,
                 const std::string& learn = "[ilc]\nphi = 0.5\n")
      : files(prefix) {
    reference = files("reference.zvs");
    writeReference(reference, seconds);
    follow = writeFollowConfig(files, "follow.toml", reference);
    config = writeLearnConfig(files, "learn.toml", learn);
    zero = files("offset0.csv");
    fl::runInit(follow, zero);
    ref = fl::loadFollowReference(follow);
  }
};

// ------------------------------------------------------------ the log

// One cycle of a follow log as the test writes it.
struct Cycle {
  std::string phase;
  double phase_time_s = 0.0;
  bool accepted = true;
  JointArray qref{};
  JointArray q{};
  JointArray qcmd{};
};

// Column for column FollowCsvLog's layout and number formats
// (apps/follow/follow_run.hpp), with offset_ramp rows the follower in this
// branch cannot write yet.
std::string logText(const std::vector<Cycle>& cycles) {
  std::string text =
      "schema_version,time_s,phase,command_mode,phase_time_s,"
      "feedback_time_s,feedback_seq,receipt_accepted,submitted_seq,"
      "applied_seq";
  char buffer[512];
  for (int j = 0; j < kDof; ++j) {
    std::snprintf(buffer, sizeof buffer,
                  ",qref%d_rad,dqref%d_rad_s,ddqref%d_rad_s2,q%d_rad,"
                  "dq%d_rad_s,tau%d_nm,qcmd%d_rad,dqcmd%d_rad_s,"
                  "effort_limit%d_nm,error%d_rad,applied%d,"
                  "applied_effort_limit%d_nm,flags%d",
                  j, j, j, j, j, j, j, j, j, j, j, j, j);
    text += buffer;
  }
  text += "\n";
  for (std::size_t k = 0; k < cycles.size(); ++k) {
    const Cycle& c = cycles[k];
    std::snprintf(buffer, sizeof buffer, "1,%.9f,%s,1,%.9f,%.9f,%zu,%d,%zu,%zu",
                  static_cast<double>(k) * kDt, c.phase.c_str(), c.phase_time_s,
                  static_cast<double>(k) * kDt, k, c.accepted ? 1 : 0, k + 1,
                  k);
    text += buffer;
    for (int j = 0; j < kDof; ++j) {
      std::snprintf(buffer, sizeof buffer,
                    ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.9g,%u",
                    c.qref[j], 0.0, 0.0, c.q[j], 0.0, 0.0, c.qcmd[j], 0.0, 0.0,
                    c.q[j] - c.qref[j], c.qcmd[j], 0.0, 0u);
      text += buffer;
    }
    text += "\n";
  }
  return text;
}

fs::path writeLog(Files& files, const std::string& name,
                  const std::vector<Cycle>& cycles) {
  const fs::path path = files(name);
  writeText(path, logText(cycles));
  return path;
}

// One trial's rows: homing, a ramp of the offset to its first sample, the
// tracking block on the grid, and the final hold. The plant is perfect
// (q = qref) until a test sets q.
struct Trial {
  std::vector<Cycle> cycles;
  std::vector<JointArray> delta;    // the offset each row commanded
  std::vector<std::size_t> sample;  // the reference sample of each row
  std::size_t tracking_begin = 0;
};

constexpr std::size_t kHomeRows = 40;
constexpr std::size_t kRampRows = 30;
constexpr double kRampSteps = 20.0;
constexpr std::size_t kHoldRows = 100;

Trial plannedTrial(const fl::FollowReference& ref,
                   const rl::CommandOffset& offset,
                   std::size_t hold_rows = kHoldRows) {
  Trial trial;
  const auto add = [&trial](const char* phase, double time,
                            const JointArray& qref, const JointArray& delta,
                            std::size_t sample) {
    Cycle c;
    c.phase = phase;
    c.phase_time_s = time;
    c.qref = qref;
    c.q = qref;
    for (int j = 0; j < kDof; ++j) c.qcmd[j] = qref[j] + delta[j];
    trial.cycles.push_back(c);
    trial.delta.push_back(delta);
    trial.sample.push_back(sample);
  };
  const std::size_t n = ref.grid.samples();
  for (std::size_t k = 0; k < kHomeRows; ++k) {
    add("home", static_cast<double>(k) * kDt, ref.q[0], JointArray{}, 0);
  }
  for (std::size_t k = 0; k < kRampRows; ++k) {
    const double s = minimumJerk((static_cast<double>(k) + 1.0) / kRampSteps);
    JointArray delta{};
    for (int j = 0; j < kDof; ++j) delta[j] = s * offset.delta_rad[0][j];
    add("offset_ramp", static_cast<double>(k) * kDt, ref.q[0], delta, 0);
  }
  trial.tracking_begin = trial.cycles.size();
  for (std::size_t i = 0; i < n; ++i) {
    const double t =
        std::min(ref.duration_s, static_cast<double>(i) * ref.grid.dt_s);
    add("tracking", t, ref.q[i], offset.delta_rad[i], i);
  }
  for (std::size_t k = 0; k < hold_rows; ++k) {
    add("finalizing", static_cast<double>(k) * ref.control_dt_s, ref.q_final,
        offset.delta_rad.back(), n - 1);
  }
  return trial;
}

// q = qref - e on every row and arm joint: e = qref - q is +e.
void constantError(Trial* trial, double e) {
  for (auto& c : trial->cycles) {
    for (int j = 0; j < kArm; ++j) c.q[j] = c.qref[j] - e;
  }
}

// A valid learned offset on the reference's grid, delta = f(i, j).
template <class F>
rl::CommandOffset fakeOffset(const fl::FollowReference& ref,
                             const std::string& method, int trial, F&& f) {
  rl::CommandOffset offset;
  offset.method = method;
  offset.trial = trial;
  offset.dt_s = ref.grid.dt_s;
  offset.reference_duration_s = ref.duration_s;
  offset.reference_sha256 = ref.reference_sha256;
  offset.parent_sha256 = std::string(64, 'a');
  offset.source_log_sha256 = std::string(64, 'b');
  offset.delta_rad.resize(ref.grid.samples());
  for (std::size_t i = 0; i < offset.delta_rad.size(); ++i) {
    for (int j = 0; j < kDof; ++j) offset.delta_rad[i][j] = f(i, j);
  }
  return offset;
}

fl::IlcInputs ilcInputs(Setup& s, const fs::path& log, const fs::path& offset,
                        const std::string& output) {
  return {s.config, s.follow, log, offset, s.files(output), std::nullopt};
}

// ------------------------------------------------------------ plants

// The ILC plant, per joint: the whole command through two samples of delay
// and a first-order lag of time constant tau (unit DC gain), from rest at
// the first command, with an output disturbance d (per reference sample):
//   x[k+1] = x[k] + (1 - a) (u[k-1] - x[k]),  q[k] = x[k] - d,
// so u[k] first shows in q[k+2]. a = exp(-dt / tau).
void lagPlant(Trial* trial, double tau,
              const std::vector<JointArray>& disturbance) {
  const double a = std::exp(-kDt / tau);
  JointArray x = trial->cycles.front().qcmd;
  for (std::size_t k = 0; k < trial->cycles.size(); ++k) {
    Cycle& c = trial->cycles[k];
    const JointArray& d = disturbance[trial->sample[k]];
    for (int j = 0; j < kDof; ++j) c.q[j] = x[j] - d[j];
    const JointArray& u = trial->cycles[k == 0 ? 0 : k - 1].qcmd;
    for (int j = 0; j < kDof; ++j) x[j] += (1.0 - a) * (u[j] - x[j]);
  }
}

// The FEL plant: the reference passes through, and the offset and an input
// disturbance d act on the position two samples late (unit gain):
//   q[k] = qref[k] + delta[k-2] - d[k-2]
// (rows before the first held at the first). Its tracking error
// e[k] = (d - delta)[k-2] is, with d = Phi w*, exactly Phi (w* - w) two
// samples late.
void delayPlant(Trial* trial, const std::vector<JointArray>& disturbance) {
  for (std::size_t k = 0; k < trial->cycles.size(); ++k) {
    const std::size_t m = k >= 2 ? k - 2 : 0;
    Cycle& c = trial->cycles[k];
    const JointArray& d = disturbance[trial->sample[m]];
    for (int j = 0; j < kDof; ++j) {
      c.q[j] = c.qref[j] + trial->delta[m][j] - d[j];
    }
  }
}

// The log's %.9g rounding of qref and q bounds the error of e = qref - q
// on any row: 5e-9 relative to each value (9 significant digits), with a
// margin for the conversion back.
double printBound(const Trial& trial) {
  double qref = 0.0;
  double q = 0.0;
  for (const auto& c : trial.cycles) {
    for (int j = 0; j < kDof; ++j) {
      qref = std::max(qref, std::fabs(c.qref[j]));
      q = std::max(q, std::fabs(c.q[j]));
    }
  }
  return 5.1e-9 * (qref + q);
}

double trackingRms(const Trial& trial, std::size_t n, int j) {
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const Cycle& c = trial.cycles[trial.tracking_begin + i];
    sum += (c.qref[j] - c.q[j]) * (c.qref[j] - c.q[j]);
  }
  return std::sqrt(sum / static_cast<double>(n));
}

}  // namespace

// ========================================================= learn config

TEST_CASE("follow learn: the learn config defaults are the library's",
          "[learn][follow_learn][config]") {
  const fl::LearnConfig config = fl::parseLearnConfig(
      "format = \"rtctrl-x7-follow-learn\"\nversion = 1\n", "case.toml");
  const rl::OffsetLimits limits;
  const rl::IlcOptions ilc;
  const rl::FelOptions fel;
  const rl::RegressorOptions regressors;
  CHECK(config.limits.max_abs_rad == limits.max_abs_rad);
  CHECK(config.limits.max_rate_rad_s == limits.max_rate_rad_s);
  CHECK(config.ilc.phi == ilc.phi);
  CHECK(config.ilc.psi_s == ilc.psi_s);
  CHECK(config.ilc.advance_s == ilc.advance_s);
  CHECK(config.ilc.cutoff_hz == ilc.cutoff_hz);
  CHECK(config.ilc.max_trial_step_rad == ilc.max_trial_step_rad);
  CHECK(config.ilc.max_abs_rad == limits.max_abs_rad);
  CHECK(config.fel.step == fel.step);
  CHECK(config.fel.gain == fel.gain);
  CHECK(config.fel.advance_s == fel.advance_s);
  CHECK(config.fel.error_lead_s == fel.error_lead_s);
  CHECK(config.fel.error_smooth_sigma_s == fel.error_smooth_sigma_s);
  CHECK(config.fel.ridge == fel.ridge);
  CHECK(config.fel.negligible_rms == fel.negligible_rms);
  CHECK(config.fel.joint_enabled == fel.joint_enabled);
  CHECK(config.regressors.smooth_sigma_s == regressors.smooth_sigma_s);
  CHECK(config.regressors.coulomb_v0_rad_s == regressors.coulomb_v0_rad_s);
}

TEST_CASE(
    "follow learn: the example learn config has every key, at its "
    "default",
    "[learn][follow_learn][config]") {
  const fs::path path = "config/follow_learn_example.toml";
  const fl::LearnConfig example = fl::loadLearnConfig(path);
  const fl::LearnConfig defaults = fl::parseLearnConfig(
      "format = \"rtctrl-x7-follow-learn\"\nversion = 1\n", "case.toml");
  CHECK(example.limits.max_abs_rad == defaults.limits.max_abs_rad);
  CHECK(example.limits.max_rate_rad_s == defaults.limits.max_rate_rad_s);
  CHECK(example.ilc.phi == defaults.ilc.phi);
  CHECK(example.ilc.psi_s == defaults.ilc.psi_s);
  CHECK(example.ilc.advance_s == defaults.ilc.advance_s);
  CHECK(example.ilc.cutoff_hz == defaults.ilc.cutoff_hz);
  CHECK(example.ilc.max_trial_step_rad == defaults.ilc.max_trial_step_rad);
  CHECK(example.fel.step == defaults.fel.step);
  CHECK(example.fel.gain == defaults.fel.gain);
  CHECK(example.fel.advance_s == defaults.fel.advance_s);
  CHECK(example.fel.error_lead_s == defaults.fel.error_lead_s);
  CHECK(example.fel.error_smooth_sigma_s == defaults.fel.error_smooth_sigma_s);
  CHECK(example.fel.ridge == defaults.fel.ridge);
  CHECK(example.regressors.smooth_sigma_s ==
        defaults.regressors.smooth_sigma_s);
  CHECK(example.regressors.coulomb_v0_rad_s ==
        defaults.regressors.coulomb_v0_rad_s);
  // Every key is written out.
  const toml::table root = toml::parse_file(path.string());
  const std::vector<std::pair<std::string, std::vector<std::string>>> keys = {
      {"limits", {"max_offset_rad", "max_offset_rate_rad_s"}},
      {"ilc", {"phi", "psi_s", "advance_s", "cutoff_hz", "max_trial_step_rad"}},
      {"fel",
       {"step", "gain", "advance_s", "error_lead_s", "error_smooth_sigma_s",
        "ridge"}},
  };
  for (const auto& [table, names] : keys) {
    for (const auto& name : names) {
      INFO(table << "." << name);
      CHECK(root[table][name]);
    }
  }
  CHECK(root["fel"]["regressors"]["smooth_sigma_s"]);
  CHECK(root["fel"]["regressors"]["coulomb_v0_rad_s"]);
}

TEST_CASE(
    "follow learn: a gain is one number for the arm or an array of "
    "eight",
    "[learn][follow_learn][config]") {
  const std::string head = "format = \"rtctrl-x7-follow-learn\"\nversion = 1\n";
  const auto parse = [&head](const std::string& body) {
    return fl::parseLearnConfig(head + body, "case.toml");
  };
  const auto scalar = parse("[ilc]\nphi = 0.5\npsi_s = 2\n");
  for (int j = 0; j < kArm; ++j) {
    CHECK(scalar.ilc.phi[j] == 0.5);
    CHECK(scalar.ilc.psi_s[j] == 2.0);  // an integer is a number
  }
  CHECK(scalar.ilc.phi[kGripper] == 0.0);
  CHECK(scalar.ilc.psi_s[kGripper] == 0.0);

  const auto array = parse(
      "[ilc]\nphi = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.0]\n"
      "psi_s = [0, 0, 0.01, 0, 0, 0, 0, -0.0]\n");
  CHECK(array.ilc.phi == JointArray{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.0});
  CHECK(array.ilc.psi_s[2] == 0.01);
  CHECK_FALSE(std::signbit(array.ilc.psi_s[kGripper]));

  CHECK_THAT(refusal([&] { parse("[ilc]\nphi = [0.1, 0.2, 0.3]\n"); }),
             ContainsSubstring("ilc.phi must be one number (the seven arm "
                               "joints) or an array of 8"));
  CHECK_THAT(refusal([&] { parse("[ilc]\nphi = -0.1\n"); }),
             ContainsSubstring("ilc.phi[0] must be >= 0"));
  CHECK_THAT(
      refusal([&] { parse("[ilc]\nphi = [0, 0, 0, -1, 0, 0, 0, 0]\n"); }),
      ContainsSubstring("ilc.phi[3] must be >= 0"));
  CHECK_THAT(refusal([&] { parse("[ilc]\npsi_s = \"0.1\"\n"); }),
             ContainsSubstring("ilc.psi_s must be a number or an array of 8"));
  CHECK_THAT(
      refusal([&] { parse("[ilc]\nphi = [0, 0, 0, \"x\", 0, 0, 0, 0]\n"); }),
      ContainsSubstring("ilc.phi[3] must be a number"));
  CHECK_THAT(refusal([&] { parse("[ilc]\nphi = nan\n"); }),
             ContainsSubstring("ilc.phi must be finite"));
}

TEST_CASE("follow learn: the gripper gain must be zero",
          "[learn][follow_learn][config]") {
  const std::string head = "format = \"rtctrl-x7-follow-learn\"\nversion = 1\n";
  for (const std::string key : {"phi", "psi_s"}) {
    INFO(key);
    const std::string message = refusal([&] {
      fl::parseLearnConfig(head + "[ilc]\n" + key +
                               " = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]\n",
                           "case.toml");
    });
    CHECK_THAT(message, ContainsSubstring("learn config 'case.toml'"));
    CHECK_THAT(message, ContainsSubstring("ilc." + key +
                                          "[7] (the gripper) must be 0: the "
                                          "gripper is never learned"));
  }
}

TEST_CASE("follow learn: the limits may only be lowered",
          "[learn][follow_learn][config]") {
  const std::string head = "format = \"rtctrl-x7-follow-learn\"\nversion = 1\n";
  const auto parse = [&head](const std::string& body) {
    return fl::parseLearnConfig(head + body, "case.toml");
  };
  const auto lowered =
      parse("[limits]\nmax_offset_rad = 0.05\nmax_offset_rate_rad_s = 0.2\n");
  CHECK(lowered.limits.max_abs_rad == 0.05);
  CHECK(lowered.limits.max_rate_rad_s == 0.2);
  CHECK(lowered.ilc.max_abs_rad == 0.05);  // the ILC clamp follows
  const auto ceiling =
      parse("[limits]\nmax_offset_rad = 0.15\nmax_offset_rate_rad_s = 0.5\n");
  CHECK(ceiling.limits.max_abs_rad == 0.15);
  CHECK(ceiling.limits.max_rate_rad_s == 0.5);
  for (const std::string value : {"0.150001", "1", "0", "-0.1"}) {
    INFO(value);
    CHECK_THAT(
        refusal([&] { parse("[limits]\nmax_offset_rad = " + value + "\n"); }),
        ContainsSubstring("limits.max_offset_rad must be > 0 and at "
                          "most 0.15 (the default; the limits may "
                          "only be lowered)"));
  }
  for (const std::string value : {"0.5000001", "0"}) {
    INFO(value);
    CHECK_THAT(refusal([&] {
                 parse("[limits]\nmax_offset_rate_rad_s = " + value + "\n");
               }),
               ContainsSubstring("limits.max_offset_rate_rad_s must be > 0 "
                                 "and at most 0.5"));
  }
}

TEST_CASE(
    "follow learn: unknown keys, wrong types and bad values are "
    "refused",
    "[learn][follow_learn][config]") {
  const std::string head = "format = \"rtctrl-x7-follow-learn\"\nversion = 1\n";
  const auto message = [&head](const std::string& body) {
    return refusal([&] { fl::parseLearnConfig(head + body, "case.toml"); });
  };
  CHECK_THAT(message("extra = 1\n"),
             ContainsSubstring("unknown key 'extra' in the root table"));
  CHECK_THAT(message("[limits]\nmax_offset = 0.1\n"),
             ContainsSubstring("unknown key 'max_offset' in [limits]"));
  CHECK_THAT(message("[ilc]\ngain = 0.1\n"),
             ContainsSubstring("unknown key 'gain' in [ilc]"));
  CHECK_THAT(message("[fel]\nphi = 0.1\n"),
             ContainsSubstring("unknown key 'phi' in [fel]"));
  CHECK_THAT(message("[fel.regressors]\nsigma = 0.1\n"),
             ContainsSubstring("unknown key 'sigma' in [fel.regressors]"));
  CHECK_THAT(message("limits = 1\n"),
             ContainsSubstring("[limits] must be a table"));
  CHECK_THAT(message("[fel]\nstep = \"gradient\"\n"),
             ContainsSubstring("fel.step must be \"normalised-gradient\" or "
                               "\"least-squares\", got \"gradient\""));
  CHECK_THAT(message("[fel]\nstep = 1\n"),
             ContainsSubstring("fel.step must be a string"));
  CHECK_THAT(message("[fel]\ngain = 0\n"),
             ContainsSubstring("fel.gain must be in (0, 1]"));
  CHECK_THAT(message("[fel]\ngain = 1.5\n"),
             ContainsSubstring("fel.gain must be in (0, 1]"));
  CHECK_THAT(message("[fel]\nridge = -1e-6\n"),
             ContainsSubstring("fel.ridge must be >= 0"));
  CHECK_THAT(message("[fel]\nadvance_s = true\n"),
             ContainsSubstring("fel.advance_s must be a number"));
  CHECK_THAT(message("[fel.regressors]\ncoulomb_v0_rad_s = 0\n"),
             ContainsSubstring("fel.regressors.coulomb_v0_rad_s must be > 0"));
  CHECK_THAT(message("[ilc]\ncutoff_hz = 0\n"),
             ContainsSubstring("ilc.cutoff_hz must be > 0"));
  CHECK_THAT(message("[ilc]\nadvance_s = -0.01\n"),
             ContainsSubstring("ilc.advance_s must be >= 0"));
  CHECK_THAT(message("[ilc]\nmax_trial_step_rad = inf\n"),
             ContainsSubstring("ilc.max_trial_step_rad must be finite"));
  CHECK_THAT(message("[ilc]\nphi = 9007199254740993\n"),
             ContainsSubstring("an integer a double cannot hold exactly"));
  CHECK_THAT(refusal([] {
               fl::parseLearnConfig(
                   "format = \"rtctrl-x7-follow\"\nversion = 1\n", "case.toml");
             }),
             ContainsSubstring("format must be \"rtctrl-x7-follow-learn\""));
  CHECK_THAT(refusal([] {
               fl::parseLearnConfig(
                   "format = \"rtctrl-x7-follow-learn\"\nversion = 2\n",
                   "case.toml");
             }),
             ContainsSubstring("version must be the integer 1"));
  CHECK_THAT(
      refusal([] { fl::parseLearnConfig("version = 1\n", "case.toml"); }),
      ContainsSubstring("missing key format"));
  CHECK_THAT(refusal([] { fl::parseLearnConfig("format = [", "case.toml"); }),
             ContainsSubstring("cannot parse"));

  const auto parsed =
      fl::parseLearnConfig(head +
                               "[fel]\nstep = \"least-squares\"\ngain = 1\n"
                               "[fel.regressors]\nsmooth_sigma_s = 0\n",
                           "case.toml");
  CHECK(parsed.fel.step == rl::FelOptions::Step::LeastSquares);
  CHECK(parsed.fel.gain == 1.0);
  CHECK(parsed.regressors.smooth_sigma_s == 0.0);
}

// ============================================================ the log

TEST_CASE(
    "follow learn: a log written by the follower's FollowCsvLog reads "
    "back",
    "[learn][follow_learn][log]") {
  namespace arm = rtctrl::arm;
  namespace model = rtctrl::model;
  Files files("csv_log");
  const fs::path path = files("log.csv");
  const std::array<x7::follow::Phase, 6> phases = {
      x7::follow::Phase::Home,       x7::follow::Phase::HomeCorrection,
      x7::follow::Phase::Tracking,   x7::follow::Phase::Tracking,
      x7::follow::Phase::Finalizing, x7::follow::Phase::Finalizing};
  const auto qref = [](std::size_t k, int j) {
    return 0.1 * j - 0.37 + 0.01 * k;
  };
  {
    x7::follow::FollowCsvLog log(path.string());
    arm::JointState state;
    arm::JointCommand command;
    model::ZVector q(kDof), dq(kDof), ddq(kDof);
    arm::CommandSnapshot snapshot;
    for (std::size_t k = 0; k < phases.size(); ++k) {
      for (int j = 0; j < kDof; ++j) {
        q[j] = qref(k, j);
        state.q[j] = q[j] - 0.001 * (j + 1);
        command.q[j] = q[j] + 0.002 * j;
      }
      arm::CommandReceipt receipt;
      receipt.accepted = k != 3;
      REQUIRE(log.record({phases[k], 0.01 * k, 0.02 * k, &state, &command, &q,
                          &dq, &ddq, &snapshot, &receipt}));
    }
    REQUIRE(log.close());
  }
  const std::vector<fl::LogRow> rows = fl::readFollowLog(path);
  REQUIRE(rows.size() == phases.size());
  const std::array<fl::LogPhase, 6> expected = {
      fl::LogPhase::Home,       fl::LogPhase::HomeCorrection,
      fl::LogPhase::Tracking,   fl::LogPhase::Tracking,
      fl::LogPhase::Finalizing, fl::LogPhase::Finalizing};
  for (std::size_t k = 0; k < rows.size(); ++k) {
    INFO("row " << k);
    CHECK(rows[k].line == k + 2);
    CHECK(rows[k].phase == expected[k]);
    CHECK(std::fabs(rows[k].phase_time_s - 0.02 * k) <= 1e-9);
    CHECK(rows[k].accepted == (k != 3));
    for (int j = 0; j < kDof; ++j) {
      CHECK(std::fabs(rows[k].qref[j] - qref(k, j)) <= 1e-8);
      CHECK(std::fabs(rows[k].q[j] - (qref(k, j) - 0.001 * (j + 1))) <= 1e-8);
      CHECK(std::fabs(rows[k].qcmd[j] - (qref(k, j) + 0.002 * j)) <= 1e-8);
    }
  }
}

TEST_CASE("follow learn: a malformed log is refused naming the line",
          "[learn][follow_learn][log]") {
  Cycle c;
  c.phase = "offset_ramp";  // the sibling phase reads
  const std::string good = logText({c, c});
  REQUIRE_NOTHROW([&] {
    std::istringstream in(good);
    const auto rows = fl::parseFollowLog(in, "follow log 'case.csv'");
    REQUIRE(rows.size() == 2);
    CHECK(rows[0].phase == fl::LogPhase::OffsetRamp);
  }());
  const auto message = [](const std::string& text) {
    return refusal([&] {
      std::istringstream in(text);
      fl::parseFollowLog(in, "follow log 'case.csv'");
    });
  };
  const auto replaced = [&good](const std::string& from, const std::string& to,
                                std::size_t occurrence = 0) {
    std::string text = good;
    std::size_t at = text.find(from);
    for (std::size_t k = 0; k < occurrence; ++k) at = text.find(from, at + 1);
    REQUIRE(at != std::string::npos);
    return text.replace(at, from.size(), to);
  };
  CHECK_THAT(message(""), ContainsSubstring("follow log 'case.csv': empty"));
  CHECK_THAT(message(good.substr(0, good.find('\n') + 1)),
             ContainsSubstring("no data rows"));
  CHECK_THAT(message(good.substr(0, good.size() - 1)),
             ContainsSubstring("line 3: missing final newline"));
  CHECK_THAT(message(replaced(",offset_ramp,", ",ramp,")),
             ContainsSubstring("line 2: unknown phase 'ramp'"));
  CHECK_THAT(message(replaced("\n1,", "\n2,", 1)),
             ContainsSubstring("line 3: schema_version '2', expected 1"));
  CHECK_THAT(message(replaced(",qcmd3_rad,", ",qcmd3,")),
             ContainsSubstring("line 1: missing column(s): qcmd3_rad"));
  CHECK_THAT(message(replaced(",q5_rad,", ",q4_rad,")),
             ContainsSubstring("line 1: duplicate column 'q4_rad'"));
  {
    // receipt_accepted is the 8th field of a row
    std::string text = good;
    std::size_t at = text.find('\n') + 1;
    for (int k = 0; k < 7; ++k) at = text.find(',', at) + 1;
    text.replace(at, 1, "2");
    CHECK_THAT(message(text),
               ContainsSubstring("line 2: column 'receipt_accepted' must be 0 "
                                 "or 1, got '2'"));
  }
  {
    std::string text = good;
    text.insert(text.size() - 1, ",extra");
    CHECK_THAT(message(text),
               ContainsSubstring("line 3: 115 fields, the header has 114"));
  }
  {
    // qref2_rad of the first row: field 10 + 2 * 13
    std::string text = good;
    std::size_t at = text.find('\n') + 1;
    for (int k = 0; k < 10 + 2 * 13; ++k) at = text.find(',', at) + 1;
    text.replace(at, text.find(',', at) - at, "nan");
    CHECK_THAT(message(text),
               ContainsSubstring("line 2: column 'qref2_rad': not a finite "
                                 "number: 'nan'"));
  }
}

TEST_CASE(
    "follow learn: the trial record is e = qref - q over the tracking "
    "block and the tail",
    "[learn][follow_learn][log]") {
  Setup s("record");
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  const std::size_t n = s.ref.grid.samples();
  REQUIRE(n == 101);  // a 1 s reference at 100 Hz
  for (const std::size_t hold : {std::size_t{3}, std::size_t{250}}) {
    INFO("hold rows " << hold);
    Trial trial = plannedTrial(s.ref, zero, hold);
    constantError(&trial, 0.003);  // q = qref - 0.003: e = +0.003
    const fs::path log =
        writeLog(s.files, "log_" + std::to_string(hold) + ".csv", trial.cycles);
    const auto rows = fl::readFollowLog(log);
    // the log's own error column is q - qref = -0.003
    CHECK(rows[kHomeRows + kRampRows].phase == fl::LogPhase::Tracking);
    CHECK(rows[kHomeRows].phase == fl::LogPhase::OffsetRamp);
    const fl::TrialRecord record =
        fl::trialRecord(rows, s.ref, zero, "follow log", "offset");
    CHECK(record.tracking_samples == n);
    CHECK(record.tail_samples == std::min<std::size_t>(hold, 200));
    REQUIRE(record.error.size() == n + record.tail_samples);
    for (const auto& e : record.error) {
      for (int j = 0; j < kArm; ++j) CHECK(std::fabs(e[j] - 0.003) <= 1e-8);
      CHECK(e[kGripper] == 0.0);
    }
  }
}

// ===================================================== the ilc refusals

TEST_CASE("follow learn: ilc refuses a log of another reference",
          "[learn][follow_learn][ilc]") {
  Setup s("other_reference");
  // The same duration and grid, the moves 10 % larger.
  const fs::path other = s.files("other.zvs");
  writeReference(other, 1.0, 1.1);
  const fs::path other_follow = writeFollowConfig(s.files, "other.toml", other);
  const fl::FollowReference other_ref = fl::loadFollowReference(other_follow);
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  const fs::path log =
      writeLog(s.files, "log.csv", plannedTrial(other_ref, zero).cycles);
  const auto in = ilcInputs(s, log, s.zero, "next.csv");
  const std::string message = refusal([&] { fl::runIlc(in); });
  CHECK_THAT(message, ContainsSubstring("follow log '" + log.string() + "'"));
  CHECK_THAT(message, ContainsSubstring("tracking sample "));
  CHECK_THAT(message, ContainsSubstring(" s): qref"));
  CHECK_THAT(message, ContainsSubstring("(tolerance 1e-6 rad): the log is not "
                                        "a run of reference"));
  CHECK_FALSE(fs::exists(in.output));
}

TEST_CASE("follow learn: ilc refuses a log run with another offset",
          "[learn][follow_learn][ilc]") {
  Setup s("other_offset");
  const rl::CommandOffset applied =
      fakeOffset(s.ref, "ilc", 1, [](std::size_t i, int j) {
        return j == 2 && i > 50 ? 2e-6 * static_cast<double>(i - 50) : 0.0;
      });
  const fs::path log =
      writeLog(s.files, "log.csv", plannedTrial(s.ref, applied).cycles);
  const auto in = ilcInputs(s, log, s.zero, "next.csv");
  const std::string message = refusal([&] { fl::runIlc(in); });
  CHECK_THAT(message, ContainsSubstring("tracking sample 51"));
  CHECK_THAT(message, ContainsSubstring("qcmd2_rad - qref2_rad = "));
  CHECK_THAT(message, ContainsSubstring("of offset '" + s.zero.string() + "'"));
  CHECK_THAT(message,
             ContainsSubstring("the log is not a run with this offset"));
  CHECK_FALSE(fs::exists(in.output));
}

TEST_CASE(
    "follow learn: ilc refuses an incomplete or malformed tracking "
    "block",
    "[learn][follow_learn][ilc]") {
  Setup s("tracking_block");
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  const Trial good = plannedTrial(s.ref, zero);
  const std::size_t begin = good.tracking_begin;
  const std::size_t n = s.ref.grid.samples();
  const auto run = [&s](const std::string& name,
                        const std::vector<Cycle>& cycles) {
    const fs::path log = writeLog(s.files, name + ".csv", cycles);
    const auto in = ilcInputs(s, log, s.zero, name + "_next.csv");
    const std::string message = refusal([&] { fl::runIlc(in); });
    CHECK_FALSE(fs::exists(in.output));
    return message;
  };
  {
    // the last tracking row is missing: the trial did not complete
    auto cycles = good.cycles;
    cycles.erase(cycles.begin() + static_cast<long>(begin + n - 1));
    const std::string message = run("short", cycles);
    CHECK_THAT(message, ContainsSubstring("has 100 rows, the tracking grid of "
                                          "reference"));
    CHECK_THAT(message, ContainsSubstring("has 101 samples: the trial did not "
                                          "complete"));
  }
  {
    // an aborted run: tracking stops and the abort hold follows
    auto cycles = good.cycles;
    cycles.resize(begin + 60);
    Cycle hold = cycles.back();
    hold.phase = "abort_hold";
    cycles.push_back(hold);
    CHECK_THAT(run("aborted", cycles),
               ContainsSubstring("phase 'abort_hold': the run aborted"));
  }
  {
    auto cycles = good.cycles;
    cycles.resize(begin);
    CHECK_THAT(run("never", cycles),
               ContainsSubstring("no tracking rows: the trial never reached "
                                 "tracking"));
  }
  {
    // a finalizing row inside the tracking block
    auto cycles = good.cycles;
    Cycle hold = cycles[begin + 40];
    hold.phase = "finalizing";
    cycles.insert(cycles.begin() + static_cast<long>(begin + 40), hold);
    CHECK_THAT(run("split", cycles),
               ContainsSubstring("the tracking rows are not one contiguous "
                                 "block"));
  }
  {
    // one row too many
    auto cycles = good.cycles;
    cycles.insert(cycles.begin() + static_cast<long>(begin + n),
                  cycles[begin + n - 1]);
    CHECK_THAT(run("long", cycles),
               ContainsSubstring("has 102 rows, the tracking grid of "));
  }
  {
    auto cycles = good.cycles;
    cycles[begin + 17].accepted = false;
    CHECK_THAT(run("rejected", cycles),
               ContainsSubstring("tracking sample 17 (t = 0.17 s): "
                                 "receipt_accepted is 0"));
  }
  {
    auto cycles = good.cycles;
    cycles[begin + 9].phase_time_s += 0.01;
    CHECK_THAT(run("time", cycles),
               ContainsSubstring("tracking sample 9 (t = 0.09 s): "
                                 "phase_time_s"));
  }
  {
    auto cycles = good.cycles;
    cycles[3].phase = "finalizing";
    CHECK_THAT(run("before", cycles),
               ContainsSubstring("phase 'finalizing' before the tracking "
                                 "block"));
  }
  {
    auto cycles = good.cycles;
    cycles.back().phase = "home";
    CHECK_THAT(run("after", cycles),
               ContainsSubstring("phase 'home' after the tracking block"));
  }
}

TEST_CASE(
    "follow learn: ilc refuses a log without the tail the advance "
    "reads",
    "[learn][follow_learn][ilc]") {
  Setup s("tail");  // advance 0.05 s at 100 Hz: 5 final-hold rows
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  for (const std::size_t hold : {std::size_t{0}, std::size_t{4}}) {
    INFO("hold rows " << hold);
    const fs::path log =
        writeLog(s.files, "log" + std::to_string(hold) + ".csv",
                 plannedTrial(s.ref, zero, hold).cycles);
    const auto in =
        ilcInputs(s, log, s.zero, "next" + std::to_string(hold) + ".csv");
    const std::string message = refusal([&] { fl::runIlc(in); });
    CHECK_THAT(message,
               ContainsSubstring("ilc: the update refuses follow log"));
    CHECK_THAT(message,
               ContainsSubstring("error needs 5 samples past the 101"));
    CHECK_FALSE(fs::exists(in.output));
  }
  // Five rows are enough, and a final-hold row must hold the final value.
  Trial trial = plannedTrial(s.ref, zero, 5);
  const fs::path ok = writeLog(s.files, "log5.csv", trial.cycles);
  CHECK_NOTHROW(fl::runIlc(ilcInputs(s, ok, s.zero, "next5.csv")));
  trial.cycles.back().qref[3] += 1e-5;
  trial.cycles.back().qcmd[3] += 1e-5;
  const fs::path moved = writeLog(s.files, "moved.csv", trial.cycles);
  CHECK_THAT(refusal([&] {
               fl::runIlc(ilcInputs(s, moved, s.zero, "next_moved.csv"));
             }),
             ContainsSubstring("final-hold row 4: qref3_rad"));
}

TEST_CASE(
    "follow learn: ilc refuses an offset of another reference or "
    "beyond the limits",
    "[learn][follow_learn][ilc]") {
  Setup s("prev_offset");
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  const fs::path log =
      writeLog(s.files, "log.csv", plannedTrial(s.ref, zero).cycles);

  const fs::path other = s.files("other.zvs");
  writeReference(other, 1.0, 1.1);
  const fs::path other_follow = writeFollowConfig(s.files, "other.toml", other);
  const fs::path other_zero = s.files("other_zero.csv");
  fl::runInit(other_follow, other_zero);
  auto in = ilcInputs(s, log, other_zero, "next.csv");
  std::string message = refusal([&] { fl::runIlc(in); });
  CHECK_THAT(message, ContainsSubstring("offset '" + other_zero.string() +
                                        "' does not belong to the run of "
                                        "reference"));
  CHECK_THAT(message,
             ContainsSubstring("does not match the reference being run"));
  CHECK_FALSE(fs::exists(in.output));

  const fs::path big = s.files("big.csv");
  rl::writeCommandOffset(
      big, fakeOffset(s.ref, "ilc", 1,
                      [](std::size_t, int j) { return j == 1 ? 0.02 : 0.0; }));
  in = ilcInputs(s, log, big, "next_big.csv");
  in.config =
      writeLearnConfig(s.files, "low.toml",
                       "[limits]\nmax_offset_rad = 0.01\n[ilc]\nphi = 0.5\n");
  message = refusal([&] { fl::runIlc(in); });
  CHECK_THAT(message, ContainsSubstring("under the configured limits"));
  CHECK_THAT(message, ContainsSubstring("exceeds max_abs_rad 0.01"));
  CHECK_FALSE(fs::exists(in.output));

  const fs::path gripper = s.files("gripper.csv");
  rl::writeCommandOffset(gripper,
                         fakeOffset(s.ref, "ilc", 1, [](std::size_t i, int j) {
                           return j == kGripper && i == 7 ? 1e-3 : 0.0;
                         }));
  CHECK_THAT(
      refusal([&] { fl::runIlc(ilcInputs(s, log, gripper, "next_g.csv")); }),
      ContainsSubstring("the gripper (joint 7) offset at sample 7"));
}

TEST_CASE(
    "follow learn: ilc refuses gains that change nothing and an update "
    "beyond the limits",
    "[learn][follow_learn][ilc]") {
  Setup s("ilc_gains");
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  Trial trial = plannedTrial(s.ref, zero);
  // An error that grows along the reference: 0.01 rad/s on the arm.
  for (std::size_t k = 0; k < trial.cycles.size(); ++k) {
    const double t =
        k < trial.tracking_begin
            ? 0.0
            : std::min(1.0,
                       static_cast<double>(k - trial.tracking_begin) * kDt);
    for (int j = 0; j < kArm; ++j) {
      trial.cycles[k].q[j] = trial.cycles[k].qref[j] - 0.01 * t;
    }
  }
  const fs::path log = writeLog(s.files, "log.csv", trial.cycles);
  auto in = ilcInputs(s, log, s.zero, "next.csv");
  in.config = writeLearnConfig(s.files, "off.toml", "");
  CHECK_THAT(refusal([&] { fl::runIlc(in); }),
             ContainsSubstring("ilc.phi and ilc.psi_s are 0 on every joint"));
  CHECK_FALSE(fs::exists(in.output));
  in.config = writeLearnConfig(s.files, "slow.toml",
                               "[limits]\nmax_offset_rate_rad_s = 0.005\n"
                               "[ilc]\nphi = 1.0\n");
  const std::string message = refusal([&] { fl::runIlc(in); });
  CHECK_THAT(message,
             ContainsSubstring("ilc: the updated offset is refused and "
                               "nothing was written"));
  CHECK_THAT(message, ContainsSubstring("exceeds max_rate_rad_s 0.005"));
  CHECK_THAT(message, ContainsSubstring("lower [ilc] cutoff_hz or the gains"));
  CHECK_FALSE(fs::exists(in.output));
}

// ============================================== lineage and the outputs

TEST_CASE("follow learn: init writes the zero offset of the reference",
          "[learn][follow_learn][init]") {
  Setup s("init");
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  const rl::TrackingGrid grid = rl::trackingGrid(1.0, 0.01);
  CHECK(zero.method == "zero");
  CHECK(zero.trial == 0);
  CHECK(zero.delta_rad.size() == grid.samples());
  CHECK(zero.dt_s == grid.dt_s);
  CHECK(zero.reference_duration_s == 1.0);
  CHECK(zero.reference_sha256 == sha(s.reference));
  CHECK(zero.parent_sha256.empty());
  CHECK(zero.source_log_sha256.empty());
  CHECK_THAT(refusal([&] { fl::runInit(s.follow, s.zero); }),
             ContainsSubstring("--output: refusing to overwrite"));
}

TEST_CASE(
    "follow learn: an ilc offset carries its lineage, and the report "
    "its numbers and hashes",
    "[learn][follow_learn][ilc]") {
  Setup s("ilc_lineage");
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  Trial trial = plannedTrial(s.ref, zero);
  constantError(&trial, 0.004);
  const fs::path log = writeLog(s.files, "log.csv", trial.cycles);
  const fs::path next = s.files("next.csv");
  const fs::path report = s.files("report.toml");
  const fl::LearnResult result =
      fl::runIlc({s.config, s.follow, log, s.zero, next, report});
  const rl::CommandOffset offset = rl::readCommandOffset(next);
  CHECK(offset.method == "ilc");
  CHECK(offset.trial == 1);
  CHECK(offset.parent_sha256 == sha(s.zero));
  CHECK(offset.source_log_sha256 == sha(log));
  CHECK(offset.reference_sha256 == sha(s.reference));
  CHECK(offset.dt_s == zero.dt_s);
  CHECK(offset.reference_duration_s == zero.reference_duration_s);
  // phi 0.5 on a constant 0.004 rad error: 0.002 rad on the arm, none on
  // the gripper (Q passes a constant unchanged)
  for (const auto& sample : offset.delta_rad) {
    for (int j = 0; j < kArm; ++j) CHECK(std::fabs(sample[j] - 0.002) <= 1e-9);
    CHECK(sample[kGripper] == 0.0);
  }
  CHECK(result.measured_trial == 0);
  CHECK(result.output_trial == 1);
  for (int j = 0; j < kArm; ++j) {
    CHECK(std::fabs(result.error_rms[j] - 0.004) <= 1e-8);
  }
  CHECK(result.error_rms[kGripper] == 0.0);
  CHECK(std::fabs(result.arm_rms - 0.004) <= 1e-8);
  CHECK(std::fabs(result.aggregate_rms - 0.004 * std::sqrt(7.0 / 8.0)) <= 1e-8);

  const toml::table toml = toml::parse_file(report.string());
  CHECK(toml["format"].value_or(std::string()) ==
        "rtctrl-x7-follow-learn-report");
  CHECK(toml["command"].value_or(std::string()) == "ilc");
  CHECK(toml["measured_trial"].value_or(-1) == 0);
  CHECK(toml["output_trial"].value_or(-1) == 1);
  CHECK(toml["tracking"]["aggregate_rms_rad"].value_or(0.0) ==
        result.aggregate_rms);
  CHECK(toml["tracking"]["arm_rms_rad"].value_or(0.0) == result.arm_rms);
  CHECK(toml["tracking"]["error_rms_rad"][3].value_or(0.0) ==
        result.error_rms[3]);
  CHECK(toml["offset"]["max_abs_rad"][2].value_or(0.0) ==
        result.offset_max_abs[2]);
  CHECK(toml["ilc"]["step_clamped"][0].value_or(-1) == 0);
  const auto* inputs = toml["input"].as_array();
  REQUIRE(inputs != nullptr);
  std::vector<std::pair<std::string, fs::path>> expected = {
      {"config", s.config},
      {"follow_config", s.follow},
      {"model", "models/crane_x7/crane_x7.ztk"},
      {"reference", s.reference},
      {"offset", s.zero},
      {"log", log}};
  REQUIRE(inputs->size() == expected.size());
  for (std::size_t k = 0; k < expected.size(); ++k) {
    INFO(expected[k].first);
    const auto& entry = *(*inputs)[k].as_table();
    CHECK(entry["role"].value_or(std::string()) == expected[k].first);
    CHECK(entry["sha256"].value_or(std::string()) == sha(expected[k].second));
  }
  const auto* outputs = toml["output"].as_array();
  REQUIRE(outputs != nullptr);
  REQUIRE(outputs->size() == 1);
  CHECK((*(*outputs)[0].as_table())["sha256"].value_or(std::string()) ==
        sha(next));

  const std::string summary = fl::formatSummary(result);
  CHECK_THAT(summary, ContainsSubstring("ilc: wrote offset " + next.string()));
  CHECK_THAT(summary, ContainsSubstring("tracking RMS of trial 0: "));
  CHECK_THAT(summary, ContainsSubstring(" rad over all 8 joints, "));
}

TEST_CASE(
    "follow learn: outputs are created exclusively, and nothing is "
    "left on a refusal",
    "[learn][follow_learn][outputs]") {
  Setup s("exclusive");
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  const fs::path log =
      writeLog(s.files, "log.csv", plannedTrial(s.ref, zero).cycles);
  const fs::path next = s.files("next.csv");
  const fs::path report = s.files("report.toml");
  writeText(next, "keep\n");
  CHECK_THAT(refusal([&] {
               fl::runIlc({s.config, s.follow, log, s.zero, next, report});
             }),
             ContainsSubstring("--output: refusing to overwrite"));
  CHECK(fileText(next) == "keep\n");
  CHECK_FALSE(fs::exists(report));
  fs::remove(next);
  writeText(report, "keep\n");
  CHECK_THAT(refusal([&] {
               fl::runIlc({s.config, s.follow, log, s.zero, next, report});
             }),
             ContainsSubstring("--report: refusing to overwrite"));
  CHECK(fileText(report) == "keep\n");
  CHECK_FALSE(fs::exists(next));
  fs::remove(report);
  CHECK_THAT(refusal([&] {
               fl::runIlc({s.config, s.follow, log, s.zero, next, next});
             }),
             ContainsSubstring("is also --output: every output needs its own "
                               "file"));
  CHECK_FALSE(fs::exists(next));
  // a dangling symlink is an existing output too
  fs::create_symlink("no_such_target", next);
  CHECK_THAT(
      refusal([&] {
        fl::runIlc({s.config, s.follow, log, s.zero, next, std::nullopt});
      }),
      ContainsSubstring("--output: refusing to overwrite"));
  CHECK(fs::is_symlink(fs::symlink_status(next)));
  fs::remove(next);

  // fel: an existing offset output refuses before the weights are written
  const fs::path weights0 = s.files("weights0.toml");
  fl::runInitWeights(s.config, weights0);
  const fs::path weights1 = s.files("weights1.toml");
  writeText(next, "keep\n");
  CHECK_THAT(refusal([&] {
               fl::runFel({s.config, s.follow, log, s.zero, weights0, weights1,
                           next, std::nullopt});
             }),
             ContainsSubstring("--output: refusing to overwrite"));
  CHECK_FALSE(fs::exists(weights1));
  CHECK(fileText(next) == "keep\n");
  fs::remove(next);
  // fel: the offset cannot be created after the weights were: the weights
  // file is removed again
  const fs::path nowhere =
      scratchDir() / "follow_learn_no_such_dir" / "next.csv";
  const std::string message = refusal([&] {
    fl::runFel({s.config, s.follow, log, s.zero, weights0, weights1, nowhere,
                std::nullopt});
  });
  CHECK_THAT(message, ContainsSubstring("cannot create"));
  CHECK_FALSE(fs::exists(weights1));
}

// ================================================================= fel

namespace {

fl::FelInputs felInputs(Setup& s, const fs::path& log, const fs::path& offset,
                        const fs::path& weights, const std::string& tag) {
  return {s.config,
          s.follow,
          log,
          offset,
          weights,
          s.files("weights_" + tag + ".toml"),
          s.files("offset_" + tag + ".csv"),
          std::nullopt};
}

}  // namespace

TEST_CASE("follow learn: fel weights and offset carry their lineage",
          "[learn][follow_learn][fel]") {
  Setup s("fel_lineage", 1.0, "[fel]\nstep = \"least-squares\"\ngain = 0.5\n");
  const fs::path weights0 = s.files("weights0.toml");
  const fl::LearnResult init = fl::runInitWeights(s.config, weights0);
  const rl::FeedforwardWeights w0 = rl::readFeedforwardWeights(weights0);
  CHECK(w0.trial == 0);
  CHECK(w0.regressors.smooth_sigma_s == 0.04);
  CHECK(init.outputs.at(0).sha256 == sha(weights0));

  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  Trial trial = plannedTrial(s.ref, zero);
  constantError(&trial, 0.004);
  const fs::path log = writeLog(s.files, "log.csv", trial.cycles);
  const auto in = felInputs(s, log, s.zero, weights0, "1");
  const fl::LearnResult result = fl::runFel(in);
  const rl::FeedforwardWeights w1 =
      rl::readFeedforwardWeights(in.output_weights);
  CHECK(w1.trial == 1);
  CHECK(w1.parent_sha256 == sha(weights0));
  CHECK(w1.source_log_sha256 == sha(log));
  CHECK(w1.reference_sha256 == sha(s.reference));
  CHECK(w1.regressors.smooth_sigma_s == w0.regressors.smooth_sigma_s);
  const rl::CommandOffset offset1 = rl::readCommandOffset(in.output);
  CHECK(offset1.method == "fel");
  CHECK(offset1.trial == 1);
  CHECK(offset1.parent_sha256 == sha(in.output_weights));  // the weights file
  CHECK(offset1.source_log_sha256 == sha(log));
  CHECK(offset1.reference_sha256 == sha(s.reference));
  // the offset is the synthesis of the weights as written
  fl::FollowReference ref = fl::loadFollowReference(s.follow);
  const auto phi = rl::buildRegressors(*ref.chain, *ref.map, ref.q,
                                       ref.grid.dt_s, w1.regressors);
  CHECK(offset1.delta_rad == rl::synthesizeOffset(phi, w1));
  REQUIRE(result.fel.has_value());
  CHECK(result.fel->joints[kGripper].enabled == false);
  CHECK(result.outputs.size() == 2);

  // trial 2 from trial 1: the chain continues
  const fs::path log1 =
      writeLog(s.files, "log1.csv", plannedTrial(s.ref, offset1).cycles);
  const auto in2 = felInputs(s, log1, in.output, in.output_weights, "2");
  fl::runFel(in2);
  const rl::FeedforwardWeights w2 =
      rl::readFeedforwardWeights(in2.output_weights);
  CHECK(w2.trial == 2);
  CHECK(w2.parent_sha256 == sha(in.output_weights));
  const rl::CommandOffset offset2 = rl::readCommandOffset(in2.output);
  CHECK(offset2.parent_sha256 == sha(in2.output_weights));
  CHECK(offset2.source_log_sha256 == sha(log1));
}

TEST_CASE(
    "follow learn: fel refuses an offset that is not the synthesis of "
    "the weights",
    "[learn][follow_learn][fel]") {
  Setup s("fel_consistency", 1.0,
          "[fel]\nstep = \"least-squares\"\ngain = 0.5\n");
  const fs::path weights0 = s.files("weights0.toml");
  fl::runInitWeights(s.config, weights0);
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  Trial trial = plannedTrial(s.ref, zero);
  constantError(&trial, 0.004);
  const fs::path log0 = writeLog(s.files, "log0.csv", trial.cycles);
  const auto in1 = felInputs(s, log0, s.zero, weights0, "1");
  fl::runFel(in1);
  const rl::CommandOffset offset1 = rl::readCommandOffset(in1.output);
  const fs::path log1 =
      writeLog(s.files, "log1.csv", plannedTrial(s.ref, offset1).cycles);
  REQUIRE_NOTHROW(
      fl::runFel(felInputs(s, log1, in1.output, in1.output_weights, "ok")));

  {
    // one sample moved by 2e-9 rad, the header (and so the parent) kept
    rl::CommandOffset tampered = offset1;
    tampered.delta_rad[30][1] += 2e-9;
    const fs::path path = s.files("tampered.csv");
    rl::writeCommandOffset(path, tampered);
    const auto in = felInputs(s, log1, path, in1.output_weights, "t");
    const std::string message = refusal([&] { fl::runFel(in); });
    CHECK_THAT(message, ContainsSubstring("sample 30, joint 1"));
    CHECK_THAT(message, ContainsSubstring("differs from the synthesis"));
    CHECK_THAT(message, ContainsSubstring("(tolerance 1e-9 rad)"));
    CHECK_FALSE(fs::exists(in.output_weights));
    CHECK_FALSE(fs::exists(in.output));
  }
  {
    // the trial-0 offset with trial-1 weights
    const auto in = felInputs(s, log0, s.zero, in1.output_weights, "m");
    CHECK_THAT(
        refusal([&] { fl::runFel(in); }),
        ContainsSubstring("is trial 0, weights '" +
                          in1.output_weights.string() + "' are trial 1"));
    CHECK_FALSE(fs::exists(in.output_weights));
  }
  {
    // an ilc offset with the weights' numbers is not theirs
    rl::CommandOffset ilc = offset1;
    ilc.method = "ilc";
    const fs::path path = s.files("ilc.csv");
    rl::writeCommandOffset(path, ilc);
    CHECK_THAT(refusal([&] {
                 fl::runFel(felInputs(s, log1, path, in1.output_weights, "i"));
               }),
               ContainsSubstring("has method 'ilc', not 'fel'"));
  }
  {
    // the same weights in another file are not its parent
    rl::FeedforwardWeights copy =
        rl::readFeedforwardWeights(in1.output_weights);
    copy.reference_sha256 = std::string(64, 'c');
    const fs::path path = s.files("copy.toml");
    rl::writeFeedforwardWeights(path, copy);
    CHECK_THAT(
        refusal([&] { fl::runFel(felInputs(s, log1, in1.output, path, "c")); }),
        ContainsSubstring("was synthesised from the weights file with "
                          "sha256"));
  }
  {
    // weights learned with other regressor options
    const fs::path other = writeLearnConfig(
        s.files, "other.toml", "[fel.regressors]\nsmooth_sigma_s = 0.03\n");
    auto in = felInputs(s, log0, s.zero, weights0, "o");
    in.config = other;
    const std::string message = refusal([&] { fl::runFel(in); });
    CHECK_THAT(message, ContainsSubstring("weights '" + weights0.string() +
                                          "' were learned with "
                                          "regressors.smooth_sigma_s = 0.04"));
    CHECK_THAT(message,
               ContainsSubstring("has fel.regressors.smooth_sigma_s = 0.03"));
    CHECK_FALSE(fs::exists(in.output_weights));
  }
}

TEST_CASE(
    "follow learn: fel removes the weights again when their offset is "
    "refused",
    "[learn][follow_learn][fel]") {
  Setup s("fel_refused", 1.0,
          "[limits]\nmax_offset_rad = 0.02\n"
          "[fel]\nstep = \"least-squares\"\ngain = 1.0\n");
  const fs::path weights0 = s.files("weights0.toml");
  fl::runInitWeights(s.config, weights0);
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  Trial trial = plannedTrial(s.ref, zero);
  constantError(&trial, 0.05);  // the constant column learns 0.05 rad
  const fs::path log = writeLog(s.files, "log.csv", trial.cycles);
  const auto in = felInputs(s, log, s.zero, weights0, "1");
  const std::string message = refusal([&] { fl::runFel(in); });
  CHECK_THAT(message,
             ContainsSubstring("fel: the offset of the updated weights "
                               "is refused, and nothing was written"));
  CHECK_THAT(message, ContainsSubstring("exceeds max_abs_rad 0.02"));
  CHECK_THAT(message, ContainsSubstring("lower [fel] gain"));
  CHECK_FALSE(fs::exists(in.output_weights));
  CHECK_FALSE(fs::exists(in.output));
}

TEST_CASE(
    "follow learn: synth gives the offset of trained weights on "
    "another reference",
    "[learn][follow_learn][synth]") {
  Setup s("synth", 1.0, "[fel]\nstep = \"least-squares\"\ngain = 0.5\n");
  const fs::path weights0 = s.files("weights0.toml");
  fl::runInitWeights(s.config, weights0);
  const rl::CommandOffset zero = rl::readCommandOffset(s.zero);
  Trial trial = plannedTrial(s.ref, zero);
  constantError(&trial, 0.004);
  const fs::path log = writeLog(s.files, "log.csv", trial.cycles);
  const auto in = felInputs(s, log, s.zero, weights0, "1");
  fl::runFel(in);
  const rl::FeedforwardWeights w1 =
      rl::readFeedforwardWeights(in.output_weights);

  // A longer reference with smaller, faster moves: another grid.
  const fs::path other = s.files("other.zvs");
  writeReference(other, 2.0, 0.8, 0.75);
  const fs::path other_follow = writeFollowConfig(s.files, "other.toml", other);
  const fs::path output = s.files("transfer.csv");
  const fl::LearnResult result =
      fl::runSynth({s.config, in.output_weights, other_follow, output});
  const rl::CommandOffset offset = rl::readCommandOffset(output);
  CHECK(offset.method == "fel");
  CHECK(offset.trial == 1);
  CHECK(offset.parent_sha256 == sha(in.output_weights));
  CHECK(offset.source_log_sha256 == w1.source_log_sha256);
  CHECK(offset.reference_sha256 == sha(other));
  CHECK(offset.reference_duration_s == 2.0);
  CHECK(offset.delta_rad.size() == 201);
  fl::FollowReference ref = fl::loadFollowReference(other_follow);
  const auto phi = rl::buildRegressors(*ref.chain, *ref.map, ref.q,
                                       ref.grid.dt_s, w1.regressors);
  CHECK(offset.delta_rad == rl::synthesizeOffset(phi, w1));
  CHECK(result.measured_trial == -1);
  // the transferred offset is a valid trial-1 start for fel on the other
  // reference
  REQUIRE_NOTHROW(
      rl::checkOffsetForRun(offset, 2.0, 0.01, sha(other), rl::OffsetLimits{}));

  CHECK_THAT(
      refusal([&] {
        fl::runSynth({s.config, weights0, other_follow, s.files("t0.csv")});
      }),
      ContainsSubstring("are trial 0 (never trained)"));
  const fs::path different = writeLearnConfig(
      s.files, "different.toml", "[fel.regressors]\ncoulomb_v0_rad_s = 0.1\n");
  CHECK_THAT(refusal([&] {
               fl::runSynth({different, in.output_weights, other_follow,
                             s.files("t1.csv")});
             }),
             ContainsSubstring("coulomb_v0_rad_s = 0.1"));
  CHECK_THAT(
      refusal([&] {
        fl::runSynth({s.config, in.output_weights, other_follow, output});
      }),
      ContainsSubstring("--output: refusing to overwrite"));
  CHECK_FALSE(fs::exists(s.files("t0.csv")));
  CHECK_FALSE(fs::exists(s.files("t1.csv")));
}

// ============================================================== the CLI

TEST_CASE("follow learn: the command line", "[learn][follow_learn][cli]") {
  const auto parse = [](std::vector<std::string> args) {
    args.insert(args.begin(), "x7_follow_learn");
    std::vector<char*> argv;
    for (auto& arg : args) argv.push_back(arg.data());
    return fl::parseLearnCli(static_cast<int>(argv.size()), argv.data());
  };
  const auto message = [&parse](std::vector<std::string> args) {
    return refusal([&] { parse(std::move(args)); });
  };
  const fl::LearnCli ilc =
      parse({"ilc", "--config", "l.toml", "--follow-config", "f.toml", "--log",
             "t.csv", "--offset", "p.csv", "--output", "n.csv", "--report",
             "r.toml"});
  REQUIRE(ilc.command == fl::LearnCommand::Ilc);
  CHECK(*ilc.config == "l.toml");
  CHECK(*ilc.follow_config == "f.toml");
  CHECK(*ilc.log == "t.csv");
  CHECK(*ilc.offset == "p.csv");
  CHECK(*ilc.output == "n.csv");
  CHECK(*ilc.report == "r.toml");
  CHECK_FALSE(ilc.help);
  const fl::LearnCli fel =
      parse({"fel", "--config", "l.toml", "--follow-config", "f.toml", "--log",
             "t.csv", "--offset", "p.csv", "--weights", "w.toml",
             "--output-weights", "nw.toml", "--output", "n.csv"});
  CHECK(fel.command == fl::LearnCommand::Fel);
  CHECK(*fel.output_weights == "nw.toml");
  CHECK_FALSE(fel.report);
  CHECK(parse({"init", "--follow-config", "f.toml", "--output", "o.csv"})
            .command == fl::LearnCommand::Init);
  CHECK(parse({"init-weights", "--config", "l.toml", "--output", "w.toml"})
            .command == fl::LearnCommand::InitWeights);
  CHECK(parse({"synth", "--config", "l.toml", "--weights", "w.toml",
               "--follow-config", "f.toml", "--output", "o.csv"})
            .command == fl::LearnCommand::Synth);

  const fl::LearnCli help = parse({"--help"});
  CHECK(help.help);
  CHECK_FALSE(help.command);
  const fl::LearnCli ilc_help = parse({"ilc", "--help"});
  CHECK(ilc_help.help);
  CHECK(ilc_help.command == fl::LearnCommand::Ilc);
  CHECK_THAT(fl::learnUsage(fl::LearnCommand::Ilc),
             ContainsSubstring("usage: x7_follow_learn ilc"));
  CHECK_THAT(fl::learnUsage(std::nullopt),
             ContainsSubstring("never opens the bus"));

  CHECK_THAT(message({}), ContainsSubstring("a command is required"));
  CHECK_THAT(message({"train"}), ContainsSubstring("unknown command 'train'"));
  CHECK_THAT(message({"ilc", "--port", "x"}),
             ContainsSubstring("unknown option: --port"));
  CHECK_THAT(message({"ilc", "--weights", "w.toml"}),
             ContainsSubstring("--weights is not an option of 'ilc'"));
  CHECK_THAT(message({"init", "--report", "r.toml"}),
             ContainsSubstring("--report is not an option of 'init'"));
  CHECK_THAT(message({"ilc", "--config"}),
             ContainsSubstring("--config requires a value"));
  CHECK_THAT(message({"ilc", "--config", "--log"}),
             ContainsSubstring("--config requires a value"));
  CHECK_THAT(message({"ilc", "--config", "a", "--config", "b"}),
             ContainsSubstring("--config is given twice"));
  CHECK_THAT(message({"ilc", "--config", "l.toml", "--follow-config", "f.toml",
                      "--offset", "p.csv", "--output", "n.csv"}),
             ContainsSubstring("--log is required for 'ilc'"));
  CHECK_THAT(message({"fel", "--config", "l.toml", "--follow-config", "f.toml",
                      "--log", "t.csv", "--offset", "p.csv", "--weights",
                      "w.toml", "--output", "n.csv"}),
             ContainsSubstring("--output-weights is required for 'fel'"));
  CHECK_THAT(message({"--help", "ilc"}),
             ContainsSubstring("--help takes no further arguments"));
}

// ========================================================= closed loops

namespace {

constexpr double kReferenceSeconds = 3.0;

// The ILC plant's smooth, repeatable output disturbance on reference time:
// a level and a sag that builds with the move, zero on the gripper.
std::vector<JointArray> sagDisturbance(const fl::FollowReference& ref) {
  std::vector<JointArray> d(ref.grid.samples());
  for (std::size_t i = 0; i < d.size(); ++i) {
    const double t =
        std::min(ref.duration_s, static_cast<double>(i) * ref.grid.dt_s);
    for (int j = 0; j < kArm; ++j) {
      const double level = 0.008 * (1.0 + 0.3 * j) * (j % 2 == 0 ? 1.0 : -1.0);
      const double sag = 0.012 * std::cos(1.1 * j + 0.4);
      d[i][j] = level + sag * minimumJerk((t - 0.2) / 2.4);
    }
  }
  return d;
}

}  // namespace

TEST_CASE("follow learn: ilc closes the loop through its own files",
          "[learn][follow_learn][closed_loop]") {
  // Plant: first-order lag of 40 ms and two samples of delay on the whole
  // command (unit DC gain), an output disturbance (a level and a sag up to
  // 0.016 rad); 100 Hz, 3 s reference. ILC: phi 0.5 on the arm (|1 - phi G|
  // = 0.5 per trial at low frequency), advance 60 ms (the delay plus the
  // lag), the default 2 Hz cutoff and clamps. Eight trials (0..7) from the
  // zero offset, an update after each.
  Setup s("ilc_loop", kReferenceSeconds,
          "[ilc]\nphi = 0.5\nadvance_s = 0.06\n");
  constexpr double kLag_s = 0.04;
  constexpr int kTrials = 8;
  const std::vector<JointArray> disturbance = sagDisturbance(s.ref);
  const std::size_t n = s.ref.grid.samples();
  std::vector<fs::path> offsets = {s.zero};
  std::vector<double> aggregate, arm;
  for (int k = 0; k < kTrials; ++k) {
    INFO("trial " << k);
    const rl::CommandOffset offset = rl::readCommandOffset(offsets[k]);
    Trial trial = plannedTrial(s.ref, offset);
    lagPlant(&trial, kLag_s, disturbance);
    const fs::path log =
        writeLog(s.files, "trial" + std::to_string(k) + ".csv", trial.cycles);
    offsets.push_back(s.files("offset" + std::to_string(k + 1) + ".csv"));
    const fl::LearnResult result = fl::runIlc(
        {s.config, s.follow, log, offsets[k], offsets[k + 1], std::nullopt});
    aggregate.push_back(result.aggregate_rms);
    arm.push_back(result.arm_rms);
    // the tool's tracking RMS is the plant's, to the log's rounding
    const double rounding = printBound(trial);
    for (int j = 0; j < kDof; ++j) {
      CHECK(std::fabs(result.error_rms[j] - trackingRms(trial, n, j)) <=
            rounding);
    }
    CHECK(result.error_rms[kGripper] == 0.0);
    // the lineage: each offset's parent is the previous offset file
    const rl::CommandOffset next = rl::readCommandOffset(offsets[k + 1]);
    CHECK(next.method == "ilc");
    CHECK(next.trial == k + 1);
    CHECK(next.parent_sha256 == sha(offsets[k]));
    CHECK(next.source_log_sha256 == sha(log));
    CHECK(next.reference_sha256 == sha(s.reference));
    for (const auto& joint : result.ilc->joints) {
      CHECK(joint.step_clamped == 0);  // the convergence is the law's
      CHECK(joint.abs_clamped == 0);
    }
  }
  std::string curve;
  for (int k = 0; k < kTrials; ++k) {
    char text[64];
    std::snprintf(text, sizeof text, "%s%.4e", k == 0 ? "" : " ", aggregate[k]);
    curve += text;
  }
  INFO("aggregate tracking RMS per trial (rad): " << curve);
  for (int k = 1; k < kTrials; ++k) {
    INFO("trial " << k);
    CHECK(aggregate[k] < aggregate[k - 1]);
    CHECK(arm[k] < arm[k - 1]);
  }
  CHECK(aggregate[kTrials - 1] < 0.1 * aggregate[0]);
  CHECK(arm[kTrials - 1] < 0.1 * arm[0]);
}

namespace {

struct FelScenario {
  Setup s;
  std::vector<rl::RegressorRow> phi;
  std::array<Row, kDof> column_rms{};
  Weights planted{};
  std::vector<JointArray> disturbance;

  FelScenario(const std::string& prefix, double gain)
      : s(prefix, kReferenceSeconds,
          "[fel]\nstep = \"least-squares\"\ngain = " + std::to_string(gain) +
              "\nadvance_s = 0.02\nerror_smooth_sigma_s = 0.0\n"
              "error_lead_s = 0.0\n") {
    phi = rl::buildRegressors(*s.ref.chain, *s.ref.map, s.ref.q,
                              s.ref.grid.dt_s, rl::RegressorOptions{});
    const double count = static_cast<double>(phi.size());
    for (int j = 0; j < kDof; ++j) {
      for (int l = 0; l < kColumns; ++l) {
        double sum = 0.0;
        for (const auto& row : phi) sum += row[j][l] * row[j][l];
        column_rms[j][l] = std::sqrt(sum / count);
      }
    }
    // 4 mrad per unit-RMS column on every well-excited arm column: the
    // disturbance is exactly Phi w*.
    for (int j = 0; j < kArm; ++j) {
      for (int l = 0; l < kColumns; ++l) {
        if (column_rms[j][l] >= 1e-3) planted[j][l] = 0.004 / column_rms[j][l];
      }
    }
    disturbance.resize(phi.size());
    for (std::size_t i = 0; i < phi.size(); ++i) {
      for (int j = 0; j < kDof; ++j) {
        double sum = 0.0;
        for (int l = 0; l < kColumns; ++l) sum += phi[i][j][l] * planted[j][l];
        disturbance[i][j] = sum;
      }
    }
  }

  // |u|: the weight error rms_l (w* - w)_l over the columns the update
  // keeps (RMS >= the library's negligible_rms).
  double weightError(const rl::FeedforwardWeights& w, int j) const {
    double sum = 0.0;
    for (int l = 0; l < kColumns; ++l) {
      if (column_rms[j][l] < rl::FelOptions{}.negligible_rms) continue;
      const double u = column_rms[j][l] * (planted[j][l] - w.w[j][l]);
      sum += u * u;
    }
    return std::sqrt(sum);
  }

  // RMS over the grid of Phi (w* - w): the error the update sees.
  double gridError(const rl::FeedforwardWeights& w, int j) const {
    double sum = 0.0;
    for (const auto& row : phi) {
      double e = 0.0;
      for (int l = 0; l < kColumns; ++l)
        e += row[j][l] * (planted[j][l] - w.w[j][l]);
      sum += e * e;
    }
    return std::sqrt(sum / static_cast<double>(phi.size()));
  }
};

struct FelTrial {
  fl::LearnResult result;
  rl::FeedforwardWeights weights;  // the weights the trial ran with
  double rounding = 0.0;           // printBound of its log
};

// Trials 0..trials-1 from zero weights, fel after each, the lineage checked
// at every step.
std::vector<FelTrial> felLoop(FelScenario& f, int trials) {
  Setup& s = f.s;
  std::vector<fs::path> offsets = {s.zero};
  std::vector<fs::path> weights = {s.files("weights0.toml")};
  fl::runInitWeights(s.config, weights[0]);
  std::vector<FelTrial> history;
  for (int k = 0; k < trials; ++k) {
    INFO("trial " << k);
    FelTrial step;
    step.weights = rl::readFeedforwardWeights(weights[k]);
    const rl::CommandOffset offset = rl::readCommandOffset(offsets[k]);
    Trial trial = plannedTrial(s.ref, offset);
    delayPlant(&trial, f.disturbance);
    step.rounding = printBound(trial);
    const fs::path log =
        writeLog(s.files, "trial" + std::to_string(k) + ".csv", trial.cycles);
    weights.push_back(s.files("weights" + std::to_string(k + 1) + ".toml"));
    offsets.push_back(s.files("offset" + std::to_string(k + 1) + ".csv"));
    step.result = fl::runFel({s.config, s.follow, log, offsets[k], weights[k],
                              weights[k + 1], offsets[k + 1], std::nullopt});
    const rl::FeedforwardWeights next =
        rl::readFeedforwardWeights(weights[k + 1]);
    CHECK(next.trial == k + 1);
    CHECK(next.parent_sha256 == sha(weights[k]));
    CHECK(next.source_log_sha256 == sha(log));
    CHECK(next.reference_sha256 == sha(s.reference));
    const rl::CommandOffset offset_next = rl::readCommandOffset(offsets[k + 1]);
    CHECK(offset_next.method == "fel");
    CHECK(offset_next.trial == k + 1);
    CHECK(offset_next.parent_sha256 == sha(weights[k + 1]));
    CHECK(offset_next.source_log_sha256 == sha(log));
    CHECK(step.result.error_rms[kGripper] == 0.0);
    history.push_back(std::move(step));
  }
  return history;
}

}  // namespace

// The bound. In RMS-normalised weight units u = rms_l (w* - w)_l (kept
// columns K, C their normalised Gram matrix), the plant makes the update's
// error signal exactly s = Phi_K (w* - w) on the grid (the advance is the
// plant's two-sample delay, no smoothing, no lead), up to the log's
// rounding eta (|eta| <= q, printBound). The least-squares step with gain g
// and ridge r is u' = u - g (C + r I)^-1 (C u + Phi^T eta / N). On C's
// eigenvectors the noise-free part scales mode k by (1 - g) + g r /
// (lambda_k + r), so with G(u) = sqrt(u^T C u) the RMS of Phi_K u over the
// grid (Minkowski, and lambda (r / (lambda + r))^2 <= r / 4):
//   G(u') <= (1 - g) G(u) + (g / 2) sqrt(r) |u| + g q,
// the last term because Phi (C + r I)^-1 Phi^T / N has eigenvalues below 1.
// The log's tracking rows see Phi[max(i - 2, 0)] u: two samples of the grid
// replaced by copies of the first, so the measured tracking RMS M is at most
// sqrt(3) G(u) + q (2 / N of the mean square moves, and a sample's square is
// at most N G^2).

TEST_CASE(
    "follow learn: one least-squares fel update removes a disturbance "
    "the regressors span",
    "[learn][follow_learn][closed_loop]") {
  // Gain 1: the bound leaves (1/2) sqrt(r) |u0| + q of the grid error, a
  // few micro-radians against the 13 mrad the disturbance causes.
  FelScenario f("fel_once", 1.0);
  const double r = rl::FelOptions{}.ridge;
  const auto history = felLoop(f, 2);
  const auto& before = history[0];
  const auto& after = history[1];
  double bound_square = 0.0;  // the gripper's error is exactly zero
  for (int j = 0; j < kArm; ++j) {
    INFO("joint " << j);
    const double g0 = f.gridError(before.weights, j);
    const double g1 = f.gridError(after.weights, j);
    const double u0 = f.weightError(before.weights, j);
    const double bound_grid = 0.5 * std::sqrt(r) * u0 + before.rounding;
    INFO("grid RMS " << g0 << " -> " << g1 << ", bound " << bound_grid);
    CHECK(g1 <= bound_grid + 1e-12);
    const double bound_measured = std::sqrt(3.0) * bound_grid + after.rounding;
    INFO("measured " << before.result.error_rms[j] << " -> "
                     << after.result.error_rms[j] << ", bound "
                     << bound_measured);
    CHECK(after.result.error_rms[j] <= bound_measured + 1e-12);
    bound_square += bound_measured * bound_measured;
  }
  // The bound itself proves a reduction of more than 1000 in one update.
  const double bound = std::sqrt(bound_square / kDof);
  INFO("aggregate " << before.result.aggregate_rms << " -> "
                    << after.result.aggregate_rms << ", bound " << bound);
  CHECK(after.result.aggregate_rms <= bound + 1e-12);
  CHECK(bound < 1e-3 * before.result.aggregate_rms);
}

TEST_CASE("follow learn: fel closes the loop through its own files",
          "[learn][follow_learn][closed_loop]") {
  // Gain 0.5: every update at least halves the error up to the stated
  // bound's small terms, so the error falls at every trial.
  FelScenario f("fel_loop", 0.5);
  constexpr double kGain = 0.5;
  constexpr int kTrials = 8;
  const double r = rl::FelOptions{}.ridge;
  const auto history = felLoop(f, kTrials);
  std::string curve;
  for (int k = 0; k < kTrials; ++k) {
    char text[64];
    std::snprintf(text, sizeof text, "%s%.4e", k == 0 ? "" : " ",
                  history[k].result.aggregate_rms);
    curve += text;
  }
  INFO("aggregate tracking RMS per trial (rad): " << curve);
  for (int k = 1; k < kTrials; ++k) {
    INFO("trial " << k);
    CHECK(history[k].result.aggregate_rms <
          history[k - 1].result.aggregate_rms);
    CHECK(history[k].result.arm_rms < history[k - 1].result.arm_rms);
    for (int j = 0; j < kArm; ++j) {
      INFO("joint " << j);
      const auto& w = history[k - 1].weights;
      const double bound = (1.0 - kGain) * f.gridError(w, j) +
                           0.5 * kGain * std::sqrt(r) * f.weightError(w, j) +
                           kGain * history[k - 1].rounding;
      CHECK(f.gridError(history[k].weights, j) <= bound + 1e-12);
    }
  }
  CHECK(history[kTrials - 1].result.aggregate_rms <
        0.1 * history[0].result.aggregate_rms);
}
