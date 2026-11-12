#include "rtctrl/learn/command_offset.hpp"

#include <sys/resource.h>

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using rtctrl::learn::CommandOffset;
using rtctrl::learn::JointArray;
using rtctrl::learn::OffsetLimits;
using rtctrl::model::kCanonicalDof;

namespace learn = rtctrl::learn;

namespace {

namespace fs = std::filesystem;

const std::string kReferenceHash =
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
const std::string kParentHash =
    "9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08";
const std::string kSourceHash =
    "2c26b46b68ffc68ff99b453c1d30413413422d706483bfa0f98a5e886266e7ae";
constexpr double kPeriod = 0.01;  // the follower's 100 Hz
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// The grid formula as written when the offset format was introduced (it was
// then FollowRun::trackReference's inline expression; the follower now calls
// learn::trackingGrid itself). The test below pins learn::trackingGrid to
// it bit for bit: a change of the formula changes the grid of every offset
// file already written, so it must be deliberate, not a refactoring.
struct FollowerGrid {
  long intervals;
  double dt;
};
FollowerGrid followerGrid(double duration, double control_dt) {
  const long intervals =
      std::max(1L, static_cast<long>(std::ceil(duration / control_dt)));
  const double dt = duration / intervals;
  return {intervals, dt};
}

bool sameBits(double a, double b) {
  std::uint64_t x = 0;
  std::uint64_t y = 0;
  std::memcpy(&x, &a, sizeof x);
  std::memcpy(&y, &b, sizeof y);
  return x == y;
}

// A learned offset on the follower's grid of a 0.3 s reference: small and
// smooth, well inside the default limits.
CommandOffset learned() {
  const auto grid = learn::trackingGrid(0.3, kPeriod);
  CommandOffset offset;
  offset.method = "ilc";
  offset.trial = 2;
  offset.dt_s = grid.dt_s;
  offset.reference_duration_s = 0.3;
  offset.reference_sha256 = kReferenceHash;
  offset.parent_sha256 = kParentHash;
  offset.source_log_sha256 = kSourceHash;
  offset.delta_rad.resize(grid.samples());
  for (std::size_t i = 0; i < grid.samples(); ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) {
      offset.delta_rad[i][j] = 0.001 * (j + 1) * std::sin(0.1 * i);
    }
  }
  return offset;
}

// On the exactly representable grid of a 1 s reference at 1/64 s: joint 5
// ramps up at exactly max_rate_rad_s to exactly max_abs_rad, joint 2 the
// same downward, so both default limits are met with equality.
constexpr double kDyadicPeriod = 1.0 / 64.0;
CommandOffset atLimits() {
  const auto grid = learn::trackingGrid(1.0, kDyadicPeriod);
  REQUIRE(grid.dt_s == kDyadicPeriod);
  CommandOffset offset;
  offset.method = "ilc";
  offset.trial = 1;
  offset.dt_s = grid.dt_s;
  offset.reference_duration_s = 1.0;
  offset.reference_sha256 = kReferenceHash;
  offset.parent_sha256 = kParentHash;
  offset.source_log_sha256 = kSourceHash;
  offset.delta_rad.resize(grid.samples());
  for (std::size_t i = 0; i < grid.samples(); ++i) {
    const double ramp = std::min(static_cast<double>(i) / 128.0, 0.15);
    offset.delta_rad[i][5] = ramp;
    offset.delta_rad[i][2] = -ramp;
  }
  return offset;
}

// Three samples on a 0.02 s reference at 100 Hz: short lines for the
// reader's rejections (preamble lines 1-10, header 11, rows 12-14).
CommandOffset small() {
  CommandOffset offset;
  offset.method = "ilc";
  offset.trial = 3;
  offset.dt_s = 0.01;
  offset.reference_duration_s = 0.02;
  offset.reference_sha256 = kReferenceHash;
  offset.parent_sha256 = kParentHash;
  offset.source_log_sha256 = kSourceHash;
  offset.delta_rad = {
      JointArray{},
      JointArray{0.001, -0.002, 0.003, -0.004, 0.005, -0.006, 0.007, -0.008},
      JointArray{0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08}};
  return offset;
}

template <typename Mutate>
CommandOffset changed(CommandOffset offset, Mutate mutate) {
  mutate(offset);
  return offset;
}

void expectStructureError(const CommandOffset& offset,
                          const std::vector<std::string>& phrases) {
  try {
    learn::checkOffsetStructure(offset);
    FAIL("structural check passed, expected an error");
  } catch (const std::runtime_error& error) {
    CHECK_THAT(error.what(), StartsWith("command offset: "));
    for (const auto& phrase : phrases) {
      CHECK_THAT(error.what(), ContainsSubstring(phrase));
    }
  }
}

void expectRunError(const CommandOffset& offset, double duration,
                    double control_dt, const std::string& hash,
                    const OffsetLimits& limits,
                    const std::vector<std::string>& phrases) {
  try {
    learn::checkOffsetForRun(offset, duration, control_dt, hash, limits);
    FAIL("run check passed, expected an error");
  } catch (const std::runtime_error& error) {
    for (const auto& phrase : phrases) {
      CHECK_THAT(error.what(), ContainsSubstring(phrase));
    }
  }
}

CommandOffset parse(const std::string& text) {
  std::istringstream input(text);
  return learn::parseCommandOffset(input);
}

void expectReadError(const std::string& text,
                     const std::vector<std::string>& phrases) {
  std::istringstream input(text);
  try {
    learn::parseCommandOffset(input);
    FAIL("read succeeded, expected an error");
  } catch (const std::runtime_error& error) {
    CHECK_THAT(error.what(), StartsWith("command offset: "));
    for (const auto& phrase : phrases) {
      CHECK_THAT(error.what(), ContainsSubstring(phrase));
    }
  }
}

std::vector<std::string> splitLines(const std::string& text) {
  std::vector<std::string> lines;
  std::size_t start = 0;
  while (start < text.size()) {
    const std::size_t newline = text.find('\n', start);
    if (newline == std::string::npos) {
      lines.push_back(text.substr(start));
      break;
    }
    lines.push_back(text.substr(start, newline - start));
    start = newline + 1;
  }
  return lines;
}

std::string joinLines(const std::vector<std::string>& lines) {
  std::string text;
  for (const auto& line : lines) text += line + "\n";
  return text;
}

// The text with 1-based line `number` replaced.
std::string withLine(const std::string& text, std::size_t number,
                     const std::string& replacement) {
  auto lines = splitLines(text);
  lines.at(number - 1) = replacement;
  return joinLines(lines);
}

// The text with 0-based comma field `field` of 1-based line `number`
// replaced.
std::string withField(const std::string& text, std::size_t number,
                      std::size_t field, const std::string& replacement) {
  auto lines = splitLines(text);
  std::vector<std::string> fields;
  std::stringstream row(lines.at(number - 1));
  for (std::string value; std::getline(row, value, ',');) {
    fields.push_back(value);
  }
  fields.at(field) = replacement;
  std::string joined;
  for (std::size_t f = 0; f < fields.size(); ++f) {
    joined += (f == 0 ? "" : ",") + fields[f];
  }
  lines.at(number - 1) = joined;
  return joinLines(lines);
}

void checkSameOffset(const CommandOffset& a, const CommandOffset& b) {
  CHECK(a.method == b.method);
  CHECK(a.trial == b.trial);
  CHECK(sameBits(a.dt_s, b.dt_s));
  CHECK(sameBits(a.reference_duration_s, b.reference_duration_s));
  CHECK(a.reference_sha256 == b.reference_sha256);
  CHECK(a.parent_sha256 == b.parent_sha256);
  CHECK(a.source_log_sha256 == b.source_log_sha256);
  REQUIRE(a.delta_rad.size() == b.delta_rad.size());
  for (std::size_t i = 0; i < a.delta_rad.size(); ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) {
      INFO("sample " << i << ", joint " << j);
      CHECK(sameBits(a.delta_rad[i][j], b.delta_rad[i][j]));
    }
  }
}

std::string slurp(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  std::ostringstream text;
  text << input.rdbuf();
  return text.str();
}

// A file in build/command_offset_test, removed before (a leftover of an
// aborted run of this test) and after use. Only the named file is ever
// removed, never a directory.
class TempPath {
 public:
  explicit TempPath(const std::string& name) {
    fs::create_directories("build/command_offset_test");
    path_ = fs::path("build/command_offset_test") / name;
    std::error_code ignored;
    fs::remove(path_, ignored);
  }
  ~TempPath() {
    std::error_code ignored;
    fs::remove(path_, ignored);
  }
  TempPath(const TempPath&) = delete;
  TempPath& operator=(const TempPath&) = delete;
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

bool absent(const fs::path& path) {
  std::error_code error;
  return fs::symlink_status(path, error).type() == fs::file_type::not_found;
}

// Runs body with the soft file-size limit lowered to `bytes` and SIGXFSZ
// ignored, so a write past the limit fails with EFBIG instead of killing
// the process; both are restored before returning. Body must not throw or
// report to Catch (stdout may be a file). False if the limit was not set.
template <typename Body>
bool withFileSizeLimit(rlim_t bytes, Body body) {
  rlimit saved{};
  if (getrlimit(RLIMIT_FSIZE, &saved) != 0) return false;
  rlimit limited = saved;
  limited.rlim_cur = bytes;
  const auto previous = std::signal(SIGXFSZ, SIG_IGN);
  const bool set = setrlimit(RLIMIT_FSIZE, &limited) == 0;
  if (set) body();
  setrlimit(RLIMIT_FSIZE, &saved);
  std::signal(SIGXFSZ, previous);
  return set;
}

}  // namespace

// Pins the formula (see FollowerGrid). That the follower commands this grid
// and its preflight validates the same one is tested in follow_run_test.cpp.
TEST_CASE("command offset: the tracking grid is the follower's",
          "[learn][command_offset]") {
  for (const double period : {kPeriod, 0.005, kDyadicPeriod}) {
    for (const double duration :
         {1.0, 0.3, 0.035, 0.07, 0.7, 2.0000001, 12.345, 60.0, 0.005, 0.0049,
          1e-6}) {
      INFO("duration " << duration << " s, period " << period << " s");
      const auto grid = learn::trackingGrid(duration, period);
      const auto mirror = followerGrid(duration, period);
      CHECK(grid.intervals == mirror.intervals);
      CHECK(sameBits(grid.dt_s, mirror.dt));
      CHECK(grid.samples() == static_cast<std::size_t>(mirror.intervals) + 1);
    }
  }
  // a multiple of the period
  CHECK(learn::trackingGrid(1.0, kPeriod).intervals == 100);
  // not a multiple: rounded up, the step shrinks to fit
  const auto odd = learn::trackingGrid(0.035, kPeriod);
  CHECK(odd.intervals == 4);
  CHECK(odd.samples() == 5);
  CHECK(odd.dt_s == 0.035 / 4);
  // 0.07 / 0.01 rounds above 7: the follower uses 8 intervals
  CHECK(learn::trackingGrid(0.07, kPeriod).intervals == 8);
  // shorter than one period: one interval, both ends sampled
  const auto brief = learn::trackingGrid(0.005, kPeriod);
  CHECK(brief.intervals == 1);
  CHECK(brief.samples() == 2);
  CHECK(brief.dt_s == 0.005);
}

TEST_CASE("command offset: the tracking grid refuses invalid inputs",
          "[learn][command_offset]") {
  for (const double duration : {0.0, -1.0, kNaN, kInf}) {
    CHECK_THROWS_WITH(learn::trackingGrid(duration, kPeriod),
                      ContainsSubstring("reference duration"));
  }
  for (const double period : {0.0, -0.01, kNaN, kInf}) {
    CHECK_THROWS_WITH(learn::trackingGrid(1.0, period),
                      ContainsSubstring("control period"));
  }
  CHECK_THROWS_WITH(learn::trackingGrid(1e20, kPeriod),
                    ContainsSubstring("too many intervals"));
  CHECK_THROWS_WITH(learn::trackingGrid(1e300, 1e-300),
                    ContainsSubstring("too many intervals"));
}

TEST_CASE("command offset: the zero offset", "[learn][command_offset]") {
  const CommandOffset zero = learn::zeroOffset(0.3, kPeriod, kReferenceHash);
  const auto grid = learn::trackingGrid(0.3, kPeriod);
  CHECK(zero.method == "zero");
  CHECK(zero.trial == 0);
  CHECK(sameBits(zero.dt_s, grid.dt_s));
  CHECK(zero.reference_duration_s == 0.3);
  CHECK(zero.reference_sha256 == kReferenceHash);
  CHECK(zero.parent_sha256.empty());
  CHECK(zero.source_log_sha256.empty());
  REQUIRE(zero.delta_rad.size() == grid.samples());
  for (const auto& sample : zero.delta_rad) {
    for (const double value : sample) CHECK(sameBits(value, 0.0));
  }
  CHECK_NOTHROW(learn::checkOffsetStructure(zero));
  CHECK_NOTHROW(learn::checkOffsetForRun(zero, 0.3, kPeriod, kReferenceHash,
                                         OffsetLimits{}));

  CHECK(learn::zeroOffset(0.005, kPeriod, kReferenceHash).delta_rad.size() ==
        2);
  std::string upper = kReferenceHash;
  for (char& c : upper) c = static_cast<char>(std::toupper(c));
  CHECK_THROWS_WITH(learn::zeroOffset(0.3, kPeriod, upper),
                    ContainsSubstring("reference_sha256"));
  CHECK_THROWS_WITH(learn::zeroOffset(0.0, kPeriod, kReferenceHash),
                    ContainsSubstring("tracking grid"));
}

TEST_CASE("command offset: the structural check names each problem",
          "[learn][command_offset]") {
  const CommandOffset good = learned();
  REQUIRE_NOTHROW(learn::checkOffsetStructure(good));
  const CommandOffset zero = learn::zeroOffset(0.3, kPeriod, kReferenceHash);

  SECTION("method") {
    for (const std::string method : {"pid", "", "ILC", "zero "}) {
      expectStructureError(
          changed(good, [&method](CommandOffset& o) { o.method = method; }),
          {"method '" + method + "'", "not one of"});
    }
    CHECK_NOTHROW(learn::checkOffsetStructure(
        changed(good, [](CommandOffset& o) { o.method = "fel"; })));
  }
  SECTION("trial") {
    expectStructureError(changed(good, [](CommandOffset& o) { o.trial = -1; }),
                         {"trial -1 is negative"});
  }
  SECTION("dt_s") {
    for (const double dt : {0.0, -0.01, kNaN, kInf}) {
      expectStructureError(
          changed(good, [dt](CommandOffset& o) { o.dt_s = dt; }),
          {"dt_s", "must be finite and > 0"});
    }
  }
  SECTION("reference_duration_s") {
    for (const double duration : {0.0, -0.3, kNaN, kInf}) {
      expectStructureError(
          changed(good, [duration](CommandOffset& o) {
            o.reference_duration_s = duration;
          }),
          {"reference_duration_s", "must be finite and > 0"});
    }
  }
  SECTION("sample count") {
    expectStructureError(
        changed(good, [](CommandOffset& o) { o.delta_rad.resize(1); }),
        {"at least 2 samples needed, got 1"});
    expectStructureError(
        changed(good, [](CommandOffset& o) { o.delta_rad.clear(); }),
        {"at least 2 samples needed, got 0"});
  }
  SECTION("span against the reference duration") {
    expectStructureError(changed(good,
                                 [](CommandOffset& o) {
                                   o.reference_duration_s += 1.1e-9;
                                 }),
                         {"(samples - 1) * dt_s", "reference_duration_s",
                          "tolerance 1e-9 s"});
    expectStructureError(
        changed(good, [](CommandOffset& o) { o.delta_rad.pop_back(); }),
        {"(samples - 1) * dt_s"});
    CHECK_NOTHROW(learn::checkOffsetStructure(changed(
        good, [](CommandOffset& o) { o.reference_duration_s += 0.9e-9; })));
  }
  SECTION("non-finite values") {
    expectStructureError(
        changed(good, [](CommandOffset& o) { o.delta_rad[5][3] = kNaN; }),
        {"non-finite delta", "joint 3, sample 5 (t = 0.05 s)"});
    expectStructureError(
        changed(good, [](CommandOffset& o) { o.delta_rad[0][7] = kInf; }),
        {"non-finite delta", "joint 7, sample 0"});
    expectStructureError(
        changed(good, [](CommandOffset& o) { o.delta_rad[30][0] = -kInf; }),
        {"non-finite delta", "joint 0, sample 30"});
  }
  SECTION("reference_sha256") {
    std::string upper = kReferenceHash;
    for (char& c : upper) c = static_cast<char>(std::toupper(c));
    for (const std::string& hash :
         {std::string(), kReferenceHash.substr(1), kReferenceHash + "0", upper,
          "g" + kReferenceHash.substr(1), " " + kReferenceHash.substr(1)}) {
      expectStructureError(changed(good,
                                   [&hash](CommandOffset& o) {
                                     o.reference_sha256 = hash;
                                   }),
                           {"reference_sha256", "64 lowercase hex"});
    }
  }
  SECTION("parent_sha256 and source_log_sha256") {
    expectStructureError(changed(good,
                                 [](CommandOffset& o) {
                                   o.parent_sha256 = kParentHash.substr(1);
                                 }),
                         {"parent_sha256", "neither empty nor 64"});
    expectStructureError(changed(good,
                                 [](CommandOffset& o) {
                                   o.source_log_sha256 = kSourceHash + "a";
                                 }),
                         {"source_log_sha256", "neither empty nor 64"});
  }
  SECTION("trial 0 if and only if method zero") {
    expectStructureError(changed(zero, [](CommandOffset& o) { o.trial = 1; }),
                         {"method 'zero' requires trial 0"});
    for (const std::string method : {"ilc", "fel"}) {
      expectStructureError(changed(good,
                                   [&method](CommandOffset& o) {
                                     o.method = method;
                                     o.trial = 0;
                                   }),
                           {"trial 0 requires method 'zero'"});
    }
  }
  SECTION("a zero offset is exactly zero, without lineage") {
    expectStructureError(
        changed(zero, [](CommandOffset& o) { o.delta_rad[4][2] = 1e-300; }),
        {"all zeros", "joint 2, sample 4"});
    expectStructureError(
        changed(zero, [](CommandOffset& o) { o.parent_sha256 = kParentHash; }),
        {"zero offset", "has no parent_sha256"});
    expectStructureError(
        changed(zero,
                [](CommandOffset& o) { o.source_log_sha256 = kSourceHash; }),
        {"zero offset", "has no source_log_sha256"});
    // negative zero is zero
    CHECK_NOTHROW(learn::checkOffsetStructure(
        changed(zero, [](CommandOffset& o) { o.delta_rad[4][2] = -0.0; })));
  }
  SECTION("a learned offset names its lineage") {
    expectStructureError(
        changed(good, [](CommandOffset& o) { o.parent_sha256.clear(); }),
        {"trial 2 requires a parent_sha256"});
    expectStructureError(
        changed(good, [](CommandOffset& o) { o.source_log_sha256.clear(); }),
        {"trial 2 requires a source_log_sha256"});
  }
}

TEST_CASE("command offset: limits met with equality are accepted",
          "[learn][command_offset]") {
  const CommandOffset offset = atLimits();
  REQUIRE(offset.delta_rad[64][5] == 0.15);
  REQUIRE((offset.delta_rad[1][5] - offset.delta_rad[0][5]) / offset.dt_s ==
          0.5);
  CHECK_NOTHROW(learn::checkOffsetForRun(offset, 1.0, kDyadicPeriod,
                                         kReferenceHash, OffsetLimits{}));
  CHECK_NOTHROW(learn::checkOffsetForRun(learned(), 0.3, kPeriod,
                                         kReferenceHash, OffsetLimits{}));
}

TEST_CASE("command offset: the run check names each problem",
          "[learn][command_offset]") {
  const CommandOffset offset = atLimits();
  const OffsetLimits limits;

  SECTION("limits must be finite and positive") {
    for (const double value : {0.0, -0.1, kNaN, kInf}) {
      expectRunError(offset, 1.0, kDyadicPeriod, kReferenceHash,
                     OffsetLimits{value, 0.5}, {"max_abs_rad"});
      expectRunError(offset, 1.0, kDyadicPeriod, kReferenceHash,
                     OffsetLimits{0.15, value}, {"max_rate_rad_s"});
    }
  }
  SECTION("the rate limit has a floor") {
    // Even an offset that never moves: the limit is the follower's ramp
    // rate, and a rate near zero makes that ramp arbitrarily long.
    const CommandOffset zero =
        learn::zeroOffset(1.0, kDyadicPeriod, kReferenceHash);
    CHECK_NOTHROW(learn::checkOffsetForRun(
        zero, 1.0, kDyadicPeriod, kReferenceHash,
        OffsetLimits{0.15, learn::kMinOffsetRateLimitRadS}));
    REQUIRE(std::nextafter(learn::kMinOffsetRateLimitRadS, 0.0) ==
            0.009999999999999998);
    for (const auto& [value, text] :
         {std::pair<double, std::string>{0.009999999999999998,
                                         "0.009999999999999998"},
          {1e-3, "0.001"},
          {1e-300, "1e-300"}}) {
      expectRunError(zero, 1.0, kDyadicPeriod, kReferenceHash,
                     OffsetLimits{0.15, value},
                     {"command offset limits: max_rate_rad_s " + text +
                      " is below the floor 0.01 rad/s"});
    }
  }
  SECTION("the structural check comes first") {
    expectRunError(
        changed(offset, [](CommandOffset& o) { o.method = "pid"; }), 1.0,
        kDyadicPeriod, kReferenceHash, limits, {"method 'pid'"});
  }
  SECTION("the run's tracking grid") {
    // another duration, another period: another sample count
    expectRunError(learned(), 0.31, kPeriod, kReferenceHash, limits,
                   {"31 samples", "has 32"});
    expectRunError(learned(), 0.3, 0.005, kReferenceHash, limits,
                   {"31 samples", "has 61"});
    // the same count, a step off by more than 1e-12 s
    const CommandOffset base = learned();
    expectRunError(
        changed(base, [](CommandOffset& o) { o.dt_s += 2e-12; }), 0.3,
        kPeriod, kReferenceHash, limits,
        {"dt_s", "tracking grid step", "tolerance 1e-12 s"});
    CHECK_NOTHROW(learn::checkOffsetForRun(
        changed(base, [](CommandOffset& o) { o.dt_s += 0.5e-12; }), 0.3,
        kPeriod, kReferenceHash, limits));
    // an invalid run
    expectRunError(base, 0.0, kPeriod, kReferenceHash, limits,
                   {"tracking grid"});
  }
  SECTION("the reference") {
    expectRunError(offset, 1.0, kDyadicPeriod, kParentHash, limits,
                   {"reference_sha256", "does not match"});
  }
  SECTION("the magnitude limit") {
    expectRunError(changed(offset,
                           [](CommandOffset& o) {
                             o.delta_rad[30][5] =
                                 std::nextafter(0.15, 1.0);
                           }),
                   1.0, kDyadicPeriod, kReferenceHash, limits,
                   {"joint 5, sample 30 (t = 0.46875 s)",
                    "delta 0.15000000000000002 rad", "max_abs_rad 0.15"});
    expectRunError(
        changed(offset, [](CommandOffset& o) { o.delta_rad[40][2] = -0.16; }),
        1.0, kDyadicPeriod, kReferenceHash, limits,
        {"joint 2, sample 40 (t = 0.625 s)", "delta -0.16 rad",
         "exceeds max_abs_rad"});
    expectRunError(offset, 1.0, kDyadicPeriod, kReferenceHash,
                   OffsetLimits{0.1, 0.5},
                   {"joint 2, sample 13", "max_abs_rad 0.1"});
  }
  SECTION("the rate limit") {
    expectRunError(changed(offset,
                           [](CommandOffset& o) {
                             o.delta_rad[1][5] += std::ldexp(1.0, -30);
                           }),
                   1.0, kDyadicPeriod, kReferenceHash, limits,
                   {"joint 5, sample 0 (t = 0 s) to sample 1",
                    "rate 0.5000000596046448 rad/s", "max_rate_rad_s 0.5"});
    expectRunError(
        changed(offset, [](CommandOffset& o) { o.delta_rad[50][0] = 0.01; }),
        1.0, kDyadicPeriod, kReferenceHash, limits,
        {"joint 0, sample 49 (t = 0.765625 s) to sample 50", "rate 0.64",
         "delta 0 to 0.01 rad"});
    expectRunError(offset, 1.0, kDyadicPeriod, kReferenceHash,
                   OffsetLimits{0.15, 0.25},
                   {"joint 2, sample 0", "max_rate_rad_s 0.25"});
  }
}

TEST_CASE("command offset: the file text", "[learn][command_offset]") {
  const std::string zero = learn::formatCommandOffset(
      learn::zeroOffset(0.02, kPeriod, kReferenceHash));
  const std::string zeros = ",0,0,0,0,0,0,0,0\n";
  CHECK(zero ==
        "# format: rtctrl-x7-follow-offset\n"
        "# version: 1\n"
        "# method: zero\n"
        "# trial: 0\n"
        "# dt_s: 0.01\n"
        "# samples: 3\n"
        "# reference_duration_s: 0.02\n"
        "# reference_sha256: " + kReferenceHash + "\n"
        "# parent_sha256:\n"
        "# source_log_sha256:\n"
        "time_s,delta0_rad,delta1_rad,delta2_rad,delta3_rad,delta4_rad,"
        "delta5_rad,delta6_rad,delta7_rad\n"
        "0.000000000" + zeros + "0.010000000" + zeros + "0.020000000" + zeros);

  const auto lines = splitLines(learn::formatCommandOffset(small()));
  REQUIRE(lines.size() == 14);
  CHECK(lines[2] == "# method: ilc");
  CHECK(lines[3] == "# trial: 3");
  CHECK(lines[8] == "# parent_sha256: " + kParentHash);
  CHECK(lines[9] == "# source_log_sha256: " + kSourceHash);
  CHECK(lines[12] ==
        "0.010000000,0.001,-0.002,0.0030000000000000001,-0.0040000000000000001,"
        "0.0050000000000000001,-0.0060000000000000001,0.0070000000000000001,"
        "-0.0080000000000000002");

  CHECK_THROWS_WITH(learn::formatCommandOffset(changed(
                        small(), [](CommandOffset& o) { o.trial = 0; })),
                    ContainsSubstring("trial 0 requires method 'zero'"));
}

TEST_CASE("command offset: the file round trip is exact",
          "[learn][command_offset]") {
  constexpr double kMax = std::numeric_limits<double>::max();
  const std::array<double, 15> awkward = {
      0.1,
      1e-300,
      -0.0,
      kMax,
      -kMax,
      std::numeric_limits<double>::denorm_min(),
      std::numeric_limits<double>::min(),
      1.0 / 3.0,
      -2.5e-12,
      0.15,
      std::nextafter(0.15, 1.0),
      std::nextafter(0.15, 0.0),
      -0.15,
      0.0,
      123456.789};
  // an awkward grid: 2.0000001 s at 100 Hz is 201 steps of 9.95... ms
  const auto grid = learn::trackingGrid(2.0000001, kPeriod);
  CommandOffset offset;
  offset.method = "fel";
  offset.trial = 17;
  offset.dt_s = grid.dt_s;
  offset.reference_duration_s = 2.0000001;
  offset.reference_sha256 = kReferenceHash;
  offset.parent_sha256 = kParentHash;
  offset.source_log_sha256 = kSourceHash;
  offset.delta_rad.resize(grid.samples());
  for (std::size_t i = 0; i < grid.samples(); ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) {
      offset.delta_rad[i][j] = awkward[(i + 3 * j) % awkward.size()];
    }
  }
  REQUIRE_NOTHROW(learn::checkOffsetStructure(offset));

  for (const CommandOffset& source :
       {offset, learn::zeroOffset(0.3, kPeriod, kReferenceHash), atLimits()}) {
    INFO("method " << source.method << ", trial " << source.trial);
    TempPath first("round_trip_first.csv");
    TempPath second("round_trip_second.csv");
    learn::writeCommandOffset(first.path(), source);
    const std::string bytes = slurp(first.path());
    CHECK(bytes == learn::formatCommandOffset(source));

    const CommandOffset read = learn::readCommandOffset(first.path());
    checkSameOffset(read, source);
    checkSameOffset(parse(bytes), source);

    learn::writeCommandOffset(second.path(), read);
    CHECK(slurp(second.path()) == bytes);
  }
  CHECK(std::signbit(parse(learn::formatCommandOffset(offset))
                         .delta_rad[2][0]));  // -0.0 stays negative
}

TEST_CASE("command offset: the reader rejects malformed files",
          "[learn][command_offset]") {
  const std::string good = learn::formatCommandOffset(small());
  REQUIRE_NOTHROW(parse(good));
  const auto lines = splitLines(good);
  REQUIRE(lines.size() == 14);
  auto without = [&lines](std::size_t number) {
    auto edited = lines;
    edited.erase(edited.begin() + static_cast<long>(number - 1));
    return joinLines(edited);
  };
  auto inserted = [&lines](std::size_t number, const std::string& line) {
    auto edited = lines;
    edited.insert(edited.begin() + static_cast<long>(number - 1), line);
    return joinLines(edited);
  };

  SECTION("format and version") {
    expectReadError(withLine(good, 1, "# format: rtctrl-x7-teach-log"),
                    {"line 1:", "wrong format 'rtctrl-x7-teach-log'"});
    expectReadError(withLine(good, 2, "# version: 2"),
                    {"line 2:", "unsupported version '2'"});
    expectReadError(withLine(good, 2, "# version: 01"),
                    {"line 2:", "unsupported version '01'"});
  }
  SECTION("missing, duplicate, unknown and reordered keys") {
    expectReadError(without(4), {"line 4:", "missing key 'trial'"});
    expectReadError(without(10),
                    {"line 10:", "missing key 'source_log_sha256'"});
    expectReadError(inserted(4, lines[2]),
                    {"line 4:", "duplicate key 'method'"});
    expectReadError(inserted(11, lines[9]),
                    {"line 11:", "duplicate key 'source_log_sha256'"});
    expectReadError(withLine(good, 5, "# dt: 0.01"),
                    {"line 5:", "unknown key 'dt'"});
    expectReadError(inserted(11, "# status: ok"),
                    {"line 11:", "unknown key 'status'"});
    auto swapped = lines;
    std::swap(swapped[2], swapped[3]);
    expectReadError(joinLines(swapped),
                    {"line 3:", "missing key 'method'", "found 'trial'"});
  }
  SECTION("malformed preamble lines") {
    for (const std::string line :
         {"# trial 3", "# trial: ", "# trial:3", "# trial:  3", "# trial: 3 ",
          "#trial: 3"}) {
      INFO(line);
      expectReadError(withLine(good, 4, line), {"line 4:"});
    }
  }
  SECTION("preamble values") {
    for (const std::string trial : {"x", "1.5", "+3", "99999999999"}) {
      expectReadError(withLine(good, 4, "# trial: " + trial),
                      {"line 4:", "trial: not an integer"});
    }
    expectReadError(withLine(good, 4, "# trial:"),
                    {"line 4:", "trial: not an integer"});
    for (const std::string dt : {"nan", "inf", "0.01x", "1e999"}) {
      expectReadError(withLine(good, 5, "# dt_s: " + dt),
                      {"line 5:", "dt_s: not a finite number"});
    }
    expectReadError(withLine(good, 5, "# dt_s:"),
                    {"line 5:", "dt_s: not a finite number"});
    for (const std::string samples : {"-1", "3.0", "x"}) {
      expectReadError(withLine(good, 6, "# samples: " + samples),
                      {"line 6:", "samples: not a non-negative integer"});
    }
    expectReadError(withLine(good, 7, "# reference_duration_s: inf"),
                    {"line 7:", "reference_duration_s: not a finite number"});
  }
  SECTION("end of file inside the preamble or before the header") {
    expectReadError("", {"line 1:", "end of file", "'format'"});
    std::vector<std::string> head(lines.begin(), lines.begin() + 5);
    expectReadError(joinLines(head), {"line 6:", "end of file", "'samples'"});
    head.assign(lines.begin(), lines.begin() + 10);
    expectReadError(joinLines(head),
                    {"line 11:", "end of file", "column header"});
  }
  SECTION("the column header") {
    std::string short_header = lines[10];
    short_header.erase(short_header.rfind(','));
    expectReadError(withLine(good, 11, short_header),
                    {"line 11:", "wrong column header"});
    expectReadError(withLine(good, 11, "time," + lines[10].substr(7)),
                    {"line 11:", "wrong column header"});
  }
  SECTION("field counts") {
    std::string short_row = lines[12];
    short_row.erase(short_row.rfind(','));
    expectReadError(withLine(good, 13, short_row),
                    {"line 13:", "8 fields, expected 9"});
    expectReadError(withLine(good, 13, lines[12] + ",0"),
                    {"line 13:", "10 fields, expected 9"});
    expectReadError(inserted(13, ""), {"line 13:", "1 fields, expected 9"});
  }
  SECTION("numbers") {
    for (const std::string bad : {"nan", "inf", "-inf", "", "abc", "1.5x",
                                  " 1", "1 ", "1e999", "+1", "0x1p3"}) {
      INFO("'" << bad << "'");
      expectReadError(
          withField(good, 14, 4, bad),
          {"line 14:", "column 'delta3_rad'", "not a finite number"});
      expectReadError(withField(good, 12, 0, bad),
                      {"line 12:", "column 'time_s'", "not a finite number"});
    }
  }
  SECTION("row count against samples") {
    expectReadError(
        withLine(good, 6, "# samples: 4"),
        {"line 15:", "end of file after 3 data rows, samples is 4"});
    expectReadError(withLine(good, 6, "# samples: 2"),
                    {"line 14:", "more data rows than samples (2)"});
  }
  SECTION("time_s against i * dt_s") {
    expectReadError(withField(good, 13, 0, "0.010002000"),
                    {"line 13:", "time_s", "sample 1", "1e-6 s"});
    expectReadError(withField(good, 14, 0, "0.010000000"),
                    {"line 14:", "time_s", "sample 2"});
    CHECK_NOTHROW(parse(withField(good, 13, 0, "0.010000500")));
  }
  SECTION("line ends") {
    expectReadError(good.substr(0, good.size() - 1),
                    {"line 14:", "missing final newline"});
    std::string crlf;
    for (const char c : good) {
      if (c == '\n') crlf += '\r';
      crlf += c;
    }
    expectReadError(crlf, {"line 1:", "wrong format"});
  }
  SECTION("the structural check follows") {
    expectReadError(withLine(good, 3, "# method: pid"), {"method 'pid'"});
    expectReadError(withLine(good, 4, "# trial: 0"),
                    {"trial 0 requires method 'zero'"});
    expectReadError(withLine(good, 7, "# reference_duration_s: 0.03"),
                    {"(samples - 1) * dt_s"});
    expectReadError(withLine(good, 9, "# parent_sha256:"),
                    {"trial 3 requires a parent_sha256"});
    std::string upper = kReferenceHash;
    for (char& c : upper) c = static_cast<char>(std::toupper(c));
    expectReadError(withLine(good, 8, "# reference_sha256: " + upper),
                    {"reference_sha256", "64 lowercase hex"});
  }
}

TEST_CASE("command offset: read from a file names the file",
          "[learn][command_offset]") {
  TempPath file("named.csv");
  {
    std::ofstream output(file.path(), std::ios::binary);
    output << withLine(learn::formatCommandOffset(small()), 2, "# version: 9");
  }
  CHECK_THROWS_WITH(
      learn::readCommandOffset(file.path()),
      ContainsSubstring("command offset '" + file.path().string() +
                        "': line 2: unsupported version"));
  CHECK_THROWS_WITH(
      learn::readCommandOffset("build/command_offset_test/no_such_file.csv"),
      ContainsSubstring("cannot open"));
}

TEST_CASE("command offset: the file is created exclusively",
          "[learn][command_offset]") {
  const CommandOffset offset = small();
  TempPath file("exclusive.csv");
  learn::writeCommandOffset(file.path(), offset);
  const std::string written = slurp(file.path());
  CHECK(written == learn::formatCommandOffset(offset));

  // an existing file is refused and left untouched
  CHECK_THROWS_WITH(learn::writeCommandOffset(file.path(), learned()),
                    ContainsSubstring("refusing to overwrite"));
  CHECK(slurp(file.path()) == written);

  // so is a symlink, to an existing file or to nothing
  TempPath target("symlink_target.csv");
  {
    std::ofstream output(target.path(), std::ios::binary);
    output << "keep\n";
  }
  TempPath link("symlink.csv");
  fs::create_symlink(target.path().filename(), link.path());
  CHECK_THROWS_WITH(learn::writeCommandOffset(link.path(), offset),
                    ContainsSubstring("refusing to overwrite"));
  CHECK(slurp(target.path()) == "keep\n");
  TempPath nowhere("dangling_target.csv");
  TempPath dangling("dangling.csv");
  fs::create_symlink(nowhere.path().filename(), dangling.path());
  CHECK_THROWS_WITH(learn::writeCommandOffset(dangling.path(), offset),
                    ContainsSubstring("refusing to overwrite"));
  CHECK(absent(nowhere.path()));
  CHECK(fs::is_symlink(dangling.path()));

  // an invalid offset creates no file
  TempPath invalid("invalid.csv");
  CHECK_THROWS_WITH(
      learn::writeCommandOffset(
          invalid.path(),
          changed(offset, [](CommandOffset& o) { o.dt_s = 0.0; })),
      ContainsSubstring("dt_s"));
  CHECK(absent(invalid.path()));

  // a missing directory is not created
  CHECK_THROWS_WITH(
      learn::writeCommandOffset(
          "build/command_offset_test/no_such_dir/x.csv", offset),
      ContainsSubstring("cannot create"));
}

TEST_CASE("command offset: a failed write removes the partial file",
          "[learn][command_offset]") {
  const CommandOffset offset = learned();
  constexpr rlim_t kLimit = 256;
  REQUIRE(learn::formatCommandOffset(offset).size() > 4 * kLimit);
  TempPath file("partial.csv");
  std::string message;
  const bool limited = withFileSizeLimit(kLimit, [&] {
    try {
      learn::writeCommandOffset(file.path(), offset);
    } catch (const std::runtime_error& error) {
      message = error.what();
    }
  });
  REQUIRE(limited);
  CHECK_THAT(message, ContainsSubstring("cannot write"));
  CHECK(absent(file.path()));

  // the same write succeeds once the limit is lifted
  learn::writeCommandOffset(file.path(), offset);
  CHECK(slurp(file.path()) == learn::formatCommandOffset(offset));
}
