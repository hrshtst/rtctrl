// FollowRun on SimArm with a link-force disturbance: the partial-join
// contract between the run's main records and the per-attempt interval
// log, and the rejected-execution path through the shared run engine.
#include "common/link_force_run.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "follow/follow_config.hpp"
#include "follow/follow_run.hpp"
#include "rtctrl/arm/link_force_log.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/link_force.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace arm = rtctrl::arm;
namespace follow = x7::follow;
namespace model = rtctrl::model;

namespace {

namespace fs = std::filesystem;

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
const std::vector<double> kStart = {0.3, -0.6, 0.2, -1.4, 0.25, -0.5, 0.4, 0.0};

using Row = std::map<std::string, std::string>;

struct MainRecord {
  follow::Phase phase;
  std::uint64_t seq;
};

struct Recorder : follow::CycleSink {
  bool record(const follow::CycleRecord& cycle) override {
    records.push_back({cycle.phase, cycle.state->seq});
    return true;
  }
  std::vector<MainRecord> records;
};

class Scratch {
 public:
  Scratch() {
    fs::create_directories(dir_);
    std::ofstream out(reference());
    // 0.6 s moving joint 0 from the start pose
    for (int k = 0; k < 3; ++k) {
      out << "0.2 8 (";
      for (int i = 0; i < 8; ++i) {
        out << ' ' << (kStart[i] + (i == 0 ? 0.025 * k : 0.0));
      }
      out << " )\n";
    }
  }
  ~Scratch() { fs::remove_all(dir_); }
  std::string reference() const { return dir_ + "/reference.zvs"; }
  std::string log() const { return dir_ + "/disturbance.csv"; }

 private:
  std::string dir_ = "build/link_force_run_test";
};

std::vector<Row> readRows(const std::string& path) {
  std::ifstream input(path);
  std::string line;
  REQUIRE(std::getline(input, line));
  std::vector<std::string> header;
  std::stringstream fields(line);
  for (std::string field; std::getline(fields, field, ',');) {
    header.push_back(field);
  }
  std::vector<Row> rows;
  while (std::getline(input, line)) {
    std::stringstream values(line);
    Row row;
    std::size_t i = 0;
    for (std::string value; std::getline(values, value, ','); ++i) {
      row[header.at(i)] = value;
    }
    rows.push_back(row);
  }
  return rows;
}

std::uint64_t num(const Row& row, const char* key) {
  return std::stoull(row.at(key));
}

follow::Config runConfig() {
  follow::Config config;
  config.mode = arm::ControlMode::Position;
  config.home.motion_time = 0.05;
  config.home.velocity_limit = 2.0;
  config.home.strict = false;
  config.home.tolerance_rad = 0.05;
  config.home.settle_time_s = 0.02;
  config.finalization.simulation_hold_time_s = 0.2;
  return config;
}

model::LinkForceSpec pulse() {
  model::LinkForceSpec spec;
  spec.link = "crane_x7_tcp_link";
  spec.direction_world = {0.0, 1.0, 0.0};
  spec.magnitude_n = 3.0;
  spec.profile.onset_s = 0.1;
  spec.profile.rise_s = 0.05;
  spec.profile.hold_s = 0.1;
  spec.profile.fall_s = 0.05;
  return spec;
}

struct Outcome {
  follow::RunResult result;
  std::vector<MainRecord> records;
  std::vector<Row> rows;
  x7::LinkForceRunReport report;
};

Outcome runWith(const model::LinkForceSpec& spec) {
  Scratch scratch;
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZvsTrajectory reference(scratch.reference(), map);
  const auto config = runConfig();
  arm::SimArm::Options options;
  options.model_path = kModelPath;
  options.initial_q8 = kStart;
  Outcome outcome;
  {
    arm::LinkForceCsvLog log(scratch.log());
    arm::SimArm robot(options);
    REQUIRE(robot.setMode(config.mode));
    REQUIRE(robot.activate());
    Recorder recorder;
    follow::FollowRun run(robot, reference, config, true, &recorder);
    x7::armLinkForce(robot, spec, &log, run, &outcome.report);
    outcome.result = run.run();
    x7::finishLinkForce(robot, log, &outcome.report);
    outcome.records = recorder.records;
  }
  outcome.rows = readRows(scratch.log());
  return outcome;
}

}  // namespace

TEST_CASE("interval rows join main records by seq on a successful run",
          "[link_force][join]") {
  const auto out = runWith(pulse());
  REQUIRE(out.result.status == follow::RunStatus::Success);
  CHECK(out.report.outcome.started);
  CHECK(out.report.outcome.applied);
  CHECK(out.report.log_ok);
  const auto& rows = out.rows;
  const auto& records = out.records;
  // follow records every cycle before stepping it: one row per record
  REQUIRE(rows.size() == records.size());
  for (std::size_t k = 0; k < rows.size(); ++k) {
    CHECK(num(rows[k], "attempt") == k);
    CHECK(rows[k].at("complete") == "1");
  }
  auto mainRowsWithSeq = [&](std::uint64_t seq) {
    return std::count_if(records.begin(), records.end(),
                         [seq](const MainRecord& r) { return r.seq == seq; });
  };
  // the initial main row has no preceding interval
  const std::uint64_t first_seq = records.front().seq;
  CHECK(std::none_of(rows.begin(), rows.end(), [&](const Row& row) {
    return num(row, "end_seq") == first_seq;
  }));
  // every complete row but the terminal one joins exactly one main row
  for (std::size_t k = 0; k + 1 < rows.size(); ++k) {
    CHECK(mainRowsWithSeq(num(rows[k], "end_seq")) == 1);
  }
  CHECK(mainRowsWithSeq(num(rows.back(), "end_seq")) == 0);  // terminal
  // and every main row is followed by exactly one interval
  for (const auto& record : records) {
    CHECK(std::count_if(rows.begin(), rows.end(), [&](const Row& row) {
            return num(row, "start_seq") == record.seq;
          }) == 1);
  }

  const auto tracking = std::find_if(
      records.begin(), records.end(),
      [](const MainRecord& r) { return r.phase == follow::Phase::Tracking; });
  REQUIRE(tracking != records.end());
  const auto before = std::find_if(rows.begin(), rows.end(), [&](const Row& r) {
    return num(r, "end_seq") == tracking->seq;
  });
  const auto after = std::find_if(rows.begin(), rows.end(), [&](const Row& r) {
    return num(r, "start_seq") == tracking->seq;
  });
  REQUIRE(before != rows.end());
  REQUIRE(after != rows.end());
  CHECK(before->at("started") == "0");  // ends at the first tracking state
  CHECK(before->at("impulse_y_ns") == "0");
  CHECK(after->at("started") == "1");  // the first that can carry force
  for (auto it = rows.begin(); it != after; ++it) {
    CHECK(it->at("active") == "0");
  }
  CHECK(std::any_of(rows.begin(), rows.end(),
                    [](const Row& r) { return r.at("active") == "1"; }));
}

TEST_CASE("a rejected substep ends the run with no later main rows",
          "[link_force][join]") {
  // a world-fixed point with the anchor on it faults at onset, during
  // tracking
  auto spec = pulse();
  spec.link = "world";
  spec.direction = model::ForceDirection::Anchor;
  spec.direction_world = {};
  spec.anchor_world_m = {0.0, 0.0, 0.0};
  const auto out = runWith(spec);
  CHECK(out.result.status != follow::RunStatus::Success);
  CHECK(out.report.faulted);
  CHECK(out.report.outcome.started);
  CHECK_FALSE(out.report.outcome.applied);
  const auto rejected =
      std::find_if(out.rows.begin(), out.rows.end(),
                   [](const Row& r) { return r.at("rejected") == "1"; });
  REQUIRE(rejected != out.rows.end());
  CHECK(rejected->at("status") == "anchor_fault");
  CHECK(rejected->at("complete") == "0");
  CHECK(rejected->at("impulse_y_ns") == "0");
  const std::uint64_t seq = num(*rejected, "start_seq");
  CHECK(num(*rejected, "end_seq") == seq);
  // no main row after the rejected attempt; later rows only latched
  for (const auto& record : out.records) CHECK(record.seq <= seq);
  CHECK(out.records.back().seq == seq);
  for (auto it = std::next(rejected); it != out.rows.end(); ++it) {
    CHECK(it->at("status") == "fault_latched");
    CHECK(num(*it, "start_seq") == seq);
    CHECK(num(*it, "end_seq") == seq);
  }
}
