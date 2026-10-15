#include "rtctrl/arm/link_force_log.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/link_force.hpp"

using Catch::Matchers::ContainsSubstring;
using rtctrl::arm::ControlMode;
using rtctrl::arm::JointState;
using rtctrl::arm::LinkForceCsvLog;
using rtctrl::arm::LinkForceInterval;
using rtctrl::arm::LinkForceStatus;
using rtctrl::arm::SimArm;
using rtctrl::model::ForceDirection;
using rtctrl::model::LinkForceSpec;

namespace {

namespace fs = std::filesystem;

class TempPath {
 public:
  explicit TempPath(const std::string& name) {
    fs::create_directories("build/link_force_log_test");
    path_ = "build/link_force_log_test/" + name;
    std::remove(path_.c_str());
  }
  ~TempPath() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

using Row = std::map<std::string, std::string>;

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
      REQUIRE(i < header.size());
      row[header[i]] = value;
    }
    REQUIRE(i == header.size());
    rows.push_back(row);
  }
  return rows;
}

SimArm::Options options() {
  SimArm::Options opt;
  opt.model_path = "models/crane_x7/crane_x7.ztk";
  opt.initial_q8 = {0.3, -0.6, 0.2, -1.4, 0.25, -0.5, 0.4, 0.0};
  return opt;
}

LinkForceSpec holding(double magnitude) {
  LinkForceSpec spec;
  spec.link = "crane_x7_tcp_link";
  spec.direction_world = {0.0, 1.0, 0.0};
  spec.magnitude_n = magnitude;
  spec.profile.rise_s = 0.05;
  spec.profile.hold_s = 100.0;
  spec.profile.fall_s = 0.05;
  return spec;
}

}  // namespace

TEST_CASE("link-force log writes a schema-versioned row per interval",
          "[link_force][log]") {
  TempPath out("rows.csv");
  {
    LinkForceCsvLog log(out.path());
    LinkForceInterval iv;
    iv.attempt = 7;
    iv.start_seq = 3;
    iv.end_seq = 4;
    iv.start_s = 0.25;
    iv.end_s = 0.26;
    iv.substeps_completed = 100;
    iv.complete = true;
    iv.prepared = iv.started = iv.active = true;
    iv.last_eval_s = 0.2599;
    iv.last_substep_dt = 1e-4;
    iv.last_scale = 0.5;
    iv.last_force_world = {0.0, 1.5, 0.0};
    iv.last_point_world = {0.1, 0.2, 0.3};
    iv.last_joint_torque_nm[2] = -0.25;
    iv.force_impulse_ns = {0.0, 0.015, 0.0};
    iv.joint_impulse_nms[2] = -0.0025;
    CHECK(log.write(iv));
    CHECK(log.close());
    CHECK(log.close());  // idempotent
  }
  const auto rows = readRows(out.path());
  REQUIRE(rows.size() == 1);
  const Row& row = rows[0];
  CHECK(row.at("schema_version") == "1");
  CHECK(row.at("attempt") == "7");
  CHECK(row.at("start_seq") == "3");
  CHECK(row.at("end_seq") == "4");
  CHECK(row.at("start_s") == "0.250000000");
  CHECK(row.at("end_s") == "0.260000000");
  CHECK(row.at("complete") == "1");
  CHECK(row.at("status") == "ok");
  CHECK(row.at("rejected") == "0");
  CHECK(row.at("rejected_eval_s") == "nan");
  CHECK(row.at("active") == "1");
  CHECK(row.at("last_force_y_n") == "1.5");
  CHECK(row.at("last_point_z_m") == "0.3");
  CHECK(row.at("last_tau_ext2_nm") == "-0.25");
  CHECK(row.at("impulse_y_ns") == "0.015");
  CHECK(row.at("joint_impulse2_nms") == "-0.0025");
  CHECK(row.count("joint_impulse7_nms") == 1);

  // exclusive creation
  CHECK_THROWS_WITH(LinkForceCsvLog(out.path()),
                    ContainsSubstring("may already exist"));
}

TEST_CASE("SimArm logs every attempt, rejected and latched ones included",
          "[link_force][log]") {
  TempPath out("attempts.csv");
  {
    LinkForceCsvLog log(out.path());
    SimArm arm(options());
    REQUIRE(arm.activate());
    arm.linkForceLogTo(&log);
    REQUIRE(arm.step());  // plain step: logged, not prepared
    LinkForceSpec pinned = holding(3.0);
    pinned.link = "world";
    pinned.direction = ForceDirection::Anchor;
    pinned.anchor_world_m = {0.0, 0.0, 0.0};
    arm.setLinkForce(pinned);
    arm.startLinkForce(-0.5);
    CHECK_FALSE(arm.step());  // rejected on its first substep
    CHECK_FALSE(arm.step());  // latched
    CHECK(log.close());
  }
  const auto rows = readRows(out.path());
  REQUIRE(rows.size() == 3);
  CHECK(rows[0].at("prepared") == "0");
  CHECK(rows[0].at("complete") == "1");
  CHECK(rows[0].at("end_seq") == "1");
  CHECK(rows[1].at("status") == "anchor_fault");
  CHECK(rows[1].at("rejected") == "1");
  CHECK(rows[1].at("substeps_completed") == "0");
  CHECK(rows[1].at("start_seq") == "1");
  CHECK(rows[1].at("end_seq") == "1");
  CHECK(rows[2].at("status") == "fault_latched");
  CHECK(rows[2].at("attempt") == "2");
  CHECK(rows[2].at("start_s") == rows[2].at("end_s"));
  CHECK(rows[2].at("start_s") == rows[1].at("end_s"));
}

TEST_CASE("a failed log write fails the step without rejecting physics",
          "[link_force][log]") {
  // An injected stream that fails on flush: the exclusive path constructor
  // cannot open an existing device, so the stream is adopted.
  std::FILE* full = std::fopen("/dev/full", "w");
  REQUIRE(full != nullptr);
  LinkForceCsvLog log(full, LinkForceCsvLog::AdoptStream{});
  SimArm arm(options());
  REQUIRE(arm.activate());
  arm.setLinkForce(holding(4.0));
  arm.startLinkForce(-0.1);
  arm.linkForceLogTo(&log);

  CHECK_FALSE(arm.step());
  CHECK(arm.linkForceLogFailed());
  CHECK(log.failed());
  CHECK_FALSE(arm.linkForceFaulted());
  const LinkForceInterval first = arm.lastLinkForceInterval();
  CHECK(first.complete);
  CHECK_FALSE(first.rejected);
  CHECK(first.status == LinkForceStatus::Ok);
  CHECK(first.end_seq == first.start_seq + 1);
  CHECK(std::abs(first.force_impulse_ns[1]) > 0.0);

  JointState state;
  REQUIRE(arm.readState(state));  // feedback stays valid
  CHECK(state.seq == 1);
  const double before = arm.time();
  CHECK_FALSE(arm.step());  // physics advances, the step still fails
  CHECK(arm.time() > before);
  CHECK(arm.lastLinkForceInterval().complete);
  CHECK(arm.lastLinkForceInterval().end_seq == 2);
  CHECK_FALSE(log.close());
}
