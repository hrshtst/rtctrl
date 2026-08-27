#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <limits>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "teach/teach_recording.hpp"

using Catch::Approx;
namespace fs = std::filesystem;
namespace model = rtctrl::model;
namespace teach = x7::teach;

TEST_CASE("teach recording resamples irregular measurements uniformly",
          "[teach]") {
  std::vector<teach::TimedPosition> input(4);
  input[0].time_s = 0.0;
  input[1].time_s = 0.07;
  input[2].time_s = 0.13;
  input[3].time_s = 0.21;
  for (auto& sample : input) sample.q[2] = 2.0 * sample.time_s;

  const auto output = teach::uniformResample(input, 10.0);
  REQUIRE(output.size() == 3);
  CHECK(output[0].time_s == Approx(0.0));
  CHECK(output[1].time_s == Approx(0.1));
  CHECK(output[2].time_s == Approx(0.2));
  CHECK(output[1].q[2] == Approx(0.2));
  CHECK(output[2].q[2] == Approx(0.4));
}

TEST_CASE("teach recording rejects malformed and undersampled input",
          "[teach]") {
  std::vector<teach::TimedPosition> one(1);
  CHECK_THROWS(teach::uniformResample(one, 100.0));
  std::vector<teach::TimedPosition> short_run(2);
  short_run[1].time_s = 0.005;
  CHECK_THROWS(teach::uniformResample(short_run, 100.0));
  short_run[1].time_s = 0.02;
  short_run[1].q[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS(teach::uniformResample(short_run, 100.0));
}

TEST_CASE("teach ZVS output is model-width and exclusive", "[teach]") {
  const fs::path path = "/tmp/rtctrl-teach-recording-test.zvs";
  fs::remove(path);
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  std::vector<teach::TimedPosition> samples(2);
  samples[1].time_s = 0.01;
  samples[1].q[7] = 0.2;
  {
    teach::ExclusiveZvsOutput output(path);
    CHECK(output.write(samples, 100.0, map) == 2);
  }
  model::ZvsTrajectory trajectory(path.string(), map);
  REQUIRE(trajectory.frames() == 2);
  CHECK(trajectory.frame(1)[7] == Approx(0.2));
  CHECK_THROWS(teach::ExclusiveZvsOutput(path));
  fs::remove(path);
}
