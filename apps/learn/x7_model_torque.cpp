// Offline model torques along a table of joint trajectories: for every
// row of IN.csv (columns qref{j}_rad, dqref{j}_rad_s, ddqref{j}_rad_s2 by
// name, e.g. an x7_follow log), the model gravity torque and the inverse
// dynamics torque, as a new CSV table. Never opens the bus.
// Usage: x7_model_torque --model ZTK --input IN.csv --output OUT.csv

#include <cstdio>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "learn/model_torque.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"

namespace fs = std::filesystem;
namespace learn = x7::learn;
namespace model = rtctrl::model;

namespace {

void usage() {
  std::printf(
      "usage: x7_model_torque --model ZTK --input IN.csv --output OUT.csv\n"
      "  evaluates the model gravity and inverse-dynamics torques at the\n"
      "  qref/dqref/ddqref columns of every IN.csv row; OUT.csv must not\n"
      "  exist\n");
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto cli = learn::parseCli(argc, argv);
    if (cli.help) {
      usage();
      return 0;
    }
    // Early refusal; the exclusive create below is the authority.
    if (fs::exists(fs::symlink_status(cli.output_path))) {
      throw std::runtime_error("output: refusing to overwrite '" +
                               cli.output_path.string() + "'");
    }
    const auto rows = learn::readTrajectoryCsv(cli.input_path);
    model::ChainModel chain(cli.model_path.string());
    model::JointMap map(chain);
    const auto torques = learn::evaluateModelTorque(chain, map, rows);
    learn::writeModelTorqueCsv(cli.output_path, torques);
    std::printf("model torque: %zu rows\n", torques.size());
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_model_torque: %s\n", error.what());
    return 1;
  }
}
