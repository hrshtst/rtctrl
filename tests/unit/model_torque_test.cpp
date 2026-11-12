#include "learn/model_torque.hpp"

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::EndsWith;
using Catch::Matchers::StartsWith;
using rtctrl::model::ChainModel;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;
using rtctrl::model::ZVector;
using x7::learn::ModelTorqueRow;
using x7::learn::TrajectoryRow;

namespace {

namespace fs = std::filesystem;

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

// A posture that loads the arm against gravity, and the same posture with
// every rate and acceleration nonzero.
TrajectoryRow restRow() {
  TrajectoryRow row;
  row.q = {0.3, -0.6, 0.2, -1.1, 0.4, 0.7, -0.5, 0.2};
  return row;
}

TrajectoryRow movingRow() {
  TrajectoryRow row = restRow();
  for (int j = 0; j < kCanonicalDof; ++j) {
    row.dq[j] = 0.4 - 0.09 * j;
    row.ddq[j] = -0.7 + 0.15 * j;
  }
  return row;
}

std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof buffer, "%.17g", value);
  return buffer;
}

// One CSV column: its header name and its text in each data row.
struct Column {
  std::string name;
  std::vector<std::string> cells;
};

// The 24 required columns in canonical order for the given rows.
std::vector<Column> requiredColumns(const std::vector<TrajectoryRow>& rows) {
  std::vector<Column> columns;
  for (int k = 0; k < x7::learn::kRequiredColumnCount; ++k) {
    Column column{x7::learn::requiredColumnName(k), {}};
    const int j = k % kCanonicalDof;
    for (const auto& row : rows) {
      const auto& values = k < kCanonicalDof       ? row.q
                           : k < 2 * kCanonicalDof ? row.dq
                                                   : row.ddq;
      column.cells.push_back(number(values[j]));
    }
    columns.push_back(column);
  }
  return columns;
}

std::string csvText(const std::vector<Column>& columns) {
  std::string text;
  for (std::size_t c = 0; c < columns.size(); ++c) {
    text += (c == 0 ? "" : ",") + columns[c].name;
  }
  text += "\n";
  const std::size_t rows = columns.empty() ? 0 : columns[0].cells.size();
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 0; c < columns.size(); ++c) {
      text += (c == 0 ? "" : ",") + columns[c].cells[r];
    }
    text += "\n";
  }
  return text;
}

std::vector<TrajectoryRow> parse(const std::string& text) {
  std::istringstream input(text);
  return x7::learn::parseTrajectoryCsv(input);
}

void expectParseError(const std::string& text,
                      const std::vector<std::string>& phrases) {
  std::istringstream input(text);
  try {
    x7::learn::parseTrajectoryCsv(input);
    FAIL("parse succeeded, expected an error");
  } catch (const std::runtime_error& error) {
    for (const auto& phrase : phrases) {
      CHECK_THAT(error.what(), ContainsSubstring(phrase));
    }
  }
}

Column& columnNamed(std::vector<Column>& columns, const std::string& name) {
  for (auto& column : columns) {
    if (column.name == name) return column;
  }
  FAIL("no column " << name);
  return columns.front();
}

void checkSameRows(const std::vector<TrajectoryRow>& a,
                   const std::vector<TrajectoryRow>& b) {
  REQUIRE(a.size() == b.size());
  for (std::size_t r = 0; r < a.size(); ++r) {
    CHECK(a[r].q == b[r].q);
    CHECK(a[r].dq == b[r].dq);
    CHECK(a[r].ddq == b[r].ddq);
  }
}

x7::learn::Cli parseArgs(std::vector<std::string> args) {
  std::vector<char*> argv;
  args.insert(args.begin(), "x7_model_torque");
  for (auto& arg : args) argv.push_back(arg.data());
  return x7::learn::parseCli(static_cast<int>(argv.size()), argv.data());
}

}  // namespace

TEST_CASE("model torque: zero rate and acceleration give the gravity torque",
          "[learn][model_torque]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const auto rows = parse(csvText(requiredColumns({restRow()})));
  REQUIRE(rows.size() == 1);
  const auto torques = x7::learn::evaluateModelTorque(chain, map, rows);
  REQUIRE(torques.size() == 1);

  ZVector q(kCanonicalDof), tau(kCanonicalDof);
  for (int j = 0; j < kCanonicalDof; ++j) q[j] = restRow().q[j];
  chain.gravityTorque(map, q, tau);
  double largest = 0.0;
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("canonical joint " << j);
    CHECK(torques[0].gravity_nm[j] == tau[j]);
    CHECK(torques[0].inverse_dynamics_nm[j] == torques[0].gravity_nm[j]);
    largest = std::max(largest, std::fabs(tau[j]));
  }
  CHECK(largest > 0.1);  // the posture loads the arm
}

TEST_CASE("model torque: a moving row equals a direct inverse dynamics call",
          "[learn][model_torque]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const TrajectoryRow moving = movingRow();
  const auto rows = parse(csvText(requiredColumns({restRow(), moving})));
  REQUIRE(rows.size() == 2);
  const auto torques = x7::learn::evaluateModelTorque(chain, map, rows);
  REQUIRE(torques.size() == 2);

  ZVector q(kCanonicalDof), dq(kCanonicalDof), ddq(kCanonicalDof);
  ZVector gravity(kCanonicalDof), tau(kCanonicalDof);
  for (int j = 0; j < kCanonicalDof; ++j) {
    q[j] = moving.q[j];
    dq[j] = moving.dq[j];
    ddq[j] = moving.ddq[j];
  }
  chain.inverseDynamics(map, q, dq, ddq, tau);
  chain.gravityTorque(map, q, gravity);
  double dynamic = 0.0;
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("canonical joint " << j);
    CHECK(torques[1].inverse_dynamics_nm[j] == tau[j]);
    CHECK(torques[1].gravity_nm[j] == gravity[j]);
    // same posture, so the same gravity as the resting row
    CHECK(torques[1].gravity_nm[j] == torques[0].gravity_nm[j]);
    dynamic = std::max(dynamic, std::fabs(tau[j] - gravity[j]));
  }
  CHECK(dynamic > 1e-3);  // the rates and accelerations matter
}

TEST_CASE("model torque: extra columns and column order are free",
          "[learn][model_torque]") {
  const std::vector<TrajectoryRow> reference = {restRow(), movingRow()};
  const auto canonical = parse(csvText(requiredColumns(reference)));
  checkSameRows(canonical, reference);

  // the x7_follow log layout: text and NaN columns around the references
  auto required = requiredColumns(reference);
  std::vector<Column> follow = {{"schema_version", {"1", "1"}},
                                {"time_s", {"0.000000000", "0.010000000"}},
                                {"phase", {"tracking", "tracking"}}};
  for (int j = 0; j < kCanonicalDof; ++j) {
    const std::string i = std::to_string(j);
    for (int kind = 0; kind < 3; ++kind) {
      follow.push_back(required[kind * kCanonicalDof + j]);
    }
    follow.push_back({"q" + i + "_rad", {"nan", "inf"}});
    follow.push_back({"flags" + i, {"0", "not a number"}});
  }
  checkSameRows(parse(csvText(follow)), reference);

  // reversed, with the ignored columns interleaved
  std::vector<Column> reversed(follow.rbegin(), follow.rend());
  checkSameRows(parse(csvText(reversed)), reference);

  // CRLF line ends parse the same
  std::string crlf;
  for (char c : csvText(reversed)) {
    if (c == '\n') crlf += '\r';
    crlf += c;
  }
  checkSameRows(parse(crlf), reference);

  ChainModel chain(kModelPath);
  JointMap map(chain);
  const auto a = x7::learn::evaluateModelTorque(chain, map, canonical);
  const auto b =
      x7::learn::evaluateModelTorque(chain, map, parse(csvText(reversed)));
  REQUIRE(a.size() == b.size());
  for (std::size_t r = 0; r < a.size(); ++r) {
    CHECK(a[r].gravity_nm == b[r].gravity_nm);
    CHECK(a[r].inverse_dynamics_nm == b[r].inverse_dynamics_nm);
  }
}

TEST_CASE("model torque: a missing required column is named",
          "[learn][model_torque]") {
  auto columns = requiredColumns({restRow()});
  for (auto it = columns.begin(); it != columns.end(); ++it) {
    if (it->name == "dqref4_rad_s") {
      columns.erase(it);
      break;
    }
  }
  expectParseError(csvText(columns),
                   {"missing required column", "dqref4_rad_s"});
  // a near-miss name does not count
  auto renamed = requiredColumns({restRow()});
  columnNamed(renamed, "ddqref7_rad_s2").name = "ddqref7_rad_s";
  expectParseError(csvText(renamed), {"ddqref7_rad_s2"});
  // a duplicated required column is ambiguous
  auto doubled = requiredColumns({restRow()});
  doubled.push_back(columnNamed(doubled, "qref0_rad"));
  expectParseError(csvText(doubled), {"duplicate column", "qref0_rad"});
}

TEST_CASE("model torque: a bad row fails naming the row",
          "[learn][model_torque]") {
  const std::vector<TrajectoryRow> rows = {restRow(), movingRow(), restRow()};
  const std::string good = csvText(requiredColumns(rows));
  const std::string header = good.substr(0, good.find('\n') + 1);
  std::vector<std::string> line;
  std::istringstream lines(good);
  for (std::string text; std::getline(lines, text);) line.push_back(text);
  REQUIRE(line.size() == 4);

  SECTION("a short row") {
    const std::string short_row = line[2].substr(0, line[2].rfind(','));
    expectParseError(header + line[1] + "\n" + short_row + "\n" + line[3],
                     {"data row 2", "23 fields", "has 24"});
  }
  SECTION("a long row") {
    expectParseError(header + line[1] + ",0\n",
                     {"data row 1", "25 fields", "has 24"});
  }
  SECTION("an empty line") {
    expectParseError(header + line[1] + "\n\n" + line[2] + "\n",
                     {"data row 2", "1 fields"});
  }
  for (const std::string bad :
       {"nan", "inf", "-inf", "", "abc", "1.5x", " 1", "1e999"}) {
    DYNAMIC_SECTION("a non-finite value '" << bad << "'") {
      auto columns = requiredColumns(rows);
      columnNamed(columns, "ddqref5_rad_s2").cells[2] = bad;
      expectParseError(
          csvText(columns),
          {"data row 3", "(line 4)", "ddqref5_rad_s2", "not a finite number"});
    }
  }
}

TEST_CASE("model torque: an input without data rows fails",
          "[learn][model_torque]") {
  const std::string good = csvText(requiredColumns({restRow()}));
  expectParseError(good.substr(0, good.find('\n') + 1), {"no data rows"});
  expectParseError(good.substr(0, good.find('\n')), {"no data rows"});
  expectParseError("", {"no header line"});
}

TEST_CASE("model torque: a non-finite model torque fails naming the row",
          "[learn][model_torque]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  TrajectoryRow huge = restRow();
  huge.dq.fill(1e200);  // finite input, overflowing velocity products
  try {
    x7::learn::evaluateModelTorque(chain, map, {restRow(), huge});
    FAIL("evaluation succeeded, expected an error");
  } catch (const std::runtime_error& error) {
    CHECK_THAT(error.what(), ContainsSubstring("data row 2"));
    CHECK_THAT(error.what(), ContainsSubstring("non-finite model torque"));
  }
}

TEST_CASE("model torque: the output table format", "[learn][model_torque]") {
  const std::string header = x7::learn::modelTorqueHeader();
  CHECK_THAT(header, StartsWith("schema_version,row,gravity0_nm,gravity1_nm,"));
  CHECK_THAT(header, ContainsSubstring(",gravity7_nm,inverse_dynamics0_nm,"));
  CHECK_THAT(header, EndsWith(",inverse_dynamics6_nm,inverse_dynamics7_nm"));

  ModelTorqueRow torque;
  for (int j = 0; j < kCanonicalDof; ++j) {
    torque.gravity_nm[j] = 1.0 / 3.0 + j;
    torque.inverse_dynamics_nm[j] = -2.5e-12 * (j + 1);
  }
  const std::string line = x7::learn::modelTorqueCsvRow(7, torque);
  CHECK_THAT(line, StartsWith("1,7,0.333333333,1.33333333,2.33333333,"));
  CHECK_THAT(line, ContainsSubstring(",7.33333333,-2.5e-12,-5e-12,"));
  CHECK_THAT(line, EndsWith(",-2e-11"));
  std::size_t commas = 0;
  for (char c : header) commas += c == ',';
  CHECK(commas == 2 * kCanonicalDof + 1);
  for (char c : line) commas -= c == ',';
  CHECK(commas == 0);
}

TEST_CASE("model torque: the output is created exclusively",
          "[learn][model_torque]") {
  const fs::path dir = "build/model_torque_test";
  fs::create_directories(dir);
  const fs::path path = dir / "exclusive.csv";
  fs::remove(path);
  ModelTorqueRow torque;
  torque.gravity_nm.fill(0.5);
  torque.inverse_dynamics_nm.fill(-0.25);
  x7::learn::writeModelTorqueCsv(path, {torque, torque});

  const auto read = [&path] {
    std::ifstream input(path);
    return std::string(std::istreambuf_iterator<char>(input), {});
  };
  const std::string written = read();
  CHECK(written == x7::learn::modelTorqueHeader() + "\n" +
                       x7::learn::modelTorqueCsvRow(0, torque) + "\n" +
                       x7::learn::modelTorqueCsvRow(1, torque) + "\n");

  CHECK_THROWS_WITH(x7::learn::writeModelTorqueCsv(path, {torque}),
                    ContainsSubstring("refusing to overwrite"));
  CHECK(read() == written);  // left untouched

  CHECK_THROWS_WITH(
      x7::learn::writeModelTorqueCsv(dir / "no_such_dir" / "x.csv", {torque}),
      ContainsSubstring("cannot create"));
  fs::remove(path);
}

TEST_CASE("model torque: the command line", "[learn][model_torque]") {
  const auto cli = parseArgs(
      {"--model", "m.ztk", "--input", "in.csv", "--output", "out.csv"});
  CHECK(cli.model_path == fs::path("m.ztk"));
  CHECK(cli.input_path == fs::path("in.csv"));
  CHECK(cli.output_path == fs::path("out.csv"));
  CHECK_FALSE(cli.help);
  CHECK(parseArgs({"--help"}).help);
  CHECK_THROWS_WITH(parseArgs({"--input", "a", "--output", "b"}),
                    ContainsSubstring("--model is required"));
  CHECK_THROWS_WITH(parseArgs({"--model", "m", "--output", "b"}),
                    ContainsSubstring("--input is required"));
  CHECK_THROWS_WITH(parseArgs({"--model", "m", "--input", "a"}),
                    ContainsSubstring("--output is required"));
  CHECK_THROWS_WITH(parseArgs({"--model", "m", "--input", "a", "--output", "b",
                               "--port", "x"}),
                    ContainsSubstring("unknown option: --port"));
  CHECK_THROWS_WITH(parseArgs({"--model", "--input", "a", "--output", "b"}),
                    ContainsSubstring("--model requires a value"));
  CHECK_THROWS_WITH(parseArgs({"--model", "m", "--input", "a", "--output"}),
                    ContainsSubstring("--output requires a value"));
}
