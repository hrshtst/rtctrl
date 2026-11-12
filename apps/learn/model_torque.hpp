// Model torques along a table of joint trajectories (x7_model_torque):
// the bridge that hands the repo's rigid-body model to offline analysis
// tools. It selects the reference columns of a CSV BY NAME (the names of
// the x7_follow log, FollowCsvLog::writeHeader), evaluates
// ChainModel::gravityTorque and ChainModel::inverseDynamics per row, and
// writes the result as a CSV table. Pure logic: never opens the bus.
#pragma once

#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <istream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7::learn {

namespace fs = std::filesystem;
namespace model = rtctrl::model;

using JointArray = std::array<double, model::kCanonicalDof>;

inline constexpr int kModelTorqueSchemaVersion = 1;
// qref, dqref, ddqref for each canonical joint.
inline constexpr int kRequiredColumnCount = 3 * model::kCanonicalDof;

// One input row: the reference state in canonical order (rad, rad/s,
// rad/s^2).
struct TrajectoryRow {
  JointArray q{};
  JointArray dq{};
  JointArray ddq{};
};

// The model torques of one row in canonical order (Nm).
struct ModelTorqueRow {
  JointArray gravity_nm{};
  JointArray inverse_dynamics_nm{};
};

struct Cli {
  fs::path model_path;
  fs::path input_path;
  fs::path output_path;
  bool help = false;
};

inline Cli parseCli(int argc, char* argv[]) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      cli.help = true;
    } else if (arg == "--model" || arg == "--input" || arg == "--output") {
      if (i + 1 >= argc || argv[i + 1][0] == '-') {
        throw std::runtime_error(arg + " requires a value");
      }
      const fs::path value = argv[++i];
      if (arg == "--model") {
        cli.model_path = value;
      } else if (arg == "--input") {
        cli.input_path = value;
      } else {
        cli.output_path = value;
      }
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (cli.help) return cli;
  if (cli.model_path.empty()) throw std::runtime_error("--model is required");
  if (cli.input_path.empty()) throw std::runtime_error("--input is required");
  if (cli.output_path.empty()) {
    throw std::runtime_error("--output is required");
  }
  return cli;
}

// Required column k: qref{j}_rad for k = j, dqref{j}_rad_s for k = 8 + j,
// ddqref{j}_rad_s2 for k = 16 + j (j canonical).
inline std::string requiredColumnName(int k) {
  const std::string j = std::to_string(k % model::kCanonicalDof);
  switch (k / model::kCanonicalDof) {
    case 0:
      return "qref" + j + "_rad";
    case 1:
      return "dqref" + j + "_rad_s";
    default:
      return "ddqref" + j + "_rad_s2";
  }
}

namespace detail {

inline double& requiredValue(TrajectoryRow* row, int k) {
  const int j = k % model::kCanonicalDof;
  switch (k / model::kCanonicalDof) {
    case 0:
      return row->q[j];
    case 1:
      return row->dq[j];
    default:
      return row->ddq[j];
  }
}

// Plain comma separation: no quoting (the x7_follow log has none; a quoted
// comma changes the field count and so fails the row).
inline void splitFields(std::string_view line,
                        std::vector<std::string_view>* fields) {
  fields->clear();
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string_view::npos) {
      fields->push_back(line.substr(start));
      return;
    }
    fields->push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
}

// Tolerates CRLF line ends (e.g. Python's csv module default).
inline void stripCarriageReturn(std::string* line) {
  if (!line->empty() && line->back() == '\r') line->pop_back();
}

// The whole field is one finite number: no whitespace, no sign-only or
// empty text, locale independent.
inline bool parseFinite(std::string_view text, double* value) {
  if (text.empty()) return false;
  double parsed = 0.0;
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(text.data(), end, parsed);
  if (result.ec != std::errc() || result.ptr != end || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

inline std::string quotedField(std::string_view text) {
  constexpr std::size_t kMaxShown = 64;
  if (text.size() <= kMaxShown) return "'" + std::string(text) + "'";
  return "'" + std::string(text.substr(0, kMaxShown)) + "...'";
}

inline std::string rowContext(std::size_t data_row) {
  // data rows are 1-based; the header is line 1
  return "input data row " + std::to_string(data_row) + " (line " +
         std::to_string(data_row + 1) + ")";
}

}  // namespace detail

// Parses a CSV whose header names all kRequiredColumnCount reference
// columns (any order; other columns, numeric or not, are ignored). Every
// data row needs the header's field count and finite numbers in the
// required columns. Throws std::runtime_error naming the 1-based data row
// and the column; a header without data rows is an error.
inline std::vector<TrajectoryRow> parseTrajectoryCsv(std::istream& input) {
  std::string line;
  if (!std::getline(input, line)) {
    if (input.bad()) throw std::runtime_error("input: read error");
    throw std::runtime_error("input: empty (no header line)");
  }
  detail::stripCarriageReturn(&line);
  std::vector<std::string_view> fields;
  detail::splitFields(line, &fields);
  const std::size_t width = fields.size();

  std::array<std::string, kRequiredColumnCount> names;
  std::array<std::size_t, kRequiredColumnCount> column{};
  for (int k = 0; k < kRequiredColumnCount; ++k) {
    names[k] = requiredColumnName(k);
    column[k] = std::string_view::npos;
  }
  for (std::size_t c = 0; c < width; ++c) {
    for (int k = 0; k < kRequiredColumnCount; ++k) {
      if (fields[c] != names[k]) continue;
      if (column[k] != std::string_view::npos) {
        throw std::runtime_error("input: duplicate column '" + names[k] + "'");
      }
      column[k] = c;
    }
  }
  std::string missing;
  for (int k = 0; k < kRequiredColumnCount; ++k) {
    if (column[k] != std::string_view::npos) continue;
    missing += (missing.empty() ? "" : ", ") + names[k];
  }
  if (!missing.empty()) {
    throw std::runtime_error("input: missing required column(s): " + missing);
  }

  std::vector<TrajectoryRow> rows;
  std::size_t data_row = 0;
  while (std::getline(input, line)) {
    ++data_row;
    detail::stripCarriageReturn(&line);
    detail::splitFields(line, &fields);
    if (fields.size() != width) {
      throw std::runtime_error(
          detail::rowContext(data_row) + ": " + std::to_string(fields.size()) +
          " fields, the header has " + std::to_string(width));
    }
    TrajectoryRow row;
    for (int k = 0; k < kRequiredColumnCount; ++k) {
      const std::string_view field = fields[column[k]];
      if (!detail::parseFinite(field, &detail::requiredValue(&row, k))) {
        throw std::runtime_error(
            detail::rowContext(data_row) + ": column '" + names[k] +
            "': not a finite number: " + detail::quotedField(field));
      }
    }
    rows.push_back(row);
  }
  if (input.bad()) throw std::runtime_error("input: read error");
  if (rows.empty()) throw std::runtime_error("input: no data rows");
  return rows;
}

inline std::vector<TrajectoryRow> readTrajectoryCsv(const fs::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("input: cannot open '" + path.string() + "'");
  }
  return parseTrajectoryCsv(input);
}

// Per-row evaluation with reused model-sized scratch vectors.
class ModelTorqueEvaluator {
 public:
  ModelTorqueEvaluator(model::ChainModel& chain, const model::JointMap& map)
      : chain_(chain),
        map_(map),
        q_(model::kCanonicalDof),
        dq_(model::kCanonicalDof),
        ddq_(model::kCanonicalDof),
        tau_(model::kCanonicalDof) {}

  ModelTorqueRow evaluate(const TrajectoryRow& row) {
    for (int j = 0; j < model::kCanonicalDof; ++j) {
      q_[j] = row.q[j];
      dq_[j] = row.dq[j];
      ddq_[j] = row.ddq[j];
    }
    ModelTorqueRow result;
    chain_.gravityTorque(map_, q_, tau_);
    for (int j = 0; j < model::kCanonicalDof; ++j) {
      result.gravity_nm[j] = tau_[j];
    }
    chain_.inverseDynamics(map_, q_, dq_, ddq_, tau_);
    for (int j = 0; j < model::kCanonicalDof; ++j) {
      result.inverse_dynamics_nm[j] = tau_[j];
    }
    return result;
  }

 private:
  model::ChainModel& chain_;
  const model::JointMap& map_;
  model::ZVector q_;
  model::ZVector dq_;
  model::ZVector ddq_;
  model::ZVector tau_;
};

// Evaluates every row; a non-finite torque (finite but huge inputs) is an
// error naming the 1-based data row, never a value in the table.
inline std::vector<ModelTorqueRow> evaluateModelTorque(
    model::ChainModel& chain, const model::JointMap& map,
    const std::vector<TrajectoryRow>& rows) {
  ModelTorqueEvaluator evaluator(chain, map);
  std::vector<ModelTorqueRow> result;
  result.reserve(rows.size());
  for (std::size_t r = 0; r < rows.size(); ++r) {
    const ModelTorqueRow torque = evaluator.evaluate(rows[r]);
    for (int j = 0; j < model::kCanonicalDof; ++j) {
      if (!std::isfinite(torque.gravity_nm[j]) ||
          !std::isfinite(torque.inverse_dynamics_nm[j])) {
        throw std::runtime_error(detail::rowContext(r + 1) +
                                 ": non-finite model torque at joint " +
                                 std::to_string(j));
      }
    }
    result.push_back(torque);
  }
  return result;
}

inline std::string modelTorqueHeader() {
  std::string header = "schema_version,row";
  for (int j = 0; j < model::kCanonicalDof; ++j) {
    header += ",gravity" + std::to_string(j) + "_nm";
  }
  for (int j = 0; j < model::kCanonicalDof; ++j) {
    header += ",inverse_dynamics" + std::to_string(j) + "_nm";
  }
  return header;
}

// One output line without its newline; row is the 0-based input row.
inline std::string modelTorqueCsvRow(std::size_t row,
                                     const ModelTorqueRow& torque) {
  std::string line =
      std::to_string(kModelTorqueSchemaVersion) + "," + std::to_string(row);
  char buffer[32];
  for (double value : torque.gravity_nm) {
    std::snprintf(buffer, sizeof buffer, ",%.9g", value);
    line += buffer;
  }
  for (double value : torque.inverse_dynamics_nm) {
    std::snprintf(buffer, sizeof buffer, ",%.9g", value);
    line += buffer;
  }
  return line;
}

// Creates path exclusively (an existing file, or symlink, is refused and
// left untouched) and writes the table; on any failure after creation the
// partial file is removed before the error propagates.
inline void writeModelTorqueCsv(const fs::path& path,
                                const std::vector<ModelTorqueRow>& rows) {
  std::FILE* file = std::fopen(path.string().c_str(), "wx");
  if (file == nullptr) {
    const int error = errno;
    if (error == EEXIST) {
      throw std::runtime_error("output: refusing to overwrite '" +
                               path.string() + "'");
    }
    throw std::runtime_error("output: cannot create '" + path.string() +
                             "': " + std::strerror(error));
  }
  struct Created {
    std::FILE* file;
    const fs::path& path;
    bool keep = false;
    ~Created() {
      if (file != nullptr) std::fclose(file);
      if (!keep) {
        std::error_code ignored;
        fs::remove(path, ignored);
      }
    }
  } created{file, path};

  bool ok = std::fprintf(file, "%s\n", modelTorqueHeader().c_str()) >= 0;
  for (std::size_t r = 0; ok && r < rows.size(); ++r) {
    ok = std::fprintf(file, "%s\n", modelTorqueCsvRow(r, rows[r]).c_str()) >= 0;
  }
  ok = ok && std::fflush(file) == 0 && std::ferror(file) == 0;
  const int closed = std::fclose(file);
  created.file = nullptr;
  if (!ok || closed != 0) {
    throw std::runtime_error("output: cannot write '" + path.string() + "'");
  }
  created.keep = true;
}

}  // namespace x7::learn
