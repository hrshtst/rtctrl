#include "rtctrl/learn/command_offset.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <istream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace rtctrl::learn {

namespace fs = std::filesystem;

namespace {

constexpr int kDof = model::kCanonicalDof;

// The preamble keys, in file order.
constexpr std::size_t kKeyCount = 10;
constexpr std::array<std::string_view, kKeyCount> kKeys = {
    "format",           "version",
    "method",           "trial",
    "dt_s",             "samples",
    "reference_duration_s",
    "reference_sha256", "parent_sha256",
    "source_log_sha256"};
// Indexes into kKeys.
enum KeyIndex : std::size_t {
  kFormat,
  kVersion,
  kMethod,
  kTrial,
  kDt,
  kSamples,
  kDuration,
  kReferenceHash,
  kParentHash,
  kSourceHash
};

// Times and spans in messages.
std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof buffer, "%.9g", value);
  return buffer;
}

// Values compared against a bound, in messages: the shortest text that
// reads back as the same double, so a value just past a bound never prints
// equal to it.
std::string exact(double value) {
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value);
  return std::string(buffer, result.ptr);
}

std::string shown(std::string_view text) {
  constexpr std::size_t kMaxShown = 80;
  if (text.size() <= kMaxShown) return "'" + std::string(text) + "'";
  return "'" + std::string(text.substr(0, kMaxShown)) + "...'";
}

std::string sampleContext(int joint, std::size_t sample, double dt_s) {
  return "joint " + std::to_string(joint) + ", sample " +
         std::to_string(sample) + " (t = " +
         number(static_cast<double>(sample) * dt_s) + " s)";
}

bool isSha256(const std::string& text) {
  return text.size() == 64 &&
         std::all_of(text.begin(), text.end(), [](char c) {
           return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
         });
}

bool isMethod(const std::string& method) {
  return method == "zero" || method == "ilc" || method == "fel";
}

std::string columnHeader() {
  std::string header = "time_s";
  for (int joint = 0; joint < kDof; ++joint) {
    header += ",delta" + std::to_string(joint) + "_rad";
  }
  return header;
}

// Empty when the offset is structurally sound, else its first problem.
std::string structureProblem(const CommandOffset& offset) {
  if (!isMethod(offset.method)) {
    return "method " + shown(offset.method) +
           " is not one of 'zero', 'ilc', 'fel'";
  }
  if (offset.trial < 0) {
    return "trial " + std::to_string(offset.trial) + " is negative";
  }
  if (!std::isfinite(offset.dt_s) || offset.dt_s <= 0.0) {
    return "dt_s " + number(offset.dt_s) + " must be finite and > 0";
  }
  if (!std::isfinite(offset.reference_duration_s) ||
      offset.reference_duration_s <= 0.0) {
    return "reference_duration_s " + number(offset.reference_duration_s) +
           " must be finite and > 0";
  }
  const std::size_t samples = offset.delta_rad.size();
  if (samples < 2) {
    return "at least 2 samples needed, got " + std::to_string(samples);
  }
  const double span = static_cast<double>(samples - 1) * offset.dt_s;
  const double gap = std::fabs(span - offset.reference_duration_s);
  if (!(gap <= kOffsetSpanToleranceS)) {
    return "(samples - 1) * dt_s = " + number(span) +
           " s differs from reference_duration_s = " +
           number(offset.reference_duration_s) + " s by " + number(gap) +
           " s (tolerance 1e-9 s)";
  }
  for (std::size_t i = 0; i < samples; ++i) {
    for (int joint = 0; joint < kDof; ++joint) {
      if (!std::isfinite(offset.delta_rad[i][joint])) {
        return "non-finite delta at " +
               sampleContext(joint, i, offset.dt_s) + ": " +
               number(offset.delta_rad[i][joint]);
      }
    }
  }
  if (!isSha256(offset.reference_sha256)) {
    return "reference_sha256 " + shown(offset.reference_sha256) +
           " is not 64 lowercase hex characters";
  }
  if (!offset.parent_sha256.empty() && !isSha256(offset.parent_sha256)) {
    return "parent_sha256 " + shown(offset.parent_sha256) +
           " is neither empty nor 64 lowercase hex characters";
  }
  if (!offset.source_log_sha256.empty() &&
      !isSha256(offset.source_log_sha256)) {
    return "source_log_sha256 " + shown(offset.source_log_sha256) +
           " is neither empty nor 64 lowercase hex characters";
  }
  if (offset.method == "zero") {
    if (offset.trial != 0) {
      return "method 'zero' requires trial 0, got trial " +
             std::to_string(offset.trial);
    }
    if (!offset.parent_sha256.empty()) {
      return "a zero offset (trial 0) has no parent_sha256";
    }
    if (!offset.source_log_sha256.empty()) {
      return "a zero offset (trial 0) has no source_log_sha256";
    }
    for (std::size_t i = 0; i < samples; ++i) {
      for (int joint = 0; joint < kDof; ++joint) {
        if (offset.delta_rad[i][joint] != 0.0) {
          return "a zero offset must be all zeros, but " +
                 sampleContext(joint, i, offset.dt_s) + " is " +
                 exact(offset.delta_rad[i][joint]);
        }
      }
    }
    return {};
  }
  if (offset.trial == 0) {
    return "trial 0 requires method 'zero', got " + shown(offset.method);
  }
  if (offset.parent_sha256.empty()) {
    return "trial " + std::to_string(offset.trial) +
           " requires a parent_sha256";
  }
  if (offset.source_log_sha256.empty()) {
    return "trial " + std::to_string(offset.trial) +
           " requires a source_log_sha256";
  }
  return {};
}

void requireLimit(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(std::string("command offset limits: ") + name +
                             " " + exact(value) + " must be finite and > 0");
  }
}

// Appends value printed with format (one double conversion); any length.
void appendNumber(std::string* text, const char* format, double value) {
  char buffer[64];
  const int length = std::snprintf(buffer, sizeof buffer, format, value);
  if (length < 0) {
    throw std::runtime_error("command offset: cannot format a number");
  }
  if (static_cast<std::size_t>(length) < sizeof buffer) {
    text->append(buffer, static_cast<std::size_t>(length));
    return;
  }
  std::string wide(static_cast<std::size_t>(length) + 1, '\0');
  std::snprintf(wide.data(), wide.size(), format, value);
  text->append(wide.data(), static_cast<std::size_t>(length));
}

void appendKey(std::string* text, std::string_view key,
               const std::string& value) {
  *text += "# ";
  *text += key;
  *text += ':';
  if (!value.empty()) *text += " " + value;
  *text += '\n';
}

std::string g17(double value) {
  std::string text;
  appendNumber(&text, "%.17g", value);
  return text;
}

// --- reading ----------------------------------------------------------

// Plain comma separation, no quoting.
void splitFields(std::string_view line, std::vector<std::string_view>* fields) {
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

// The whole field is one finite number: no whitespace, no empty text,
// locale independent.
bool parseFinite(std::string_view text, double* value) {
  if (text.empty()) return false;
  double parsed = 0.0;
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(text.data(), end, parsed);
  if (result.ec != std::errc() || result.ptr != end ||
      !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

template <typename Integer>
bool parseInteger(std::string_view text, Integer* value) {
  if (text.empty()) return false;
  Integer parsed = 0;
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(text.data(), end, parsed);
  if (result.ec != std::errc() || result.ptr != end) return false;
  *value = parsed;
  return true;
}

std::size_t keyIndex(std::string_view key) {
  for (std::size_t k = 0; k < kKeyCount; ++k) {
    if (kKeys[k] == key) return k;
  }
  return kKeyCount;
}

enum class PreambleLine { NotPreamble, Malformed, Ok };

// A preamble line is "# key:" (empty value) or "# key: value" with value
// non-empty and free of surrounding whitespace.
PreambleLine splitPreamble(std::string_view line, std::string_view* key,
                           std::string_view* value) {
  if (line.substr(0, 2) != "# ") return PreambleLine::NotPreamble;
  const std::size_t colon = line.find(':', 2);
  if (colon == std::string_view::npos) return PreambleLine::Malformed;
  *key = line.substr(2, colon - 2);
  const std::string_view rest = line.substr(colon + 1);
  if (rest.empty()) {
    *value = rest;
    return PreambleLine::Ok;
  }
  if (rest.size() < 2 || rest[0] != ' ') return PreambleLine::Malformed;
  *value = rest.substr(1);
  const auto blank = [](char c) { return c == ' ' || c == '\t'; };
  if (blank(value->front()) || blank(value->back())) {
    return PreambleLine::Malformed;
  }
  return PreambleLine::Ok;
}

class Reader {
 public:
  Reader(std::istream& input, const std::string& source)
      : input_(input), source_(source) {}

  // Next line into line(); false at end of input. A last line without its
  // newline is an error.
  bool next() {
    if (!std::getline(input_, line_)) {
      if (input_.bad()) throw std::runtime_error(source_ + ": read error");
      return false;
    }
    ++number_;
    if (input_.eof()) fail(number_, "missing final newline");
    return true;
  }

  const std::string& line() const { return line_; }
  std::size_t lineNumber() const { return number_; }

  [[noreturn]] void fail(std::size_t line_number,
                         const std::string& problem) const {
    throw std::runtime_error(source_ + ": line " +
                             std::to_string(line_number) + ": " + problem);
  }
  [[noreturn]] void fail(const std::string& problem) const {
    fail(number_, problem);
  }

  // A "# key..." line where key was not expected.
  [[noreturn]] void failKey(std::string_view key, std::size_t expected) const {
    const std::size_t index = keyIndex(key);
    if (index == kKeyCount) fail("unknown key " + shown(key));
    if (index < expected) fail("duplicate key " + shown(key));
    fail("missing key " + shown(kKeys[expected]) + " (found " + shown(key) +
         " in its place)");
  }

 private:
  std::istream& input_;
  const std::string& source_;
  std::string line_;
  std::size_t number_ = 0;
};

}  // namespace

TrackingGrid trackingGrid(double reference_duration_s, double control_dt_s) {
  if (!std::isfinite(reference_duration_s) || reference_duration_s <= 0.0) {
    throw std::runtime_error("tracking grid: reference duration " +
                             number(reference_duration_s) +
                             " s must be finite and > 0");
  }
  if (!std::isfinite(control_dt_s) || control_dt_s <= 0.0) {
    throw std::runtime_error("tracking grid: control period " +
                             number(control_dt_s) +
                             " s must be finite and > 0");
  }
  // The interval count must fit a long, or the conversion below is
  // undefined: the bound is LONG_MAX + 1 = 2^63 exactly, and the ceiling of
  // a double below 2^63 is below it too.
  if (!(reference_duration_s / control_dt_s <
        static_cast<double>(std::numeric_limits<long>::max()))) {
    throw std::runtime_error("tracking grid: reference duration " +
                             number(reference_duration_s) +
                             " s over control period " + number(control_dt_s) +
                             " s is too many intervals");
  }
  // FollowRun::trackReference (apps/follow/follow_run.hpp), expression for
  // expression: the follower commands samples 0..intervals at step * dt.
  // Keep the two identical, or an offset sample stops matching its command.
  const long intervals = std::max(
      1L, static_cast<long>(std::ceil(reference_duration_s / control_dt_s)));
  const double dt = reference_duration_s / intervals;
  return {intervals, dt};
}

CommandOffset zeroOffset(double reference_duration_s, double control_dt_s,
                         const std::string& reference_sha256) {
  const TrackingGrid grid = trackingGrid(reference_duration_s, control_dt_s);
  CommandOffset offset;
  offset.method = "zero";
  offset.trial = 0;
  offset.dt_s = grid.dt_s;
  offset.reference_duration_s = reference_duration_s;
  offset.reference_sha256 = reference_sha256;
  offset.delta_rad.assign(grid.samples(), JointArray{});
  checkOffsetStructure(offset);
  return offset;
}

void checkOffsetStructure(const CommandOffset& offset) {
  const std::string problem = structureProblem(offset);
  if (!problem.empty()) throw std::runtime_error("command offset: " + problem);
}

void checkOffsetForRun(const CommandOffset& offset, double reference_duration_s,
                       double control_dt_s, const std::string& reference_sha256,
                       const OffsetLimits& limits) {
  requireLimit(limits.max_abs_rad, "max_abs_rad");
  requireLimit(limits.max_rate_rad_s, "max_rate_rad_s");
  checkOffsetStructure(offset);
  const TrackingGrid grid = trackingGrid(reference_duration_s, control_dt_s);
  if (offset.delta_rad.size() != grid.samples()) {
    throw std::runtime_error(
        "command offset: " + std::to_string(offset.delta_rad.size()) +
        " samples, the tracking grid of a " + number(reference_duration_s) +
        " s reference at a " + number(control_dt_s) + " s control period has " +
        std::to_string(grid.samples()));
  }
  const double step_gap = std::fabs(offset.dt_s - grid.dt_s);
  if (!(step_gap <= kOffsetGridStepToleranceS)) {
    throw std::runtime_error("command offset: dt_s " + exact(offset.dt_s) +
                             " differs from the tracking grid step " +
                             exact(grid.dt_s) + " by " + number(step_gap) +
                             " s (tolerance 1e-12 s)");
  }
  if (offset.reference_sha256 != reference_sha256) {
    throw std::runtime_error("command offset: reference_sha256 " +
                             shown(offset.reference_sha256) +
                             " does not match the reference being run " +
                             shown(reference_sha256));
  }
  const auto& delta = offset.delta_rad;
  for (std::size_t i = 0; i < delta.size(); ++i) {
    for (int joint = 0; joint < kDof; ++joint) {
      if (std::fabs(delta[i][joint]) > limits.max_abs_rad) {
        throw std::runtime_error(
            "command offset: " + sampleContext(joint, i, offset.dt_s) +
            ": delta " + exact(delta[i][joint]) + " rad exceeds max_abs_rad " +
            exact(limits.max_abs_rad));
      }
    }
  }
  for (std::size_t i = 0; i + 1 < delta.size(); ++i) {
    for (int joint = 0; joint < kDof; ++joint) {
      const double rate =
          std::fabs(delta[i + 1][joint] - delta[i][joint]) / offset.dt_s;
      if (!(rate <= limits.max_rate_rad_s)) {
        throw std::runtime_error(
            "command offset: " + sampleContext(joint, i, offset.dt_s) +
            " to sample " + std::to_string(i + 1) + ": rate " + exact(rate) +
            " rad/s (delta " + exact(delta[i][joint]) + " to " +
            exact(delta[i + 1][joint]) + " rad) exceeds max_rate_rad_s " +
            exact(limits.max_rate_rad_s));
      }
    }
  }
}

std::string formatCommandOffset(const CommandOffset& offset) {
  checkOffsetStructure(offset);
  std::string text;
  appendKey(&text, kKeys[kFormat], kCommandOffsetFormat);
  appendKey(&text, kKeys[kVersion], std::to_string(kCommandOffsetVersion));
  appendKey(&text, kKeys[kMethod], offset.method);
  appendKey(&text, kKeys[kTrial], std::to_string(offset.trial));
  appendKey(&text, kKeys[kDt], g17(offset.dt_s));
  appendKey(&text, kKeys[kSamples], std::to_string(offset.delta_rad.size()));
  appendKey(&text, kKeys[kDuration], g17(offset.reference_duration_s));
  appendKey(&text, kKeys[kReferenceHash], offset.reference_sha256);
  appendKey(&text, kKeys[kParentHash], offset.parent_sha256);
  appendKey(&text, kKeys[kSourceHash], offset.source_log_sha256);
  text += columnHeader();
  text += '\n';
  for (std::size_t i = 0; i < offset.delta_rad.size(); ++i) {
    appendNumber(&text, "%.9f", static_cast<double>(i) * offset.dt_s);
    for (const double value : offset.delta_rad[i]) {
      text += ',';
      appendNumber(&text, "%.17g", value);
    }
    text += '\n';
  }
  return text;
}

void writeCommandOffset(const fs::path& path, const CommandOffset& offset) {
  const std::string text = formatCommandOffset(offset);
  std::FILE* file = std::fopen(path.string().c_str(), "wx");
  if (file == nullptr) {
    const int error = errno;
    if (error == EEXIST) {
      throw std::runtime_error("command offset: refusing to overwrite '" +
                               path.string() + "'");
    }
    throw std::runtime_error("command offset: cannot create '" +
                             path.string() + "': " + std::strerror(error));
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

  bool ok = std::fwrite(text.data(), 1, text.size(), file) == text.size();
  ok = ok && std::fflush(file) == 0 && std::ferror(file) == 0;
  const int closed = std::fclose(file);
  created.file = nullptr;
  if (!ok || closed != 0) {
    throw std::runtime_error("command offset: cannot write '" + path.string() +
                             "'");
  }
  created.keep = true;
}

CommandOffset parseCommandOffset(std::istream& input,
                                 const std::string& source) {
  Reader reader(input, source);
  CommandOffset offset;
  unsigned long long samples = 0;

  for (std::size_t k = 0; k < kKeyCount; ++k) {
    if (!reader.next()) {
      reader.fail(reader.lineNumber() + 1,
                  "end of file, expected key " + shown(kKeys[k]));
    }
    std::string_view key;
    std::string_view value;
    switch (splitPreamble(reader.line(), &key, &value)) {
      case PreambleLine::NotPreamble:
        reader.fail("missing key " + shown(kKeys[k]) + " (found " +
                    shown(reader.line()) + ")");
      case PreambleLine::Malformed:
        reader.fail("malformed preamble line " + shown(reader.line()) +
                    ", expected '# " + std::string(kKeys[k]) + ": <value>'");
      case PreambleLine::Ok:
        break;
    }
    if (key != kKeys[k]) reader.failKey(key, k);
    switch (k) {
      case kFormat:
        if (value != kCommandOffsetFormat) {
          reader.fail("wrong format " + shown(value) + ", expected '" +
                      kCommandOffsetFormat + "'");
        }
        break;
      case kVersion:
        if (value != std::to_string(kCommandOffsetVersion)) {
          reader.fail("unsupported version " + shown(value) + ", expected " +
                      std::to_string(kCommandOffsetVersion));
        }
        break;
      case kMethod:
        offset.method = std::string(value);
        break;
      case kTrial:
        if (!parseInteger(value, &offset.trial)) {
          reader.fail("trial: not an integer: " + shown(value));
        }
        break;
      case kDt:
        if (!parseFinite(value, &offset.dt_s)) {
          reader.fail("dt_s: not a finite number: " + shown(value));
        }
        break;
      case kSamples:
        if (!parseInteger(value, &samples)) {
          reader.fail("samples: not a non-negative integer: " + shown(value));
        }
        break;
      case kDuration:
        if (!parseFinite(value, &offset.reference_duration_s)) {
          reader.fail("reference_duration_s: not a finite number: " +
                      shown(value));
        }
        break;
      case kReferenceHash:
        offset.reference_sha256 = std::string(value);
        break;
      case kParentHash:
        offset.parent_sha256 = std::string(value);
        break;
      default:
        offset.source_log_sha256 = std::string(value);
        break;
    }
  }

  const std::string header = columnHeader();
  if (!reader.next()) {
    reader.fail(reader.lineNumber() + 1,
                "end of file, expected the column header");
  }
  {
    std::string_view key;
    std::string_view value;
    if (splitPreamble(reader.line(), &key, &value) == PreambleLine::Ok) {
      reader.failKey(key, kKeyCount);
    }
  }
  if (reader.line() != header) {
    reader.fail("wrong column header " + shown(reader.line()) +
                ", expected '" + header + "'");
  }

  std::vector<std::string_view> fields;
  while (reader.next()) {
    const std::size_t i = offset.delta_rad.size();
    if (i == samples) {
      reader.fail("more data rows than samples (" + std::to_string(samples) +
                  ")");
    }
    splitFields(reader.line(), &fields);
    if (fields.size() != static_cast<std::size_t>(kDof) + 1) {
      reader.fail(std::to_string(fields.size()) + " fields, expected " +
                  std::to_string(kDof + 1));
    }
    double time_s = 0.0;
    if (!parseFinite(fields[0], &time_s)) {
      reader.fail("column 'time_s': not a finite number: " +
                  shown(fields[0]));
    }
    JointArray row{};
    for (int joint = 0; joint < kDof; ++joint) {
      if (!parseFinite(fields[joint + 1], &row[joint])) {
        reader.fail("column 'delta" + std::to_string(joint) +
                    "_rad': not a finite number: " + shown(fields[joint + 1]));
      }
    }
    const double expected = static_cast<double>(i) * offset.dt_s;
    if (!(std::fabs(time_s - expected) <= kOffsetTimeToleranceS)) {
      reader.fail("time_s " + exact(time_s) + " of sample " +
                  std::to_string(i) + " differs from i * dt_s = " +
                  exact(expected) +
                  " by more than 1e-6 s");
    }
    offset.delta_rad.push_back(row);
  }
  if (offset.delta_rad.size() != samples) {
    reader.fail(reader.lineNumber() + 1,
                "end of file after " + std::to_string(offset.delta_rad.size()) +
                    " data rows, samples is " + std::to_string(samples));
  }

  const std::string problem = structureProblem(offset);
  if (!problem.empty()) throw std::runtime_error(source + ": " + problem);
  return offset;
}

CommandOffset readCommandOffset(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("command offset: cannot open '" + path.string() +
                             "'");
  }
  return parseCommandOffset(input, "command offset '" + path.string() + "'");
}

}  // namespace rtctrl::learn
