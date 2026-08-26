#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/model/joint_map.hpp"

namespace x7 {

// Compatibility reader for archived identification .dwells.json sidecars.
// This deliberately is not the authored-posture path: it finds the generated
// "anchor" array and validates its eight finite numeric values.
inline bool loadLegacyAnchorSidecar(const std::string& path, double* out) {
  std::FILE* file = std::fopen(path.c_str(), "r");
  if (file == nullptr) return false;
  std::string text;
  char buffer[4096];
  std::size_t count = 0;
  while ((count = std::fread(buffer, 1, sizeof buffer, file)) > 0) {
    text.append(buffer, count);
  }
  std::fclose(file);

  const auto key = text.find("\"anchor\"");
  if (key == std::string::npos) return false;
  const auto colon = text.find(':', key + std::strlen("\"anchor\""));
  if (colon == std::string::npos) return false;
  const auto open = text.find('[', colon + 1);
  if (open == std::string::npos) return false;
  const auto close = text.find(']', open + 1);
  if (close == std::string::npos) return false;

  std::array<double, rtctrl::model::kCanonicalDof> values{};
  std::size_t position = open + 1;
  int found = 0;
  while (position < close) {
    const char character = text[position];
    if ((character >= '0' && character <= '9') ||
        ((character == '-' || character == '+') &&
         position + 1 < close && text[position + 1] >= '0' &&
         text[position + 1] <= '9')) {
      if (found == rtctrl::model::kCanonicalDof) return false;
      char* end = nullptr;
      const double value = std::strtod(text.c_str() + position, &end);
      if (end == text.c_str() + position) return false;
      if (!std::isfinite(value) || end > text.c_str() + close) return false;
      values[found++] = value;
      position = static_cast<std::size_t>(end - text.c_str());
    } else {
      ++position;
    }
  }
  if (found != rtctrl::model::kCanonicalDof) return false;
  std::copy(values.begin(), values.end(), out);
  return true;
}

}  // namespace x7
