// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: MIT

#ifndef GHAND_HARDWARE_INTERFACE_COMMUNICATION_UTIL_HPP_
#define GHAND_HARDWARE_INTERFACE_COMMUNICATION_UTIL_HPP_

#include <cstddef>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ghand_comm {

/// Split a string by a delimiter, dropping empty tokens.
inline std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream stream(text);
  while (std::getline(stream, token, delimiter)) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  return tokens;
}

/// Parse an integer, returning default_value on any failure.
inline int parse_int_default(const std::string& text, int default_value) {
  try {
    return std::stoi(text);
  } catch (...) {
    return default_value;
  }
}

/// Parse a comma-separated list of tactile region counts into named regions.
/// Defaults to the canonical five-finger names {THUMB, FF, MF, RF, LF}.
inline std::vector<std::pair<std::string, size_t>> parse_tactile_regions(
    const std::string& region_counts) {
  std::vector<std::pair<std::string, size_t>> regions;
  if (region_counts.empty()) {
    return regions;
  }

  const auto counts = split(region_counts, ',');
  const std::vector<std::string> names = {"THUMB", "FF", "MF", "RF", "LF"};
  for (size_t i = 0; i < counts.size() && i < names.size(); ++i) {
    try {
      regions.emplace_back(names[i], std::stoul(counts[i]));
    } catch (...) {
      // malformed region count, skip
    }
  }
  return regions;
}

}  // namespace ghand_comm

#endif  // GHAND_HARDWARE_INTERFACE_COMMUNICATION_UTIL_HPP_
