#pragma once
#include <cerrno>
#include <cstdlib>
#include "config.h"

namespace autonomous_target {
inline bool selectable(float value) {
  for (float allowed : Config::ENERGY_CONTROL_AUTONOMOUS_TARGET_CHOICES_DEG)
    if (value == allowed) return true;
  return false;
}

// Validate the entire HTTP argument before accepting a START or pausing HTTP.
// On failure, leave the caller's previous value untouched.
inline bool parse(const char* text, float& target_deg) {
  if (!text || !*text) return false;
  char* end = nullptr;
  errno = 0;
  const float value = std::strtof(text, &end);
  if (errno || end == text || *end || !selectable(value)) return false;
  target_deg = value;
  return true;
}
} // namespace autonomous_target
