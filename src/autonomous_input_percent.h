#pragma once
#include <cmath>
#include <cerrno>
#include <cstdlib>
namespace autonomous_input_percent {
constexpr float kDefaultPercent = 0.0f;
constexpr float kMaxPercent = 0.0f;
inline bool valid(float value) { return std::isfinite(value) && value == 0.0f; }
inline bool parse(const char* text, float& result) {
  if (!text || !*text) return false;
  char* end = nullptr; errno = 0;
  const float value = std::strtof(text, &end);
  if (errno || end == text || *end || !valid(value)) return false;
  result = value; return true;
}
} // namespace autonomous_input_percent
