#pragma once
#include <cerrno>
#include <cstdlib>
#include <cstdint>
#include <cmath>

namespace autonomous_input_current {
constexpr int16_t kDefaultMa = 300;
constexpr int16_t kMinMa = 100;
constexpr int16_t kMaxMa = 1200;
inline bool valid(long value) { return value >= kMinMa && value <= kMaxMa && value % 10 == 0; }
inline bool parse(const char* text, int16_t& result) {
  if (!text || !*text) return false;
  char* end = nullptr; errno = 0;
  const long value = std::strtol(text, &end, 10);
  if (errno || end == text || *end || !valid(value)) return false;
  result = static_cast<int16_t>(value); return true;
}
} // namespace autonomous_input_current

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

namespace autonomous_brake_limit {
constexpr uint16_t kDefaultMs = 50;
inline bool valid(long value) { return value == 0 || value == 15 || value == 20 || value == 25 || value == 30 || value == 40 || value == 50; }
inline bool probe(long value) { return value == 0 || value == 15 || value == 30; }
inline bool batch(long value) { return value == 25; }
inline bool extendedBatch(long value) { return value == 40; }
inline bool positiveCurrentBatch(long value) { return value == 50; }
inline bool quadratic(long value) { return value == 0 || value == 20 || value == 50; }
inline bool parse(const char* text, uint16_t& result) {
  if (!text || !*text) return false;
  char* end = nullptr; errno = 0;
  const long value = std::strtol(text, &end, 10);
  if (errno || end == text || *end || !valid(value)) return false;
  result = static_cast<uint16_t>(value); return true;
}
} // namespace autonomous_brake_limit

namespace autonomous_target {
constexpr float kChoicesDeg[] = {8.0f, 10.0f, 12.0f};
inline bool selectable(float value) {
  for (float allowed : kChoicesDeg)
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
