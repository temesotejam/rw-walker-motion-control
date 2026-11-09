#pragma once
#include <cerrno>
#include <cstdlib>
#include <cstdint>
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
