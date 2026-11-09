#pragma once
#include <cerrno>
#include <cstdlib>
#include <cstdint>

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
