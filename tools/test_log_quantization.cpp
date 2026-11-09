#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include "../src/log_quantization.h"

// Frozen encoding from 0.47.9: compare the on-wire result, including missing
// values, saturation and rounding boundaries, rather than approximate angles.
static int16_t previous(float value, float scale) {
  if (!std::isfinite(value)) return -32768;
  value *= scale;
  if (value > 32767.0f) return 32767;
  if (value < -32767.0f) return -32767;
  return static_cast<int16_t>(std::lround(value));
}
int main() {
  uint64_t checked = 0;
  const auto check = [&](float value, float scale) {
    assert(log_quantization::scaledI16(value, scale) == previous(value, scale));
    ++checked;
  };
  const float inf = std::numeric_limits<float>::infinity();
  for (float scale : {1.0f, 100.0f, 1000.0f, 10000.0f}) {
    for (int n = -32770; n <= 32770; ++n) {
      const float half = (static_cast<float>(n) + 0.5f) / scale;
      check(half, scale);
      check(std::nextafter(half, -inf), scale);
      check(std::nextafter(half, inf), scale);
      check(static_cast<float>(n) / scale, scale);
    }
    for (float value : {0.0f, -0.0f, inf, -inf, std::numeric_limits<float>::quiet_NaN(),
         std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
         std::numeric_limits<float>::denorm_min(), -std::numeric_limits<float>::denorm_min()})
      check(value, scale);
  }
  uint32_t seed = 32653719;
  for (unsigned i = 0; i < 500000; ++i) {
    seed = seed * 1664525U + 1013904223U;
    float value; std::memcpy(&value, &seed, sizeof(value));
    for (float scale : {100.0f, 1000.0f, 10000.0f}) check(value, scale);
  }
  std::cout << "RWLOG int16 encoding equals 0.47.9 lroundf at all half-integer boundaries, adjacent floats, random bit patterns and nonfinite inputs: "
            << checked << " comparisons PASS\n";
}
