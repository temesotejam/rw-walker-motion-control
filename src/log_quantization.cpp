#include "log_quantization.h"
#include <cmath>

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("Os", "no-fast-math")
#endif
namespace log_quantization {
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((noinline, noclone))
#elif defined(__clang__)
__attribute__((noinline))
#endif
int16_t scaledI16(float value, float scale) {
  if (!std::isfinite(value)) return kMissing;
  value *= scale;
  if (value > 32767.0f) return 32767;
  if (value < -32767.0f) return -32767;
  int32_t integer = static_cast<int32_t>(value);
  const float fraction = value - static_cast<float>(integer);
  if (fraction >= 0.5f) ++integer;
  else if (fraction <= -0.5f) --integer;
  return static_cast<int16_t>(integer);
}
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
