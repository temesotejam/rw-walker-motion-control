#pragma once
#include <cstdint>
#include "realtime_code.h"

namespace log_quantization {
constexpr int16_t kMissing = -32768;

// RWLOG's existing saturated int16 encoding, including ties away from zero.
// The bounded conversion uses the FPU's truncation/conversion instructions
// instead of a libm lroundf call for every logged field. Adding +/-0.5 first
// is NOT equivalent immediately below a half-integer, so compare the exact
// fractional remainder after truncation instead.
// One non-inline definition also keeps the Xtensa literal pool with its IRAM
// body; a header COMDAT IRAM function can leave literals in a flash section.
int16_t RW_HOT_CODE scaledI16(float value, float scale);
}  // namespace log_quantization
