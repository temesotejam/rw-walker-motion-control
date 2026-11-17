#pragma once
#include <stdint.h>
namespace foot_timing_policy {
#if defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
// Sensor frames arrive near 20ms; allow timestamp quantization/jitter while
// predictors still reject duplicates, reverse time, stale data and jumps.
constexpr uint32_t kMinFrameIntervalMs=15;
#elif defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
constexpr uint32_t kMinFrameIntervalMs=25;
#else
constexpr uint32_t kMinFrameIntervalMs=40;
#endif
}
