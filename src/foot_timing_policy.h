#pragma once
#include <stdint.h>
namespace foot_timing_policy {
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
constexpr uint32_t kMinFrameIntervalMs=25;
#else
constexpr uint32_t kMinFrameIntervalMs=40;
#endif
}
