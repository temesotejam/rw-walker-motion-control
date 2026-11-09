#pragma once

// Placement only: numerical expressions and optimization policies stay intact.
// The target link audit checks these routines actually land in internal RAM.
// Calls into libm / other code and constant data can still access flash; this
// is not an assertion that the complete path is safe with caches disabled.
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_attr.h>
#define RW_HOT_CODE IRAM_ATTR
#else
#define RW_HOT_CODE
#endif

// Selected estimator, acquisition and motion/pulse routines use speed-oriented compilation.
// Keep IEEE finite/NaN handling and the original expression order. This does
// not enable fast-math, change the filter algorithm or suppress any samples.
// Enable on the host as well so differential tests exercise this policy.
#if defined(__GNUC__) && !defined(__clang__)
#define RW_SPEED_CODE RW_HOT_CODE __attribute__((optimize("O2", "no-fast-math")))
#else
#define RW_SPEED_CODE RW_HOT_CODE
#endif
