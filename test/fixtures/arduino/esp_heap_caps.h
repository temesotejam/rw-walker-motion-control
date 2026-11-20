#pragma once
#include <cstdlib>
constexpr int MALLOC_CAP_SPIRAM=1, MALLOC_CAP_8BIT=2;
inline bool host_psram_failure=false;
inline void* heap_caps_malloc(size_t n, int caps) {
  if (caps != (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)) std::abort();
  return host_psram_failure ? nullptr : std::malloc(n);
}
