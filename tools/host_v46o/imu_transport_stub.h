#pragma once
#include "../../src/imu_i2c_transport.h"
namespace imu_i2c {
inline bool begin_ok = true;
inline bool begin(int p, int sda, int scl, uint8_t addr, uint32_t hz) {
  return begin_ok && p == 1 && sda == 45 && scl == 0 && addr == 0x68 && hz == 1000000;
}
inline bool read(uint8_t, uint8_t*, size_t) { return false; }
inline bool ready() { return begin_ok; }
inline int lastError() { return 0; }
}
