#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Transport-only adaptation for pinned M5Unified 0.2.18. No axis, scale,
// calibration, ODR, filter, or estimator changes. One reader task owns this.
namespace bmi270_timing {
// Installed only after boot validation and exclusive I2C1 handoff. A failed
// runtime read never falls back to the old controller driver.
using Transport = bool (*)(uint8_t, uint8_t*, size_t);
inline Transport& transport() { static Transport fn = nullptr; return fn; }
struct LastRead {
  uint32_t status_us = 0, data_us = 0;
  uint16_t data_bytes = 0;
  uint8_t ready = 0, returned = 0, failures = 0;
  bool called = false;
};
inline LastRead& lastRead() { static LastRead r; return r; }
inline int16_t signed16(const uint8_t* p) {
  const uint16_t u = uint16_t(p[0]) | (uint16_t(p[1]) << 8);
  int16_t v; memcpy(&v, &u, sizeof(v)); return v;
}
template<class Axis> inline void copyAxis(Axis& out, const uint8_t* p) {
  out.x = signed16(p); out.y = signed16(p + 2); out.z = signed16(p + 4);
}
template<class Raw, class Read, class Clock>
uint8_t readRaw(Raw* data, Read read, Clock clock) {
  LastRead& trace = lastRead(); trace = LastRead{}; trace.called = true;
  uint8_t status = 0;
  const uint32_t status_start = clock();
  const bool status_ok = read(0x03, &status, 1); // STATUS: unread-data flags.
  trace.status_us = uint32_t(clock() - status_start);
  if (!status_ok) { ++trace.failures; return 0; }
  trace.ready = status & 0xe0U;
  uint8_t result = 0;
  uint8_t buf[12] = {};
  const uint32_t data_start = clock();
  const uint8_t ag = status & 0xc0U;
  // Never touch another sensor's DATA registers when its ready flag was clear.
  // A sample arriving between STATUS and this transfer must remain unread.
  if (ag == 0xc0U) {
    trace.data_bytes += 12;
    if (read(0x0c, buf, 12)) {
      copyAxis(data->accel, buf); copyAxis(data->gyro, buf + 6); result |= 3;
    } else ++trace.failures;
  } else if (ag == 0x40U) {
    trace.data_bytes += 6;
    if (read(0x12, buf, 6)) { copyAxis(data->gyro, buf); result |= 2; }
    else ++trace.failures;
  } else if (ag == 0x80U) {
    trace.data_bytes += 6;
    if (read(0x0c, buf, 6)) { copyAxis(data->accel, buf); result |= 1; }
    else ++trace.failures;
  }
  // Preserve the library's auxiliary magnetometer behavior, but only read it
  // when ready. Its bytes are no longer fetched for every gyro-only sample.
  if (status & 0x20U) {
    trace.data_bytes += 8;
    if (read(0x04, buf, 8)) {
      data->mag.x = signed16(buf) >> 2;
      data->mag.y = signed16(buf + 2) >> 2;
      data->mag.z = signed16(buf + 4) & 0xfffe;
      result |= 4;
    } else ++trace.failures;
  }
  trace.data_us = uint32_t(clock() - data_start);
  trace.returned = result;
  return result;
}
}
