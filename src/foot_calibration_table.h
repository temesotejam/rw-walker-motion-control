#pragma once
#include <stddef.h>
namespace foot_calibration {
struct Point { float x; float angle; };
static constexpr Point right[] = {
  {67.5192000f, 20.6265010f},
  {111.0505000f, 14.1099580f},
  {152.5749000f, 8.3712810f},
  {182.4695000f, 3.6884650f},
  {203.0819000f, 0.0000000f},
};
static constexpr Point left[] = {
  {63.6918000f, 20.6265010f},
  {102.7030000f, 14.1099580f},
  {144.3562000f, 8.3712810f},
  {173.8169000f, 3.6884650f},
  {201.6569000f, 0.0000000f},
};
static constexpr size_t count = 5;
// Interpolate inside measured support; use the nearest end segment outside.
inline float angleAt(const Point* p, float x, float* gain = nullptr) {
  size_t i = 0;
  while (i + 2 < count && x > p[i+1].x) ++i;
  const float slope = (p[i+1].angle-p[i].angle)/(p[i+1].x-p[i].x);
  if (gain) *gain = -slope;
  return p[i].angle + slope*(x-p[i].x);
}
} // namespace foot_calibration
