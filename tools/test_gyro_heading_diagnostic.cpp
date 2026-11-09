#include "gyro_heading_diagnostic.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
  GyroHeadingDiagnostic h;
  assert(!h.start(0, 0, -1, 1000, false, true));
  assert(!h.valid());
  assert(!h.start(0, 0, -1, 1000, true, false));
  assert(!h.start(0, 0, 0, 1000, true, true));
  assert(h.start(0, 0, -1, 1000, true, true));
  // Raw Z is inverted by the sensor-to-body transform. Fixed raw bias is
  // applied before the transform. 90 degrees in 10 s at 400 Hz.
  for (unsigned i=1; i<=4000; ++i)
    h.sample(.2f, -.3f, -9.1f, .2f, -.3f, -.1f, .908911f, 1000+i*2500);
  assert(h.valid() && std::fabs(h.yaw()-90.f)<.02f);
  const float saved=h.yaw();
  h.sample(0,0,0,0,0,0,1,1000+4000*2500); // duplicate sample
  assert(h.valid() && h.yaw()==saved);
  h.sample(0,0,0,0,0,0,1,1000+4000*2500+10001);
  assert(!h.valid() && !std::isfinite(h.yaw()));
  h.sample(0,0,0,0,0,0,1,1000+4000*2500+12501);
  assert(!h.valid()); // gap invalidity is latched until new run
  assert(h.start(0,0,-1,1234,true,true));
  h.sample(NAN,0,0,0,0,0,1,3734);
  assert(!h.valid());
  std::puts("gyro heading diagnostic: frame, bias, invalid/gap latch, reset PASS");
}
