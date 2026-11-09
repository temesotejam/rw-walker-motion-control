#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include "tilt_stop.h"
using mekf6::Quaternion;
Quaternion multiply(Quaternion a, Quaternion b) {
  return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z, a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
      a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x, a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
const mekf6::Vec3 upright{-0.021626f,0.033568f,0.999202f};
Quaternion reference;
Quaternion pose(float forward, float lateral, float yaw) {
  const float r=mekf6::degToRad(forward)*.5f, p=mekf6::degToRad(lateral)*.5f,
      y=mekf6::degToRad(yaw)*.5f;
  return multiply(multiply(multiply({cosf(y),0,0,sinf(y)},
      {cosf(p),0,sinf(p),0}),{cosf(r),sinf(r),0,0}),reference);
}
const char* sample(tilt_stop::ForeAftGuard& guard, float forward, float lateral, float yaw) {
  auto q=pose(forward,lateral,yaw);
  const char* result=guard.reason(q,true,2500,upright);
  q={-q.w,-q.x,-q.y,-q.z};
  assert((guard.reason(q,true,2500,upright)!=nullptr)==(result!=nullptr));
  return result;
}
int main() {
  mekf6::Mekf6 m; assert(m.initializeFromAccel(upright)); reference=m.quaternion();
  for (float yaw : {-170.f,0.f,135.f}) {
    // Sideways full turns in either direction, including +/-90 and +/-180,
    // must not STOP even with a nonzero (but <90) fore/aft lean.
    for (float forward : {-80.f,-20.f,0.f,20.f,80.f}) {
      for (float side : {-1.f,1.f}) {
        tilt_stop::ForeAftGuard guard;
        for (int step=0;step<=80;++step) assert(!sample(guard,forward*step/80,0,yaw));
        for (int step=0;step<=1440;++step)
          assert(!sample(guard,forward,side*step*.25f,yaw+step*.1f));
      }
    }
    // Forward and backward +/-90 edges, including simultaneous lateral lean.
    for (float lateral : {-60.f,0.f,60.f}) for (float sign : {-1.f,1.f}) {
      tilt_stop::ForeAftGuard guard;
      for (int step=0;step<=60;++step) assert(!sample(guard,0,lateral*step/60,yaw));
      for (int step=0;step<900;++step) assert(!sample(guard,sign*step*.1f,lateral,yaw));
      assert(!sample(guard,sign*89.99f,lateral,yaw));
      const char* reason=sample(guard,sign*90.f,lateral,yaw);
      assert(reason && !strcmp(reason,"fore_aft_tilt_90deg"));
      for (float angle : {90.01f,100.f,150.f,180.f}) assert(sample(guard,sign*angle,lateral,yaw));
      guard.reset(); assert(!sample(guard,0,0,yaw));
    }
    // Fore/aft STOP remains available after a sideways 90-degree crossing.
    for (float lateral : {-179.f,-100.f,-89.f,89.f,100.f,179.f}) for (float sign : {-1.f,1.f}) {
      tilt_stop::ForeAftGuard guard;
      for (int step=0;step<=180;++step) assert(!sample(guard,0,lateral*step/180,yaw));
      for (int step=0;step<=89;++step) assert(!sample(guard,sign*step,lateral,yaw));
      assert(sample(guard,sign*90.02f,lateral,yaw));
    }
  }
  tilt_stop::ForeAftGuard guard;
  // Exactly side-on has no fore/aft gravity projection. Hold the branch,
  // rather than promoting this singular posture to a sideways STOP.
  for (int step=0;step<=90;++step) assert(!sample(guard,0,step,0));
  for (int step=0;step<=80;++step) assert(!sample(guard,step,90,0));
  assert(!sample(guard,80,89,0));
  guard.reset();
  assert(!guard.reason(reference,true,10000,upright));
  assert(!strcmp(guard.reason(reference,true,10001,upright),"tilt_stale_mekf"));
  assert(guard.reason({NAN,0,0,0},true,0,upright));
  assert(guard.reason({0,0,0,0},true,0,upright));
  assert(guard.reason(reference,false,0,upright));
  std::cout << "fore/aft +/-90 STOP; sideways +/-360 excluded; mixed tilt, mechanical reference, yaw/sign, reset and stale/invalid attitude PASS\n";
}
