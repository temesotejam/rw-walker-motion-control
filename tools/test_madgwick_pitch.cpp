#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include "fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_Madgwick.h"
// Arduino exposes this function-like macro even for namespaced functions.
#define degrees(rad) ((rad) * 57.29577951308232)
#include "../src/madgwick_pitch.h"

static uint32_t cases=0;
static bool same(float a,float b) {
  if(std::isnan(a) && std::isnan(b)) return true;
  return a==b && std::signbit(a)==std::signbit(b);
}
static void check(float w,float x,float y,float z) {
  // New objects keep upstream's lazy angle cache invalid. The unmodified
  // upstream implementation is the reference, including its nonunit behavior.
  Adafruit_Madgwick reference, candidate;
  reference.setQuaternion(w,x,y,z);candidate.setQuaternion(w,x,y,z);
  float before[4],after[4];
  candidate.getQuaternion(before,before+1,before+2,before+3);
  const float pitch=madgwick_pitch::readPitchDeg(candidate);
  assert(same(pitch,reference.getPitch()));
  candidate.getQuaternion(after,after+1,after+2,after+3);
  assert(std::memcmp(before,after,sizeof(before))==0);
  // Deferred standard getters must retain the same roll/yaw/gravity output.
  assert(same(candidate.getPitch(),reference.getPitch()));
  assert(same(candidate.getRoll(),reference.getRoll()));
  assert(same(candidate.getYaw(),reference.getYaw()));
  float ga[3],gb[3];
  candidate.getGravityVector(ga,ga+1,ga+2);reference.getGravityVector(gb,gb+1,gb+2);
  for(int i=0;i<3;++i)assert(same(ga[i],gb[i]));
  ++cases;
}
int main() {
  for(float pitch:{-90.0f,-89.999f,-45.0f,-0.0f,0.0f,45.0f,89.999f,90.0f}) {
    const float half=pitch*0.017453292519943295f*0.5f;
    for(float scale:{0.0f,0.5f,0.99999f,1.0f,1.00001f,2.0f,-1.0f})
      check(scale*std::cos(half),0,scale*std::sin(half),0);
  }
  uint32_t seed=33554009;
  const auto random=[&]() {seed=1664525U*seed+1013904223U;return float(seed>>8)/8388608.0f-1.0f;};
  for(unsigned i=0;i<20000;++i) {
    float w=random(),x=random(),y=random(),z=random();
    const float n=std::sqrt(w*w+x*x+y*y+z*z);
    check(w/n,x/n,y/n,z/n);
    if(!(i%10))check(w,x,y,z);
  }
  check(NAN,0,0,0);check(INFINITY,0,1,0);check(1,0,NAN,0);
  std::printf("Madgwick pitch matches unmodified Adafruit 2.4.0; signed zero, poles, nonunit, nonfinite, frozen quaternion and all-axis getters: %u cases PASS\n",cases);
}
