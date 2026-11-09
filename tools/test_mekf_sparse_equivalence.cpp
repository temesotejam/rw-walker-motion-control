#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <vector>

// Compare all posterior state, including cross covariance, against the frozen
// pre-optimization implementation. The reference is never linked into firmware.
#define private public
#include "../src/mekf6.hpp"
#include "fixtures/mekf6_dense_reference_0478.hpp"
#undef private

static uint64_t checked = 0;
static void same(float a, float b) {
  ++checked;
  if (a == b || (std::isnan(a) && std::isnan(b))) return;
  std::fprintf(stderr, "MEKF dense/sparse mismatch %.9g != %.9g at scalar %llu\n",
      a, b, static_cast<unsigned long long>(checked));
  std::abort();
}
static void same(const mekf6::Mekf6& a, const mekf6_dense_reference::Mekf6& b) {
  auto qa = a.quaternion(); auto qb = b.quaternion();
  same(qa.w,qb.w); same(qa.x,qb.x); same(qa.y,qb.y); same(qa.z,qb.z);
  auto ba = a.gyroBiasRadS(); auto bb = b.gyroBiasRadS();
  same(ba.x,bb.x); same(ba.y,bb.y); same(ba.z,bb.z);
  for (int r=0;r<6;++r) for (int c=0;c<6;++c) same(a.P_[r][c],b.P_[r][c]);
  auto da=a.diagnostics(); auto db=b.diagnostics();
  same(da.accel_norm_g,db.accel_norm_g);
  same(da.accel_magnitude_error_g,db.accel_magnitude_error_g);
  same(da.accel_direction_residual_deg,db.accel_direction_residual_deg);
  same(da.accel_confidence,db.accel_confidence);
  assert(da.accel_used==db.accel_used);
  auto ea=a.eulerDeg(); auto eb=b.eulerDeg();
  same(ea.roll,eb.roll); same(ea.pitch,eb.pitch); same(ea.yaw,eb.yaw);
  same(mekf6::Mekf6::pitchDegFromQuaternion(qa),eb.pitch);
}
struct Input { float gx,gy,gz,ax,ay,az,dt; bool accel; };
static uint32_t seed=45028066;
static float noise() {
  seed=1664525U*seed+1013904223U;
  return static_cast<float>(seed>>8)/8388608.0f-1.0f;
}

static void copiedQuaternionConversions() {
  mekf6_dense_reference::Mekf6 reference;
  uint32_t cases = 0;
  const auto check = [&](const mekf6::Quaternion& q) {
    reference.q_ = {q.w, q.x, q.y, q.z};
    const auto expected = reference.eulerDeg();
    const auto display = mekf6::Mekf6::eulerDegFromQuaternion(q);
    same(display.roll, expected.roll);
    same(display.pitch, expected.pitch);
    same(display.yaw, expected.yaw);
    same(mekf6::Mekf6::pitchDegFromQuaternion(q), expected.pitch);
    ++cases;
  };
  // Non-unit and sign-reversed snapshots must keep the previous normalization
  // and clamp behavior, including zero/tiny norm and pitch near +/-90 degrees.
  for(float pitch : {-90.0f,-89.999f,-30.0f,0.0f,30.0f,89.999f,90.0f}) {
    for(float scale : {0.0f,1.0e-9f,0.5f,1.0f,1.01f,5.0f,-1.0f}) {
      const float half=mekf6::degToRad(pitch)*0.5f;
      check({scale*std::cos(half),0,scale*std::sin(half),0});
    }
  }
  for(unsigned i=0;i<4096;++i) check({noise(),noise(),noise(),noise()});
  std::printf("Copied-quaternion control pitch and all-axis display: %u normalization/pole/random cases exactly match frozen reference PASS\n",cases);
}

template<class Filter>
static double benchmark(const std::vector<Input>& inputs) {
  Filter f; assert(f.initializeFromAccel({0,0,1}));
  const auto start=std::chrono::steady_clock::now();
  for (const auto& in:inputs) {
    f.predict({in.gx,in.gy,in.gz},in.dt);
    if(in.accel) f.updateAccel({in.ax,in.ay,in.az});
  }
  const auto end=std::chrono::steady_clock::now();
  assert(std::isfinite(f.quaternion().w));
  return std::chrono::duration<double,std::micro>(end-start).count()/inputs.size();
}

int main() {
  uint32_t predictions=0, corrections=0, rejected=0;
  std::vector<Input> benchmark_inputs;
  for (unsigned scenario=0;scenario<4;++scenario) {
    mekf6::Mekf6 a; mekf6_dense_reference::Mekf6 b;
    const float roll0=scenario==3?2.8f:0.13f*scenario;
    const float pitch0=scenario==2?1.3f:-0.08f*scenario;
    float ax=-std::sin(pitch0),ay=std::sin(roll0)*std::cos(pitch0),az=std::cos(roll0)*std::cos(pitch0);
    assert(a.initializeFromAccel({ax,ay,az})==b.initializeFromAccel({ax,ay,az}));
    a.setGyroBiasRadS({0.001f,-0.002f,0.0003f});
    b.setGyroBiasRadS({0.001f,-0.002f,0.0003f});
    same(a,b);
    float time=0;
    for (unsigned i=0;i<12000;++i) {
      const float dt=0.0025f+0.0005f*noise(); time+=dt;
      const float roll=roll0+0.14f*std::sin(7*time),pitch=pitch0+0.08f*std::sin(4*time);
      const float rd=0.98f*std::cos(7*time),pd=0.32f*std::cos(4*time),yd=0.25f;
      Input in{rd-yd*std::sin(pitch)+0.001f,
          pd*std::cos(roll)+yd*std::sin(roll)*std::cos(pitch)-0.002f,
          -pd*std::sin(roll)+yd*std::cos(roll)*std::cos(pitch)+0.0003f,
          -std::sin(pitch),std::sin(roll)*std::cos(pitch),std::cos(roll)*std::cos(pitch),dt,!(i%2)};
      const float scale=1+0.04f*noise();
      in.ax=in.ax*scale+0.004f*noise();
      in.ay=in.ay*scale+0.004f*noise();
      in.az=in.az*scale+0.004f*noise();
      if(i%151==0) {in.ax+=0.8f;in.az+=0.4f;} // rejected translational acceleration
      if(i%251==0) in.ax=in.ay=in.az=0; // invalid norm
      if(i%601==0) in.ax=NAN; // sensor invalid input rejection
      if(i%701==0) in.dt=0.06f; // timing rejection
      if(i%997==0) in.dt=NAN;
      const bool pa=a.predict({in.gx,in.gy,in.gz},in.dt);
      assert(pa==b.predict({in.gx,in.gy,in.gz},in.dt)); ++predictions;
      same(a,b);
      if(in.accel) {
        const bool ca=a.updateAccel({in.ax,in.ay,in.az});
        assert(ca==b.updateAccel({in.ax,in.ay,in.az}));
        ++corrections; if(!ca) ++rejected;
        same(a,b);
      }
      if(scenario==0) benchmark_inputs.push_back(in);
    }
  }
  // Dense cross covariance is essential: a diagonal-only simplification must
  // fail this test even if a short upright-angle replay happens to agree.
  for (unsigned trial=0;trial<128;++trial) {
    mekf6::Mekf6 a; mekf6_dense_reference::Mekf6 b;
    float matrix[6][6];
    for(auto& row:matrix) for(auto& v:row) v=0.03f*noise();
    for(int r=0;r<6;++r) for(int c=0;c<6;++c) {
      float p=0;for(int k=0;k<6;++k) p+=matrix[r][k]*matrix[c][k];
      if(r==c) p+=0.0001f;
      a.P_[r][c]=b.P_[r][c]=p;
    }
    const float gx=noise()*4,gy=noise()*4,gz=noise()*4;
    assert(a.predict({gx,gy,gz},0.0025f)==b.predict({gx,gy,gz},0.0025f)); same(a,b);
    assert(a.updateAccel({0.02f,-0.03f,0.999f})==b.updateAccel({0.02f,-0.03f,0.999f})); same(a,b);
  }
  copiedQuaternionConversions();
  std::printf("MEKF dense/sparse exact finite-state equality: %u predictions, %u accel attempts (%u rejected), 128 full covariance cases, %llu scalar comparisons PASS\n",
      predictions,corrections,rejected,static_cast<unsigned long long>(checked));
  const double dense=benchmark<mekf6_dense_reference::Mekf6>(benchmark_inputs);
  const double sparse=benchmark<mekf6::Mekf6>(benchmark_inputs);
  std::printf("Informational host-only time/update: dense %.3f us, sparse %.3f us; ESP32 timing unverified\n",dense,sparse);
}
