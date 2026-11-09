#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include "../src/mekf_attitude_diagnostics.h"
#include "../src/foot_range_diagnostics.h"
#include "../src/foot_calibration_diagnostics.h"
static void pureForeAftReplay() {
  mekf6::Mekf6 filter;
  const float pitch=mekf6::degToRad(.544745f), start=1.881578f;
  const auto gravity=[&](float roll_deg) {
    const float roll=mekf6::degToRad(roll_deg);
    return mekf6::Vec3{-std::sin(pitch),std::cos(pitch)*std::sin(roll),std::cos(pitch)*std::cos(roll)};
  };
  assert(filter.initializeFromAccel(gravity(start)));
  float roll=start;
  // Fixed-axis out/hold/return/hold, with accelerometer corrections enabled.
  for(int step=0;step<1600;++step) {
    const float rate=step<400 ? -10.0f : step<800 ? 0.0f : step<1200 ? 10.0f : 0.0f;
    roll+=rate*.005f;
    assert(filter.predict({mekf6::degToRad(rate),0,0},.005f));
    filter.updateAccel(gravity(roll));
    const auto e=filter.eulerDeg();
    assert(std::abs(e.roll-roll)<.02f);
    assert(std::abs(e.pitch-mekf6::radToDeg(pitch))<.02f);
    assert(std::abs(e.yaw)<.02f);
  }
  std::cout<<"Pure fore/aft out/hold/return with accel correction: no spurious yaw >0.02 deg PASS\n";
}
int main(int argc,char** argv) {
  assert(argc==2);
  pureForeAftReplay();
  mekf6::Mekf6 filter;
  std::ofstream out(argv[1]);
  out<<"{\"invalid\":"<<mekfAttitudeJson(captureMekfAttitude(filter,false,0),1000,true).c_str();
  out<<",\"axes\":[";
  for(int axis=0;axis<3;++axis){
    filter.reset();assert(filter.initializeFromAccel({0,0,1}));
    const float rate=mekf6::degToRad(20);
    mekf6::Vec3 gyro{axis==0?rate:0,axis==1?rate:0,axis==2?rate:0};
    for(int i=0;i<200;++i)assert(filter.predict(gyro,.005f));
    const auto before=filter.quaternion();
    const auto s=captureMekfAttitude(filter,true,1000000);
    const auto after=filter.quaternion();
    assert(s.valid && s.sample_us==1000000);
    const auto display=mekf6::Mekf6::eulerDegFromQuaternion(s.quaternion);
    assert(std::abs(display.roll-(axis==0?20:0))<.001);
    assert(std::abs(s.pitch_deg-(axis==1?20:0))<.001);
    assert(display.pitch==s.pitch_deg);
    assert(std::abs(display.yaw-(axis==2?20:0))<.001);
    assert(before.w==after.w&&before.x==after.x&&before.y==after.y&&before.z==after.z);
    assert(std::abs(s.quaternion.w-std::cos(mekf6::degToRad(10)))<.00001);
    const std::string frozen_json=mekfAttitudeJson(s,1002500,true).c_str();
    // Changing the live filter must not change deferred all-axis output from
    // this snapshot, including when it is attached to a frozen camera image.
    for(int i=0;i<100;++i)assert(filter.predict(gyro,.005f));
    assert(frozen_json==mekfAttitudeJson(s,1002500,true).c_str());
    assert(std::abs(filter.eulerDeg().roll-display.roll)+
           std::abs(filter.eulerDeg().pitch-display.pitch)+
           std::abs(filter.eulerDeg().yaw-display.yaw)>9.9f);
    if(axis)out<<",";
    out<<mekfAttitudeJson(s,1002500,true).c_str();
  }
  const auto snapshot=captureMekfAttitude(filter,true,1000000);
  out<<"],\"stale\":"<<mekfAttitudeJson(snapshot,1500000,true).c_str();
  out<<",\"sensor_failed\":"<<mekfAttitudeJson(snapshot,1002500,false).c_str();
  const auto wrapped=captureMekfAttitude(filter,true,UINT32_MAX-100);
  out<<",\"wrapped\":"<<mekfAttitudeJson(wrapped,150,true).c_str();
  filter.setGyroBiasRadS({mekf6::degToRad(.1f),mekf6::degToRad(-.2f),mekf6::degToRad(.3f)});
  filter.updateAccel({0,0,1});
  const auto inputs=captureMekfAttitude(filter,true,1000000,{.01f,.02f,1.0f},
      {mekf6::degToRad(1),mekf6::degToRad(2),mekf6::degToRad(3)},999000);
  const auto frozen=inputs;
  filter.reset();
  assert(inputs.bias_rad_s.z==frozen.bias_rad_s.z && inputs.gyro_rad_s.y==frozen.gyro_rad_s.y);
  out<<",\"inputs\":"<<mekfAttitudeJson(inputs,1002500,true).c_str();
  out<<",\"range\":"<<footRangeDiagnosticsJson().c_str();
  out<<",\"calibration\":"<<footCalibrationDiagnosticsJson().c_str()<<"}";
  std::cout<<"MEKF pitch-only capture, immutable deferred all-axis JSON and quaternion consistency PASS\n";
}
