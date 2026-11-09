// Real ImuManager + sensor/RTOS doubles. Functional tests, NOT device timing.
#include "host_v46o/Arduino.h"
#include "host_v46o/M5Unified.h"
#include <cassert>
#include <cmath>
#include <iostream>
#define private public
#include "../src/imu_manager.h"
#undef private
#include "host_v46o/imu_transport_stub.h"
#include "../src/imu_manager.cpp"
static void init(ImuManager& imu) {
  host_us=1000; host_tasks_created=0; M5=HostM5{};
  assert(imu.begin()); assert(host_tasks_created==1);
}
int main() {
  unsigned cases=0;
  {
    ImuManager imu; init(imu);
    uint32_t seed=12345;
    auto random=[&](){seed=1664525U*seed+1013904223U;return (static_cast<int32_t>(seed>>8)-8388608)*0.0000002f;};
    for (unsigned i=0;i<5000;++i) {
      M5.Imu.data.accel={random(),random(),random()};
      host_us+=5000;
      imu.capture_.acc_norm_g=-1234; imu.capture_.pitch_accel_only_deg=-5678;
      imu.poll_observation_=ImuPollObservation{};
      imu.captureSensor();
      assert(imu.capture_.acc_norm_g==-1234&&imu.capture_.pitch_accel_only_deg==-5678);
      const auto raw=imu.capture_;
      imu.update(); const auto& v=imu.reading();
      const float norm=sqrtf(raw.ax_g*raw.ax_g+raw.ay_g*raw.ay_g+raw.az_g*raw.az_g);
      const float angle=Config::PITCH_SIGN*atan2f(-raw.ax_g,sqrtf(raw.ay_g*raw.ay_g+raw.az_g*raw.az_g))*57.2957795f;
      assert(v.acc_norm_g==norm&&v.acc_norm_error_g==norm-1.0f&&v.pitch_accel_only_deg==angle);
      assert(v.gyro_sequence==raw.gyro_sequence&&v.last_gyro_update_us==raw.last_gyro_update_us);
      assert(v.gyro_update_dt_us==raw.gyro_update_dt_us);
      // Same accel sequence: keep the previous derived value, not queued garbage.
      ImuReading next=v; next.acc_norm_g=-999; next.pitch_accel_only_deg=-999;
      ++next.gyro_sequence; assert(xQueueSend(imu.sample_queue_,&next,0)==pdTRUE);
      imu.update(); assert(imu.reading().acc_norm_g==norm&&imu.reading().pitch_accel_only_deg==angle);
      assert(!imu.reading().accel_fresh); ++cases;
    }
  }
  {ImuManager imu;init(imu);host_us+=5000;M5.Imu.update_cost_us=123;M5.Imu.convert_cost_us=47;
   imu.poll_observation_=ImuPollObservation{};imu.captureSensor();
   assert(imu.poll_observation_.update_us==123&&imu.poll_observation_.convert_us==47);
   assert(imu.poll_observation_.fresh_gyro);++cases;}
  {ImuManager imu;init(imu);host_us+=5000;M5.Imu.data.gyro.y=NAN;
   imu.captureSensor();assert(!imu.acquisitionHealthy());assert(std::string(imu.fault_reason_)=="imu_nonfinite_sample");++cases;}
  {ImuManager imu;init(imu);host_us=UINT32_MAX-100;
   ImuManager::timerCallback(&imu);host_us=99;ImuManager::timerCallback(&imu);
   assert(imu.notify_stamp_.seen&&imu.notify_stamp_.gap_us==200&&imu.notify_stamp_.sequence==2);++cases;}
  {ImuManager imu;init(imu);host_us=1000000;imu.setAcquisitionContext(true,true,3);
   ImuPollObservation o;o.start_us=host_us;o.update_us=400;o.total_us=470;
   imu.recordPollProfile(o);assert(imu.poll_profile_.polls==1);
   assert(imu.pollProfileJson().value.find("measurement_active")!=std::string::npos);
   imu.setAcquisitionContext(false,false,7);imu.recordPollProfile(o);
   assert(imu.poll_profile_.polls==1); // No idle writer can race stopped export.
   assert(imu.pollProfileJson().value.find("\"polls\":1")!=std::string::npos);
   host_us+=100000;imu.setAcquisitionContext(true,true,3);o.start_us=host_us;
   imu.recordPollProfile(o);assert(imu.poll_profile_.polls==1&&imu.poll_profile_.epoch_us==host_us);++cases;}
  {ImuPollProfile p;p.start(UINT32_MAX-1000000);unsigned gap_count=0,gyro=0;
   for(unsigned i=1;i<=30000;++i){
     ImuPollObservation o;o.start_us=p.epoch_us+i*1000;o.sample_us=o.start_us;
     o.sequence=i;o.mask=(i%5<2)?3:0;o.fresh_gyro=o.mask!=0;o.dt_us=2500;
     o.has_notify=true;o.notify_age_us=40;o.callback_sequence=i;o.callback_gap_us=1000;
     o.wakes=1;o.period_us=1000;o.update_us=350;o.convert_us=20;o.pack_us=8;o.publish_us=9;o.total_us=400;
     o.forced_yield=(i%100==0);
     if(i%80==1){o.fresh_gyro=true;o.mask=3;o.dt_us=5100;++gap_count;}
     if(o.fresh_gyro) ++gyro;
     p.record(o);
   }
   assert(p.polls==30000&&p.gyro==gyro&&p.long_gaps==gap_count&&p.forced_yields==300);
   assert(p.stage[ImuPollProfile::UPDATE].count==30000&&p.stage[ImuPollProfile::UPDATE].sum_us==10500000);
   assert(p.worst_gap[299].dt_us>4000); // Late Run coverage, not only first 128 gaps.
   auto count=p.polls;ImuPollObservation early;early.start_us=p.epoch_us-1;p.record(early);assert(p.polls==count);
   p.start(123);assert(p.polls==0&&p.long_gaps==0&&p.worst_gap[299].dt_us==0);++cases;}
  {ImuManager imu;init(imu);host_us=1000000;imu.setAcquisitionContext(true,true,3);
   ImuPollObservation o;o.start_us=host_us;o.sample_us=host_us;o.fresh_gyro=true;o.mask=3;o.dt_us=5100;
   o.has_notify=true;o.notify_age_us=50;o.callback_sequence=1;o.callback_gap_us=1000;
   o.update_us=400;o.convert_us=40;o.pack_us=10;o.publish_us=20;o.total_us=480;
   imu.recordPollProfile(o);imu.setAcquisitionContext(false,false,7);
   std::cout<<"profile_json="<<imu.pollProfileJson().value<<'\n';++cases;}
  std::cout<<"V46q real-ImuManager relocation/profile regression: "<<cases<<" cases PASS; profile_bytes="<<sizeof(ImuPollProfile)<<'\n';
}
