// Exercises the real ImuManager implementation against deterministic sensor/RTOS
// doubles. This is NOT an ESP32 scheduler, electrical or cold-power hardware test.
#include "host_v46o/Arduino.h"
#include "host_v46o/M5Unified.h"
#include <cassert>
#include <iostream>
#include <limits>
#define private public
#include "../src/imu_manager.h"
#undef private
#include "host_v46o/imu_transport_stub.h"
#include "../src/imu_manager.cpp"

static void resetHost(){host_us=1000;host_core=1;host_task_result=pdPASS;host_tasks_created=0;host_isr_notifications=0;timer_stub::reset();M5=HostM5{};imu_i2c::begin_ok=true;bmi270_timing::transport()=nullptr;}
static void initialized(ImuManager& imu){resetHost();assert(imu.begin());assert(imu.ok());assert(host_tasks_created==1);}
static void put(ImuManager& imu,uint32_t seq,uint32_t stamp){
  ImuReading r=imu.reading_;r.gyro_sequence=seq;r.last_gyro_update_us=stamp;r.queue_submit_us=stamp;r.gyro_update_dt_us=2500;r.last_update_ms=stamp/1000;
  imu.latest_capture_sequence_=seq;imu.latest_capture_us_=stamp;imu.latest_capture_ms_=stamp/1000;
  assert(xQueueSend(imu.sample_queue_,&r,0)==pdTRUE);
}
int main(){
  unsigned checks=0;
  {ImuManager imu;initialized(imu);const auto reads=M5.Imu.data.usec;
   host_us=UINT32_MAX-100;assert(timer_stub::callback(timer_stub::context));
   host_us=899;assert(timer_stub::callback(timer_stub::context));
   assert(host_isr_notifications==2&&imu.notify_stamp_.gap_us==1000&&imu.notify_stamp_.core==1);
   assert(M5.Imu.data.usec==reads);++checks;}
  {resetHost();timer_stub::fail="start";ImuManager imu;assert(!imu.begin());
   assert(!imu.acquisition_task_&&!timer_stub::registered&&!timer_stub::initialized);++checks;}
  {resetHost();M5.In_I2C.release_ok=false;ImuManager imu;assert(!imu.begin() && imu.fault_ && !host_tasks_created);++checks;}
  {resetHost();imu_i2c::begin_ok=false;ImuManager imu;assert(!imu.begin() && imu.fault_ && !host_tasks_created);assert(!bmi270_timing::transport());++checks;}

  {ImuManager imu;initialized(imu);assert(imu.init_valid_accel_>=8&&imu.init_valid_gyro_>=16);assert(UprightPoseGuide::isUprightStableSample(imu.reading()));++checks;}
  // Cold-init failure followed by success. No duplicate acquisition task.
  {resetHost();M5.Imu.fail_begins=2;ImuManager imu;assert(imu.begin());assert(imu.init_attempts_==3);assert(host_tasks_created==1);++checks;}
  // begin() succeeds but no fresh values: retry instead of announcing readiness.
  {resetHost();M5.Imu.no_stream_attempts=1;ImuManager imu;assert(imu.begin());assert(imu.init_attempts_==2);assert(host_tasks_created==1);++checks;}
  {resetHost();M5.Imu.fail_begins=8;ImuManager imu;assert(!imu.begin());assert(imu.init_attempts_==5);assert(!imu.ok());assert(host_tasks_created==0);++checks;}
  {resetHost();M5.Imu.dev.status=2;ImuManager imu;assert(!imu.begin());assert(host_tasks_created==0);++checks;}
  {resetHost();M5.Imu.dev.power=0;ImuManager imu;assert(!imu.begin());assert(host_tasks_created==0);++checks;}
  {resetHost();M5.Imu.data.accel.z=0;ImuManager imu;assert(!imu.begin());assert(host_tasks_created==0);++checks;}
  // Reproduce 8/11/30/79 ms-old queued idle history. Never count it as a
  // newly started run's backlog. Test empty/busy/full queues and sequence wrap.
  for(uint32_t age:{8005U,11001U,30000U,79000U})
    for(uint32_t count=1;count<=32;++count)
      for(uint32_t start:{100U,UINT32_MAX-8}) {
        ImuManager imu;initialized(imu);host_us=1000000;
        for(uint32_t i=0;i<count;++i)put(imu,start+i,host_us-age);
        imu.setAcquisitionContext(true,false,6);
        const uint32_t cut=start+count-1;
        if(count<32)put(imu,cut+1,host_us-100);
        imu.update();assert(!imu.fault_);assert(imu.boundary_.discarded_idle_samples==count);
        if(count==32){put(imu,cut+1,host_us-100);imu.update();}
        assert(imu.reading_.gyro_sequence==cut+1);
        assert(imu.start_sync_deliveries_==1);
        // Repeated context refresh must not slide the boundary or discard later data.
        imu.setAcquisitionContext(true,false,6);
        assert(imu.boundary_.cutoff==cut);
        ++checks;
      }
  // Producer wake before consumer at a FULL idle->run boundary must not
  // misclassify old history as a live queue overflow.
  {ImuManager imu;initialized(imu);host_us=1500000;
   for(unsigned i=1;i<=32;++i)put(imu,100+i,host_us-79000);
   imu.setAcquisitionContext(true,false,6);
   imu.capture_=imu.reading_;imu.capture_.gyro_sequence=133;
   imu.capture_.last_gyro_update_us=host_us;imu.publishSample();
   assert(!imu.fault_&&imu.boundary_producer_discards_==1);
   imu.update();assert(!imu.fault_&&imu.reading_.gyro_sequence==133);
   assert(imu.boundary_.discarded_idle_samples==31);++checks;}
  // A REAL after-boundary 10.001ms delay still stops, recording the first fault
  // even before the measurement audit has started. Clear/context cannot rearm.
  {ImuManager imu;initialized(imu);host_us=2000000;imu.setAcquisitionContext(true,false,6);
   put(imu,imu.boundary_.cutoff+1,host_us-10001);imu.update();assert(imu.fault_);
   assert(imu.fault_snapshot_.age_us==10001&&imu.fault_snapshot_.state_id==6);
   const auto before=imu.fault_snapshot_.time_us;
   imu.setAcquisitionContext(false,false,5);imu.setAcquisitionContext(true,false,6);
   assert(!imu.begin());assert(host_tasks_created==1);
   host_us+=30000;imu.latchFault("later_fault");assert(imu.fault_snapshot_.time_us==before);++checks;}
  // RUNNING_BATCH_SWEEP does not rebase/flush the START_SYNC stream.
  {ImuManager imu;initialized(imu);host_us=3000000;imu.setAcquisitionContext(true,false,6);auto cut=imu.boundary_.cutoff;
   imu.setAcquisitionContext(true,true,3);assert(imu.boundary_.cutoff==cut);
   for(uint32_t i=1;i<=12000;++i){host_us+=2500;put(imu,cut+i,host_us);imu.update();}
   assert(!imu.fault_&&imu.audit_.delivered==12000&&imu.audit_.delivery_sequence_gaps==0);++checks;}
  // A full live queue is still a latched overflow, not an overwrite/mailbox.
  {ImuManager imu;initialized(imu);host_us=4000000;imu.setAcquisitionContext(true,false,6);
   const auto cut=imu.boundary_.cutoff;for(unsigned i=1;i<=32;++i)put(imu,cut+i,host_us);
   imu.capture_.gyro_sequence=cut+33;imu.capture_.last_gyro_update_us=host_us;imu.publishSample();
   assert(imu.fault_&&std::string(imu.fault_reason_)=="imu_acquisition_queue_overflow");++checks;}
  // Timestamp wrap: a valid recent sample stays recent.
  {ImuManager imu;initialized(imu);host_us=UINT32_MAX-1000;imu.setAcquisitionContext(true,false,6);
   auto seq=imu.boundary_.cutoff+1;host_us=300;put(imu,seq,UINT32_MAX-300);imu.update();assert(!imu.fault_);++checks;}
  // Diagnostics contain initialization/first-fault details even without a Run.
  {ImuManager imu;initialized(imu);auto j=imu.acquisitionDiagnosticsJson().value;
   assert(j.find("first_fault")!=std::string::npos&&j.find("init_attempts")!=std::string::npos);
   std::cout<<"diagnostic_json="<<j<<'\n';++checks;}
  std::cout<<"V46o real-ImuManager deterministic regression: "<<checks<<" cases PASS\n";
}
