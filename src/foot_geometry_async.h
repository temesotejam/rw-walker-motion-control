#pragma once
// Optional ESP32/FreeRTOS adapter. The timing controller never performs the
// geometry search. Only enable in the explicit experimental firmware build.
#include "foot_geometry_worker.h"
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_timer.h>

namespace foot_geometry_worker {
struct PoseMessage { uint32_t run_id=0; Pose pose{}; };
struct Request { Input input{}; uint32_t run_epoch_ms=0; uint32_t max_pose_gap_ms=0; };
enum class AsyncReason : uint8_t { None, PoseTimeout, Replaced, GenerationChanged, CalculationRejected, CpuBudget };
struct Reply { Result result{}; Diagnostic diagnostic{}; bool accepted=false; AsyncReason async_reason=AsyncReason::None;
  uint32_t compute_us=0,max_step_us=0,stack_high_water_raw=0; };

class Async {
 public:
  // Call once at setup, before use. Lifetime must exceed that of the task.
  // No delete/stop method: firmware owns this object until reset.
  bool begin() {
    if(task_)return true;
    poses_=xQueueCreate(128,sizeof(PoseMessage));
    jobs_=xQueueCreate(1,sizeof(Request));
    replies_=xQueueCreate(1,sizeof(Reply));
    if(!poses_ || !jobs_ || !replies_) {cleanupQueues();return false;}
    if(xTaskCreatePinnedToCore(entry,"foot_geometry",8192,this,1,&task_,0)!=pdPASS) {
      task_=nullptr;cleanupQueues();return false;
    }
    return true;
  }
  // Call from the control owner, in this order, at run/peak/job replacement.
  // New generation invalidates an in-flight calculation before publishing.
  void generation(uint32_t run,uint32_t peak,uint32_t generation) {
    run_.store(run,std::memory_order_relaxed);peak_.store(peak,std::memory_order_relaxed);
    generation_.store(generation,std::memory_order_release);
  }
  bool pose(const PoseMessage& p) {return poses_ && xQueueSend(poses_,&p,0)==pdTRUE;}
  bool submit(const Request& r) {
    if(!jobs_ || r.input.generation!=generation_.load(std::memory_order_acquire) ||
       r.input.run_id!=run_.load(std::memory_order_relaxed) ||
       r.input.peak_id!=peak_.load(std::memory_order_relaxed))return false;
    return xQueueOverwrite(jobs_,&r)==pdTRUE;
  }
  bool receive(Reply& out) {return replies_ && xQueueReceive(replies_,&out,0)==pdTRUE;}
 private:
  void cleanupQueues(){
    if(poses_)vQueueDelete(poses_);
    if(jobs_)vQueueDelete(jobs_);
    if(replies_)vQueueDelete(replies_);
    poses_=jobs_=replies_=nullptr;
  }
  void publish(Reply& r) {
    r.stack_high_water_raw=static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr));
    xQueueOverwrite(replies_,&r);
  }
  static void entry(void* arg) {static_cast<Async*>(arg)->loop();}
  uint32_t now(const Request& r) const {
    return static_cast<uint32_t>(esp_timer_get_time()/1000)-r.run_epoch_ms;
  }
  bool current(const Request& r)const {
    const uint32_t g=generation_.load(std::memory_order_acquire);
    return g && r.input.generation==g && r.input.run_id==run_.load(std::memory_order_relaxed) &&
           r.input.peak_id==peak_.load(std::memory_order_relaxed) &&
           g==generation_.load(std::memory_order_acquire);
  }
  void loop() {
    PoseHistory<256> history;Worker worker;Request active;bool pending=false;
    uint32_t history_run=0,compute_us=0,max_step_us=0;
    for(;;) {
      PoseMessage p;
      // Bounded drain; never starve candidate cancellation.
      for(unsigned i=0;i<32 && xQueueReceive(poses_,&p,0)==pdTRUE;++i) {
        if(p.run_id!=history_run){history.reset();history_run=p.run_id;}
        history.push(p.pose);
      }
      Request replacement;
      if(xQueueReceive(jobs_,&replacement,0)==pdTRUE) {active=replacement;worker.cancel();pending=true;compute_us=max_step_us=0;}
      if(pending) {
        if(!current(active) || now(active)>=active.input.expiry_ms) {
          Reply out;out.diagnostic=worker.diagnostic();
          out.diagnostic.run_id=active.input.run_id;out.diagnostic.peak_id=active.input.peak_id;
          out.diagnostic.sequence=active.input.sequence;out.diagnostic.generation=active.input.generation;
          out.diagnostic.frame_ms=active.input.frame_ms;out.diagnostic.exposure=active.input.exposure;
          out.diagnostic.exposure_earliest_ms=active.input.exposure_earliest_ms;
          out.diagnostic.exposure_latest_ms=active.input.exposure_latest_ms;
          out.async_reason=current(active)?AsyncReason::PoseTimeout:AsyncReason::GenerationChanged;
          publish(out);
          worker.cancel();pending=false;
        } else if(worker.status()==Status::Idle) {
          float lo,hi;
          if(history_run==active.input.run_id && history.range(active.input.exposure_earliest_ms,
                active.input.exposure_latest_ms,active.max_pose_gap_ms,lo,hi)) {
            active.input.pitch_lo_deg=lo;active.input.pitch_hi_deg=hi;
            active.input.pose_interval_bracketed=true;
            if(!worker.submit(active.input,now(active))) {
              Reply out;out.diagnostic=worker.diagnostic();out.async_reason=AsyncReason::CalculationRejected;
              publish(out);pending=false;
            }
          }
          // A not-yet-bracketed observation waits until expiry or replacement.
        } else {
          Status state=Status::Working;
          const int64_t batch_start=esp_timer_get_time();bool budget_exceeded=false;
          for(unsigned i=0;i<8 && state==Status::Working;++i) {
            const int64_t step_start=esp_timer_get_time();
            state=worker.step(now(active),run_.load(std::memory_order_relaxed),
                    peak_.load(std::memory_order_relaxed),generation_.load(std::memory_order_acquire));
            const uint32_t step_us=static_cast<uint32_t>(esp_timer_get_time()-step_start);
            compute_us+=step_us;if(step_us>max_step_us)max_step_us=step_us;
            if(step_us>250 || compute_us>5000) {budget_exceeded=true;break;}
            if(esp_timer_get_time()-batch_start>=250)break;
          }
          if(budget_exceeded) {
            Reply out;out.diagnostic=worker.diagnostic();out.async_reason=AsyncReason::CpuBudget;
            out.compute_us=compute_us;out.max_step_us=max_step_us;
            if(current(active))publish(out);
            worker.cancel();pending=false;
          } else if(state==Status::Ready) {
            Reply out;out.accepted=worker.result(now(active),run_.load(std::memory_order_relaxed),
                 peak_.load(std::memory_order_relaxed),generation_.load(std::memory_order_acquire),out.result);
            out.diagnostic=worker.diagnostic();
            out.compute_us=compute_us;out.max_step_us=max_step_us;
            if(current(active))publish(out);
            pending=false;
          } else if(state==Status::Rejected) {
            Reply out;out.diagnostic=worker.diagnostic();out.async_reason=AsyncReason::CalculationRejected;
            out.compute_us=compute_us;out.max_step_us=max_step_us;
            if(current(active))publish(out);
            pending=false;
          }
        }
      }
      // Give the rest of core 0 execution time. This delay is included in the
      // deadline; if 96 hypotheses cannot complete, ZERO remains unchanged.
      vTaskDelay(1);
    }
  }
  QueueHandle_t poses_=nullptr,jobs_=nullptr,replies_=nullptr;TaskHandle_t task_=nullptr;
  std::atomic<uint32_t> run_{0},peak_{0},generation_{0};
};
}
