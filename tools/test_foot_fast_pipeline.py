"""Run the production v20 camera/foot loop with host ownership/time doubles.

The loop, camera acquisition/release/configuration, tracker, sparse-row cache,
zero calibration, angle mapping and both predictors come from current source.
This verifies software ownership and scheduling contracts, not board FPS,
exposure time, physical angle accuracy or control deadline performance.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    """Extract a complete current C++ definition (including its original body)."""
    start = source.index(signature)
    opening = source.index('{', start)
    # These selected methods have no braces inside string literals/comments.
    depth = 1
    for pos in range(opening + 1, len(source)):
        depth += (source[pos] == '{') - (source[pos] == '}')
        if depth == 0:
            return source[start:pos+1]
    raise AssertionError('unclosed method: ' + signature)


def declarations(path):
    return re.sub(r'^\s*(?:#include[^\n]*|#pragma once)\n', '',
        path.read_text(encoding='utf8'), flags=re.M).replace(' private:', ' public:')


def fixture():
    observer = (ROOT/'src/foot_observer.cpp').read_text(encoding='utf8')
    camera = (ROOT/'src/camera_coexistence.cpp').read_text(encoding='utf8')
    priority = (ROOT/'src/camera_task_priority_patch.cpp').read_text(encoding='utf8')
    # Current hardware contracts, independent of the historical whole-file SHA.
    assert 'constexpr UBaseType_t kCameraInternalTaskPriority = 3;' in priority
    assert 'effective_core = 0;' in priority and 'strcmp(pcName, "cam_task") == 0' in priority
    init = function(camera, 'bool OneShotCamera::initCameraOnTemporaryI2c0()')
    assert 'I2C_NUM_1' not in init
    assert init.count('i2c_driver_install(I2C_NUM_0,') == 1
    assert init.count('i2c_driver_delete(I2C_NUM_0)') >= 2
    assert 'sccb.sda_io_num = static_cast<gpio_num_t>(PIN_CAM_SDA);' in init
    assert 'sccb.scl_io_num = static_cast<gpio_num_t>(PIN_CAM_SCL);' in init
    assert 'sccb.master.clk_speed = 100000;' in init
    continuous = function(camera, 'camera_fb_t* OneShotCamera::acquireContinuous(')
    code = re.sub(r'//[^\n]*', '', continuous)
    assert code.count('cam_take(') == 1 and 'cam_take(0)' not in code
    config = init[init.index('camera_config_t c = {};'):init.index('  err = esp_camera_init(&c);')]
    fields = sorted(set(re.findall(r'c\.(\w+)\s*=', config)))
    constants = '\n'.join(re.findall(r'^constexpr (?:int|uint32_t) (?:PIN_CAM_\w+|k\w+) = [^;]+;', camera, re.M))
    preamble = r'''
#include "Arduino.h"
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <sys/time.h>
#include "foot_image_rows.h"
#include "foot_fast_policy.h"
#include "camera_speed_profile.h"
#include "foot_zero.h"
#include "white_marker_tracker.h"
#include "foot_angle_estimator.h"
#include "foot_phase_prediction.h"
#include "foot_local_prediction.h"
#include "foot_control_range.h"
using TickType_t=uint32_t;
using TaskHandle_t=void*;
using SemaphoreHandle_t=void*;
using portMUX_TYPE=int;
constexpr int portMUX_INITIALIZER_UNLOCKED=0,pdPASS=1,pdTRUE=1;
constexpr int MALLOC_CAP_INTERNAL=1,MALLOC_CAP_8BIT=2;
constexpr int PIXFORMAT_GRAYSCALE=1,FRAMESIZE_QVGA=2,LEDC_TIMER_0=0,LEDC_CHANNEL_0=0;
constexpr int CAMERA_FB_IN_PSRAM=1,CAMERA_GRAB_LATEST=2,CAMERA_GRAB_WHEN_EMPTY=1,I2C_NUM_0=0;
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define pdMS_TO_TICKS(x) (x)
static void* heap_caps_malloc(size_t size,int caps){assert(caps==3);return std::malloc(size);}
static SemaphoreHandle_t xSemaphoreCreateMutex(){return reinterpret_cast<void*>(1);}
static int xSemaphoreTake(SemaphoreHandle_t s,TickType_t){return s?pdTRUE:0;}
static void xSemaphoreGive(SemaphoreHandle_t s){assert(s);}
static int xTaskCreatePinnedToCore(void(*)(void*),const char* name,uint32_t stack,void*,int priority,TaskHandle_t* task,int core){
 assert(std::string(name)=="foot_observer" && stack==10240 && priority==1 && core==0);
 *task=reinterpret_cast<void*>(1);return pdPASS;
}
static int xPortGetCoreID(){return 0;}
static int uxTaskPriorityGet(void*){return 1;}
namespace RuntimeDiag {
 enum class Lane {Camera};
 enum class Phase {Snapshot,CameraStop,CameraCapture,CameraProcess,CameraRelease,Publish,Wait};
 static void phase(Lane,Phase){} static void beat(Lane,uint32_t){}
}
struct camera_fb_t {uint8_t* buf=nullptr;size_t len=0,width=0,height=0;int format=0;timeval timestamp{};};
struct MekfAttitudeSnapshot {bool valid=true;int sentinel=73;};
struct ControlSnapshot {
 bool running=false,imu_ok=true,upright_stable=true;
 uint8_t state_id=2,led_state=1,sync_event_id=2;
 uint16_t run_id=7;uint32_t imu_sample_us=123,upright_epoch=1;
 uint64_t upright_since_us=1,measurement_epoch_us=500000;
 MekfAttitudeSnapshot mekf_attitude;
};
class RunControlWorker {public:bool ready()const{return true;}ControlSnapshot snapshot()const;};
class PsramString;
struct LoopDone{};
static void vTaskDelay(TickType_t ticks);
static camera_fb_t* cam_take(TickType_t timeout);
static void cam_give(camera_fb_t* fb);
static void cam_start();static void cam_stop();
static void cameraTaskPriorityPatchSampleStack(){}
'''
    methods = '\n'.join(function(observer, signature) for signature in [
        'bool FootObserver::begin(', 'bool FootObserver::copyPreview(',
        'void FootObserver::publishPreview(', 'FootSnapshot FootObserver::snapshot()',
        'void FootObserver::recordControlObservation(', 'void FootObserver::beginRun(',
        'void FootObserver::finishRun()', 'void FootObserver::clearRun()',
        'void FootObserver::loop()'])
    camera_methods = '\n'.join(function(camera, signature) for signature in [
        'void OneShotCamera::setError(', 'void OneShotCamera::flushQueuedFrames()',
        'camera_fb_t* OneShotCamera::acquire(', 'void OneShotCamera::release(',
        'bool OneShotCamera::debugCaptureOnce()',
        'CameraOneShotSnapshot OneShotCamera::snapshot()',
        'bool OneShotCamera::startContinuous()',
        'camera_fb_t* OneShotCamera::acquireContinuous(',
        'void OneShotCamera::releaseContinuous(', 'void OneShotCamera::stopContinuous()'])
    return (preamble + declarations(ROOT/'src/camera_coexistence.h') + '\n' +
        declarations(ROOT/'src/foot_observer.h') + '\n' + constants + '\n' +
        'struct camera_config_t {' + ''.join('int '+f+'=0;' for f in fields) + '};\n' +
        'static camera_config_t testedConfig(){\n' + config + '\nreturn c;}\n' +
        'void OneShotCamera::setXclkEnabled(bool enabled){snapshot_.xclk_active=enabled;}\n' +
        camera_methods + '\n' + methods + r'''

using Image=std::vector<uint8_t>;
static Image image(int right=203,int left=202){
 Image out(320*240,30);
 for(int row=-10;row<=10;++row)for(int dx=-10;dx<=10;++dx){
  out[(66+row)*320+right+dx]=230;out[(160+row)*320+left+dx]=230;
 }
 return out;
}
enum class Invalid {None,NullFrame,Short,NullBuffer,FutureTimestamp,OldTimestamp};
struct Plan {
 Image original=image();ControlSnapshot before,after;
 Invalid invalid=Invalid::None;uint32_t wait_us=2000;bool stop=false;
};
struct Iteration {FootSnapshot snapshot;FootPreviewInfo info;Image preview;bool preview_ok=false;
 uint64_t time=0;uint32_t delay=0;foot_image_rows::ReadCounts reads;};
static std::vector<Plan> plans;
static std::vector<Iteration> iterations;
static size_t plan_index=0;static unsigned context_calls=0,takes=0,gives=0,flush_takes=0,starts=0,stops=0;
static bool live=true,borrowed=false,poison=false,enforce_rows=true;
static Image buffer;
static camera_fb_t frame;
static FootObserver* current=nullptr;
ControlSnapshot RunControlWorker::snapshot()const {
 const auto& p=plans.at(plan_index);return context_calls++==0?p.before:p.after;
}
static camera_fb_t* cam_take(TickType_t timeout){
 if(timeout==0){assert(!live && !borrowed);++flush_takes;return nullptr;}
 assert(live && !borrowed);++takes;
 auto& p=plans.at(plan_index);host_clock+=p.wait_us;
 foot_image_rows::readCounts()={};
 if(p.invalid==Invalid::NullFrame)return nullptr;
 buffer=p.original;frame={};frame.buf=p.invalid==Invalid::NullBuffer?nullptr:buffer.data();
 frame.len=p.invalid==Invalid::Short?123:buffer.size();
 uint64_t stamp=host_clock-50000;
 if(p.invalid==Invalid::FutureTimestamp)stamp=host_clock+5000;
 if(p.invalid==Invalid::OldTimestamp)stamp=host_clock-600000;
 frame.timestamp.tv_sec=stamp/1000000;frame.timestamp.tv_usec=stamp%1000000;
 borrowed=true;return &frame;
}
static void cam_give(camera_fb_t* fb){
 assert(borrowed && fb==&frame);borrowed=false;++gives;
 if(enforce_rows){
  const auto& r=foot_image_rows::readCounts();
  const auto invalid=plans.at(plan_index).invalid;
  const bool cache_valid=invalid!=Invalid::Short && invalid!=Invalid::NullBuffer;
  assert(r.capture_rows==(cache_valid?foot_image_rows::kRequiredRows:0));
  assert(r.packed_rows==0 && r.full_rows==0); // actual recognition has not read any row
 }
 if(poison)std::fill(buffer.begin(),buffer.end(),0);
}
static void cam_start(){assert(!borrowed);live=true;++starts;}
static void cam_stop(){if(enforce_rows)assert(!borrowed);live=false;++stops;}
static void vTaskDelay(TickType_t ticks){
 assert(!borrowed);
 Iteration it;it.snapshot=current->snapshot();it.preview.resize(FootObserver::kPreviewBytes);
 it.preview_ok=current->copyPreview(it.preview.data(),it.info);
 it.time=host_clock;it.delay=static_cast<uint32_t>(ticks);it.reads=foot_image_rows::readCounts();
 iterations.push_back(it);host_clock+=ticks*1000;context_calls=0;
 if(++plan_index==plans.size())throw LoopDone{};
}
static bool equal(float a,float b){return (std::isnan(a)&&std::isnan(b)) || std::abs(a-b)<1e-6f;}
static void detectionEqual(const FootFrame& a,const FootFrame& b,bool projections=true){
 assert(a.sequence==b.sequence && a.frame_valid==b.frame_valid && a.timestamp_valid==b.timestamp_valid);
 assert(a.right_valid==b.right_valid && a.left_valid==b.left_valid && a.zero_ready==b.zero_ready);
 assert(equal(a.right_x,b.right_x) && equal(a.left_x,b.left_x));
 assert(equal(a.right_deg,b.right_deg) && equal(a.left_deg,b.left_deg));
 assert(equal(a.right_contrast,b.right_contrast) && equal(a.left_contrast,b.left_contrast));
 assert(equal(a.right_weight,b.right_weight) && equal(a.left_weight,b.left_weight));
 assert(equal(a.right_scan_y,b.right_scan_y) && equal(a.left_scan_y,b.left_scan_y));
 assert(a.right_reason==b.right_reason && a.left_reason==b.left_reason);
 assert(a.right_templates==b.right_templates && a.left_templates==b.left_templates);
 assert(a.right_candidates==b.right_candidates && a.left_candidates==b.left_candidates);
 assert(equal(a.right_ambiguity,b.right_ambiguity) && equal(a.left_ambiguity,b.left_ambiguity));
 assert(a.zero_reason==b.zero_reason);
 if(!projections)return; // preview is published before pose/local computation
 assert(a.pose_projection.reason==b.pose_projection.reason && a.local_projection.reason==b.local_projection.reason);
 for(unsigned i=0;i<4;++i){assert(equal(a.pose_projection.left[i],b.pose_projection.left[i]));
  assert(equal(a.pose_projection.right[i],b.pose_projection.right[i]));}
 assert(equal(a.local_projection.left_model.observed,b.local_projection.left_model.observed));
 assert(equal(a.local_projection.right_model.observed,b.local_projection.right_model.observed));
}
static Image thumbnail(const Image& source){
 Image out(FootObserver::kPreviewBytes);
 for(unsigned y=0;y<120;++y)for(unsigned x=0;x<160;++x)out[y*160+x]=source[(y*2)*320+x*2];
 return out;
}
struct Result {std::vector<Iteration> history;unsigned take=0,give=0,flush=0,stop=0;};
static Result run(std::vector<Plan> input,bool poison_image,bool archive_full=false){
 plans=std::move(input);iterations.clear();plan_index=0;context_calls=0;
 takes=gives=flush_takes=starts=stops=0;live=true;borrowed=false;poison=poison_image;enforce_rows=true;
 host_clock=1000000;foot_image_rows::readCounts()={};
 OneShotCamera camera;camera.snapshot_.camera_ok=true;camera.snapshot_.sensor_powered=true;
 camera.snapshot_.camera_driver_active=true;camera.snapshot_.receiver_active=true;camera.snapshot_.xclk_active=true;
 RunControlWorker control;FootObserver observer;current=&observer;
 assert(observer.begin(camera,control));assert(observer.snapshot().row_cache_ready);
 observer.beginRun(7,500000);
 FootFrame canary;canary.sequence=0xdeadbeef;
 if(archive_full){
  observer.frames_=static_cast<FootFrame*>(std::realloc(observer.frames_,sizeof(FootFrame)*(FootObserver::kCapacity+1)));
  assert(observer.frames_);observer.frames_[FootObserver::kCapacity-1]=observer.frames_[FootObserver::kCapacity]=canary;
  observer.status_.count=FootObserver::kCapacity;
 }
 try{observer.loop();assert(false);}catch(const LoopDone&){}
 assert(!borrowed);
 auto status=observer.snapshot();
 if(archive_full){assert(status.overflow && status.count==FootObserver::kCapacity);
  assert(observer.frames_[FootObserver::kCapacity-1].sequence==canary.sequence);
  assert(observer.frames_[FootObserver::kCapacity].sequence==canary.sequence);}
 else {
  assert(status.count==takes);
  if(status.count){
   auto& last=observer.frames_[status.count-1];
   observer.recordControlObservation(7,last.sequence,last.ready_us+1000);
   assert(last.control_observed_us==last.ready_us+1000);
   assert(observer.snapshot().latest.control_observed_us==last.control_observed_us);
   observer.recordControlObservation(7,last.sequence,last.ready_us+2000);
   observer.recordControlObservation(7,last.sequence+1,last.ready_us+3000);
   assert(observer.control_observation_attempts_==3 && observer.control_observation_unmatched_==2);
   assert(last.control_observed_us==last.ready_us+1000);
  }
 }
 assert(camera.snapshot().frame_count==gives && camera.snapshot().frame_failures==takes-gives);
 assert(status.recognition_max_us>0 && status.row_copy_max_us>0 && status.processing_max_us>0);
 observer.finishRun();assert(!observer.snapshot().recording);
 observer.clearRun();assert(observer.snapshot().count==0 && !observer.snapshot().overflow);
 Result out{iterations,takes,gives,flush_takes,stops};
 std::free(observer.frames_);std::free(observer.preview_);std::free(observer.row_storage_);std::free(observer.preview_staging_);
 current=nullptr;return out;
}
static std::vector<Plan> scenario(){
 std::vector<Plan> out(70); // >=2s even at the v22 worker cadence
 for(int i=0;i<8;++i){Plan p;p.before.running=p.after.running=true;p.before.state_id=p.after.state_id=3;
  p.original=image(202-i,201-i);if(i==0){p.before.running=false;p.before.state_id=2;}
  out.push_back(p);}
 for(auto invalid:{Invalid::NullFrame,Invalid::Short,Invalid::NullBuffer}){
  Plan p;p.before.running=p.after.running=true;p.before.state_id=p.after.state_id=3;p.invalid=invalid;out.push_back(p);}
 for(int i=0;i<4;++i){Plan p;p.before.running=p.after.running=true;p.before.state_id=p.after.state_id=3;
  p.original=image(195-i,194-i);out.push_back(p);}
 Plan stop;stop.before.state_id=5;stop.stop=true;out.push_back(stop);return out;
}
static void fullPipeline(){
 const auto source=scenario();const auto baseline=run(source,false),poisoned=run(source,true);
 assert(baseline.history.size()==source.size() && poisoned.history.size()==source.size());
 assert(poisoned.take==source.size()-1 && poisoned.give==poisoned.take-1);
 assert(poisoned.flush==1 && poisoned.stop==1);
 bool phase_ready=false,local_ready=false;
 for(size_t i=0;i<source.size()-1;++i){
  const auto& a=baseline.history[i];const auto& b=poisoned.history[i];const auto& f=b.snapshot.latest;
  detectionEqual(a.snapshot.latest,f);
  assert(a.preview_ok==b.preview_ok && a.preview==b.preview);
  assert(f.sequence==i+1 && f.capture_wait_us>=source[i].wait_us);
  assert(f.ready_us>f.delivered_us && f.processing_us>=f.recognition_us);
  assert(f.pose_compute_us>=f.local_compute_us && f.pose_projection.received_ms==f.local_projection.received_ms);
  assert(f.ready_us>=500000+uint64_t(f.pose_projection.received_ms)*1000);
  const bool null_frame=source[i].invalid==Invalid::NullFrame;
  if(null_frame)assert(f.released_us==0);
  else assert(f.released_us>f.delivered_us && f.ready_us>f.released_us);
  assert(b.delay>=foot_fast_policy::kWorkerPeriodUs/1000-3 && b.delay<=foot_fast_policy::kWorkerPeriodUs/1000);
  if(i+1<source.size()-1){const auto dt=poisoned.history[i+1].snapshot.latest.delivered_us-f.delivered_us;
   assert(dt>=foot_fast_policy::kWorkerPeriodUs && dt<foot_fast_policy::kWorkerPeriodUs+1100);}
  if(i<70){assert(b.preview_ok && b.preview==thumbnail(source[i].original));
   detectionEqual(b.info.frame,f,false);assert(b.info.mekf_attitude.sentinel==73 && b.info.imu_ok);
   assert(equal(b.info.right.center_x_px,f.right_x) && equal(b.info.left.center_x_px,f.left_x));}
  else {assert(b.preview_ok && b.info.frame.sequence==70 && b.preview==thumbnail(source[69].original));}
  if(i>=70 && i<78){assert(f.zero_ready && f.right_valid && f.left_valid);
   assert(f.right_templates==17 && f.left_templates==17);
   phase_ready|=f.pose_projection.reason==foot_phase_prediction::Reason::Ready;
   local_ready|=f.local_projection.reason==foot_local_prediction::Reason::Ready;}
  if(i>=78 && i<81){assert(!f.frame_valid && !f.row_cache_used && !f.right_valid && !f.left_valid);
   assert(std::isnan(f.right_x) && std::isnan(f.left_x));
   assert(f.right_reason==MarkerDetectionReason::NoFrame && f.left_reason==MarkerDetectionReason::NoFrame);
   assert(b.reads.packed_rows==0 && b.reads.capture_rows==0);}
  else assert(f.row_cache_used && b.reads.capture_rows==foot_image_rows::kRequiredRows && b.reads.packed_rows>0 && b.reads.full_rows==0);
 }
 assert(phase_ready && local_ready && poisoned.history[69].snapshot.zero_ready);
 const auto& last=poisoned.history.back();assert(last.delay==20 && last.snapshot.latest.sequence==poisoned.take);
 assert(last.snapshot.frame_failures==3 && last.snapshot.captured_frames==poisoned.take-3);
 assert(last.snapshot.right_marker_failures==0 && last.snapshot.left_marker_failures==0);
 std::cout<<"real v20 loop: early release/poison, idle preview, active preservation, zero/angles/predictors, invalidation, period/times/counters PASS\n";
}
static void edgeCases(){
 Plan good,bad;bad.invalid=Invalid::Short;
 auto result=run({good,bad},true);assert(result.history[0].preview_ok && !result.history[1].preview_ok);
 assert(result.history[1].snapshot.latest.right_reason==MarkerDetectionReason::NoFrame);
 assert(!result.history[1].snapshot.latest.row_cache_used);
 for(auto invalid:{Invalid::FutureTimestamp,Invalid::OldTimestamp}){
  Plan p;p.invalid=invalid;auto timed=run({p},true);const auto& f=timed.history[0].snapshot.latest;
  assert(f.frame_valid && !f.timestamp_valid && !f.right_valid && !f.left_valid);
  assert(!f.zero_ready && f.log_time_us==-1 && f.measurement_time_us==-1);
 }
 Plan slow;slow.wait_us=60000;auto overtime=run({slow},true);assert(overtime.history[0].delay==1);
 assert(overtime.history[0].snapshot.latest.capture_wait_us>=60000);
 auto full=run({good},true,true);assert(full.history[0].snapshot.overflow && full.history[0].snapshot.latest.sequence==1);
 auto c=testedConfig();assert(c.fb_count==2 && c.grab_mode==CAMERA_GRAB_LATEST);
 assert(c.pixel_format==PIXFORMAT_GRAYSCALE && c.frame_size==FRAMESIZE_QVGA && c.fb_location==CAMERA_FB_IN_PSRAM);
 assert(c.xclk_freq_hz==20000000 && c.sccb_i2c_port==I2C_NUM_0 && c.pin_sccb_sda==-1 && c.pin_sccb_scl==-1);
 assert(foot_fast_policy::kArchiveCapacity==1280);
 std::cout<<"invalid idle preview, bad timestamps, overrun, full archive, source camera2/LATEST/SCCB0/priority3 Core0 PASS\n";
}
static void manualOneShot(){
 // Execute the actual manual acquire/release path: stop receiver/XCLK before
 // returning, retain the borrowed image until the matching explicit release.
 plans={Plan{}};plan_index=0;context_calls=0;takes=gives=0;borrowed=false;live=false;enforce_rows=false;poison=false;
 OneShotCamera camera;camera.capture_mutex_=reinterpret_cast<void*>(1);
 camera.snapshot_.camera_ok=camera.snapshot_.camera_driver_active=camera.snapshot_.sensor_powered=true;
 auto* fb=camera.acquire(200);assert(fb && borrowed && !live);
 assert(!camera.snapshot().receiver_active && !camera.snapshot().xclk_active && gives==0);
 camera.release(fb);assert(!borrowed && takes==1 && gives==1);
 camera.snapshot_.sensor_powered=false;assert(!camera.acquire(200) && takes==1);
 std::cout<<"real manual one-shot receiver/XCLK stop and exactly one ownership return PASS\n";
}
static void idleReceipt(){
#if (defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23) || (defined(FOOT_RECEIPT_AUDIT) && FOOT_RECEIPT_AUDIT)
 FootObserver observer;
 auto& f=observer.status_.latest;
 f.run_id=0;f.sequence=17;f.timestamp_valid=true;f.ready_us=1000;
 observer.recordControlObservation(1,17,1100);
 observer.recordControlObservation(0,18,1100);
 observer.recordControlObservation(0,17,999);
 assert(!f.control_observed_us);
 f.timestamp_valid=false;observer.recordControlObservation(0,17,1100);
 assert(!f.control_observed_us);f.timestamp_valid=true;
 observer.recordControlObservation(0,17,1100);
 observer.recordControlObservation(0,17,1200);
 assert(f.control_observed_us==1100);
 assert(!observer.status_.recording && !observer.status_.count);
 assert(!observer.control_observation_attempts_ && !observer.control_observation_unmatched_);
 std::cout<<"idle first receipt identity/time guards and no archive mutation PASS\n";
#endif
}
int main(){fullPipeline();edgeCases();manualOneShot();idleReceipt();}
''')


ARDUINO = r'''
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>
using std::min;using std::max;using std::isfinite;
inline uint64_t host_clock=1000000;
inline uint32_t micros(){return static_cast<uint32_t>(++host_clock);}
inline uint32_t millis(){return static_cast<uint32_t>(host_clock/1000);}
inline uint64_t esp_timer_get_time(){return ++host_clock;}
inline void delay(uint32_t ms){host_clock+=ms*1000;}
inline void* ps_malloc(size_t n){return std::malloc(n);}
'''


def main():
    compiler = shutil.which('g++')
    if not compiler:
        raise SystemExit('g++ is required for the real-source host fixture')
    with tempfile.TemporaryDirectory(prefix='foot_fast_pipeline_') as temp:
        path = Path(temp)
        (path/'Arduino.h').write_text(ARDUINO, encoding='utf8')
        (path/'pipeline.cpp').write_text(fixture(), encoding='utf8')
        binary = path/'pipeline.exe'
        command = [compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
            '-DFOOT_LOW_LATENCY_V20=1', '-DFOOT_SUPPORT_ASSIST=1', '-DFOOT_SUPPORT_LOCAL=1',
            '-DWALKING_RETURN_ENERGY=1', '-DFOOT_IMAGE_ROWS_TEST_READS=1',
            '-I'+str(path), '-I'+str(ROOT/'src'), str(path/'pipeline.cpp'),
            str(ROOT/'src/white_marker_tracker.cpp'), str(ROOT/'src/foot_angle_estimator.cpp'),
            '-o', str(binary)]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
        subprocess.run(command+['-DFOOT_SPEED_V22=1'],check=True)
        subprocess.run([str(binary)],check=True)
        subprocess.run(command+['-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1'],check=True)
        subprocess.run([str(binary)],check=True)
        subprocess.run(command+['-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_RECEIPT_AUDIT=1'],check=True)
        subprocess.run([str(binary)],check=True)


if __name__ == '__main__':
    main()
