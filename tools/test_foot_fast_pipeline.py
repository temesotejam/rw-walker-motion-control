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
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
#include "camera_native_skip_profile.h"
#endif
#include "foot_zero.h"
#include "white_marker_tracker.h"
#include "foot_angle_estimator.h"
#include "foot_phase_prediction.h"
#include "foot_local_prediction.h"
#include "foot_control_range.h"
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
#include "camera_row_overlap.h"
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
static bool mock_overlap_requested=false;
extern "C" void rw_camera_overlap_request(bool requested){
 assert(requested!=mock_overlap_requested);mock_overlap_requested=requested;
}
#endif
namespace camera_row_overlap {
static Consumer mock_consumer=nullptr;
static void* mock_context=nullptr;
static uint8_t* mock_storage=nullptr;
static size_t mock_capacity=0;
bool initialize(uint8_t* storage,size_t capacity,Consumer consumer,void* context){
 assert(storage && capacity==foot_image_rows::kUpperBytes && consumer && context);
 // Each test observer represents a separate device lifetime.
 mock_storage=storage;mock_capacity=capacity;mock_consumer=consumer;mock_context=context;return true;
}
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
static LowerConsumer mock_lower_consumer=nullptr;
static void* mock_lower_context=nullptr;
static uint8_t* mock_lower_storage=nullptr;
static size_t mock_lower_capacity=0;
bool initializeLower(uint8_t* storage,size_t capacity,LowerConsumer consumer,void* context){
 assert(storage && capacity==foot_image_rows::kLowerBytes && consumer && context);
 mock_lower_storage=storage;mock_lower_capacity=capacity;
 mock_lower_consumer=consumer;mock_lower_context=context;return true;
}
#endif
}
#endif
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
static int created_foot_core=-1,created_foot_priority=-1;
static int xTaskCreatePinnedToCore(void(*)(void*),const char* name,uint32_t stack,void*,int priority,TaskHandle_t* task,int core){
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
 constexpr uint32_t expected_stack=12288;
#else
 constexpr uint32_t expected_stack=10240;
#endif
 constexpr int expected_core=0,expected_priority=1;
 assert(std::string(name)=="foot_observer" && stack==expected_stack && priority==expected_priority && core==expected_core);
 created_foot_core=core;created_foot_priority=priority;
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
    methods += '\n#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24\n' + function(
        observer, 'void FootObserver::prepareUpper(') + '\n#endif\n'
    methods += '\n#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26\n' + function(
        observer, 'void FootObserver::prepareLower(') + '\n#endif\n'
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
 Image out(foot_image_rows::kRawFrameBytes,30);
 for(int row=-10;row<=10;++row)for(int dx=-10;dx<=10;++dx){
  out[foot_image_rows::rawYForLogical(66+row)*320+right+dx]=230;
  out[foot_image_rows::rawYForLogical(160+row)*320+left+dx]=230;
 }
 return out;
}
enum class Invalid {None,NullFrame,Short,NullBuffer,FutureTimestamp,OldTimestamp,FullLogicalLength,WrongHeight};
struct Plan {
 Image original=image();ControlSnapshot before,after;
 Invalid invalid=Invalid::None;uint32_t wait_us=2000;bool stop=false;
 bool early=true;int64_t early_stamp_delta=0;Image early_image;
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
 bool lower=true,lower_abandoned_first=false,lower_invalid_view=false;
 int64_t lower_stamp_delta=0;Image lower_image;
 std::vector<int> lower_stages{174,190,206,222};
#endif
 uint64_t force_delivered_us=0; // Focused boundary cases; zero keeps ordinary clocks.
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
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
static bool override_stamp=false;
static uint64_t full_stamp=0;
#endif
ControlSnapshot RunControlWorker::snapshot()const {
 const auto& p=plans.at(plan_index);const bool before=context_calls++==0;
 if(!before && borrowed && p.invalid==Invalid::WrongHeight)frame.height=240;
 if(!before && p.force_delivered_us)host_clock=p.force_delivered_us-1;
 return before?p.before:p.after;
}
static camera_fb_t* cam_take(TickType_t timeout){
 if(timeout==0){assert(!live && !borrowed);++flush_takes;return nullptr;}
 assert(live && !borrowed);++takes;
 auto& p=plans.at(plan_index);host_clock+=p.wait_us;
 foot_image_rows::readCounts()={};
 if(p.invalid==Invalid::NullFrame)return nullptr;
 buffer=p.original;if(p.invalid==Invalid::FullLogicalLength)buffer.resize(320*240);
 frame={};frame.buf=p.invalid==Invalid::NullBuffer?nullptr:buffer.data();
 frame.len=p.invalid==Invalid::Short?123:buffer.size();
 uint64_t stamp=host_clock-50000;
 if(p.force_delivered_us)stamp=p.force_delivered_us-50000;
 if(p.invalid==Invalid::FutureTimestamp)stamp=host_clock+5000;
 if(p.invalid==Invalid::OldTimestamp)stamp=host_clock-600000;
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
 if(override_stamp)stamp=full_stamp;
#endif
 frame.timestamp.tv_sec=stamp/1000000;frame.timestamp.tv_usec=stamp%1000000;
 borrowed=true;return &frame;
}
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
extern "C" void* rw_camera_take_overlap(uint32_t timeout){
 const auto& p=plans.at(plan_index);
 full_stamp=host_clock+p.wait_us-50000;
 if(p.force_delivered_us)full_stamp=p.force_delivered_us-50000;
 if(p.invalid==Invalid::FutureTimestamp)full_stamp=host_clock+p.wait_us+5000;
 if(p.invalid==Invalid::OldTimestamp)full_stamp=host_clock+p.wait_us-600000;
 if(p.early){
  const auto& pixels=p.early_image.empty()?p.original:p.early_image;
  foot_image_rows::UpperRowCache upper(camera_row_overlap::mock_storage,camera_row_overlap::mock_capacity);
  assert(upper.capture(pixels.data(),foot_image_rows::kUpperPrefixBytes));
  const auto before=foot_image_rows::readCounts().packed_rows;
  camera_row_overlap::mock_consumer(upper.view(),full_stamp+p.early_stamp_delta,1,camera_row_overlap::mock_context);
  assert(foot_image_rows::readCounts().packed_rows>before);
  // Original early pixels may be released before the full image arrives.
  std::memset(camera_row_overlap::mock_storage,0,camera_row_overlap::mock_capacity);
 }
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
 if(p.lower){
  auto stage=[&](const Image& pixels,uint64_t stamp,int max_y,bool invalid){
   foot_image_rows::LowerRowCache lower(camera_row_overlap::mock_lower_storage,camera_row_overlap::mock_lower_capacity);
   const size_t prefix=(foot_image_rows::rawYForLogical(max_y)+1)*320;
   assert(lower.capture(pixels.data(),prefix));
   camera_row_overlap::mock_lower_consumer(invalid?foot_image_rows::GrayRowView{}:lower.view(),
       stamp,1,max_y,camera_row_overlap::mock_lower_context);
   std::memset(camera_row_overlap::mock_lower_storage,0,camera_row_overlap::mock_lower_capacity);
  };
  // Abandoned features are deliberately inconsistent with the accepted frame.
  if(p.lower_abandoned_first)stage(image(50,50),full_stamp-1,222,false);
  const auto& pixels=p.lower_image.empty()?p.original:p.lower_image;
  for(int max_y:p.lower_stages)stage(pixels,full_stamp+p.lower_stamp_delta,max_y,p.lower_invalid_view);
 }
#endif
 override_stamp=true;auto* result=cam_take(timeout);override_stamp=false;return result;
}
#endif
static void cam_give(camera_fb_t* fb){
 assert(borrowed && fb==&frame);borrowed=false;++gives;
 if(enforce_rows){
  const auto& r=foot_image_rows::readCounts();
  const auto invalid=plans.at(plan_index).invalid;
  const bool cache_valid=invalid!=Invalid::Short && invalid!=Invalid::NullBuffer &&
      invalid!=Invalid::FullLogicalLength && invalid!=Invalid::WrongHeight;
  bool needs_cache=cache_valid;
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
  const bool valid_stamp=full_stamp && full_stamp<=host_clock && host_clock-full_stamp<500000;
  if(valid_stamp && current->upper_frame_us_==full_stamp && current->lower_frame_us_==full_stamp &&
      current->lower_scan_.complete())needs_cache=false;
#endif
  assert(r.capture_rows==(needs_cache?foot_image_rows::kRequiredRows:0));
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
 for(unsigned y=0;y<120;++y)for(unsigned x=0;x<160;++x){
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  const int raw_y=int(y)*2-2;
  out[y*160+x]=raw_y>=0 && raw_y<236?source[size_t(raw_y)*320+x*2]:0;
#else
  out[y*160+x]=source[(y*2)*320+x*2];
#endif
 }
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
 assert(created_foot_core==FootObserver::kTaskCore && created_foot_priority==FootObserver::kTaskPriority);
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
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
 std::free(observer.upper_storage_);
#endif
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
 std::free(observer.lower_storage_);
#endif
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
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
 size_t preview_index=0;uint64_t preview_delivery=0;
#endif
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
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  if(i<70){
   if(!i || f.delivered_us-preview_delivery>=200000){preview_index=i;preview_delivery=f.delivered_us;}
   assert(b.preview_ok && b.preview==thumbnail(source[preview_index].original));
   detectionEqual(b.info.frame,poisoned.history[preview_index].snapshot.latest,false);
   assert(b.info.frame.ready_us==poisoned.history[preview_index].snapshot.latest.ready_us);
   assert(b.info.mekf_attitude.sentinel==73 && b.info.imu_ok);
  }else assert(b.preview_ok && b.info.frame.sequence==preview_index+1 && b.preview==thumbnail(source[preview_index].original));
#else
  if(i<70){assert(b.preview_ok && b.preview==thumbnail(source[i].original));
   detectionEqual(b.info.frame,f,false);assert(b.info.mekf_attitude.sentinel==73 && b.info.imu_ok);
   assert(equal(b.info.right.center_x_px,f.right_x) && equal(b.info.left.center_x_px,f.left_x));}
  else {assert(b.preview_ok && b.info.frame.sequence==70 && b.preview==thumbnail(source[69].original));}
#endif
  if(i>=70 && i<78){assert(f.zero_ready && f.right_valid && f.left_valid);
   assert(f.right_templates==17 && f.left_templates==17);
   phase_ready|=f.pose_projection.reason==foot_phase_prediction::Reason::Ready;
   local_ready|=f.local_projection.reason==foot_local_prediction::Reason::Ready;}
  if(i>=78 && i<81){assert(!f.frame_valid && !f.row_cache_used && !f.right_valid && !f.left_valid);
   assert(std::isnan(f.right_x) && std::isnan(f.left_x));
   assert(f.right_reason==MarkerDetectionReason::NoFrame && f.left_reason==MarkerDetectionReason::NoFrame);
   assert(b.reads.packed_rows==0 && b.reads.capture_rows==0);}
  else {
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
   assert(b.snapshot.lower_overlap_used && f.upper_overlap_used && !f.row_cache_used);
   assert(b.reads.capture_rows==0 && b.reads.packed_rows==0 && b.reads.full_rows==0);
#else
   assert(f.row_cache_used && b.reads.capture_rows==foot_image_rows::kRequiredRows && b.reads.packed_rows>0 && b.reads.full_rows==0);
#endif
  }
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
static void overlapAdmission(){
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
 std::vector<Plan> plans(9);
 plans[1].early_stamp_delta=1;plans[1].early_image=image(280,80);
 plans[2].invalid=Invalid::Short;plans[3].invalid=Invalid::NullFrame;
 plans[4].invalid=Invalid::NullBuffer;plans[5].invalid=Invalid::FutureTimestamp;
 plans[6].invalid=Invalid::OldTimestamp;plans[7].early=false;
 for(unsigned i=2;i<=6;++i)plans[i].early_image=image(280,80);
 auto ordinary=plans;for(auto& p:ordinary)p.early=false;
 auto baseline=run(ordinary,true),overlapped=run(plans,true);
 assert(overlapped.history.size()==plans.size());
 for(unsigned i=0;i<plans.size();++i){
  const auto& a=baseline.history[i];const auto& b=overlapped.history[i];
  detectionEqual(a.snapshot.latest,b.snapshot.latest,false);
  assert(a.preview==b.preview && a.preview_ok==b.preview_ok);
  assert(a.info.right.success_count==b.info.right.success_count && a.info.right.fail_count==b.info.right.fail_count);
  const bool hit=i==0 || i==8;
  assert(b.snapshot.latest.upper_overlap_used==hit);
  assert((b.snapshot.latest.upper_prepare_us>0)==hit);
 }
 const auto& last=overlapped.history.back().snapshot;
 assert(last.upper_attempts==8 && last.upper_hits==2 && last.upper_misses==7);
 std::cout<<"real v24 observer: matching frame reuse, wrong/invalid/timeout fallback, no speculative state commit PASS\n";
#endif
}
static void nativeGeometryAdmission(){
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
 Plan good,wrong_length,wrong_height,short_frame;
 wrong_length.invalid=Invalid::FullLogicalLength;
 wrong_height.invalid=Invalid::WrongHeight;
 short_frame.invalid=Invalid::Short;
 auto result=run({good,wrong_length,wrong_height,short_frame,good},true);
 assert(result.history.size()==5);
 for(unsigned i=0;i<5;++i){
  const auto& it=result.history[i];const bool valid=i==0 || i==4;
  assert(it.snapshot.latest.frame_valid==valid);
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
  assert(!it.snapshot.latest.row_cache_used && it.snapshot.lower_overlap_used==valid);
#else
  assert(it.snapshot.latest.row_cache_used==valid);
#endif
  assert(it.snapshot.latest.upper_overlap_used==valid && it.preview_ok==valid);
  if(valid){
   assert(it.preview==thumbnail(good.original));
   for(unsigned x=0;x<160;++x)assert(it.preview[x]==0 && it.preview[119*160+x]==0);
   assert(equal(it.snapshot.latest.right_x,203) && equal(it.snapshot.latest.left_x,202));
  }else assert(it.snapshot.latest.right_reason==MarkerDetectionReason::NoFrame);
 }
 const auto& last=result.history.back().snapshot;
 assert(last.frame_failures==3 && last.captured_frames==2 && last.upper_hits==2 && last.upper_misses==3);
 std::cout<<"real v25 observer: native236 mapping, logical240/short rejection, zero-padded bounded preview and identity admission PASS\n";
#endif
}
static void nativePreviewCadence(){
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
 auto at=[](uint64_t stamp,int x){Plan p;p.force_delivered_us=stamp;p.original=image(x,x-1);return p;};
 auto first=at(2000000,203),second=at(2050000,190),invalid=at(2100000,189);
 invalid.invalid=Invalid::Short;
 auto recovery=at(2150000,188),active=at(2400000,180),idle=at(2450000,178);
 active.before.running=active.after.running=true;active.before.state_id=active.after.state_id=3;
 const auto result=run({first,second,invalid,recovery,active,idle},true);
 const auto& history=result.history;
 assert(history.size()==6 && history.back().snapshot.latest.sequence==6);
 assert(history.back().snapshot.captured_frames==5 && history.back().snapshot.frame_failures==1);
 assert(history[0].preview_ok && history[0].info.frame.sequence==1);
 assert(history[1].snapshot.latest.sequence==2 && equal(history[1].snapshot.latest.right_x,190));
 assert(history[1].preview==history[0].preview && history[1].info.frame.sequence==1);
 assert(history[1].info.right.center_x_px==history[0].info.right.center_x_px);
 assert(!history[2].preview_ok && !history[2].snapshot.latest.frame_valid);
 assert(history[3].preview_ok && history[3].info.frame.sequence==4 && history[3].preview==thumbnail(recovery.original));
 assert(history[4].snapshot.latest.sequence==5 && history[4].preview==history[3].preview && history[4].info.frame.sequence==4);
 assert(history[5].preview_ok && history[5].info.frame.sequence==6 && history[5].preview==thumbnail(idle.original));
 for(const auto& h:history)if(h.preview_ok){
  assert(h.info.frame.ready_us>h.info.frame.released_us);
  assert(h.info.frame.ready_us<=h.time);
 }
 for(uint64_t delta:{199999ULL,200000ULL}){
  auto boundary=at(2000000+delta,190);auto pair=run({first,boundary},true);
  const auto& latest=pair.history.back();
  assert(latest.snapshot.latest.sequence==2 && equal(latest.snapshot.latest.right_x,190));
  assert(latest.info.frame.sequence==(delta<200000?1:2));
  assert(latest.preview==thumbnail(delta<200000?first.original:boundary.original));
 }
 std::cout<<"native idle preview5Hz: original pixels/metadata retained, exact200ms boundary, invalid recovery, active preservation, all observations retained PASS\n";
#endif
}
static void lowerOverlapAdmission(){
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
 std::vector<Plan> samples(16);
 samples[1].lower_stamp_delta=1;samples[1].lower_image=image(280,80);
 samples[2].lower_stages={174,190,206}; // Matching partial work finishes after delivery.
 samples[3].lower=false;
 samples[4].invalid=Invalid::Short;samples[5].invalid=Invalid::NullFrame;
 samples[6].invalid=Invalid::NullBuffer;samples[7].invalid=Invalid::FutureTimestamp;
 samples[8].invalid=Invalid::OldTimestamp;samples[9].invalid=Invalid::FullLogicalLength;
 samples[10].invalid=Invalid::WrongHeight;
 for(unsigned i=4;i<=10;++i)samples[i].lower_image=image(280,80);
 samples[11].lower_abandoned_first=true; // Complete wrong frame then reset to matching partial scan.
 samples[12].lower_invalid_view=true;
 samples[13].lower_stages={222}; // Mailbox may coalesce all milestones into a final prefix.
 samples[14].lower_stages={174,174,190,174,206,222}; // Repeats/regressions do not duplicate work.
 samples[15].early=false; // Lower accepted, upper falls back to the full cache.
 auto ordinary=samples;
 for(auto& p:ordinary){p.lower=false;p.early=false;}
 const auto baseline=run(ordinary,true),streamed=run(samples,true);
 for(unsigned i=0;i<samples.size();++i){
  const auto& a=baseline.history[i];const auto& b=streamed.history[i];
  detectionEqual(a.snapshot.latest,b.snapshot.latest,false);
  assert(a.preview_ok==b.preview_ok && a.preview==b.preview);
  if(a.preview_ok){assert(a.info.left.success_count==b.info.left.success_count);
   assert(a.info.left.fail_count==b.info.left.fail_count);}
  const bool hit=i==0 || i==2 || i==11 || i==13 || i==14 || i==15;
  assert(b.snapshot.lower_overlap_used==hit && (b.snapshot.lower_prepare_us>0)==hit);
  const auto& f=b.snapshot.latest;
  const auto early_templates=b.snapshot.lower_templates_before_delivery;
  if(i==2)assert(early_templates>0 && early_templates<17 && f.left_templates==17 && b.reads.packed_rows>0);
  else assert(early_templates==(hit?17:0));
  const bool skipped=f.frame_valid && f.upper_overlap_used && hit && early_templates==17;
  assert(f.row_cache_used==(f.frame_valid && !skipped));
  assert(b.reads.capture_rows==(f.row_cache_used?foot_image_rows::kRequiredRows:0));
 }
 const auto& last=streamed.history.back().snapshot;
 assert(last.lower_hits==6 && last.lower_misses==10);
 assert(last.lower_attempts==59);
 assert(sizeof(WhiteMarker1DTracker::PreparedScan)==2132);
 std::cout<<"real v26 observer: staged/missing/repeated prefix, frame reset/admission, owned features, all-row copy skip/fallback, no speculative state commits PASS\n";
#endif
}
int main(){fullPipeline();edgeCases();manualOneShot();idleReceipt();overlapAdmission();nativeGeometryAdmission();nativePreviewCadence();lowerOverlapAdmission();}
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
        subprocess.run(command+['-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1'],check=True)
        subprocess.run([str(binary)],check=True)
        subprocess.run(command+['-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1'],check=True)
        subprocess.run([str(binary)],check=True)
        subprocess.run(command+['-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1','-DFOOT_NATIVE_STREAM_V26=1'],check=True)
        subprocess.run([str(binary)],check=True)
        subprocess.run(command+['-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1','-DFOOT_NATIVE_STREAM_V26=1','-DFOOT_EXPORT_WEBFIX=1','-DFOOT_COPY_DELTA_V27=1'],check=True)
        subprocess.run([str(binary)],check=True)


if __name__ == '__main__':
    main()
