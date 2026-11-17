#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
#include "camera_row_overlap.h"
#include "esp_timer.h"
namespace {
enum : uint32_t {Free,Writing,Ready,Reading};
uint32_t state=Free,initialized=0;
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
uint32_t requested=0;
#endif
foot_image_rows::UpperRowCache rows;
camera_row_overlap::Consumer consumer=nullptr;
void* context=nullptr;
uint64_t stamp=0,last_offered=0;
uint32_t copy_us=0,offered=0,busy=0,replaced=0,consumed=0;
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
static_assert(foot_image_rows::kRawFrameHeight==236,"Native streaming requires native sensor geometry");
uint32_t lower_state=Free,lower_initialized=0,polling=0;
foot_image_rows::LowerRowCache lower_rows;
camera_row_overlap::LowerConsumer lower_consumer=nullptr;
void* lower_context=nullptr;
uint64_t lower_stamp=0,lower_latest_frame=0;
int lower_max_y=-1,lower_last_offered_y=-1;
uint32_t lower_copy_us=0,lower_offered=0,lower_busy=0,lower_replaced=0,lower_consumed=0;
void offerLower(const uint8_t* gray,size_t prefix_bytes,uint64_t frame_us) {
  if(!__atomic_load_n(&lower_initialized,__ATOMIC_ACQUIRE) || !gray || !frame_us ||
      prefix_bytes>foot_image_rows::kRawFrameBytes || frame_us<lower_latest_frame)return;
  // One HAL producer owns the progress cursor; it never publishes older frame
  // pixels under a new stamp. Busy callbacks do not consume an unoffered stage.
  if(frame_us>lower_latest_frame) {lower_latest_frame=frame_us;lower_last_offered_y=-1;}
  const int milestone=foot_image_rows::lowerMilestoneForPrefix(prefix_bytes);
  if(milestone<0 || milestone<=lower_last_offered_y)return;
  uint32_t expected=__atomic_load_n(&lower_state,__ATOMIC_ACQUIRE);
  const bool replacing=expected==Ready;
  if((expected!=Free && expected!=Ready) ||
      !__atomic_compare_exchange_n(&lower_state,&expected,Writing,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE)) {
    __atomic_fetch_add(&lower_busy,1,__ATOMIC_RELAXED);return;
  }
  const uint64_t begin=esp_timer_get_time();
  if(!
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
     lower_rows.extend(gray,foot_image_rows::lowerPrefixThrough(milestone),frame_us)
#else
     lower_rows.capture(gray,foot_image_rows::lowerPrefixThrough(milestone))
#endif
    ) {
    __atomic_store_n(&lower_state,Free,__ATOMIC_RELEASE);return;
  }
  lower_stamp=frame_us;lower_max_y=lower_rows.maxLogicalY();
  lower_copy_us=static_cast<uint32_t>(esp_timer_get_time()-begin);
  lower_last_offered_y=milestone;
  __atomic_fetch_add(&lower_offered,1,__ATOMIC_RELAXED);
  if(replacing)__atomic_fetch_add(&lower_replaced,1,__ATOMIC_RELAXED);
  __atomic_store_n(&lower_state,Ready,__ATOMIC_RELEASE);
}
void pollLower() {
  uint32_t expected=Ready;
  if(!__atomic_compare_exchange_n(&lower_state,&expected,Reading,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return;
  lower_consumer(lower_rows.view(),lower_stamp,lower_copy_us,lower_max_y,lower_context);
  __atomic_fetch_add(&lower_consumed,1,__ATOMIC_RELAXED);
  __atomic_store_n(&lower_state,Free,__ATOMIC_RELEASE);
}
#endif
}
bool camera_row_overlap::initialize(uint8_t* storage,size_t capacity,Consumer fn,void* ctx) {
  if(!storage || capacity<foot_image_rows::kUpperBytes || !fn ||
     __atomic_load_n(&initialized,__ATOMIC_ACQUIRE))return false;
  rows=foot_image_rows::UpperRowCache(storage,capacity);consumer=fn;context=ctx;
  __atomic_store_n(&initialized,1,__ATOMIC_RELEASE);return true;
}
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
bool camera_row_overlap::initializeLower(uint8_t* storage,size_t capacity,LowerConsumer fn,void* ctx) {
  if(!storage || capacity<foot_image_rows::kLowerBytes || !fn ||
     __atomic_load_n(&lower_initialized,__ATOMIC_ACQUIRE))return false;
  lower_rows=foot_image_rows::LowerRowCache(storage,capacity);lower_consumer=fn;lower_context=ctx;
  __atomic_store_n(&lower_initialized,1,__ATOMIC_RELEASE);return true;
}
#endif
extern "C" void rw_camera_overlap_offer(const uint8_t* gray,size_t prefix_bytes,uint64_t frame_us) {
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
#if !defined(FOOT_OVERLAP_ALWAYS_V29) || !FOOT_OVERLAP_ALWAYS_V29
  if(!__atomic_load_n(&requested,__ATOMIC_ACQUIRE))return;
#endif
#endif
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
  // Every DMA chunk can advance the lower stage, even after this frame's
  // one upper offer. This is called by the HAL task, never its ISR.
  offerLower(gray,prefix_bytes,frame_us);
#endif
  if(!__atomic_load_n(&initialized,__ATOMIC_ACQUIRE) || !gray || !frame_us ||
     prefix_bytes<foot_image_rows::kUpperPrefixBytes || prefix_bytes>foot_image_rows::kRawFrameBytes ||
     frame_us==last_offered)return;
  // Single HAL producer owns last_offered. Preserve the old profile's
  // skip-on-busy behavior; V26 retries on a later DMA chunk after ownership
  // is released, just like its progressive lower stage.
#if !defined(FOOT_NATIVE_STREAM_V26) || !FOOT_NATIVE_STREAM_V26
  last_offered=frame_us;
#endif
  uint32_t expected=__atomic_load_n(&state,__ATOMIC_ACQUIRE);
  const bool replacing=expected==Ready;
  if((expected!=Free && expected!=Ready) ||
     !__atomic_compare_exchange_n(&state,&expected,Writing,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE)) {
    __atomic_fetch_add(&busy,1,__ATOMIC_RELAXED);return;
  }
  const uint64_t begin=esp_timer_get_time();
  if(!rows.capture(gray,prefix_bytes)) {__atomic_store_n(&state,Free,__ATOMIC_RELEASE);return;}
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
  last_offered=frame_us;
#endif
  stamp=frame_us;copy_us=static_cast<uint32_t>(esp_timer_get_time()-begin);
  __atomic_fetch_add(&offered,1,__ATOMIC_RELAXED);
  if(replacing)__atomic_fetch_add(&replaced,1,__ATOMIC_RELAXED);
  __atomic_store_n(&state,Ready,__ATOMIC_RELEASE);
}
extern "C" void rw_camera_overlap_poll(void) {
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
  // The foot task reuses one scratch buffer for both callbacks. Reject nested
  // polls, including a second consumer, so callbacks cannot overwrite scratch.
  if(__atomic_exchange_n(&polling,1,__ATOMIC_ACQUIRE))return;
#endif
  uint32_t expected=Ready;
  if(__atomic_compare_exchange_n(&state,&expected,Reading,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE)) {
    consumer(rows.view(),stamp,copy_us,context);
    __atomic_fetch_add(&consumed,1,__ATOMIC_RELAXED);
    __atomic_store_n(&state,Free,__ATOMIC_RELEASE);
  }
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
  pollLower();
  __atomic_store_n(&polling,0,__ATOMIC_RELEASE);
#endif
}
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
extern "C" void rw_camera_overlap_request(bool value) {
  // Does not revoke a slot already owned by producer/consumer. Frame stamps
  // still gate every prepared result; no raw camera buffer ownership changes.
  __atomic_store_n(&requested,value?1U:0U,__ATOMIC_RELEASE);
}
#endif
extern "C" rw_camera_overlap_stats_t rw_camera_overlap_stats(void) {
  return {__atomic_load_n(&offered,__ATOMIC_RELAXED),__atomic_load_n(&busy,__ATOMIC_RELAXED),
          __atomic_load_n(&replaced,__ATOMIC_RELAXED),__atomic_load_n(&consumed,__ATOMIC_RELAXED)};
}
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
extern "C" rw_camera_overlap_stats_t rw_camera_lower_overlap_stats(void) {
  return {__atomic_load_n(&lower_offered,__ATOMIC_RELAXED),__atomic_load_n(&lower_busy,__ATOMIC_RELAXED),
          __atomic_load_n(&lower_replaced,__ATOMIC_RELAXED),__atomic_load_n(&lower_consumed,__ATOMIC_RELAXED)};
}
#endif
#endif
