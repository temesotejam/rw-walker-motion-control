#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
#include "camera_row_overlap.h"
#include "esp_timer.h"
namespace {
enum : uint32_t {Free,Writing,Ready,Reading};
uint32_t state=Free,initialized=0;
foot_image_rows::UpperRowCache rows;
camera_row_overlap::Consumer consumer=nullptr;
void* context=nullptr;
uint64_t stamp=0,last_offered=0;
uint32_t copy_us=0,offered=0,busy=0,replaced=0,consumed=0;
}
bool camera_row_overlap::initialize(uint8_t* storage,size_t capacity,Consumer fn,void* ctx) {
  if(!storage || capacity<foot_image_rows::kUpperBytes || !fn ||
     __atomic_load_n(&initialized,__ATOMIC_ACQUIRE))return false;
  rows=foot_image_rows::UpperRowCache(storage,capacity);consumer=fn;context=ctx;
  __atomic_store_n(&initialized,1,__ATOMIC_RELEASE);return true;
}
extern "C" void rw_camera_overlap_offer(const uint8_t* gray,size_t prefix_bytes,uint64_t frame_us) {
  if(!__atomic_load_n(&initialized,__ATOMIC_ACQUIRE) || !gray || !frame_us ||
     prefix_bytes<foot_image_rows::kUpperPrefixBytes || prefix_bytes>foot_image_rows::kFrameBytes ||
     frame_us==last_offered)return;
  // Single HAL producer owns last_offered. A busy slot skips this frame.
  last_offered=frame_us;
  uint32_t expected=__atomic_load_n(&state,__ATOMIC_ACQUIRE);
  const bool replacing=expected==Ready;
  if((expected!=Free && expected!=Ready) ||
     !__atomic_compare_exchange_n(&state,&expected,Writing,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE)) {
    __atomic_fetch_add(&busy,1,__ATOMIC_RELAXED);return;
  }
  const uint64_t begin=esp_timer_get_time();
  if(!rows.capture(gray,prefix_bytes)) {__atomic_store_n(&state,Free,__ATOMIC_RELEASE);return;}
  stamp=frame_us;copy_us=static_cast<uint32_t>(esp_timer_get_time()-begin);
  __atomic_fetch_add(&offered,1,__ATOMIC_RELAXED);
  if(replacing)__atomic_fetch_add(&replaced,1,__ATOMIC_RELAXED);
  __atomic_store_n(&state,Ready,__ATOMIC_RELEASE);
}
extern "C" void rw_camera_overlap_poll(void) {
  uint32_t expected=Ready;
  if(!__atomic_compare_exchange_n(&state,&expected,Reading,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return;
  consumer(rows.view(),stamp,copy_us,context);
  __atomic_fetch_add(&consumed,1,__ATOMIC_RELAXED);
  __atomic_store_n(&state,Free,__ATOMIC_RELEASE);
}
extern "C" rw_camera_overlap_stats_t rw_camera_overlap_stats(void) {
  return {__atomic_load_n(&offered,__ATOMIC_RELAXED),__atomic_load_n(&busy,__ATOMIC_RELAXED),
          __atomic_load_n(&replaced,__ATOMIC_RELAXED),__atomic_load_n(&consumed,__ATOMIC_RELAXED)};
}
#endif
