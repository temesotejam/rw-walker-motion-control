#pragma once
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
// HAL producer copies only a complete prefix into an independently owned slot.
void rw_camera_overlap_offer(const uint8_t* gray,size_t prefix_bytes,uint64_t frame_us);
// Runs image feature preparation in the calling consumer task, never the ISR.
void rw_camera_overlap_poll(void);
void* rw_camera_take_overlap(uint32_t timeout_ticks);
typedef struct {
  uint32_t offered,busy,replaced,consumed;
} rw_camera_overlap_stats_t;
rw_camera_overlap_stats_t rw_camera_overlap_stats(void);
#ifdef __cplusplus
}
#include "foot_upper_rows.h"
namespace camera_row_overlap {
using Consumer=void(*)(const foot_image_rows::GrayRowView&,uint64_t,uint32_t,void*);
// Initialize once at boot, before the foot consumer starts; storage outlives it.
bool initialize(uint8_t* storage,size_t capacity,Consumer consumer,void* context);
}
#endif
