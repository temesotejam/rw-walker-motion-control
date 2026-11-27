#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
// HAL producer copies only a complete prefix into an independently owned slot.
void rw_camera_overlap_offer(const uint8_t* gray,size_t prefix_bytes,uint64_t frame_us);
// Runs image feature preparation in the calling consumer task, never the ISR.
void rw_camera_overlap_poll(void);
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
// Single foot owner brackets acquisition. Producer skips unused preparation.
void rw_camera_overlap_request(bool requested);
#endif
void* rw_camera_take_overlap(uint32_t timeout_ticks);
typedef struct {
  uint32_t offered,busy,replaced,consumed;
} rw_camera_overlap_stats_t;
rw_camera_overlap_stats_t rw_camera_overlap_stats(void);
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
rw_camera_overlap_stats_t rw_camera_lower_overlap_stats(void);
#endif
#ifdef __cplusplus
}
#include "foot_image_rows.h"
namespace camera_row_overlap {
using Consumer=void(*)(const foot_image_rows::GrayRowView&,uint64_t,uint32_t,void*);
// Initialize once at boot, before the foot consumer starts; storage outlives it.
bool initialize(uint8_t* storage,size_t capacity,Consumer consumer,void* context);
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
using LowerConsumer=void(*)(const foot_image_rows::GrayRowView&,uint64_t,uint32_t,int,void*);
// Separate storage must outlive callbacks and must not alias the upper slot.
bool initializeLower(uint8_t* storage,size_t capacity,LowerConsumer consumer,void* context);
#endif
}
#endif
