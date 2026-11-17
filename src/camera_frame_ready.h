#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t early_frames;
    uint32_t vsync_frames;
    uint32_t queue_drops;
    uint32_t last_full_age_us;
} rw_camera_frame_ready_stats_t;
rw_camera_frame_ready_stats_t rw_camera_frame_ready_stats(void);
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
// Independent boot-lifetime counters; all delays are ISR/task elapsed time,
// not sensor exposure latency. Unsigned counters wrap modulo 2^32.
typedef struct {
    uint32_t events, eof_events, vsync_events, event_drops, short_frames;
    uint32_t last_short_bytes, last_vsync_delay_us, max_vsync_delay_us;
    uint32_t max_eof_delay_us, events_over_1ms, events_over_4ms;
    uint32_t short_start_delay_us, full_start_delay_us;
    uint32_t max_copy_us, max_offer_us;
} rw_camera_receive_stats_t;
rw_camera_receive_stats_t rw_camera_receive_stats(void);
#endif
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
// Completed queued frames deliberately returned at fresh acquisition entry.
uint32_t rw_camera_native_stale_discard_count(void);
#endif
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
// Bench only: copied bytes are a whole-EOF prefix, not a physical pixel count.
// The untransferred final fragment is unknown and may be shorter than one chunk.
// Times name LCD_CAM VSYNC events; independent GPIO edges establish pin timing.
typedef struct {
    uint32_t enabled, boundaries, published_frames, empty_frames, overflow_frames;
    uint32_t event_queue_drops, last_prefix_bytes, dma_chunk_bytes, frame_capacity_bytes;
    uint64_t last_start_vsync_event_us, last_end_vsync_event_us;
    uint64_t last_eof_event_us, last_capture_start_us;
} rw_camera_geometry_probe_stats_t;
rw_camera_geometry_probe_stats_t rw_camera_geometry_probe_stats(void);
#endif
#ifdef __cplusplus
}
#endif
