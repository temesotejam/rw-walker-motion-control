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
#ifdef __cplusplus
}
#endif
