// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <stdalign.h>
#include "esp_heap_caps.h"
#include "ll_cam.h"
#include "cam_hal.h"
#include "../../src/camera_frame_ready.h"
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
#include "../../src/camera_row_overlap.h"
#endif
#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
#include "../../src/camera_capture_dma.h"
#endif
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25 && defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
#error "Native walking capture and the variable-length geometry probe are separate profiles"
#endif

#if (ESP_IDF_VERSION_MAJOR == 3) && (ESP_IDF_VERSION_MINOR == 3)
#include "rom/ets_sys.h"
#else
#include "esp_timer.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/ets_sys.h"  // will be removed in idf v5.0
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/ets_sys.h"
#endif
#endif // ESP_IDF_VERSION_MAJOR
#define ESP_CAMERA_ETS_PRINTF ets_printf

#if CONFIG_CAMERA_TASK_STACK_SIZE
#define CAM_TASK_STACK             CONFIG_CAMERA_TASK_STACK_SIZE
#else
#define CAM_TASK_STACK             (2*1024)
#endif

static const char *TAG = "cam_hal";
static cam_obj_t *cam_obj = NULL;
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
typedef struct {cam_event_t type; uint32_t isr_us;} rw_receive_event_t;
static rw_camera_receive_stats_t rw_receive;
static uint32_t rw_receive_start_delay_us;
// Each field has one writer (ISR drops, otherwise camera task).
static void rw_receive_max(uint32_t *value, uint32_t sample) {
    if (sample > __atomic_load_n(value, __ATOMIC_RELAXED))
        __atomic_store_n(value, sample, __ATOMIC_RELAXED);
}
rw_camera_receive_stats_t rw_camera_receive_stats(void) {
    rw_camera_receive_stats_t s;
#define RW_RX_LOAD(field) s.field = __atomic_load_n(&rw_receive.field, __ATOMIC_RELAXED)
    RW_RX_LOAD(events); RW_RX_LOAD(eof_events); RW_RX_LOAD(vsync_events);
    RW_RX_LOAD(event_drops); RW_RX_LOAD(short_frames); RW_RX_LOAD(last_short_bytes);
    RW_RX_LOAD(last_vsync_delay_us); RW_RX_LOAD(max_vsync_delay_us);
    RW_RX_LOAD(max_eof_delay_us); RW_RX_LOAD(events_over_1ms); RW_RX_LOAD(events_over_4ms);
    RW_RX_LOAD(short_start_delay_us); RW_RX_LOAD(full_start_delay_us);
    RW_RX_LOAD(max_copy_us); RW_RX_LOAD(max_offer_us);
#undef RW_RX_LOAD
    return s;
}
#endif
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
typedef struct {cam_event_t type; uint64_t isr_us;} rw_probe_event_t;
static uint32_t rw_probe_event_queue_drops;
#endif

static const uint32_t JPEG_SOI_MARKER = 0xFFD8FF;  // written in little-endian for esp32
static const uint16_t JPEG_EOI_MARKER = 0xD9FF;  // written in little-endian for esp32

static int cam_verify_jpeg_soi(const uint8_t *inbuf, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        if (memcmp(&inbuf[i], &JPEG_SOI_MARKER, 3) == 0) {
            //ESP_LOGW(TAG, "SOI: %d", (int) i);
            return i;
        }
    }
    ESP_LOGW(TAG, "NO-SOI");
    return -1;
}

static int cam_verify_jpeg_eoi(const uint8_t *inbuf, uint32_t length)
{
    int offset = -1;
    uint8_t *dptr = (uint8_t *)inbuf + length - 2;
    while (dptr > inbuf) {
        if (memcmp(dptr, &JPEG_EOI_MARKER, 2) == 0) {
            offset = dptr - inbuf;
            //ESP_LOGW(TAG, "EOI: %d", length - (offset + 2));
            return offset;
        }
        dptr--;
    }
    return -1;
}

static bool cam_get_next_frame(int * frame_pos)
{
    if(!cam_obj->frames[*frame_pos].en){
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (cam_obj->frames[x].en) {
                *frame_pos = x;
                return true;
            }
        }
    } else {
        return true;
    }
    return false;
}

static bool cam_start_frame(int * frame_pos)
{
    if (cam_get_next_frame(frame_pos)) {
        if(ll_cam_start(cam_obj, *frame_pos)){
            // Vsync the frame manually
            ll_cam_do_vsync(cam_obj);
            uint64_t us = (uint64_t)esp_timer_get_time();
            cam_obj->frames[*frame_pos].fb.timestamp.tv_sec = us / 1000000UL;
            cam_obj->frames[*frame_pos].fb.timestamp.tv_usec = us % 1000000UL;
            return true;
        }
    }
    return false;
}

void IRAM_ATTR ll_cam_send_event(cam_obj_t *cam, cam_event_t cam_event, BaseType_t * HPTaskAwoken)
{
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
    const rw_receive_event_t tagged_event = {cam_event, (uint32_t)esp_timer_get_time()};
    void *event_data = (void *)&tagged_event;
#elif defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
    const rw_probe_event_t tagged_event = {cam_event, (uint64_t)esp_timer_get_time()};
    void *event_data = (void *)&tagged_event;
#else
    void *event_data = (void *)&cam_event;
#endif
    if (xQueueSendFromISR(cam->event_queue, event_data, HPTaskAwoken) != pdTRUE) {
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
        __atomic_fetch_add(&rw_probe_event_queue_drops, 1, __ATOMIC_RELAXED);
#endif
        ll_cam_stop(cam);
        cam->state = CAM_STATE_IDLE;
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
        // Never format/write console text in the camera ISR. Keep every drop.
        __atomic_fetch_add(&rw_receive.event_drops, 1, __ATOMIC_RELAXED);
#else
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: EV-%s-OVF\r\n"), cam_event==CAM_IN_SUC_EOF_EVENT ? DRAM_STR("EOF") : DRAM_STR("VSYNC"));
#endif
    }
}

// Walker v21: publish only a complete QVGA Y8 frame; no sensor/timestamp changes.
static bool rw_early_enabled = false;
static uint32_t rw_early_count, rw_vsync_count, rw_queue_drops, rw_last_full_age_us;
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
static bool rw_native_enabled;
static uint32_t rw_native_stale_discards;
uint32_t rw_camera_native_stale_discard_count(void)
{
    return __atomic_load_n(&rw_native_stale_discards, __ATOMIC_RELAXED);
}
#endif

static bool rw_early_y8_geometry(void)
{
    if (!rw_early_enabled || cam_obj->jpeg_mode || cam_obj->psram_mode ||
        cam_obj->width != 320 || cam_obj->in_bytes_per_pixel != 1 ||
        cam_obj->fb_bytes_per_pixel != 1) return false;
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
    if (rw_native_enabled)
        return cam_obj->height == 236 && cam_obj->fb_size == 75520 && cam_obj->recv_size == 75520;
#endif
    return cam_obj->height == 240 && cam_obj->fb_size == 76800;
}

#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
static bool rw_native_dma_layout(void)
{
    if (!rw_native_enabled) return true;
    // ll_cam_dma_sizes uses the actual 236 rows, yielding 4-row nodes/EOFs.
    // 59 whole chunks complete a frame. Retain the former 30720-byte allocation
    // as 24 physical slots, rather than the SDK's rounded 32000-byte buffer.
    if (!rw_early_y8_geometry() || cam_obj->dma_bytes_per_item != 1 ||
        cam_obj->dma_node_buffer_size != 1280 || cam_obj->dma_half_buffer_size != 1280 ||
        cam_obj->dma_buffer_size < 30720 || cam_obj->dma_buffer_size % 1280) return false;
    cam_obj->dma_buffer_size = 30720;
    cam_obj->dma_node_buffer_size = 1280;
    cam_obj->dma_half_buffer_size = 1280;
    cam_obj->dma_half_buffer_cnt = 24;
    return true;
}
#endif
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
static bool rw_probe_enabled;
static portMUX_TYPE rw_probe_mux = portMUX_INITIALIZER_UNLOCKED;
static rw_camera_geometry_probe_stats_t rw_probe_stats;

rw_camera_geometry_probe_stats_t rw_camera_geometry_probe_stats(void)
{
    portENTER_CRITICAL(&rw_probe_mux);
    rw_camera_geometry_probe_stats_t s = rw_probe_stats;
    portEXIT_CRITICAL(&rw_probe_mux);
    s.event_queue_drops = __atomic_load_n(&rw_probe_event_queue_drops, __ATOMIC_RELAXED);
    return s;
}

static bool rw_geometry_probe_dma_layout(void)
{
    if (!rw_probe_enabled) return true;
    // 1280 divides VGA/QVGA/QQVGA and native 320/640 x236 Y8 lengths.
    // Keep the allocation: 24 physical slots on the pinned 30720-byte layout.
    const uint32_t chunk = 1280;
    if (cam_obj->jpeg_mode || cam_obj->psram_mode || cam_obj->width != 640 ||
        cam_obj->height != 480 || cam_obj->fb_size != 307200 ||
        cam_obj->recv_size != 307200 || cam_obj->dma_bytes_per_item != 1 ||
        cam_obj->in_bytes_per_pixel != 1 || cam_obj->fb_bytes_per_pixel != 1 ||
        cam_obj->dma_buffer_size < 2 * chunk || cam_obj->dma_buffer_size % chunk) return false;
    cam_obj->dma_node_buffer_size = chunk;
    cam_obj->dma_half_buffer_size = chunk;
    cam_obj->dma_half_buffer_cnt = cam_obj->dma_buffer_size / chunk;
    portENTER_CRITICAL(&rw_probe_mux);
    rw_probe_stats.enabled = 1;
    rw_probe_stats.dma_chunk_bytes = chunk;
    rw_probe_stats.frame_capacity_bytes = cam_obj->fb_size;
    portEXIT_CRITICAL(&rw_probe_mux);
    return true;
}

static bool rw_geometry_probe_boundary(camera_fb_t *fb, uint64_t start_event_us,
                                       uint64_t end_event_us, uint64_t eof_event_us, bool overflow)
{
    // A capacity overrun invalidates the entire frame, including a full prefix.
    const bool accepted = !overflow && fb->len && fb->len <= cam_obj->fb_size;
    portENTER_CRITICAL(&rw_probe_mux);
    ++rw_probe_stats.boundaries;
    if (overflow) ++rw_probe_stats.overflow_frames;
    else if (!fb->len) ++rw_probe_stats.empty_frames;
    rw_probe_stats.last_prefix_bytes = fb->len;
    rw_probe_stats.last_start_vsync_event_us = start_event_us;
    rw_probe_stats.last_end_vsync_event_us = end_event_us;
    rw_probe_stats.last_eof_event_us = eof_event_us;
    rw_probe_stats.last_capture_start_us = (uint64_t)fb->timestamp.tv_sec * 1000000ULL + fb->timestamp.tv_usec;
    portEXIT_CRITICAL(&rw_probe_mux);
    return accepted;
}
#endif

rw_camera_frame_ready_stats_t rw_camera_frame_ready_stats(void)
{
    rw_camera_frame_ready_stats_t s;
    s.early_frames = __atomic_load_n(&rw_early_count, __ATOMIC_RELAXED);
    s.vsync_frames = __atomic_load_n(&rw_vsync_count, __ATOMIC_RELAXED);
    s.queue_drops = __atomic_load_n(&rw_queue_drops, __ATOMIC_RELAXED);
    s.last_full_age_us = __atomic_load_n(&rw_last_full_age_us, __ATOMIC_RELAXED);
    return s; // Independent boot-lifetime counters, not a transactional snapshot.
}

static bool rw_publish_frame(int frame_pos, camera_fb_t *fb)
{
    cam_obj->frames[frame_pos].en = 0;
    if (xQueueSend(cam_obj->frame_buffer_queue, &fb, 0) == pdTRUE) return true;
    camera_fb_t *old = NULL;
    if (xQueueReceive(cam_obj->frame_buffer_queue, &old, 0) == pdTRUE) {
        bool sent = xQueueSend(cam_obj->frame_buffer_queue, &fb, 0) == pdTRUE;
        if (!sent) cam_obj->frames[frame_pos].en = 1;
        cam_give(old); // Only reclaim the queued frame, never the consumer's frame.
        __atomic_fetch_add(&rw_queue_drops, sent ? 1 : 2, __ATOMIC_RELAXED);
        return sent;
    }
    cam_obj->frames[frame_pos].en = 1;
    __atomic_fetch_add(&rw_queue_drops, 1, __ATOMIC_RELAXED);
    return false;
}

//Copy fram from DMA dma_buffer to fram dma_buffer
static void cam_task(void *arg)
{
    int cnt = 0;
    int frame_pos = 0;
    cam_obj->state = CAM_STATE_IDLE;
    cam_event_t cam_event = 0;
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
    rw_receive_event_t receive_event = {0, 0};
#endif
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
    rw_probe_event_t tagged_event = {0, 0};
    uint64_t probe_start_event_us = 0, probe_eof_event_us = 0;
    bool probe_overflow = false;
#endif

    xQueueReset(cam_obj->event_queue);

    while (1) {
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
        xQueueReceive(cam_obj->event_queue, (void *)&receive_event, portMAX_DELAY);
        cam_event = receive_event.type;
        const uint32_t delay_us = (uint32_t)esp_timer_get_time() - receive_event.isr_us;
        __atomic_fetch_add(&rw_receive.events, 1, __ATOMIC_RELAXED);
        if (delay_us >= 1000) __atomic_fetch_add(&rw_receive.events_over_1ms, 1, __ATOMIC_RELAXED);
        if (delay_us >= 4000) __atomic_fetch_add(&rw_receive.events_over_4ms, 1, __ATOMIC_RELAXED);
        if (cam_event == CAM_VSYNC_EVENT) {
            __atomic_fetch_add(&rw_receive.vsync_events, 1, __ATOMIC_RELAXED);
            __atomic_store_n(&rw_receive.last_vsync_delay_us, delay_us, __ATOMIC_RELAXED);
            rw_receive_max(&rw_receive.max_vsync_delay_us, delay_us);
        } else {
            __atomic_fetch_add(&rw_receive.eof_events, 1, __ATOMIC_RELAXED);
            rw_receive_max(&rw_receive.max_eof_delay_us, delay_us);
        }
#elif defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
        xQueueReceive(cam_obj->event_queue, (void *)&tagged_event, portMAX_DELAY);
        cam_event = tagged_event.type;
#else
        xQueueReceive(cam_obj->event_queue, (void *)&cam_event, portMAX_DELAY);
#endif
        DBG_PIN_SET(1);
        switch (cam_obj->state) {

            case CAM_STATE_IDLE: {
                if (cam_event == CAM_VSYNC_EVENT) {
                    //DBG_PIN_SET(1);
                    if(cam_start_frame(&frame_pos)){
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
                        rw_receive_start_delay_us = delay_us;
#endif
                        cam_obj->frames[frame_pos].fb.len = 0;
                        cam_obj->state = CAM_STATE_READ_BUF;
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
                        probe_start_event_us = tagged_event.isr_us;
                        probe_eof_event_us = 0; probe_overflow = false;
#endif
                    }
                    cnt = 0;
                }
            }
            break;

            case CAM_STATE_READ_BUF: {
                camera_fb_t * frame_buffer_event = &cam_obj->frames[frame_pos].fb;
                size_t pixels_per_dma = (cam_obj->dma_half_buffer_size * cam_obj->fb_bytes_per_pixel) / (cam_obj->dma_bytes_per_item * cam_obj->in_bytes_per_pixel);

                if (cam_event == CAM_IN_SUC_EOF_EVENT) {
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
                    if (rw_probe_enabled && probe_overflow) {DBG_PIN_SET(0); continue;}
#endif
                    if(!cam_obj->psram_mode){
                        if (cam_obj->fb_size < (frame_buffer_event->len + pixels_per_dma)) {
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
                            if (rw_probe_enabled) probe_overflow = true;
#endif
                            ESP_LOGW(TAG, "FB-OVF");
                            ll_cam_stop(cam_obj);
                            DBG_PIN_SET(0);
                            continue;
                        }
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
                        const uint32_t copy_start_us = (uint32_t)esp_timer_get_time();
#endif
                        frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                            &frame_buffer_event->buf[frame_buffer_event->len],
                            &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                            cam_obj->dma_half_buffer_size);
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
                        rw_receive_max(&rw_receive.max_copy_us, (uint32_t)esp_timer_get_time() - copy_start_us);
#endif
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
                        if (rw_probe_enabled) probe_eof_event_us = tagged_event.isr_us;
#endif
                    }
                    //Check for JPEG SOI in the first buffer. stop if not found
                    if (cam_obj->jpeg_mode && cnt == 0 && cam_verify_jpeg_soi(frame_buffer_event->buf, frame_buffer_event->len) != 0) {
                        ll_cam_stop(cam_obj);
                        cam_obj->state = CAM_STATE_IDLE;
                    }
                    cnt++;
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
                    if(rw_early_y8_geometry() &&
                       frame_buffer_event->len<=cam_obj->fb_size) {
                        const uint64_t frame_us=(uint64_t)frame_buffer_event->timestamp.tv_sec*1000000ULL+
                            frame_buffer_event->timestamp.tv_usec;
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
                        const uint32_t offer_start_us = (uint32_t)esp_timer_get_time();
#endif
                        rw_camera_overlap_offer(frame_buffer_event->buf,frame_buffer_event->len,frame_us);
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
                        rw_receive_max(&rw_receive.max_offer_us, (uint32_t)esp_timer_get_time() - offer_start_us);
#endif
                    }
#endif
                    if (rw_early_y8_geometry() && frame_buffer_event->len == cam_obj->fb_size) {
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
                        __atomic_store_n(&rw_receive.full_start_delay_us, rw_receive_start_delay_us, __ATOMIC_RELAXED);
#endif
                        // Stop DMA before giving ownership to the consumer. VSYNC remains
                        // enabled; start the next frame only at its physical boundary.
                        ll_cam_stop(cam_obj);
                        uint64_t start_us = (uint64_t)frame_buffer_event->timestamp.tv_sec * 1000000ULL +
                                            frame_buffer_event->timestamp.tv_usec;
                        __atomic_store_n(&rw_last_full_age_us, (uint32_t)((uint64_t)esp_timer_get_time() - start_us), __ATOMIC_RELAXED);
                        if (rw_publish_frame(frame_pos, frame_buffer_event))
                            __atomic_fetch_add(&rw_early_count, 1, __ATOMIC_RELAXED);
                        cam_obj->state = CAM_STATE_IDLE;
                        cnt = 0; // Queued late EOF events are ignored in IDLE.
                    }

                } else if (cam_event == CAM_VSYNC_EVENT) {
                    //DBG_PIN_SET(1);
                    ll_cam_stop(cam_obj);

                    if (cnt || !cam_obj->jpeg_mode || cam_obj->psram_mode) {
                        if (cam_obj->jpeg_mode) {
                            if (!cam_obj->psram_mode) {
                                if (cam_obj->fb_size < (frame_buffer_event->len + pixels_per_dma)) {
                                    ESP_LOGW(TAG, "FB-OVF");
                                    cnt--;
                                } else {
                                    frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                                        &frame_buffer_event->buf[frame_buffer_event->len],
                                        &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                                        cam_obj->dma_half_buffer_size);
                                }
                            }
                            cnt++;
                        }

                        cam_obj->frames[frame_pos].en = 0;

                        if (cam_obj->psram_mode) {
                            if (cam_obj->jpeg_mode) {
                                frame_buffer_event->len = cnt * cam_obj->dma_half_buffer_size;
                            } else {
                                frame_buffer_event->len = cam_obj->recv_size;
                            }
                        } else if (!cam_obj->jpeg_mode) {
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
                            if (rw_probe_enabled) {
                                // Never fill or invent the uncopied tail. A probe
                                // frame is only the received whole-chunk prefix.
                                if (!rw_geometry_probe_boundary(frame_buffer_event, probe_start_event_us,
                                        tagged_event.isr_us, probe_eof_event_us, probe_overflow))
                                    cam_obj->frames[frame_pos].en = 1;
                            } else
#endif
                            if (frame_buffer_event->len != cam_obj->fb_size) {
                                cam_obj->frames[frame_pos].en = 1;
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
                                __atomic_fetch_add(&rw_receive.short_frames, 1, __ATOMIC_RELAXED);
                                __atomic_store_n(&rw_receive.last_short_bytes, frame_buffer_event->len, __ATOMIC_RELAXED);
                                // Start delay belongs to the preceding frame, not this VSYNC.
                                __atomic_store_n(&rw_receive.short_start_delay_us, rw_receive_start_delay_us, __ATOMIC_RELAXED);
#else
                                ESP_LOGE(TAG, "FB-SIZE: %u != %u", frame_buffer_event->len, (unsigned) cam_obj->fb_size);
#endif
                            }
                        }
                        // Same latest-frame queue ownership policy for the fallback path.
                        if (!cam_obj->frames[frame_pos].en && rw_publish_frame(frame_pos, frame_buffer_event)) {
                            __atomic_fetch_add(&rw_vsync_count, 1, __ATOMIC_RELAXED);
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
                            if (rw_probe_enabled) {
                                portENTER_CRITICAL(&rw_probe_mux);
                                ++rw_probe_stats.published_frames;
                                portEXIT_CRITICAL(&rw_probe_mux);
                            }
#endif
                        }
                    }

                    if(!cam_start_frame(&frame_pos)){
                        cam_obj->state = CAM_STATE_IDLE;
                    } else {
                        cam_obj->frames[frame_pos].fb.len = 0;
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
                        rw_receive_start_delay_us = delay_us;
#endif
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
                        probe_start_event_us = tagged_event.isr_us;
                        probe_eof_event_us = 0; probe_overflow = false;
#endif
                    }
                    cnt = 0;
                }
            }
            break;
        }
        DBG_PIN_SET(0);
    }
}

static lldesc_t * allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t * buffer)
{
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL) {
        return dma;
    }

    for (int x = 0; x < count; x++) {
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }
    return dma;
}

static esp_err_t cam_dma_config(const camera_config_t *config)
{
    bool ret = ll_cam_dma_sizes(cam_obj);
    if (0 == ret) {
        return ESP_FAIL;
    }
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
    if (!rw_geometry_probe_dma_layout()) return ESP_FAIL;
#endif
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
    if (!rw_native_dma_layout()) return ESP_FAIL;
#endif

#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
    if(!cam_obj->jpeg_mode && !cam_obj->psram_mode && cam_obj->width==320 && cam_obj->height==240) {
        const rw_capture_dma_layout_t layout=rw_capture_dma_layout(cam_obj->dma_buffer_size,
            cam_obj->dma_half_buffer_size,cam_obj->dma_node_buffer_size,cam_obj->recv_size);
        if(!layout.valid)return ESP_FAIL;
        cam_obj->dma_half_buffer_size=layout.half_bytes;
        cam_obj->dma_half_buffer_cnt=layout.slots;
    }
#endif

    cam_obj->dma_node_cnt = (cam_obj->dma_buffer_size) / cam_obj->dma_node_buffer_size; // Number of DMA nodes
    cam_obj->frame_copy_cnt = cam_obj->recv_size / cam_obj->dma_half_buffer_size; // Number of interrupted copies, ping-pong copy

    ESP_LOGI(TAG, "buffer_size: %d, half_buffer_size: %d, node_buffer_size: %d, node_cnt: %d, total_cnt: %d",
             (int) cam_obj->dma_buffer_size, (int) cam_obj->dma_half_buffer_size, (int) cam_obj->dma_node_buffer_size,
             (int) cam_obj->dma_node_cnt, (int) cam_obj->frame_copy_cnt);

    cam_obj->dma_buffer = NULL;
    cam_obj->dma = NULL;

    cam_obj->frames = (cam_frame_t *)heap_caps_aligned_calloc(alignof(cam_frame_t), 1, cam_obj->frame_cnt * sizeof(cam_frame_t), MALLOC_CAP_DEFAULT);
    CAM_CHECK(cam_obj->frames != NULL, "frames malloc failed", ESP_FAIL);

    uint8_t dma_align = 0;
    size_t fb_size = cam_obj->fb_size;
    if (cam_obj->psram_mode) {
        dma_align = ll_cam_get_dma_align(cam_obj);
        if (cam_obj->fb_size < cam_obj->recv_size) {
            fb_size = cam_obj->recv_size;
        }
    }

    /* Allocate memory for frame buffer */
    size_t alloc_size = fb_size * sizeof(uint8_t) + dma_align;
    uint32_t _caps = MALLOC_CAP_8BIT;
    if (CAMERA_FB_IN_DRAM == config->fb_location) {
        _caps |= MALLOC_CAP_INTERNAL;
    } else {
        _caps |= MALLOC_CAP_SPIRAM;
    }
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].dma = NULL;
        cam_obj->frames[x].fb_offset = 0;
        cam_obj->frames[x].en = 0;
        ESP_LOGI(TAG, "Allocating %d Byte frame buffer in %s", alloc_size, _caps & MALLOC_CAP_SPIRAM ? "PSRAM" : "OnBoard RAM");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
        // In IDF v4.2 and earlier, memory returned by heap_caps_aligned_alloc must be freed using heap_caps_aligned_free.
        // And heap_caps_aligned_free is deprecated on v4.3.
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_aligned_alloc(16, alloc_size, _caps);
#else
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_malloc(alloc_size, _caps);
#endif
        CAM_CHECK(cam_obj->frames[x].fb.buf != NULL, "frame buffer malloc failed", ESP_FAIL);
        if (cam_obj->psram_mode) {
            //align PSRAM buffer. TODO: save the offset so proper address can be freed later
            cam_obj->frames[x].fb_offset = dma_align - ((uint32_t)cam_obj->frames[x].fb.buf & (dma_align - 1));
            cam_obj->frames[x].fb.buf += cam_obj->frames[x].fb_offset;
            ESP_LOGI(TAG, "Frame[%d]: Offset: %u, Addr: 0x%08X", x, cam_obj->frames[x].fb_offset, (unsigned) cam_obj->frames[x].fb.buf);
            cam_obj->frames[x].dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->frames[x].fb.buf);
            CAM_CHECK(cam_obj->frames[x].dma != NULL, "frame dma malloc failed", ESP_FAIL);
        }
        cam_obj->frames[x].en = 1;
    }

    if (!cam_obj->psram_mode) {
        cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
        if(NULL == cam_obj->dma_buffer) {
            ESP_LOGE(TAG,"%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__,
                     (int) cam_obj->dma_buffer_size, (int) heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            return ESP_FAIL;
        }

        cam_obj->dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->dma_buffer);
        CAM_CHECK(cam_obj->dma != NULL, "dma malloc failed", ESP_FAIL);
    }

    return ESP_OK;
}

esp_err_t cam_init(const camera_config_t *config)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

    cam_obj->swap_data = 0;
    cam_obj->vsync_pin = config->pin_vsync;
    cam_obj->vsync_invert = true;

    ll_cam_set_pin(cam_obj, config);
    ret = ll_cam_config(cam_obj, config);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam initialize failed", err);

#if CAMERA_DBG_PIN_ENABLE
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[DBG_PIN_NUM], PIN_FUNC_GPIO);
    gpio_set_direction(DBG_PIN_NUM, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DBG_PIN_NUM, GPIO_FLOATING);
#endif

    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;

err:
    free(cam_obj);
    cam_obj = NULL;
    return ESP_FAIL;
}

esp_err_t cam_config(const camera_config_t *config, framesize_t frame_size, uint16_t sensor_pid)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    ret = ll_cam_set_sample_mode(cam_obj, (pixformat_t)config->pixel_format, config->xclk_freq_hz, sensor_pid);

    cam_obj->jpeg_mode = config->pixel_format == PIXFORMAT_JPEG;
#if CONFIG_IDF_TARGET_ESP32
    cam_obj->psram_mode = false;
#else
    cam_obj->psram_mode = (config->xclk_freq_hz == 16000000);
#endif
    rw_early_enabled = sensor_pid == GC0308_PID && config->pixel_format == PIXFORMAT_GRAYSCALE &&
                       frame_size == FRAMESIZE_QVGA && config->xclk_freq_hz == 20000000;
    cam_obj->frame_cnt = config->fb_count;
    cam_obj->width = resolution[frame_size].width;
    cam_obj->height = resolution[frame_size].height;
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
    // SDK selects QVGA, but this isolated sensor profile emits 320 x236 Y8.
    // Describe the real wire geometry before sizes, DMA and buffers are made.
    rw_native_enabled = rw_early_enabled && !cam_obj->jpeg_mode && !cam_obj->psram_mode &&
        cam_obj->in_bytes_per_pixel == 1 && cam_obj->fb_bytes_per_pixel == 1;
    if (rw_native_enabled) cam_obj->height = 236;
#endif
#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
    // This bench reserves a VGA-sized buffer while sensor output may shrink.
    // No early EOF publication or foot-overlap callback is permitted here.
    rw_early_enabled = false;
    rw_probe_enabled = sensor_pid == GC0308_PID && config->pixel_format == PIXFORMAT_GRAYSCALE &&
        frame_size == FRAMESIZE_VGA && config->xclk_freq_hz == 20000000 && !cam_obj->psram_mode;
#endif

    if(cam_obj->jpeg_mode){
        cam_obj->recv_size = cam_obj->width * cam_obj->height / 5;
        cam_obj->fb_size = cam_obj->recv_size;
    } else {
        cam_obj->recv_size = cam_obj->width * cam_obj->height * cam_obj->in_bytes_per_pixel;
        cam_obj->fb_size = cam_obj->width * cam_obj->height * cam_obj->fb_bytes_per_pixel;
    }

    ret = cam_dma_config(config);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam_dma_config failed", err);

    size_t queue_size = cam_obj->dma_half_buffer_cnt - 1;
    if (queue_size == 0) {
        queue_size = 1;
    }
    cam_obj->event_queue = xQueueCreate(queue_size,
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
        sizeof(rw_receive_event_t)
#elif defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
        sizeof(rw_probe_event_t)
#else
        sizeof(cam_event_t)
#endif
    );
    CAM_CHECK_GOTO(cam_obj->event_queue != NULL, "event_queue create failed", err);

    size_t frame_buffer_queue_len = cam_obj->frame_cnt;
    if (config->grab_mode == CAMERA_GRAB_LATEST && cam_obj->frame_cnt > 1) {
        frame_buffer_queue_len = cam_obj->frame_cnt - 1;
    }
    cam_obj->frame_buffer_queue = xQueueCreate(frame_buffer_queue_len, sizeof(camera_fb_t*));
    CAM_CHECK_GOTO(cam_obj->frame_buffer_queue != NULL, "frame_buffer_queue create failed", err);

    ret = ll_cam_init_isr(cam_obj);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam intr alloc failed", err);


#if CONFIG_CAMERA_CORE0
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 0);
#elif CONFIG_CAMERA_CORE1
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 1);
#else
    xTaskCreate(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle);
#endif

    ESP_LOGI(TAG, "cam config ok");
    return ESP_OK;

err:
    cam_deinit();
    return ESP_FAIL;
}

esp_err_t cam_deinit(void)
{
    if (!cam_obj) {
        return ESP_FAIL;
    }

    cam_stop();
    if (cam_obj->task_handle) {
        vTaskDelete(cam_obj->task_handle);
    }
    if (cam_obj->event_queue) {
        vQueueDelete(cam_obj->event_queue);
    }
    if (cam_obj->frame_buffer_queue) {
        vQueueDelete(cam_obj->frame_buffer_queue);
    }

    ll_cam_deinit(cam_obj);

    if (cam_obj->dma) {
        free(cam_obj->dma);
    }
    if (cam_obj->dma_buffer) {
        free(cam_obj->dma_buffer);
    }
    if (cam_obj->frames) {
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            free(cam_obj->frames[x].fb.buf - cam_obj->frames[x].fb_offset);
            if (cam_obj->frames[x].dma) {
                free(cam_obj->frames[x].dma);
            }
        }
        free(cam_obj->frames);
    }

    free(cam_obj);
    cam_obj = NULL;
    return ESP_OK;
}

void cam_stop(void)
{
    ll_cam_vsync_intr_enable(cam_obj, false);
    ll_cam_stop(cam_obj);
}

void cam_start(void)
{
    ll_cam_vsync_intr_enable(cam_obj, true);
}

// Queue cleanup only: an empty queue must not reset DMA. Capture owner only,
// with reception stopped and no borrowed frame.
void rw_camera_discard_queued(void)
{
    if (!cam_obj || !cam_obj->frame_buffer_queue) return;
    camera_fb_t *frame = NULL;
    while (xQueueReceive(cam_obj->frame_buffer_queue, &frame, 0) == pdTRUE) {
        if (frame) cam_give(frame);
    }
}

camera_fb_t *cam_take(TickType_t timeout)
{
    camera_fb_t *dma_buffer = NULL;
    TickType_t start = xTaskGetTickCount();
    xQueueReceive(cam_obj->frame_buffer_queue, (void *)&dma_buffer, timeout);
#if CONFIG_IDF_TARGET_ESP32S3
    // Currently (22.01.2024) there is a bug in ESP-IDF v5.2, that causes
    // GDMA to fall into a strange state if it is running while WiFi STA is connecting.
    // This code tries to reset GDMA if frame is not received, to try and help with
    // this case. It is possible to have some side effects too, though none come to mind
    if (!dma_buffer) {
        ll_cam_dma_reset(cam_obj);
        xQueueReceive(cam_obj->frame_buffer_queue, (void *)&dma_buffer, timeout);
    }
#endif
    if (dma_buffer) {
        if(cam_obj->jpeg_mode){
            // find the end marker for JPEG. Data after that can be discarded
            int offset_e = cam_verify_jpeg_eoi(dma_buffer->buf, dma_buffer->len);
            if (offset_e >= 0) {
                // adjust buffer length
                dma_buffer->len = offset_e + sizeof(JPEG_EOI_MARKER);
                return dma_buffer;
            } else {
                ESP_LOGW(TAG, "NO-EOI");
                cam_give(dma_buffer);
                TickType_t ticks_spent = xTaskGetTickCount() - start;
                if (ticks_spent >= timeout) {
                    return NULL; /* We are out of time */
                }
                return cam_take(timeout - ticks_spent);//recurse!!!!
            }
        } else if(cam_obj->psram_mode && cam_obj->in_bytes_per_pixel != cam_obj->fb_bytes_per_pixel){
            //currently this is used only for YUV to GRAYSCALE
            dma_buffer->len = ll_cam_memcpy(cam_obj, dma_buffer->buf, dma_buffer->buf, dma_buffer->len);
        }
        return dma_buffer;
    } else {
        ESP_LOGW(TAG, "Failed to get the frame on time!");
// #if CONFIG_IDF_TARGET_ESP32S3
//         ll_cam_dma_print_state(cam_obj);
// #endif
    }
    return NULL;
}

#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
void* rw_camera_take_overlap(uint32_t timeout_ticks)
{
    if(!cam_obj)return NULL;
    // Used only by the continuous grayscale consumer. Keep manual/boot cam_take.
    if(!rw_early_y8_geometry())return cam_take((TickType_t)timeout_ticks);
    camera_fb_t* fb=NULL;
    const TickType_t start=xTaskGetTickCount();
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
    // The worker can run more slowly than this sensor. Drop at most the one
    // already completed LATEST-queue frame, then await a fresh completion.
    // Direct dequeue owns this buffer; it cannot return a borrowed frame.
    if(rw_native_enabled && xQueueReceive(cam_obj->frame_buffer_queue,&fb,0)==pdTRUE) {
        cam_give(fb);
        __atomic_fetch_add(&rw_native_stale_discards,1,__ATOMIC_RELAXED);
        fb=NULL;
    }
#endif
    for(;;) {
        // A completed frame takes precedence over speculative preparation.
        if(xQueueReceive(cam_obj->frame_buffer_queue,&fb,0)==pdTRUE)return fb;
        const TickType_t elapsed=xTaskGetTickCount()-start;
        if(elapsed>=(TickType_t)timeout_ticks)break;
        rw_camera_overlap_poll();
        if(xQueueReceive(cam_obj->frame_buffer_queue,&fb,0)==pdTRUE)return fb;
        if(xTaskGetTickCount()-start>=(TickType_t)timeout_ticks)break;
        if(xQueueReceive(cam_obj->frame_buffer_queue,&fb,1)==pdTRUE)return fb;
    }
    if(xQueueReceive(cam_obj->frame_buffer_queue,&fb,0)==pdTRUE)return fb;
#if CONFIG_IDF_TARGET_ESP32S3
    // Match the legacy single recovery at a FULL timeout, never per short poll.
    ll_cam_dma_reset(cam_obj);
    if(xQueueReceive(cam_obj->frame_buffer_queue,&fb,(TickType_t)timeout_ticks)==pdTRUE)return fb;
#endif
    return NULL;
}
#endif

void cam_give(camera_fb_t *dma_buffer)
{
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        if (&cam_obj->frames[x].fb == dma_buffer) {
            cam_obj->frames[x].en = 1;
            break;
        }
    }
}

void cam_give_all(void) {
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].en = 1;
    }
}
