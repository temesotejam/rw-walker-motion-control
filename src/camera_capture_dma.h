#pragma once
#include <stdint.h>
#include <stdbool.h>
// Split the same internal DMA allocation into four slots. This allows the
// boundary EOF/VSYNC burst without extending the queue past physical slots.
typedef struct {uint32_t half_bytes,slots;bool valid;} rw_capture_dma_layout_t;
static inline rw_capture_dma_layout_t rw_capture_dma_layout(uint32_t total,uint32_t half,uint32_t node,uint32_t frame) {
    rw_capture_dma_layout_t out={0,0,false};
    if(!node || !half || half%2 || (uint64_t)half*2!=total)return out;
    uint32_t next=half/2;
    if(!next || next%node || frame%next)return out;
    out.half_bytes=next;out.slots=total/next;out.valid=out.slots==4;
    return out;
}
