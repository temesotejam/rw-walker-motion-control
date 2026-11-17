"""Execute the probe HAL loop with actual ISR tagging and variable DMA prefixes.

The synthetic VSYNC schedule verifies bookkeeping/ownership, not sensor timing,
dimensions, or whether a final partial DMA chunk exists on the physical board.
"""
from pathlib import Path
import subprocess
import tempfile
from test_foot_fast_metadata import function

ROOT = Path(__file__).resolve().parents[1]
PREAMBLE = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <sys/time.h>
#include "camera_frame_ready.h"
#define DBG_PIN_SET(x)
#define ESP_LOGE(...)
#define ESP_LOGW(...)
#define ESP_CAMERA_ETS_PRINTF(...)
#define DRAM_STR(x) x
#define IRAM_ATTR
#define pdTRUE 1
#define portMAX_DELAY 0xffffffff
using BaseType_t=int;
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (++*(x))
#define portEXIT_CRITICAL(x) (--*(x))
using cam_event_t=int;
enum {CAM_IN_SUC_EOF_EVENT, CAM_VSYNC_EVENT};
enum {CAM_STATE_IDLE, CAM_STATE_READ_BUF};
struct camera_fb_t {uint8_t* buf; size_t len; timeval timestamp;};
struct cam_frame_t {camera_fb_t fb; uint8_t en;};
'''
DOUBLES = r'''
struct Queue {
    std::deque<camera_fb_t*> frames;
    std::deque<rw_probe_event_t> events;
};
struct cam_obj_t {
    cam_frame_t frames[2]; Queue *event_queue,*frame_buffer_queue;
    uint32_t frame_cnt=2,width=640,height=480,fb_size=307200,recv_size=307200;
    uint32_t dma_buffer_size=30720,dma_node_buffer_size=3840;
    uint32_t dma_half_buffer_size=15360,dma_half_buffer_cnt=2;
    uint32_t dma_bytes_per_item=1,in_bytes_per_pixel=1,fb_bytes_per_pixel=1;
    bool jpeg_mode=false,psram_mode=false;int state=CAM_STATE_IDLE;uint8_t* dma_buffer;
};
static cam_obj_t storage,*cam_obj=&storage;
static Queue events,frames;
struct Event {cam_event_t type;uint64_t isr,dispatch;std::function<void()> before;};
static std::deque<Event> schedule;
static uint64_t clock_us;
static unsigned starts,stops,sends,copies;
static bool running,fail_event_send;
static uint8_t pixels[2][307202],dma[30720];
static camera_fb_t* held;
struct Done {};
static bool ll_cam_stop(cam_obj_t*) {running=false;++stops;return true;}
static bool ll_cam_start(cam_obj_t* cam,int pos) {
    assert(cam->frames[pos].en);running=true;++starts;return true;
}
static void ll_cam_do_vsync(cam_obj_t*) {}
static uint64_t esp_timer_get_time() {return clock_us;}
static int xQueueSendFromISR(Queue* q,void* ptr,BaseType_t*) {
    assert(q==&events);
    if(fail_event_send){fail_event_send=false;return 0;}
    q->events.push_back(*static_cast<rw_probe_event_t*>(ptr));return pdTRUE;
}
void ll_cam_send_event(cam_obj_t*,cam_event_t,BaseType_t*);
static int xQueueReceive(Queue* q,void* out,unsigned) {
    if(q==&events) {
        while(q->events.empty()) {
            if(schedule.empty())throw Done{};
            auto e=schedule.front();schedule.pop_front();
            if(e.before)e.before();
            clock_us=e.isr;BaseType_t woken=0;ll_cam_send_event(cam_obj,e.type,&woken);
            clock_us=e.dispatch;
        }
        *static_cast<rw_probe_event_t*>(out)=q->events.front();q->events.pop_front();
        return pdTRUE;
    }
    if(q->frames.empty())return 0;
    *static_cast<camera_fb_t**>(out)=q->frames.front();q->frames.pop_front();return pdTRUE;
}
static int xQueueSend(Queue* q,void* ptr,unsigned) {
    assert(q==&frames && !running);
    if(!q->frames.empty())return 0;
    auto* fb=*static_cast<camera_fb_t**>(ptr);
    assert(fb->len && fb->len<=storage.fb_size);
    q->frames.push_back(fb);++sends;return pdTRUE;
}
static void xQueueReset(Queue* q) {q->events.clear();}
static size_t ll_cam_memcpy(cam_obj_t*,uint8_t* out,const uint8_t* in,size_t size) {
    assert(running);
    bool in_bounds=false;
    for(auto& p:pixels) {
        auto dst=reinterpret_cast<uintptr_t>(out),base=reinterpret_cast<uintptr_t>(p+1);
        if(dst>=base && dst+size<=base+storage.fb_size)in_bounds=true;
    }
    assert(in_bounds);
    memcpy(out,in,size);++copies;return size;
}
static int cam_verify_jpeg_soi(const uint8_t*,uint32_t) {return 0;}
static void cam_give(camera_fb_t* fb) {
    for(auto& f:storage.frames)if(&f.fb==fb){assert(!f.en);f.en=1;return;}
    assert(false);
}
'''
CASES = r'''
static void reset() {
    storage=cam_obj_t{};schedule.clear();events.events.clear();frames.frames.clear();
    storage.event_queue=&events;storage.frame_buffer_queue=&frames;storage.dma_buffer=dma;
    memset(pixels,0,sizeof(pixels));memset(dma,0x5a,sizeof(dma));
    for(int i=0;i<2;++i) {
        pixels[i][0]=pixels[i][307201]=0xa5;
        storage.frames[i].fb={pixels[i]+1,0,{0,0}};storage.frames[i].en=1;
    }
    clock_us=starts=stops=sends=copies=0;running=fail_event_send=false;held=nullptr;
    rw_early_enabled=false;rw_early_count=rw_vsync_count=rw_queue_drops=rw_last_full_age_us=0;
    rw_probe_enabled=true;rw_probe_event_queue_drops=0;rw_probe_stats={};rw_probe_mux=0;
    assert(rw_geometry_probe_dma_layout());
    assert(storage.dma_half_buffer_size==1280 && storage.dma_node_buffer_size==1280);
    assert(storage.dma_half_buffer_cnt==24 && storage.dma_buffer_size==30720);
}
static void event(cam_event_t type,uint64_t isr,std::function<void()> before={},uint64_t lag=0) {
    schedule.push_back({type,isr,isr+lag,before});
}
static void prefix(unsigned count,uint64_t start=1000) {
    event(CAM_VSYNC_EVENT,start);
    for(unsigned i=0;i<count;++i)event(CAM_IN_SUC_EOF_EVENT,start+(i+1)*100,[i] {
        memset(dma+(i%24)*1280,(i%250)+1,1280);
    });
}
static void run() {try{cam_task(nullptr);}catch(Done&) {}}
static void canaries() {
    for(auto& p:pixels)assert(p[0]==0xa5 && p[307201]==0xa5);
    assert(!rw_probe_mux);
}
int main() {
    // Includes both native geometries: 59*1280=75520 and 118*1280=151040.
    for(unsigned chunks:{1u,15u,21u,59u,60u,118u,240u}) {
        reset();prefix(chunks);
        const uint64_t end=1000+(chunks+1)*100;
        event(CAM_VSYNC_EVENT,end,[=]{assert(!sends && copies==chunks);},37);
        run();assert(sends==1 && copies==chunks && starts==2);
        auto* fb=frames.frames.front();assert(fb->len==chunks*1280);
        assert(fb->timestamp.tv_usec==1000 && fb->timestamp.tv_sec==0);
        for(unsigned i=0;i<chunks;++i)for(unsigned j=0;j<1280;++j)
            assert(fb->buf[i*1280+j]==(i%250)+1);
        for(size_t i=fb->len;i<307200;++i)assert(fb->buf[i]==0);
        const auto s=rw_camera_geometry_probe_stats();
        assert(s.enabled && s.boundaries==1 && s.published_frames==1);
        assert(!s.empty_frames && !s.overflow_frames && !s.event_queue_drops);
        assert(s.last_prefix_bytes==chunks*1280 && s.dma_chunk_bytes==1280 && s.frame_capacity_bytes==307200);
        assert(s.last_start_vsync_event_us==1000 && s.last_end_vsync_event_us==end);
        assert(s.last_eof_event_us==1000+chunks*100 && s.last_capture_start_us==1000);
        assert(!rw_early_count && rw_vsync_count==1);canaries();
    }
    // An uncopied fragment in the next DMA slot must not be invented or padded.
    reset();prefix(15);event(CAM_VSYNC_EVENT,3000,[] {memset(dma+15*1280,0xee,721);});
    run();assert(frames.frames.front()->len==19200 && copies==15);canaries();
    // No EOF (including a sub-chunk physical image) is an empty prefix, not a frame.
    reset();event(CAM_VSYNC_EVENT,1000);event(CAM_VSYNC_EVENT,2000);run();
    assert(!sends && !copies && starts==2);
    assert(rw_camera_geometry_probe_stats().empty_frames==1);canaries();
    // An overrun invalidates the entire capacity-sized prefix; later EOFs are ignored.
    reset();prefix(240);event(CAM_IN_SUC_EOF_EVENT,25200);event(CAM_IN_SUC_EOF_EVENT,25300);
    event(CAM_VSYNC_EVENT,30000,[] {assert(!running && copies==240 && !sends);});
    for(unsigned i=1;i<=5;++i)event(CAM_IN_SUC_EOF_EVENT,30000+i*100);
    event(CAM_VSYNC_EVENT,31000);run();
    auto s=rw_camera_geometry_probe_stats();
    assert(copies==245 && sends==1 && s.overflow_frames==1 && s.boundaries==2);
    assert(frames.frames.front()->len==6400 && s.last_start_vsync_event_us==30000);canaries();
    // Queue replacement returns only the old queued frame, never a borrowed buffer.
    reset();prefix(5);event(CAM_VSYNC_EVENT,2000);
    for(unsigned i=1;i<=5;++i)event(CAM_IN_SUC_EOF_EVENT,2000+i*100);
    event(CAM_VSYNC_EVENT,3000);run();
    assert(sends==2 && rw_queue_drops==1 && starts==3);
    assert(frames.frames.front()==&storage.frames[1].fb);canaries();
    reset();prefix(5);event(CAM_VSYNC_EVENT,2000);
    event(CAM_IN_SUC_EOF_EVENT,2100,[] {held=frames.frames.front();frames.frames.pop_front();});
    for(unsigned i=2;i<=5;++i)event(CAM_IN_SUC_EOF_EVENT,2000+i*100);
    event(CAM_VSYNC_EVENT,3000);event(CAM_IN_SUC_EOF_EVENT,3100);
    event(CAM_VSYNC_EVENT,4000,[] {assert(starts==2 && copies==10);cam_give(held);});
    run();assert(starts==3 && sends==2 && copies==10 && !rw_queue_drops);canaries();
    // Preserve the ISR timestamp separately from task dispatch, beyond 32-bit time.
    reset();const uint64_t base=(1ULL<<32)+1000;
    event(CAM_VSYNC_EVENT,base,{},71);event(CAM_IN_SUC_EOF_EVENT,base+100,{},53);
    event(CAM_VSYNC_EVENT,base+300,{},97);run();s=rw_camera_geometry_probe_stats();
    assert(s.last_start_vsync_event_us==base && s.last_end_vsync_event_us==base+300);
    assert(s.last_eof_event_us==base+100 && s.last_capture_start_us==base+71);
    // ISR queue loss is visible and does not publish the abandoned prefix.
    reset();event(CAM_VSYNC_EVENT,1000);event(CAM_IN_SUC_EOF_EVENT,1100);
    event(CAM_IN_SUC_EOF_EVENT,1200,[] {fail_event_send=true;});
    event(CAM_VSYNC_EVENT,2000);event(CAM_IN_SUC_EOF_EVENT,2100);event(CAM_VSYNC_EVENT,3000);
    run();s=rw_camera_geometry_probe_stats();
    assert(s.event_queue_drops==1 && sends==1 && copies==2);
    assert(s.last_prefix_bytes==1280 && s.last_start_vsync_event_us==2000);canaries();
    // Probe disabled: a short image still fails the legacy full-size check.
    reset();rw_probe_enabled=false;prefix(5);event(CAM_VSYNC_EVENT,2000);run();
    assert(!sends && !rw_probe_stats.boundaries && !rw_vsync_count);
    // The isolated DMA layout refuses unexpected capacity/format/alignment.
    for(unsigned field=0;field<10;++field) {
        reset();
        switch(field) {
        case 0:storage.width=320;break;case 1:storage.height=240;break;
        case 2:storage.fb_size=76800;break;case 3:storage.recv_size=76800;break;
        case 4:storage.psram_mode=true;break;case 5:storage.jpeg_mode=true;break;
        case 6:storage.in_bytes_per_pixel=2;break;case 7:storage.fb_bytes_per_pixel=2;break;
        case 8:storage.dma_buffer_size=30719;break;case 9:storage.dma_bytes_per_item=2;break;
        }
        assert(!rw_geometry_probe_dma_layout());
    }
    reset();rw_probe_enabled=false;storage.width=1;storage.dma_half_buffer_size=123;
    assert(rw_geometry_probe_dma_layout() && storage.dma_half_buffer_size==123);
    std::cout<<"PASS actual geometry probe HAL: variable prefixes, quantization, empty/overflow rejection, ISR timestamps, ownership, queue loss, isolated DMA layout\n";
}
'''


def main():
    source = (ROOT/'third_party/esp32_camera_f0bb429/cam_hal.c').read_text(encoding='utf8')
    tag_start = source.index('typedef struct {cam_event_t type; uint64_t isr_us;} rw_probe_event_t;')
    tag = source[tag_start:source.index('#endif', tag_start)]
    helpers = source[source.index('static bool rw_early_enabled'):source.index('//Copy fram')]
    code = PREAMBLE + tag + DOUBLES
    code += function(source, 'void IRAM_ATTR ll_cam_send_event(') + '\n'
    code += function(source, 'static bool cam_get_next_frame(') + '\n'
    code += function(source, 'static bool cam_start_frame(') + '\n' + helpers
    code += function(source, 'static void cam_task(') + '\n' + CASES
    # Ensure the tested hook is wired into actual configuration and queue sizing.
    config = function(source, 'esp_err_t cam_config(')
    assert 'rw_early_enabled = false;' in config
    assert 'frame_size == FRAMESIZE_VGA' in config and 'sizeof(rw_probe_event_t)' in config
    assert 'rw_geometry_probe_dma_layout()' in function(source, 'static esp_err_t cam_dma_config(')
    with tempfile.TemporaryDirectory(prefix='camera_geometry_probe_') as folder:
        cpp=Path(folder)/'test.cpp';exe=Path(folder)/'test.exe';cpp.write_text(code,encoding='utf8')
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                        '-Wno-unused-parameter','-Wno-sign-compare','-DCAMERA_GEOMETRY_PROBE_V25=1',
                        '-I'+str(ROOT/'src'),str(cpp),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)


if __name__ == '__main__':
    main()
