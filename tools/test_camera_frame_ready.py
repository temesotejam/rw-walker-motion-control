"""Execute the production HAL event loop against DMA/queue ownership doubles.

Timing below is a synthetic event schedule, not a measurement of the sensor.
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
#include <stdexcept>
#include <sys/time.h>
#include "camera_frame_ready.h"
#define DBG_PIN_SET(x)
#define ESP_LOGE(...)
#define ESP_LOGW(...)
#define pdTRUE 1
#define portMAX_DELAY 0xffffffff
typedef int cam_event_t;
enum {CAM_IN_SUC_EOF_EVENT, CAM_VSYNC_EVENT};
enum {CAM_STATE_IDLE, CAM_STATE_READ_BUF};
struct camera_fb_t {uint8_t* buf; size_t len; timeval timestamp;};
struct cam_frame_t {camera_fb_t fb; uint8_t en;};
struct Queue {std::deque<camera_fb_t*> frames;};
struct cam_obj_t {
 cam_frame_t frames[2]; Queue *event_queue,*frame_buffer_queue;
 uint32_t frame_cnt=2, width=320,height=240,fb_size=76800,recv_size=76800;
 uint32_t dma_half_buffer_size=7680,dma_half_buffer_cnt=2;
 uint32_t dma_bytes_per_item=1,in_bytes_per_pixel=1,fb_bytes_per_pixel=1;
 bool jpeg_mode=false,psram_mode=false; int state=CAM_STATE_IDLE; uint8_t* dma_buffer;
};
static cam_obj_t storage, *cam_obj=&storage;
static Queue events,frames;
struct Event {cam_event_t type; uint64_t time; std::function<void()> before;};
static std::deque<Event> schedule;
static uint64_t clock_us=0;
static unsigned stops=0,starts=0,sends=0,copies=0;
static int fail_send=0; static bool fail_pop=false;
static bool running=false;
static uint8_t pixels[2][76800], dma[30720];
struct Done {};
static int xQueueReceive(Queue* q, void* out, unsigned) {
 if(q==&events){
  if(schedule.empty())throw Done{};
  auto e=schedule.front();schedule.pop_front();clock_us=e.time;
  if(e.before)e.before();
  *static_cast<cam_event_t*>(out)=e.type;
  return pdTRUE;
 }
 if(fail_pop){fail_pop=false;return 0;}
 if(q->frames.empty())return 0;
 *static_cast<camera_fb_t**>(out)=q->frames.front();q->frames.pop_front();return pdTRUE;
}
static int xQueueSend(Queue* q,void* ptr,unsigned){
 assert(q==&frames && !running); // Producer must stop DMA before publication.
 if(fail_send){--fail_send;return 0;}
 if(!q->frames.empty())return 0;
 auto fb=*static_cast<camera_fb_t**>(ptr);
 assert(fb->len==76800 || cam_obj->jpeg_mode || cam_obj->width!=320);
 q->frames.push_back(fb);++sends;return pdTRUE;
}
static void xQueueReset(Queue*){}
static bool ll_cam_stop(cam_obj_t*){running=false;++stops;return true;}
static bool ll_cam_start(cam_obj_t* cam,int pos){
 assert(cam->frames[pos].en);running=true;++starts;return true;
}
static void ll_cam_do_vsync(cam_obj_t*){}
static uint64_t esp_timer_get_time(){return clock_us;}
static size_t ll_cam_memcpy(cam_obj_t*,uint8_t* out,const uint8_t* in,size_t size){
 assert(running);memcpy(out,in,size);++copies;return size;
}
static int cam_verify_jpeg_soi(const uint8_t*,uint32_t){return 0;}
static void cam_give(camera_fb_t* fb){
 for(auto& f:cam_obj->frames)if(&f.fb==fb){assert(!f.en);f.en=1;return;}
 assert(false);
}
'''
CASES = r'''
static void reset(){
 storage=cam_obj_t{};events.frames.clear();frames.frames.clear();schedule.clear();
 storage.event_queue=&events;storage.frame_buffer_queue=&frames;storage.dma_buffer=dma;
 for(int i=0;i<2;++i){storage.frames[i].fb={pixels[i],0,{0,0}};storage.frames[i].en=1;}
 memset(dma,0x5a,sizeof(dma));memset(pixels,0,sizeof(pixels));
 clock_us=stops=starts=sends=copies=0;running=false;fail_send=0;fail_pop=false;
 rw_early_enabled=true;rw_early_count=rw_vsync_count=rw_queue_drops=rw_last_full_age_us=0;
}
static void event(cam_event_t type,uint64_t t,std::function<void()> f={}){schedule.push_back({type,t,f});}
static void full(uint64_t start){
 event(CAM_VSYNC_EVENT,start);
 for(int i=1;i<=10;++i)event(CAM_IN_SUC_EOF_EVENT,start+i*4000);
}
static void run(){try{cam_task(nullptr);}catch(Done&) {}}
int main(){
 // 40ms full readout vs 50ms next VSYNC, synthetic; timestamp must be unchanged.
 for(bool enabled:{false,true}){
  reset();rw_early_enabled=enabled;full(1000);
  event(CAM_VSYNC_EVENT,51000,[=]{assert(sends==(enabled?1u:0u));});run();
  assert(sends==1 && starts==2 && copies==10);
  assert(frames.frames.front()->timestamp.tv_usec==1000);
  for(auto v:pixels[0])assert(v==0x5a);
  auto s=rw_camera_frame_ready_stats();assert(s.early_frames==(enabled?1u:0u));
  assert(s.vsync_frames==(enabled?0u:1u));if(enabled)assert(s.last_full_age_us==40000);
 }
 // No premature/duplicate publication; trailing EOF ignored until VSYNC.
 reset();full(1000);event(CAM_IN_SUC_EOF_EVENT,42000);event(CAM_IN_SUC_EOF_EVENT,43000);run();
 assert(sends==1 && copies==10 && starts==1 && !running && storage.state==CAM_STATE_IDLE);
 // Short frames fail size validation; next complete frame recovers.
 reset();event(CAM_VSYNC_EVENT,1000);event(CAM_IN_SUC_EOF_EVENT,5000);
 full(51000);run();assert(sends==1 && rw_early_count==1 && rw_vsync_count==0 && copies==11);
 // Latest frame replaces only the queued old frame.
 reset();full(1000);full(51000);run();
 assert(sends==2 && rw_queue_drops==1 && storage.frames[0].en && !storage.frames[1].en);
 assert(frames.frames.front()==&storage.frames[1].fb);
 // A borrowed frame cannot be reused, even with no free buffers at next VSYNC.
 reset();camera_fb_t* held=nullptr;full(1000);
 event(CAM_VSYNC_EVENT,51000,[&]{held=frames.frames.front();frames.frames.pop_front();});
 for(int i=1;i<=10;++i)event(CAM_IN_SUC_EOF_EVENT,51000+i*4000);
 event(CAM_VSYNC_EVENT,101000);event(CAM_IN_SUC_EOF_EVENT,105000);
 event(CAM_VSYNC_EVENT,151000,[&]{assert(starts==2 && !storage.frames[0].en);cam_give(held);});
 run();assert(starts==3 && copies==20 && sends==2 && rw_queue_drops==0);
 // Requeue failure / race with consumer popping: free current, preserve borrowed old.
 reset();full(1000);full(51000);
 schedule.back().before=[] {fail_send=2;};run();
 assert(rw_early_count==1 && rw_queue_drops==2 && frames.frames.empty());
 assert(storage.frames[0].en && storage.frames[1].en);
 reset();full(1000);full(51000);
 schedule.back().before=[&]{held=frames.frames.front();frames.frames.pop_front();fail_send=1;fail_pop=true;};run();
 assert(rw_early_count==1 && rw_queue_drops==1 && !storage.frames[0].en && storage.frames[1].en);cam_give(held);
 // Other formats/geometries retain VSYNC path; direct-PSRAM never copies early.
 reset();storage.width=160;full(1000);run();assert(!sends && !rw_early_count);
 reset();storage.psram_mode=true;full(1000);event(CAM_VSYNC_EVENT,51000);run();
 assert(sends==1 && copies==0 && rw_early_count==0 && rw_vsync_count==1);
 reset();storage.jpeg_mode=true;storage.fb_size=100000;full(1000);run();assert(!sends && !rw_early_count);
 // Overflow never publishes a partial frame.
 reset();event(CAM_VSYNC_EVENT,1000);storage.fb_size=7000;event(CAM_IN_SUC_EOF_EVENT,5000);run();
 assert(copies==0 && sends==0 && !running);
 // Driver timestamp and age stay correct beyond the 32-bit microsecond wrap.
 reset();full((1ULL<<32)+1000);run();auto fb=frames.frames.front();
 assert(uint64_t(fb->timestamp.tv_sec)*1000000+fb->timestamp.tv_usec==(1ULL<<32)+1000);
 assert(rw_camera_frame_ready_stats().last_full_age_us==40000);
 // v23 four physical slots: verify ordered pixels through multiple ring wraps.
 reset();storage.dma_half_buffer_cnt=4;event(CAM_VSYNC_EVENT,1000);
 for(int i=0;i<10;++i)event(CAM_IN_SUC_EOF_EVENT,5000+i*4000,[=]{
   memset(dma+(i%4)*7680,i+1,7680);
 });
 run();assert(sends==1 && copies==10 && !running);
 for(int i=0;i<10;++i)for(int j=0;j<7680;++j)assert(pixels[0][i*7680+j]==i+1);
 std::cout<<"PASS actual HAL loop: full/partial, fallback, DMA ownership, late EOF, borrowed buffers, queue races, timestamp wrap\n";
}
'''


def main():
    source = (ROOT/'third_party/esp32_camera_f0bb429/cam_hal.c').read_text(encoding='utf8')
    helper = source[source.index('static bool rw_early_enabled'):source.index('//Copy fram')]
    code = PREAMBLE + function(source, 'static bool cam_get_next_frame(') + '\n'
    code += function(source, 'static bool cam_start_frame(') + '\n' + helper
    code += function(source, 'static void cam_task(') + '\n' + CASES
    with tempfile.TemporaryDirectory(prefix='frame_ready_') as folder:
        cpp=Path(folder)/'test.cpp';exe=Path(folder)/'test.exe';cpp.write_text(code,encoding='utf8')
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter',
                        '-Wno-sign-compare','-I'+str(ROOT/'src'),str(cpp),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)


if __name__=='__main__':main()
