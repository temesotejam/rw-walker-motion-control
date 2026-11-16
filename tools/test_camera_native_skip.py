"""Run actual native-height HAL/DMA/wait code against ownership and time doubles.

These checks do not establish sensor geometry, pixel calibration or latency.
"""
from pathlib import Path
import subprocess
import tempfile
from test_foot_fast_metadata import function
from test_camera_frame_ready import PREAMBLE as HAL_PREAMBLE
from test_camera_overlap_wait import PREAMBLE as WAIT_PREAMBLE

ROOT = Path(__file__).resolve().parents[1]
EXTRA = r'''
#define ESP_LOGI(...)
#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE 4092
#define CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX 32768
static unsigned offers;
static std::vector<size_t> prefixes;
static void rw_camera_overlap_offer(const uint8_t* data,size_t bytes,uint64_t stamp) {
    assert(data && stamp && bytes<=storage.fb_size);
    ++offers;prefixes.push_back(bytes);
}
enum {PIXFORMAT_GRAYSCALE,PIXFORMAT_JPEG,PIXFORMAT_RGB565};
enum {FRAMESIZE_QVGA,FRAMESIZE_VGA};
constexpr unsigned GC0308_PID=0x9b;
struct camera_config_t {unsigned pixel_format=PIXFORMAT_GRAYSCALE,xclk_freq_hz=20000000,fb_count=2;};
struct Resolution {unsigned width,height;};
static constexpr Resolution resolution[]={{320,240},{640,480}};
'''
CASES = r'''
static void reset(){
    storage=cam_obj_t{};events.frames.clear();frames.frames.clear();schedule.clear();
    storage.event_queue=&events;storage.frame_buffer_queue=&frames;storage.dma_buffer=dma;
    for(int i=0;i<2;++i){storage.frames[i].fb={pixels[i],0,{0,0}};storage.frames[i].en=1;}
    memset(dma,0x5a,sizeof(dma));memset(pixels,0,sizeof(pixels));
    clock_us=stops=starts=sends=copies=offers=0;running=false;fail_send=0;fail_pop=false;prefixes.clear();
    rw_early_enabled=false;rw_native_enabled=false;
    rw_early_count=rw_vsync_count=rw_queue_drops=rw_last_full_age_us=0;
    camera_config_t config;configure_geometry(&config,FRAMESIZE_QVGA,GC0308_PID);
    assert(storage.width==320 && storage.height==236 && storage.recv_size==75520 && storage.fb_size==75520);
    assert(rw_native_enabled && rw_early_y8_geometry());
    assert(ll_cam_calc_rgb_dma(&storage));
    assert(storage.dma_buffer_size==32000 && storage.dma_node_buffer_size==1280);
    assert(storage.dma_half_buffer_size==1280 && storage.dma_half_buffer_cnt==25);
    assert(rw_native_dma_layout());
    assert(storage.dma_buffer_size==30720 && storage.dma_node_buffer_size==1280);
    assert(storage.dma_half_buffer_size==1280 && storage.dma_half_buffer_cnt==24);
    for(auto& p:pixels)memset(p+75520,0xa5,sizeof(p)-75520);
}
static void canaries(){for(auto& p:pixels)for(unsigned i=75520;i<sizeof(p);++i)assert(p[i]==0xa5);}
static void event(cam_event_t kind,uint64_t t,std::function<void()> before={}){schedule.push_back({kind,t,before});}
static void chunks(unsigned count,uint64_t start=1000){
    event(CAM_VSYNC_EVENT,start);
    for(unsigned i=0;i<count;++i)event(CAM_IN_SUC_EOF_EVENT,start+(i+1)*200,[i]{
        memset(dma+(i%24)*1280,i+1,1280);
    });
}
static void run(){try{cam_task(nullptr);}catch(Done&) {}}
int main(){
    // Publish only the 59th exact chunk, before VSYNC; all pixels retain raw coordinates.
    reset();chunks(59);event(CAM_IN_SUC_EOF_EVENT,14000);event(CAM_VSYNC_EVENT,23000,[]{assert(sends==1);});
    run();assert(sends==1 && copies==59 && offers==59 && starts==2);
    assert(rw_early_count==1 && !rw_vsync_count && rw_last_full_age_us==11800);
    auto* fb=frames.frames.front();assert(fb->len==75520 && fb->timestamp.tv_usec==1000);
    for(unsigned i=0;i<59;++i)for(unsigned j=0;j<1280;++j)assert(fb->buf[i*1280+j]==i+1);
    assert(prefixes.back()==75520);canaries();
    // Missing even one chunk (74240 bytes), an empty frame, or an early VSYNC never publishes.
    for(unsigned count:{0u,1u,58u}){
        reset();chunks(count);event(CAM_VSYNC_EVENT,23000);run();
        assert(!sends && !rw_early_count && !rw_vsync_count && copies==count);canaries();
    }
    // Recovery after a short frame still needs all59 chunks of the new timestamp.
    reset();chunks(58);chunks(59,23000);run();
    assert(sends==1 && copies==117 && frames.frames.front()->len==75520);
    assert(frames.frames.front()->timestamp.tv_usec==23000);canaries();
    // Queued replacement and borrowed-frame ownership are unchanged.
    reset();chunks(59);chunks(59,23000);run();
    assert(sends==2 && rw_queue_drops==1 && storage.frames[0].en && !storage.frames[1].en);canaries();
    reset();camera_fb_t* held=nullptr;chunks(59);
    event(CAM_VSYNC_EVENT,23000,[&]{held=frames.frames.front();frames.frames.pop_front();});
    for(unsigned i=1;i<=59;++i)event(CAM_IN_SUC_EOF_EVENT,23000+i*200);
    event(CAM_VSYNC_EVENT,45000);event(CAM_IN_SUC_EOF_EVENT,45200);
    event(CAM_VSYNC_EVENT,67000,[&]{assert(starts==2 && copies==118);cam_give(held);});
    run();assert(starts==3 && sends==2 && copies==118 && !rw_queue_drops);canaries();
    // An impossible short capacity blocks the overrun copy; it cannot publish a prefix.
    reset();storage.fb_size=74241;chunks(59);event(CAM_VSYNC_EVENT,23000);run();
    assert(!sends && copies==58 && !rw_early_count && !rw_vsync_count);canaries();
    // Raw/native geometry must agree on height, expected length and incoming byte count.
    for(unsigned field=0;field<8;++field){
        reset();
        switch(field){case 0:storage.height=240;break;case 1:storage.width=640;break;
        case 2:storage.fb_size=76800;break;case 3:storage.recv_size=76800;break;
        case 4:storage.psram_mode=true;break;case 5:storage.jpeg_mode=true;break;
        case 6:storage.in_bytes_per_pixel=2;break;case 7:storage.fb_bytes_per_pixel=2;break;}
        assert(!rw_early_y8_geometry() && !rw_native_dma_layout());
    }
    for(unsigned field=0;field<4;++field){
        reset();
        switch(field){case 0:storage.dma_node_buffer_size=3840;break;
        case 1:storage.dma_half_buffer_size=2560;break;case 2:storage.dma_buffer_size=30000;break;
        case 3:storage.dma_bytes_per_item=2;break;}
        assert(!rw_native_dma_layout());
    }
    // Native selection is limited to this sensor/Y8/QVGA/20MHz internal DMA setup.
    for(unsigned field=0;field<5;++field){
        reset();camera_config_t config;unsigned format=FRAMESIZE_QVGA,pid=GC0308_PID;
        if(field==0)pid=0;
        if(field==1)config.pixel_format=PIXFORMAT_RGB565;
        if(field==2)format=FRAMESIZE_VGA;
        if(field==3){config.xclk_freq_hz=16000000;storage.psram_mode=true;}
        if(field==4)storage.in_bytes_per_pixel=2;
        configure_geometry(&config,format,pid);assert(!rw_native_enabled);
        assert(storage.height==(format==FRAMESIZE_VGA?480u:240u));
        const auto old_size=storage.dma_buffer_size;assert(rw_native_dma_layout());
        assert(storage.dma_buffer_size==old_size);
    }
    // Existing QVGA full-size early publication remains available when native is disabled.
    reset();rw_native_enabled=false;storage.height=240;storage.fb_size=storage.recv_size=76800;
    storage.dma_half_buffer_size=7680;storage.dma_half_buffer_cnt=4;
    event(CAM_VSYNC_EVENT,1000);for(unsigned i=1;i<=10;++i)event(CAM_IN_SUC_EOF_EVENT,1000+i*4000);
    run();assert(sends==1 && frames.frames.front()->len==76800 && rw_early_count==1);
    std::cout<<"PASS actual native HAL:320x236/75520 full59chunks, partial74240 rejection, rawgeometry/DMA guards, overlap prefix, buffer ownership, legacy fallback\n";
}
'''
NATIVE_WAIT_CASES = r'''
static unsigned gives;
static std::function<void()> on_give;
static void cam_give(camera_fb_t* fb) {
    assert(fb==&frame);++gives;if(on_give)on_give();
}
static void reset_native(uint64_t now=0) {
    reset(now);storage.height=236;storage.fb_size=storage.recv_size=75520;
    rw_native_enabled=true;rw_native_stale_discards=0;gives=0;on_give={};frame.identity=17;
}
'''
NATIVE_WAIT_MAIN = r'''
int main(){
    reset_native();cam_obj=nullptr;assert(!rw_camera_take_overlap(5));
    assert(!gives && !polls && !resets && !legacy_calls);
    for(unsigned kind=0;kind<10;++kind){
        reset_native();queued=true;
        switch(kind){case 0:rw_early_enabled=false;break;case 1:storage.jpeg_mode=true;break;
        case 2:storage.psram_mode=true;break;case 3:storage.width=160;break;
        case 4:storage.height=240;break;case 5:storage.in_bytes_per_pixel=2;break;
        case 6:storage.fb_bytes_per_pixel=2;break;case 7:storage.fb_size=76800;break;
        case 8:storage.recv_size=76800;break;case 9:rw_native_enabled=false;break;}
        assert(rw_camera_take_overlap(5)==&frame && legacy_calls==1);
        assert(!gives && !polls && !resets && !rw_camera_native_stale_discard_count());
    }
    // Already-complete old frame is returned exactly once; fresh completion is accepted.
    reset_native();queued=true;
    on_poll=[] {if(polls==1){queued=true;available_at=3;frame.identity=29;}};
    assert(rw_camera_take_overlap(5)==&frame && frame.identity==29 && clock_ticks==3);
    assert(gives==1 && rw_camera_native_stale_discard_count()==1 && polls==3 && !resets);
    // Completion racing with the entry return must not be drained a second time.
    reset_native();queued=true;
    on_give=[] {queued=true;available_at=clock_ticks;frame.identity=31;};
    on_poll=[] {assert(false && "new ready frame must beat speculative work");};
    assert(rw_camera_take_overlap(5)==&frame && frame.identity==31);
    assert(gives==1 && rw_camera_native_stale_discard_count()==1 && !polls && !resets);
    // No queued frame: entry does not return a buffer; next completion has priority.
    reset_native();queued=true;available_at=2;
    assert(rw_camera_take_overlap(5)==&frame && clock_ticks==2);
    assert(!gives && !rw_camera_native_stale_discard_count() && polls==2 && !resets);
    // Stale discard does not reset DMA, restart the timeout, or free the received fresh frame.
    reset_native();queued=true;assert(!rw_camera_take_overlap(5));
    assert(gives==1 && polls==5 && resets==1 && reset_at==5 && clock_ticks==10);
    reset_native();queued=true;
    on_poll=[] {clock_ticks+=2;};assert(!rw_camera_take_overlap(5));
    assert(gives==1 && polls==2 && resets==1 && reset_at==5);
    // A full frame arriving during preparation at the deadline still avoids recovery.
    reset_native();on_poll=[] {clock_ticks=5;queued=true;available_at=5;};
    assert(rw_camera_take_overlap(5)==&frame && !gives && !resets && polls==1);
    constexpr uint64_t base=(uint64_t{1}<<32)-3;
    reset_native(base);queued=true;available_at=base;
    on_poll=[] {if(polls==1){queued=true;available_at=clock_ticks+4;}};
    assert(rw_camera_take_overlap(5)==&frame && clock_ticks==base+4);
    assert(gives==1 && !resets && polls==4);
    // Legacy QVGA in a native-capable binary retains its existing queue priority.
    reset_native();rw_native_enabled=false;storage.height=240;storage.fb_size=storage.recv_size=76800;
    queued=true;assert(rw_camera_take_overlap(5)==&frame);
    assert(!gives && !polls && !resets && !legacy_calls && !rw_camera_native_stale_discard_count());
    std::cout<<"PASS actual native wait: one stale dequeue/return, fresh priority, no borrowed-buffer return, bounded wait/recovery, wrap, legacy isolation\n";
}
'''


def build_and_run(folder, name, code, flags):
    cpp=folder/(name+'.cpp');exe=folder/(name+'.exe');cpp.write_text(code,encoding='utf8')
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                    '-Wno-unused-parameter','-Wno-sign-compare','-DFOOT_NATIVE_SKIP_V25=1',
                    '-I'+str(ROOT/'src'),*flags,str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def main():
    source=(ROOT/'third_party/esp32_camera_f0bb429/cam_hal.c').read_text(encoding='utf8')
    low=(ROOT/'analysis/2026-11-16-camera-latency/sources/ll_cam.c').read_text(encoding='utf8')
    preamble=HAL_PREAMBLE.replace('#include <stdexcept>', '#include <stdexcept>\n#include <vector>')
    preamble=preamble.replace('uint32_t dma_half_buffer_size=7680,dma_half_buffer_cnt=2;',
                             'uint32_t dma_half_buffer_size=7680,dma_half_buffer_cnt=2;\n uint32_t dma_buffer_size=30720,dma_node_buffer_size=3840;')
    preamble=preamble.replace('assert(fb->len==76800 || cam_obj->jpeg_mode || cam_obj->width!=320);',
                             'assert(fb->len==cam_obj->fb_size);')
    config=function(source,'esp_err_t cam_config(')
    begin=config.index('    rw_early_enabled = sensor_pid')
    end=config.index('    ret = cam_dma_config(config);')
    geometry='static void configure_geometry(const camera_config_t* config,unsigned frame_size,unsigned sensor_pid){\n'+config[begin:end]+'}\n'
    helper=source[source.index('static bool rw_early_enabled'):source.index('//Copy fram')]
    code=preamble+EXTRA+function(low,'static bool ll_cam_calc_rgb_dma(')+'\n'
    code+=function(source,'static bool cam_get_next_frame(')+'\n'
    code+=function(source,'static bool cam_start_frame(')+'\n'+helper+geometry
    code+=function(source,'static void cam_task(')+'\n'+CASES
    assert 'rw_native_dma_layout()' in function(source,'static esp_err_t cam_dma_config(')
    wait=WAIT_PREAMBLE.replace('static bool rw_early_enabled=true;',
                              'static bool rw_early_enabled=true,rw_native_enabled=true;\nstatic uint32_t rw_native_stale_discards;')
    wait+=function(source,'uint32_t rw_camera_native_stale_discard_count(')+'\n'
    wait+=NATIVE_WAIT_CASES
    wait+=function(source,'static bool rw_early_y8_geometry(')+'\n'
    wait+=function(source,'void* rw_camera_take_overlap(')+'\n'+NATIVE_WAIT_MAIN
    with tempfile.TemporaryDirectory(prefix='camera_native_skip_') as folder:
        path=Path(folder)
        build_and_run(path,'hal',code,['-DFOOT_ROW_OVERLAP_V24=1'])
        build_and_run(path,'wait',wait,['-DCONFIG_IDF_TARGET_ESP32S3=1'])


if __name__=='__main__':
    main()
