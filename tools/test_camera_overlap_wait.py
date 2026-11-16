"""Run the actual overlap acquisition wait with deterministic queue/time doubles.

This verifies queue priority, timeout boundaries and bounded DMA recovery;
sensor timing, physical ownership and CPU scheduling need separate checks.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
from test_foot_fast_metadata import function

ROOT = Path(__file__).resolve().parents[1]
PREAMBLE = r'''
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <vector>
using TickType_t=uint32_t;
constexpr int pdTRUE=1;
struct camera_fb_t {int identity;};
struct Queue {};
static Queue queue;
struct Camera {
    bool jpeg_mode=false,psram_mode=false;
    unsigned width=320,height=240,in_bytes_per_pixel=1,fb_bytes_per_pixel=1;
    Queue* frame_buffer_queue=&queue;
};
static Camera storage,*cam_obj=&storage;
static bool rw_early_enabled=true;
static camera_fb_t frame{17};
static uint64_t clock_ticks=0,available_at=0,reset_at=0;
static bool queued=false,recover_with_frame=false;
static unsigned polls=0,resets=0,legacy_calls=0;
static TickType_t legacy_timeout=0;
static std::vector<TickType_t> waits;
static std::function<void()> on_poll;
static TickType_t xTaskGetTickCount(){return static_cast<TickType_t>(clock_ticks);}
static int xQueueReceive(Queue* q,void* out,TickType_t wait) {
    assert(q==&queue);waits.push_back(wait);
    if(queued && available_at<=clock_ticks+wait) {
        clock_ticks=std::max(clock_ticks,available_at);
        *static_cast<camera_fb_t**>(out)=&frame;queued=false;return pdTRUE;
    }
    clock_ticks+=wait;return 0;
}
static void rw_camera_overlap_poll() {++polls;if(on_poll)on_poll();}
static void ll_cam_dma_reset(Camera* cam) {
    assert(cam==cam_obj);++resets;reset_at=clock_ticks;
    if(recover_with_frame){queued=true;available_at=clock_ticks+2;}
}
static camera_fb_t* cam_take(TickType_t timeout){++legacy_calls;legacy_timeout=timeout;return &frame;}
static void reset(uint64_t now=0) {
    storage=Camera{};cam_obj=&storage;rw_early_enabled=true;
    clock_ticks=now;available_at=reset_at=0;queued=recover_with_frame=false;
    polls=resets=legacy_calls=0;legacy_timeout=0;waits.clear();on_poll={};
}
'''

CASES = r'''
int main() {
    reset();cam_obj=nullptr;assert(!rw_camera_take_overlap(5));
    assert(waits.empty() && !legacy_calls && !resets && !polls);
    for(int kind=0;kind<7;++kind) {
        reset();
        if(kind==0)rw_early_enabled=false;
        if(kind==1)storage.jpeg_mode=true;
        if(kind==2)storage.psram_mode=true;
        if(kind==3)storage.width=160;
        if(kind==4)storage.height=120;
        if(kind==5)storage.in_bytes_per_pixel=2;
        if(kind==6)storage.fb_bytes_per_pixel=2;
        assert(rw_camera_take_overlap(11)==&frame);
        assert(legacy_calls==1 && legacy_timeout==11 && waits.empty() && !polls && !resets);
    }
    // A frame already in the driver queue always beats speculative work.
    reset(42);queued=true;available_at=42;
    on_poll=[] {assert(false && "ready frame must precede preparation");};
    assert(rw_camera_take_overlap(5)==&frame && clock_ticks==42);
    assert(!polls && !resets && waits==std::vector<TickType_t>{0});

    reset();queued=true;available_at=3;
    assert(rw_camera_take_overlap(5)==&frame);
    assert(clock_ticks==3 && polls==3 && !resets && !legacy_calls);
    assert(std::count(waits.begin(),waits.end(),1)==3);

    // Empty short probes never reset DMA. One reset follows the full timeout,
    // and the existing full-duration recovery wait remains bounded.
    reset();assert(!rw_camera_take_overlap(5));
    assert(polls==5 && resets==1 && reset_at==5 && clock_ticks==10);
    assert(std::count(waits.begin(),waits.end(),1)==5 && waits.back()==5);

    // Callback elapsed time consumes the same deadline; it is not ignored or
    // converted into a fresh timeout for every loop iteration.
    reset();on_poll=[] {clock_ticks+=2;};
    assert(!rw_camera_take_overlap(5));
    assert(polls==2 && resets==1 && reset_at==5 && clock_ticks==10);
    assert(std::count(waits.begin(),waits.end(),1)==1);
    reset();on_poll=[] {clock_ticks+=6;};
    assert(!rw_camera_take_overlap(5));
    assert(polls==1 && resets==1 && reset_at==6 && clock_ticks==11);

    // A complete image arriving during preparation at the deadline must be
    // returned before recovery resets a DMA stream which has already delivered.
    reset();on_poll=[] {clock_ticks=5;queued=true;available_at=5;};
    assert(rw_camera_take_overlap(5)==&frame);
    assert(clock_ticks==5 && polls==1 && !resets);
    reset();on_poll=[] {available_at=4;queued=true;clock_ticks=7;};
    assert(rw_camera_take_overlap(5)==&frame);
    assert(clock_ticks==7 && polls==1 && !resets);

    // Tick subtraction must remain unsigned across the 32-bit wrap.
    constexpr uint64_t start=(uint64_t{1}<<32)-3;
    reset(start);queued=true;available_at=start+4;
    assert(rw_camera_take_overlap(5)==&frame);
    assert(clock_ticks==start+4 && polls==4 && !resets);
    reset(start);assert(!rw_camera_take_overlap(5));
    assert(reset_at==start+5 && clock_ticks==start+10 && polls==5 && resets==1);

    reset();recover_with_frame=true;
    assert(rw_camera_take_overlap(5)==&frame);
    assert(polls==5 && resets==1 && reset_at==5 && clock_ticks==7);
    // The queue priority also applies to an explicitly zero-duration request.
    reset();queued=true;available_at=0;
    assert(rw_camera_take_overlap(0)==&frame && !polls && !resets);
    std::cout<<"PASS actual overlap wait: ready-frame priority, 1tick blocking, callback deadline, wrap, one full-timeout recovery, legacy geometry/format fallback\n";
}
'''


def main():
    compiler = shutil.which('g++')
    if not compiler:
        raise SystemExit('g++ is required for the actual overlap wait test')
    source = (ROOT/'third_party/esp32_camera_f0bb429/cam_hal.c').read_text(encoding='utf8')
    code = PREAMBLE + function(source, 'void* rw_camera_take_overlap(') + '\n' + CASES
    with tempfile.TemporaryDirectory(prefix='camera_overlap_wait_') as folder:
        path = Path(folder)
        (path/'test.cpp').write_text(code, encoding='utf8')
        binary = path/'test.exe'
        subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-DCONFIG_IDF_TARGET_ESP32S3=1', str(path/'test.cpp'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
