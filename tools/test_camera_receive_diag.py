"""Run the actual native HAL ownership cases with tagged receive events.

Also check silent error counters, ISR drop behavior, timing wrap, and that a
short-frame diagnostic uses its own start event rather than the next VSYNC.
"""
from pathlib import Path
import test_camera_native_skip as native
from test_foot_fast_metadata import function

ROOT = Path(__file__).resolve().parents[1]

def main():
    source = (ROOT/'third_party/esp32_camera_f0bb429/cam_hal.c').read_text(encoding='utf8')
    begin = source.index('typedef struct {cam_event_t type; uint32_t isr_us;}')
    end = source.index('\n#endif', begin)
    helper = source[begin:end]
    preamble = native.HAL_PREAMBLE.replace('struct Queue {', helper+'\nstatic uint32_t dispatch_delay;\nstruct Queue {')
    preamble = preamble.replace('*static_cast<cam_event_t*>(out)=e.type;',
        '*static_cast<rw_receive_event_t*>(out)={e.type,(uint32_t)e.time}; clock_us+=dispatch_delay;')
    preamble += r'''
#define IRAM_ATTR
using BaseType_t=int;
static bool reject_event;
static rw_receive_event_t sent_event;
static int xQueueSendFromISR(Queue*,void* ptr,BaseType_t*) {
    if(reject_event)return 0;
    sent_event=*static_cast<rw_receive_event_t*>(ptr);return pdTRUE;
}
'''+function(source,'void IRAM_ATTR ll_cam_send_event(')+'\n'
    native.HAL_PREAMBLE = preamble
    extra = r'''
    reset();rw_receive={};dispatch_delay=2000;
    chunks(58,10000);
    event(CAM_VSYNC_EVENT,40000,[]{dispatch_delay=5000;});run();
    auto rx=rw_camera_receive_stats();
    assert(rx.short_frames==1 && rx.last_short_bytes==74240 && !sends);
    assert(rx.events==60 && rx.eof_events==58 && rx.vsync_events==2);
    assert(rx.events_over_1ms==60 && rx.events_over_4ms==1);
    assert(rx.short_start_delay_us==2000 && rx.last_vsync_delay_us==5000);
    assert(rx.max_vsync_delay_us==5000 && rx.max_eof_delay_us==2000);
    reset();rw_receive={};dispatch_delay=20;
    chunks(59,0xfffffff0ULL);run();rx=rw_camera_receive_stats();
    assert(sends==1 && rx.full_start_delay_us==20 && rx.events==60);
    assert(rx.max_eof_delay_us==20 && !rx.short_frames && !rx.events_over_1ms);
    BaseType_t woken=0;clock_us=0x100000005ULL;reject_event=false;
    ll_cam_send_event(cam_obj,CAM_VSYNC_EVENT,&woken);
    assert(sent_event.type==CAM_VSYNC_EVENT && sent_event.isr_us==5);
    running=true;storage.state=CAM_STATE_READ_BUF;reject_event=true;
    ll_cam_send_event(cam_obj,CAM_IN_SUC_EOF_EVENT,&woken);
    assert(!running && storage.state==CAM_STATE_IDLE && rw_camera_receive_stats().event_drops==1);
    std::cout<<"PASS receive diagnostics: native ownership, short-frame identity, ISR drops, dispatch delay, 32-bit timer wrap\n";
'''
    at = native.CASES.rfind('}')
    native.CASES = native.CASES[:at]+extra+native.CASES[at:]
    old_build = native.build_and_run
    def build(folder,name,code,flags):
        if name=='hal': flags=flags+['-DFOOT_RECEIVE_DIAG_V30=1']
        old_build(folder,name,code,flags)
    native.build_and_run=build
    native.main()

if __name__=='__main__': main()
