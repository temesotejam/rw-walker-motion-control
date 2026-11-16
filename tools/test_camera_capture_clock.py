"""Check clock arithmetic and fail-closed source/divider admission (not an oscilloscope test)."""
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include <cassert>
#include <iostream>
#include "camera_capture_clock.h"
#include "camera_capture_dma.h"
struct Reg {
 unsigned cam_clk_sel=3,cam_clkm_div_num=8,cam_clkm_div_a=0,cam_clkm_div_b=0;
 unsigned cam_update=0,other_bits=0xa5;
};
int main(){
 auto layout=rw_capture_dma_layout(30720,15360,3840,76800);
 assert(layout.valid && layout.half_bytes==7680 && layout.slots==4);
 assert(layout.half_bytes*layout.slots==30720 && 76800/layout.half_bytes==10);
 assert(!rw_capture_dma_layout(30720,15360,5000,76800).valid);
 assert(!rw_capture_dma_layout(30720,15360,3840,76801).valid);
 assert(!rw_capture_dma_layout(30000,15360,3840,76800).valid);
 assert(!rw_capture_dma_layout(0,0,0,0).valid);
 Reg r;assert(camera_capture_clock::apply(r));
 assert(160000000ULL/r.cam_clkm_div_num==22857142 && r.cam_clkm_div_a==0 && r.cam_clkm_div_b==0);
 assert(r.cam_update==1 && r.other_bits==0xa5);
 for(int fault=0;fault<4;++fault){
  Reg bad; if(fault==0)bad.cam_clk_sel=2;if(fault==1)bad.cam_clkm_div_num=6;
  if(fault==2)bad.cam_clkm_div_a=3;if(fault==3)bad.cam_clkm_div_b=2;
  const Reg before=bad;assert(!camera_capture_clock::apply(bad));
  assert(bad.cam_clk_sel==before.cam_clk_sel && bad.cam_clkm_div_num==before.cam_clkm_div_num &&
         bad.cam_clkm_div_a==before.cam_clkm_div_a && bad.cam_clkm_div_b==before.cam_clkm_div_b && bad.cam_update==0);
 }
 std::cout<<"PASS integer divider7 and unknown source/divider refusal\n";
}
'''
with tempfile.TemporaryDirectory(prefix='capture_clock_') as folder:
    p=Path(folder);(p/'test.cpp').write_text(CPP,encoding='utf8')
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
                    '-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
    subprocess.run([str(p/'test.exe')],check=True)
source=(ROOT/'src/camera_coexistence.cpp').read_text(encoding='utf8')
start=source.index('const bool clock_ok=')
assert source.rfind('setXclkEnabled(false)',0,start)>source.index('esp_camera_init(&c)')
assert source.index('setXclkEnabled(true)',start)<source.index('sensor->set_framesize',start)
assert 'camera_capture_clock_readback_failed' in source
boot=source[source.index('bool OneShotCamera::beginOnOwnerCore()'):source.index('camera_fb_t* OneShotCamera::acquire(')]
assert boot.index('initCameraOnTemporaryI2c0()')<boot.index('cam_start();')<boot.index('camera_fb_t* boot_frame = cam_take')
