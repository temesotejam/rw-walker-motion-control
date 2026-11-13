"""The UI and actual Start share a calibrated, fresh foot-readiness gate."""
from pathlib import Path
import subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include "foot_support_start.h"
#include <cassert>
#include <iostream>
struct Frame {bool zero_ready=true,frame_valid=true,timestamp_valid=true,left_valid=true,right_valid=true,left_in_range=true,right_in_range=true;
 uint64_t frame_us=900000;float left_deg=0,right_deg=0;};
struct Snapshot {bool available=true,zero_ready=true;Frame latest;};
int main(){
 Snapshot s;assert(foot_support_start::ready(s,1000000));
 s.zero_ready=false;assert(!foot_support_start::ready(s,1000000));s.zero_ready=true;
 s.latest.zero_ready=false;assert(!foot_support_start::ready(s,1000000));s.latest.zero_ready=true;
 s.latest.left_valid=false;assert(!foot_support_start::ready(s,1000000));s.latest.left_valid=true;
 assert(!foot_support_start::ready(s,899999));assert(foot_support_start::ready(s,1100000));assert(!foot_support_start::ready(s,1100001));
 s.latest.right_deg=NAN;assert(!foot_support_start::ready(s,1000000));
 std::cout<<"fresh calibrated foot Start gate PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='foot_start_') as tmp:
 p=Path(tmp);(p/'test.cpp').write_text(CPP,encoding='utf8')
 subprocess.run(['g++','-std=c++11','-O2','-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
 subprocess.run([str(p/'test.exe')],check=True)
