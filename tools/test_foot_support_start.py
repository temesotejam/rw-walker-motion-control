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
 using R=foot_support_start::Reason;
 s=Snapshot{};s.latest.left_in_range=false;assert(foot_support_start::reason(s,1000000)==R::LeftRange);
 s=Snapshot{};s.zero_ready=false;assert(foot_support_start::reason(s,1000000)==R::Zero);
 s=Snapshot{};assert(foot_support_start::reason(s,1100001)==R::Stale);
 assert(foot_support_start::reason(s,899999)==R::Timestamp);
 // Preserve every original readiness condition while exposing its reason.
 for(unsigned mask=0;mask<512;++mask)for(uint64_t now:{899999ULL,900000ULL,1100000ULL,1100001ULL}){
   s=Snapshot{};s.available=mask&1;s.zero_ready=mask&2;s.latest.zero_ready=mask&4;
   s.latest.frame_valid=mask&8;s.latest.timestamp_valid=mask&16;s.latest.left_valid=mask&32;
   s.latest.right_valid=mask&64;s.latest.left_in_range=mask&128;s.latest.right_in_range=mask&256;
   const auto& f=s.latest;
   const bool original=s.available && s.zero_ready && f.zero_ready && f.frame_valid && f.timestamp_valid &&
     f.left_valid && f.right_valid && f.left_in_range && f.right_in_range && f.frame_us && f.frame_us<=now &&
     now-f.frame_us<=200000 && std::isfinite(f.left_deg) && std::isfinite(f.right_deg);
   assert(foot_support_start::ready(s,now)==original);
 }
 std::cout<<"fresh calibrated foot Start gate PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='foot_start_') as tmp:
 p=Path(tmp);(p/'test.cpp').write_text(CPP,encoding='utf8')
 subprocess.run(['g++','-std=c++11','-O2','-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
 subprocess.run([str(p/'test.exe')],check=True)
