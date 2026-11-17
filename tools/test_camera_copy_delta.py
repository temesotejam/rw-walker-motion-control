"""Compare v27 immutable-prefix copies and exercise actual gated mailbox code."""
from pathlib import Path
import subprocess
import tempfile
from test_camera_lower_overlap import CPP, ROOT

prefix=CPP[:CPP.index('#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26\nstruct Context')]
CASES=r'''
static uint64_t expected_stamp;
static unsigned expected_seed, lower_calls=0, upper_calls=0;
static int expected_y;
static std::function<void()> while_reading;
static void consumeLower(const GrayRowView& rows,uint64_t stamp,uint32_t,int y,void*){
 assert(stamp==expected_stamp && y==expected_y);checkLower(rows,expected_seed,y);++lower_calls;
 if(while_reading){auto once=while_reading;while_reading={};once();}
 checkLower(rows,expected_seed,y);
}
static void consumeUpper(const GrayRowView& rows,uint64_t stamp,uint32_t,void*){
 assert(stamp==expected_stamp);checkUpper(rows,expected_seed);++upper_calls;
}
int main(){
 cacheCases();viewCases();
 std::vector<uint8_t> buffer(kLowerBytes+2,0xa5),scratch_bytes(kRequiredBytes,0xa5);
 LowerRowCache cache(buffer.data()+1,kLowerBytes),scratch(scratch_bytes.data(),scratch_bytes.size());
 const uint64_t stamp=(uint64_t(1)<<40)+7;
 auto raw=image(11);unsigned copied=0,old_copied=0,scratch_copied=0;
 for(int stage:kLowerMilestones){
   assert(cache.extend(raw.data(),lowerPrefixThrough(stage),stamp));
   copied+=cache.lastCopiedBytes();checkLower(cache.view(),11,stage);
   unsigned count=0;for(int y=0;y<=stage;++y)if(lowerNeededRow(y))++count;
   old_copied+=count*appcfg::kFrameWidth;
   assert(scratch.extend(cache.view(),stage,stamp));scratch_copied+=scratch.lastCopiedBytes();
   checkLower(scratch.view(),11,stage);
   assert(cache.extend(raw.data(),lowerPrefixThrough(stage),stamp));assert(cache.lastCopiedBytes()==0);
 }
 assert(copied==kLowerBytes && scratch_copied==kLowerBytes && old_copied>copied);
 assert(buffer.front()==0xa5 && buffer.back()==0xa5);
 // A new frame overwrites its entire prefix, never reuses preceding pixels.
 auto next=image(29);assert(cache.extend(next.data(),lowerPrefixThrough(174),stamp+1));
 checkLower(cache.view(),29,174);assert(cache.lastCopiedBytes()>0);
 assert(!cache.extend(next.data(),lowerPrefixThrough(174),stamp));assert(!cache.view().valid());
 assert(!cache.extend(next.data(),kRawFrameBytes+1,stamp+2));
 assert(!cache.extend(next.data(),kRawFrameBytes,0));
 assert(cache.extend(next.data(),lowerPrefixThrough(222),stamp+2));
 assert(!cache.extend(next.data(),lowerPrefixThrough(174),stamp+2));
 // Upper/full-image processing overwrites shared scratch: invalidate first.
 std::fill(scratch_bytes.begin(),scratch_bytes.end(),0xa5);scratch.invalidate();
 assert(cache.extend(next.data(),lowerPrefixThrough(222),stamp+3));
 assert(scratch.extend(cache.view(),222,stamp+3));checkLower(scratch.view(),29,222);
 assert(scratch.lastCopiedBytes()==kLowerBytes);
 assert(!scratch.extend(GrayRowView{},222,stamp+3));assert(!scratch.view().valid());
 std::vector<uint8_t> upper(kUpperBytes),lower(kLowerBytes);
 assert(camera_row_overlap::initialize(upper.data(),upper.size(),consumeUpper,nullptr));
 assert(camera_row_overlap::initializeLower(lower.data(),lower.size(),consumeLower,nullptr));
#if !defined(FOOT_OVERLAP_ALWAYS_V29) || !FOOT_OVERLAP_ALWAYS_V29
 rw_camera_overlap_offer(raw.data(),raw.size(),stamp);rw_camera_overlap_poll();
 assert(lower_calls==0 && upper_calls==0);
#endif
 rw_camera_overlap_request(true);expected_stamp=stamp;expected_seed=11;
 for(int stage:kLowerMilestones){expected_y=stage;
   rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(stage),stamp);rw_camera_overlap_poll();
 }
 assert(lower_calls==4 && upper_calls==1);
 rw_camera_overlap_request(false);
 const auto before=rw_camera_lower_overlap_stats().offered;
#if defined(FOOT_OVERLAP_ALWAYS_V29) && FOOT_OVERLAP_ALWAYS_V29
 expected_stamp=stamp+1;expected_seed=29;expected_y=222;
#endif
 rw_camera_overlap_offer(next.data(),next.size(),stamp+1);rw_camera_overlap_poll();
#if defined(FOOT_OVERLAP_ALWAYS_V29) && FOOT_OVERLAP_ALWAYS_V29
 assert(rw_camera_lower_overlap_stats().offered==before+1 && lower_calls==5);
#else
 assert(rw_camera_lower_overlap_stats().offered==before);
#endif
 const auto calls_before_retry=lower_calls;
 rw_camera_overlap_request(true);expected_stamp=stamp+2;expected_seed=29;expected_y=174;
 rw_camera_overlap_offer(next.data(),lowerPrefixThrough(174),stamp+2);
 while_reading=[&](){
   // Producer cannot overwrite pixels being read; nested polls cannot reenter.
   rw_camera_overlap_offer(next.data(),lowerPrefixThrough(190),stamp+2);
   const auto n=lower_calls;rw_camera_overlap_poll();assert(lower_calls==n);
 };
 rw_camera_overlap_poll();assert(rw_camera_lower_overlap_stats().busy>0);
 expected_y=190;rw_camera_overlap_offer(next.data(),lowerPrefixThrough(190),stamp+2);
 rw_camera_overlap_poll();assert(lower_calls==calls_before_retry+2);
 rw_camera_overlap_request(false);
 std::cout<<"PASS v27 copies per lower lane: "<<old_copied<<" -> "<<copied
          <<" bytes at four stages; same-frame/new-frame/scratch-reset/ownership/demand gates\n";
}
'''
with tempfile.TemporaryDirectory(prefix='copy_delta_') as tmp:
    path=Path(tmp);(path/'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nextern "C" int64_t esp_timer_get_time();\n')
    (path/'test.cpp').write_text(prefix+CASES)
    for standard,always in [('c++11',0),('c++17',0),('c++11',1),('c++17',1)]:
        exe=path/(standard+'.exe')
        subprocess.run(['g++','-std='+standard,'-O2','-Wall','-Wextra','-Werror','-Wno-unused-function',
                        '-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1','-DFOOT_NATIVE_STREAM_V26=1',
                        '-DFOOT_COPY_DELTA_V27=1','-I'+str(path),'-I'+str(ROOT/'src'),
                        '-DFOOT_OVERLAP_ALWAYS_V29='+str(always),
                        str(path/'test.cpp'),str(ROOT/'src/camera_row_overlap.cpp'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        print('Continuous preparation:',bool(always),flush=True)
