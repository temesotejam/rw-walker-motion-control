"""Verify native236 cache/preview bounds and logical tracker equivalence.

Uses guard pages at the exact source end, including the earliest upper prefix.
Synthetic image/timestamp equivalence is not sensor-angle validation.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CPP = r'''
#define NOMINMAX
#include <Arduino.h>
#include <cassert>
#include <iostream>
#include <random>
#include "foot_image_rows.h"
#include "white_marker_tracker.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif
namespace rows=foot_image_rows;
struct Guarded {
 uint8_t* allocation=nullptr;uint8_t* data=nullptr;size_t total=0;
 explicit Guarded(size_t bytes){
#ifdef _WIN32
  SYSTEM_INFO info;GetSystemInfo(&info);const size_t page=info.dwPageSize;
  total=((bytes+page-1)/page+1)*page;
  allocation=static_cast<uint8_t*>(VirtualAlloc(nullptr,total,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE));assert(allocation);
  DWORD old=0;assert(VirtualProtect(allocation+total-page,page,PAGE_NOACCESS,&old));
#else
  const size_t page=sysconf(_SC_PAGESIZE);total=((bytes+page-1)/page+1)*page;
  allocation=static_cast<uint8_t*>(mmap(nullptr,total,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0));
  assert(allocation!=MAP_FAILED);assert(mprotect(allocation+total-page,page,PROT_NONE)==0);
#endif
  data=allocation+total-page-bytes;
 }
 ~Guarded(){
#ifdef _WIN32
  VirtualFree(allocation,0,MEM_RELEASE);
#else
  munmap(allocation,total);
#endif
 }
};
bool same(float a,float b){return (std::isnan(a)&&std::isnan(b)) || a==b;}
void equal(const WhiteMarkerObservation& a,const WhiteMarkerObservation& b){
 assert(a.valid==b.valid && a.reason==b.reason && a.id==b.id);
 assert(a.templates_tested==b.templates_tested && a.candidate_count==b.candidate_count);
 assert(same(a.ambiguity_ratio,b.ambiguity_ratio) && same(a.alternate_x_px,b.alternate_x_px) && same(a.alternate_y_px,b.alternate_y_px));
 assert(same(a.center_x_px,b.center_x_px) && same(a.center_y_px,b.center_y_px));
 assert(same(a.peak_contrast,b.peak_contrast) && same(a.weight_sum,b.weight_sum));
 assert(a.peak_x_px==b.peak_x_px && a.bright_width_px==b.bright_width_px);
 assert(a.success_count==b.success_count && a.fail_count==b.fail_count);
}
void band(std::vector<uint8_t>& image,int cx,int cy,int half){
 for(int y=cy-10;y<=cy+10;++y)for(int x=cx-half;x<=cx+half;++x)
  if(x>=0 && x<320 && y>=0 && y<240)image[y*320+x]=230;
}
int main(){
 static_assert(rows::kFrameBytes==76800 && rows::kRawFrameBytes==75520 && rows::kRawFrameHeight==236,"Distinct geometries");
 static_assert(rows::kRawYOrigin==-2 && rows::kRequiredRows==66 && rows::kUpperRows==26,"Full original row search");
 static_assert(rows::kUpperPrefixBytes==33600,"Last upper raw row104 inclusive");
 Guarded raw(rows::kRawFrameBytes),prefix(rows::kUpperPrefixBytes),thumbnail(160*120);
 std::vector<uint8_t> logical(rows::kFrameBytes),storage(rows::kRequiredBytes),upper_storage(rows::kUpperBytes),scratch_storage(rows::kUpperBytes);
 rows::RowCache cache(storage.data(),storage.size());
 rows::UpperRowCache upper(upper_storage.data(),upper_storage.size()),scratch(scratch_storage.data(),scratch_storage.size());
 WhiteMarker1DTracker full_a(0),full_b(1),raw_a(0),raw_b(1),early_a(0);
 std::mt19937 rng(75125);
 for(unsigned iteration=0;iteration<350;++iteration){
  for(auto& value:logical)value=uint8_t(15+rng()%20);
  const int ax=20+int(rng()%280),bx=20+int(rng()%280);
  const int ay=38+int(rng()%57),by=132+int(rng()%57);
  if(iteration%13)band(logical,ax,ay,3+int(rng()%30));
  if(iteration%17)band(logical,bx,by,3+int(rng()%30));
  if(iteration%9==0)band(logical,160,66,10); // Ambiguity/jump histories remain equivalent.
  for(int y=0;y<236;++y)memcpy(raw.data+y*320,logical.data()+(y+2)*320,320);
  memcpy(prefix.data,raw.data,rows::kUpperPrefixBytes);
  assert(!rows::GrayRowView::full(raw.data,rows::kRawFrameBytes).valid());
  assert(rows::GrayRowView::full(logical.data(),logical.size()).valid());
  assert(cache.capture(raw.data,rows::kRawFrameBytes));
  assert(upper.capture(prefix.data,rows::kUpperPrefixBytes));
  assert(scratch.capture(upper.view())); // Logical packed view must not shift twice.
  for(int y=0;y<240;++y){
   if(rows::neededRow(y))assert(memcmp(cache.view().row(y),logical.data()+y*320,320)==0);
   if(rows::upperNeededRow(y))assert(memcmp(scratch.view().row(y),logical.data()+y*320,320)==0);
  }
  const auto features=early_a.prepare(scratch.view());
  host_us=1000000+iteration*33333;
  const auto a=full_a.process(logical.data()),b=full_b.process(logical.data());
  equal(a,raw_a.process(cache.view()));equal(b,raw_b.process(cache.view()));equal(a,early_a.processPrepared(features));
  memset(thumbnail.data,0xa5,160*120);
  assert(rows::copyNativeThumbnail(raw.data,rows::kRawFrameBytes,thumbnail.data,160*120));
  for(int y=0;y<240;y+=2)for(int x=0;x<320;x+=2){
   const uint8_t expected=y>=2 && y<238?logical[y*320+x]:0;
   assert(rows::rawPixelOrZero(raw.data,rows::kRawFrameBytes,x,y)==expected);
   assert(thumbnail.data[(y/2)*160+x/2]==expected);
  }
 }
 // Every byte value, including the bounded raw source end and both padded
 // borders, remains bit-exact with the original pixel helper.
 for(unsigned iteration=0;iteration<40;++iteration){
  for(size_t i=0;i<rows::kRawFrameBytes;++i)raw.data[i]=uint8_t(rng());
  assert(rows::copyNativeThumbnail(raw.data,rows::kRawFrameBytes,thumbnail.data,160*120));
  for(int y=0;y<120;++y)for(int x=0;x<160;++x)
   assert(thumbnail.data[y*160+x]==rows::rawPixelOrZero(raw.data,rows::kRawFrameBytes,x*2,y*2));
 }
 memset(thumbnail.data,0xa5,160*120);
 for(size_t bytes:{size_t(0),rows::kRawFrameBytes-1,rows::kRawFrameBytes+1,rows::kFrameBytes})
  assert(!rows::copyNativeThumbnail(raw.data,bytes,thumbnail.data,160*120));
 for(size_t capacity:{size_t(0),size_t(160*120-1)})
  assert(!rows::copyNativeThumbnail(raw.data,rows::kRawFrameBytes,thumbnail.data,capacity));
 assert(!rows::copyNativeThumbnail(nullptr,rows::kRawFrameBytes,thumbnail.data,160*120));
 assert(!rows::copyNativeThumbnail(raw.data,rows::kRawFrameBytes,nullptr,160*120));
 for(size_t i=0;i<160*120;++i)assert(thumbnail.data[i]==0xa5);
 assert(!cache.capture(raw.data,rows::kFrameBytes) && !cache.view().valid());
 for(size_t bytes:{size_t(0),rows::kRawFrameBytes-1,rows::kRawFrameBytes+1})
  assert(!cache.capture(raw.data,bytes) && !cache.view().valid());
 assert(!cache.capture(nullptr,rows::kRawFrameBytes));
 assert(!upper.capture(prefix.data,rows::kUpperPrefixBytes-1) && !upper.view().valid());
 assert(!upper.capture(raw.data,rows::kRawFrameBytes+1));
 assert(!upper.capture(raw.data,rows::kFrameBytes));
 assert(upper.capture(raw.data,rows::kRawFrameBytes));
 assert(!scratch.capture(rows::GrayRowView{}) && !scratch.view().valid());
 assert(rows::rawPixelOrZero(raw.data,rows::kRawFrameBytes,320,222)==0);
 assert(rows::rawPixelOrZero(raw.data,rows::kRawFrameBytes,0,-1)==0);
 assert(rows::rawPixelOrZero(raw.data,rows::kRawFrameBytes,0,240)==0);
 assert(rows::rawPixelOrZero(raw.data,rows::kFrameBytes,0,222)==0);
 std::cout<<"native236: exact raw length, guard-page preview/prefix bounds, rowwise thumbnail390 bit-exact comparisons, all66 logical rows, prepared/full tracker1050comparisons PASS\n";
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix='foot_native_geometry_') as temporary:
        path = Path(temporary)
        source, binary = path/'test.cpp', path/'test.exe'
        source.write_text(CPP, encoding='utf8')
        subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
            '-DFOOT_LOW_LATENCY_V20=1', '-DFOOT_SPEED_V22=1', '-DFOOT_CAPTURE_V23=1',
            '-DFOOT_ROW_OVERLAP_V24=1', '-DFOOT_NATIVE_SKIP_V25=1',
            '-I'+str(ROOT/'test/fixtures/arduino'), '-I'+str(ROOT/'src'), str(source),
            str(ROOT/'src/white_marker_tracker.cpp'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
