"""Exercise the boot-only native-skip SCCB transaction, including bus failures."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = r'''
#include "camera_native_skip_profile.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <utility>
#include <vector>
namespace p=camera_native_skip_profile;
struct Bus {
 int regs[2][256]{},page=0,reads=0,writes=0,waits=0;
 int fail_read=-1,fail_write=-1,fail_read_reg=-1,ignore_write_reg=-1;
 int delayed_reg=-1,delayed_value=0,remaining=0,delay_reads=0;
 bool corrupt_aec=false,corrupt_orientation=false,fail_cleanup=false,cleanup_seen=false;
 int corrupt_pixel_count=-1;
 bool fail_final_pixel_count=false;
 std::vector<std::pair<int,int>> log;
 Bus() {
  for(auto r:p::baseline0)regs[0][r.address]=r.value;
  for(auto r:p::baseline1)regs[1][r.address]=r.value;
  regs[1][0x55]=3;
  for(unsigned i=0;i<sizeof(p::preserved0);++i)regs[0][p::preserved0[i]]=0x20+i;
  regs[0][0xd2]=0x90;regs[0][3]=1;regs[0][4]=224;
  regs[0][0x51]=0x73;regs[0][0x52]=0x67;
 }
 int read(int reg) {
  ++reads;
  const int full=reg==0xfe?reg:(page<<8)|reg;
  if(reads==fail_read || full==fail_read_reg)return -1;
  if(fail_final_pixel_count && full==0x153 && regs[1][0x54]==0x12)return -1;
  if(reg==0xfe)return page;
  if(full==delayed_reg && remaining>0 && --remaining==0) {
   regs[page][reg]=delayed_value;delayed_reg=-1;
  }
  return regs[page][reg];
 }
 bool write(int reg,int value) {
  ++writes;
  const int full=reg==0xfe?reg:(page<<8)|reg;
  log.emplace_back(full,value);
  if(writes==fail_write)return false;
  if(reg==0xfe) {
   if(fail_cleanup && page==0 && regs[0][0x15]==0x4a && regs[1][0x54]==0x12 && cleanup_seen)
    return false;
   if(value==0 && regs[1][0x54]==0x12)cleanup_seen=true;
   page=value;return true;
  }
  if(full==ignore_write_reg)return true;
  if(corrupt_aec && full==0x154)regs[0][0xd2]^=0x80;
  if(corrupt_orientation && full==0x154)regs[0][0x14]=0x12;
  if(corrupt_pixel_count>=0 && full==0x154)regs[1][0x53]=corrupt_pixel_count;
  if(delay_reads) {delayed_reg=full;delayed_value=value;remaining=delay_reads;}
  else regs[page][reg]=value;
  return true;
 }
 void wait(unsigned ms){assert(ms==5);waits+=ms;assert(waits<2000);}
 bool run(p::Report& r){return p::apply([&](int x){return read(x);},
   [&](int x,int y){return write(x,y);},[&](unsigned ms){wait(ms);},&r);}
 void checkWrites()const {
  for(auto item:log)assert(item.first==0xfe || item.first==0x14 || item.first==0x15 || item.first==1 || item.first==0x154);
 }
};
int main() {
 static_assert(p::kNativeHeight==488 && p::kOutputWidth==320 && p::kOutputHeight==236);
 static_assert(p::kLogicalRowOrigin==2);
 static_assert(p::kOrientation==0x10,"Actual legacy image orientation, not stale SDK intent");
 Bus reference; p::Report r;
 assert(reference.run(r) && r.applied && r.page0_restored && std::strcmp(r.stage,"applied")==0);
 assert(reference.page==0 && reference.regs[0][1]==p::kHorizontalBlank);
 assert(reference.regs[0][0x14]==0x10);
 assert(reference.regs[0][0x15]==0x4a && reference.regs[1][0x54]==0x12);
 assert(reference.waits==0);reference.checkWrites();
 Bus initial;
 for(int page=0;page<2;++page)for(int reg=0;reg<256;++reg) {
  if((page==0 && (reg==1 || reg==0x15)) || (page==1 && reg==0x54))continue;
  assert(reference.regs[page][reg]==initial.regs[page][reg]);
 }
 for(auto expected:p::baseline0) {
  Bus b;b.regs[0][expected.address]^=expected.address==0x14?4:1;
  assert(!b.run(r) && !r.applied && r.page0_restored && b.page==0);
  assert(std::strcmp(r.stage,expected.address==0x14?"baseline_orientation_bits":"baseline_readback")==0 && r.reg==expected.address);
  for(auto item:b.log)assert(item.first==0xfe);
 }
 for(auto expected:p::baseline1) {
  Bus b;b.regs[1][expected.address]^=expected.address==0x53?2:1;
  assert(!b.run(r) && !r.applied && r.page0_restored && b.page==0);
  assert(r.reg==(0x100|expected.address));
  for(auto item:b.log)assert(item.first==0xfe);
 }
 for(int sub:{0,2,0x83}) {
  Bus b;b.regs[1][0x55]=sub;
  assert(!b.run(r) && r.page0_restored && r.reg==0x155);
  for(auto item:b.log)assert(item.first==0xfe);
 }
 {Bus b;b.regs[1][0x55]=1;assert(b.run(r) && b.regs[1][0x55]==1);}
 for(int pixel_count=0;pixel_count<256;++pixel_count) {
  Bus b;b.regs[1][0x53]=pixel_count;
  if(pixel_count==0x82 || pixel_count==0x83) {
   assert(b.run(r) && r.applied && b.regs[1][0x53]==pixel_count && r.page0_restored);
   b.checkWrites(); // No write to53 is permitted, even for the accepted alternative.
  } else {
   assert(!b.run(r) && !r.applied && r.page0_restored && r.reg==0x153 && r.actual==pixel_count);
   assert(std::strcmp(r.stage,"baseline_readback")==0);
   for(auto item:b.log)assert(item.first==0xfe);
  }
 }
 for(int pixel_count:{0x82,0x83})for(int sub_mode:{1,3})for(int orientation:{0x10,0x11,0x12,0x13}) {
  Bus b;b.regs[1][0x53]=pixel_count;b.regs[1][0x55]=sub_mode;b.regs[0][0x14]=orientation;
  b.delay_reads=3;
  assert(b.run(r) && r.applied && r.page0_restored && b.regs[1][0x53]==pixel_count);
  assert(b.regs[1][0x55]==sub_mode && b.regs[0][0x14]==0x10);b.checkWrites();
 }
 for(int orientation:{0x10,0x11,0x12,0x13}) {
  Bus b;b.regs[0][0x14]=orientation;b.delay_reads=3;
  assert(b.run(r) && b.regs[0][0x14]==0x10 && r.page0_restored);
  bool orientation_seen=false;
  for(auto item:b.log) {
   if(item.first==0x14){assert(item.second==0x10);orientation_seen=true;}
   if(item.first==0x15 || item.first==1 || item.first==0x154)assert(orientation_seen);
  }
 }
 // Reproduce the legacy two-setter stale RMW:11 ->pending13 ->stale11
 // ->pending10. The actual legacy image was10; preserve that orientation
 // instead of "repairing" the image to the SDK's unobserved flip intent.
 {Bus b;b.regs[0][0x14]=0x11;b.delay_reads=3;
  assert(b.write(0x14,b.read(0x14)|2));
  assert(b.write(0x14,b.read(0x14)&~1));
  for(int i=0;i<3;++i)(void)b.read(0x14);
  assert(b.regs[0][0x14]==0x10);
  b.log.clear();b.reads=b.writes=b.waits=0;
  assert(b.run(r) && b.regs[0][0x14]==0x10 && r.applied);
 }
 // Every single bus-read/write failure in the successful sequence fails
 // closed, including the final page restoration. Cleanup remains bounded.
 for(int pixel_count:{0x82,0x83})for(int n=1;n<=reference.reads;++n) {
  Bus b;b.regs[1][0x53]=pixel_count;b.fail_read=n;
  assert(!b.run(r) && !r.applied);b.checkWrites();assert(b.page==0);
 }
 for(int pixel_count:{0x82,0x83})for(int n=1;n<=reference.writes;++n) {
  Bus b;b.regs[1][0x53]=pixel_count;b.fail_write=n;
  assert(!b.run(r) && !r.applied);b.checkWrites();assert(b.page==0);
 }
 for(int reg:{0x15,1,0x154}) {
  if(reg==1 && p::kHorizontalBlank==128)continue;
  Bus b;b.ignore_write_reg=reg;
  assert(!b.run(r) && !r.applied && r.page0_restored && r.reg==reg);
  assert(std::strcmp(r.stage,"setting_readback")==0 && r.settle_wait_ms==100);
  b.checkWrites();
 }
 for(int reversed:{0x12,0x13}){Bus b;b.regs[0][0x14]=reversed;b.ignore_write_reg=0x14;
  assert(!b.run(r) && !r.applied && r.page0_restored && r.reg==0x14 && r.settle_wait_ms==100);
  assert(b.regs[0][0x15]==0x0a && b.regs[1][0x54]==0x22 && b.regs[0][1]==128);
 }
 {Bus b;b.delay_reads=3;assert(b.run(r));assert(r.settle_wait_ms==b.waits && b.waits>=20);}
 {Bus b;b.corrupt_aec=true;assert(!b.run(r));
  assert(std::strcmp(r.stage,"preserved_readback")==0 && r.reg==0xd2 && r.page0_restored);}
 {Bus b;b.corrupt_orientation=true;assert(!b.run(r) && !r.applied);
  assert(std::strcmp(r.stage,"final_readback")==0 && r.reg==0x14 && r.expected==0x10 && r.actual==0x12 && r.page0_restored);}
 for(int pixel_count:{0x82,0x83}) {
  for(int changed:{0x80,0x82,0x83,0x84}) {
   if(changed==pixel_count)continue;
   Bus b;b.regs[1][0x53]=pixel_count;b.corrupt_pixel_count=changed;
   assert(!b.run(r) && !r.applied && r.page0_restored && r.reg==0x153);
   assert(std::strcmp(r.stage,"final_readback")==0 && r.expected==pixel_count && r.actual==changed);
  }
  Bus b;b.regs[1][0x53]=pixel_count;b.fail_final_pixel_count=true;
  assert(!b.run(r) && !r.applied && r.page0_restored && r.reg==0x153 && r.actual==-1);
  assert(std::strcmp(r.stage,"final_readback")==0 && r.expected==pixel_count);
 }
 {Bus b;b.fail_read_reg=0x154;assert(!b.run(r) && r.page0_restored && b.page==0);}
 {Bus b;b.fail_cleanup=true;assert(!b.run(r) && !r.applied && !r.page0_restored);
  assert(std::strcmp(r.stage,"page0_restore")==0);}
 {Bus b;b.regs[1][0x54]=0x23;b.fail_write=3;
  assert(!b.run(r) && !r.applied && !r.page0_restored && b.page==1);
  assert(std::strcmp(r.stage,"baseline_readback")==0 && r.reg==0x154 && r.actual==0x23);}
 {Bus b;b.fail_read_reg=0xfe;assert(!b.run(r) && !r.page0_restored);
  assert(std::strcmp(r.stage,"page_readback")==0 && b.writes==2);}
 std::cout<<"native skip transaction PASS HB="<<p::kHorizontalBlank
   <<" reads="<<reference.reads<<" writes="<<reference.writes<<"\n";
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix='native_skip_profile_') as folder:
        folder = Path(folder)
        source = folder / 'fixture.cpp'
        source.write_text(FIXTURE+'\n#if FOOT_NATIVE_HB80_V33\nstatic_assert(camera_native_skip_profile::kHorizontalBlank==80,"v33 must override inherited HB96");\n#endif\n', encoding='utf-8')
        for hb in [None, 64, 80, 96, 128, 'v33']:
            exe = folder / f'profile_{hb}.exe'
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(ROOT / 'src')] +
                           ([] if hb is None else ['-DCAMERA_NATIVE_HB=96','-DFOOT_NATIVE_HB80_V33=1'] if hb=='v33' else [f'-DCAMERA_NATIVE_HB={hb}']) +
                           [str(source), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
        for hb in [0, 63, 129, 256]:
            result = subprocess.run(['g++', '-std=c++17', '-fsyntax-only',
                                     '-I' + str(ROOT / 'src'), f'-DCAMERA_NATIVE_HB={hb}',
                                     str(source)], capture_output=True, text=True)
            assert result.returncode and 'Native HB must stay' in result.stderr
    print('Native skip profile fault injection, preservation and HB bounds PASS')


if __name__ == '__main__':
    main()
