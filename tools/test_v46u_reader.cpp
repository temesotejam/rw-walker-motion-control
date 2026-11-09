#include "bmi270_timing_reader.h"
#include "timing_deadline.h"
#include <cassert>
#include <array>
#include <vector>
#include <utility>
#include <iostream>
struct Axis {int16_t x=0,y=0,z=0;};
struct Raw {Axis accel,gyro,mag;};
struct Bus {
  std::array<uint8_t,128> mem{};
  uint8_t ready=0; bool fail_status=false;int fail_reg=-1;
  bool inject=false; bool interrupt_expired=false;
  uint32_t clock=0;std::vector<std::pair<int,int>> reads;
  Bus(){for(unsigned i=0;i<mem.size();++i)mem[i]=uint8_t(i*37);}
  bool read(uint8_t reg,uint8_t* dst,size_t n){
    reads.emplace_back(reg,n);clock+=uint32_t(9*(n+3));
    if(reg==0x03){ if(fail_status)return false;*dst=ready;return true; }
    if(reg==0x1d){*dst=interrupt_expired?0:ready;return true;}
    if(inject){ready|=0x40;inject=false;}
    if(reg==fail_reg)return false;
    for(size_t i=0;i<n;++i)dst[i]=mem[reg+i];
    if(reg<=0x0d && reg+n>0x0d)ready&=~0x80;
    if(reg<=0x13 && reg+n>0x13)ready&=~0x40;
    if(reg<=0x05 && reg+n>0x05)ready&=~0x20;
    return true;
  }
};
uint8_t updated(Bus& b, Raw& r){return bmi270_timing::readRaw(&r,[&](uint8_t a,uint8_t* p,size_t n){return b.read(a,p,n);},[&](){return b.clock;});}
uint8_t legacy(Bus& b,Raw& r){uint8_t s=0;b.read(0x1d,&s,1);uint8_t p[20];uint8_t out=0;if(s&0xe0){if(b.read(4,p,20)){if(s&0x80){bmi270_timing::copyAxis(r.accel,p+8);out|=1;}if(s&0x40){bmi270_timing::copyAxis(r.gyro,p+14);out|=2;}if(s&0x20){r.mag.x=bmi270_timing::signed16(p)>>2;r.mag.y=bmi270_timing::signed16(p+2)>>2;r.mag.z=bmi270_timing::signed16(p+4)&0xfffe;out|=4;}}}return out;}
int main(){
  for(unsigned k=0;k<8;++k){
    Bus a,b;a.ready=b.ready=uint8_t(k<<5);Raw x,y;
    assert(updated(a,x)==legacy(b,y));assert(memcmp(&x,&y,sizeof(x))==0);
    assert(a.reads.front().first==3);assert(a.reads.size()<=3);
    assert(bmi270_timing::lastRead().failures==0);
    if(k==2)assert(a.reads[1]==std::make_pair(0x12,6));
    if(k==4)assert(a.reads[1]==std::make_pair(0x0c,6));
    if(k==6)assert(a.reads[1]==std::make_pair(0x0c,12));
  }
  // An accel-only transfer must not consume a newly arriving gyro sample.
  {Bus a,b;a.ready=b.ready=0x80;a.inject=b.inject=true;Raw x,y;
    assert(updated(a,x)==1);assert(legacy(b,y)==1);
    assert((a.ready&0x40)!=0);assert((b.ready&0x40)==0);assert(updated(a,x)==2);}
  {Bus a,b;a.ready=b.ready=0x40;a.interrupt_expired=b.interrupt_expired=true;Raw x,y;
    assert(updated(a,x)==2);assert(legacy(b,y)==0);}
  {Bus b;b.ready=0xe0;b.fail_status=true;Raw r;assert(updated(b,r)==0);assert(b.reads.size()==1);assert(bmi270_timing::lastRead().failures==1);}
  for(int reg:{0x0c,0x12,0x04}){Bus b;Raw r;b.ready=reg==0x0c?0xc0:reg==0x12?0x40:0x20;b.fail_reg=reg;assert(updated(b,r)==0);assert(bmi270_timing::lastRead().failures==1);}
  {Bus b;Raw r;b.ready=0x40;b.clock=0xfffffff0U;assert(updated(b,r)==2);assert(bmi270_timing::lastRead().status_us==36);}
  timing_deadline::Counter c;assert(!c.passed());c.add(2499,2500);c.add(2500,2500);assert(c.passed());c.add(2501,2500);assert(c.over==1 && !c.passed());assert(c.count==3 && c.maximum==2501 && c.sum==7500);
  std::cout<<"PASS: 8 mask equivalence cases; short reads; ready-race; expired interrupt; failures; timestamp wrap; strict deadline boundary\n";
}
