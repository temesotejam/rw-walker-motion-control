"""Portable causal inward-deceleration checks; no motor/contact simulation."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "foot_support_deceleration.h"
#include <cstdlib>
#include <iostream>
#include <cmath>
#define CHECK(test) do {if(!(test)){std::cerr<<"failed: "<<#test<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
using foot_support_deceleration::Tracker;
static void baseline(Tracker& p){for(uint32_t t=100;t<=140;t+=5)CHECK(!p.update(1,90,t,20));}
int main(){
  Tracker p;CHECK(sizeof(Tracker)<320u);
  baseline(p);
  // A three-point median delays the first changed value. Confirmation then
  // requires three filtered observations spanning at least 10 ms.
  CHECK(!p.update(1,90,145,14));
  CHECK(!p.update(1,90,150,14));
  CHECK(!p.update(1,90,155,14));
  CHECK(p.update(1,90,160,14));
  CHECK(p.update(1,90,165,14));
  // No indefinite latch from an old peak: it must leave the rolling window.
  bool last=true;for(uint32_t t=170;t<=280;t+=5)last=p.update(1,90,t,14);
  CHECK(!last);

  // One high gyro spike cannot create a speed maximum; one low spike cannot
  // manufacture a sustained loss of speed either.
  for(bool high:{false,true}){
    p.reset();baseline(p);CHECK(!p.update(1,90,145,high?60.f:1.f));
    for(uint32_t t=150;t<=220;t+=5)CHECK(!p.update(1,90,t,20));
  }
  // A change smaller than 4 dps never confirms. Exactly 4 dps can confirm.
  p.reset();baseline(p);for(uint32_t t=145;t<=200;t+=5)CHECK(!p.update(1,90,t,16.01f));
  p.reset();baseline(p);for(uint32_t t=145;t<160;t+=5)CHECK(!p.update(1,90,t,16));
  CHECK(p.update(1,90,160,16));

  // Gaps, duplicates, reversed time, nonfinite/non-inward values, and a
  // different run or peak cannot reuse a preceding maximum/confirmation.
  for(unsigned mode=0;mode<8;++mode){
    p.reset();baseline(p);p.update(1,90,145,14);p.update(1,90,150,14);
    p.update(1,90,155,14);CHECK(p.update(1,90,160,14));
    uint16_t run=1;uint32_t peak=90,time=165;
    if(mode==0)time=176;
    if(mode==1)time=160;
    if(mode==2)time=159;
    if(mode==3){CHECK(!p.update(1,90,165,NAN));time=170;}
    if(mode==4){CHECK(!p.update(1,90,165,0));time=170;}
    if(mode==5){CHECK(!p.update(1,90,165,-1));time=170;}
    if(mode==6)run=2;
    if(mode==7)peak=91;
    CHECK(!p.update(run,peak,time,14));
    for(unsigned i=1;i<=8;++i)CHECK(!p.update(run,peak,time+i*5,14));
  }
  // Even dense calls need 10 ms: three samples alone are insufficient.
  p.reset();baseline(p);CHECK(!p.update(1,90,145,14));
  CHECK(!p.update(1,90,146,14));CHECK(!p.update(1,90,147,14));
  CHECK(!p.update(1,90,148,14));CHECK(!p.update(1,90,155,14));
  CHECK(p.update(1,90,156,14));
  // Missing identity is always invalid, including after a valid history.
  CHECK(!p.update(0,90,161,14));CHECK(!p.update(1,0,166,14));
  std::cout<<"causal deceleration median, persistence, window and identity PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix="foot_deceleration_") as temporary:
    temporary = Path(temporary)
    cpp, exe = temporary / "test.cpp", temporary / "test.exe"
    cpp.write_text(SOURCE, encoding="utf8")
    subprocess.run([
        "g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I" + str(ROOT / "src"), str(cpp), "-o", str(exe)
    ], check=True)
    subprocess.run([str(exe)], check=True)
