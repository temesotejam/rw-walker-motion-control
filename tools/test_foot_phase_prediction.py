"""Portable checks for causal delayed-foot prediction and frozen projections.

This validates inference mechanics, not physical foot contact or gait success.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "foot_phase_prediction.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace foot_phase_prediction;
static float wave(uint32_t t){return 2.f*std::sin(t*6.28318530718f/1600.f);}
int main(){
  Predictor p;
  assert(sizeof(Predictor)<1024u);
  assert(sizeof(Projection)<128u);
  assert(!p.predict(10).ready());
  assert(p.observe(1,100,163,true,0,0)==Reason::Warmup);
  assert(p.observe(1,200,263,true,1,1)==Reason::Ready);
  auto q=p.predict(277,30);
  assert(q.ready() && q.age_ms==77 && q.horizon_ms==107);
  // A two-frame slope alone does not justify a fresh extrapolated trajectory.
  assert(q.left==1.f && q.right==1.f && q.method==Method::Hold && q.confidence==0.f);
  assert(q.left_uncertainty>=1.f && q.right_uncertainty>=1.f);
  assert(p.predict(262).reason==Reason::Timestamp); // before publication
  assert(p.predict(381).reason==Reason::Stale);
  assert(p.predict(377,30).reason==Reason::Horizon);
  assert(p.predict(277,51).reason==Reason::Horizon);
  assert(p.observe(1,200,263,true,1,1)==Reason::Timestamp);
  assert(!p.predict(300).ready());
  assert(p.observe(1,300,363,true,1,1)==Reason::Warmup);
  assert(p.observe(1,400,463,false,1,1)==Reason::Invalid);
  assert(p.observe(1,500,563,true,1,1)==Reason::Warmup);
  assert(p.observe(1,600,663,true,2,2)==Reason::Ready);
  assert(p.observe(1,900,963,true,3,3)==Reason::Gap);
  assert(!p.predict(970).ready());
  assert(p.observe(1,1000,1063,true,3,3)==Reason::Ready);
  assert(p.predict(1070).method==Method::Hold);
  assert(p.observe(1,1100,1163,true,20,3)==Reason::Jump);
  assert(p.observe(1,1200,1263,true,3,3)==Reason::Warmup);
  assert(p.observe(1,1300,1481,true,3,3)==Reason::Stale);
  assert(p.observe(1,1500,1499,true,3,3)==Reason::Timestamp);
  assert(p.observe(2,100,163,true,1,1)==Reason::Warmup); // new run clears history
  assert(p.observe(2,200,263,true,NAN,1)==Reason::Invalid);
  assert(p.observe(0,300,363,true,1,1)==Reason::Invalid);
  assert(!p.predict(370).ready());

  // A repeated smooth orbit supplies causal evidence of a reversal. Only the
  // already received samples are inserted; future truth is used for assertion.
  p.reset();
  for(uint32_t t=100;t<=3600;t+=100)
    p.observe(3,t,t+63,true,wave(t),wave(t+200));
  const auto frozen=p.projection();
  const auto turn=frozen.predict(3677,13); // frame+90 ms
  assert(turn.ready() && turn.method==Method::History && turn.confidence>.8f);
  assert(turn.left_uncertainty<.75f && turn.right_uncertainty<.75f);
  assert(turn.left<1.98f && turn.left_rate<0.f);
  assert(turn.phase_left==Phase::Returning || turn.phase_left==Phase::Turning);
  assert(std::fabs(turn.left-wave(3690))<std::fabs(wave(3600)-wave(3690)));
  const float before=turn.left;
  p.observe(3,3700,3763,true,wave(3700),wave(3900));
  assert(frozen.predict(3677,13).left==before); // later data cannot alter a copy
  assert(!frozen.predict(3781).ready()); // copies age out while reused

  // Repeated projection queries perform no state update and need no allocation.
  const auto a=p.projection();const auto b=p.projection();
  assert(a.confidence==b.confidence);
  for(unsigned i=0;i<4;++i){assert(a.left[i]==b.left[i]);assert(a.right[i]==b.right[i]);}

  // Two identical three-frame histories may lead in opposite directions.
  // Their forecast remains unchanged, but high matching confidence alone
  // must not hide the disagreement from the intervention policy.
  p.reset();
  for(uint32_t t=100;t<=3600;t+=100){
    const uint32_t phase=t%1600u,cycle=t/1600u;
    float l=-3.f,r=-3.f;
    if(phase==200u){l=0;r=0;}
    if(phase==300u){l=.5f;r=.2f;}
    if(phase==400u){l=1.f;r=.5f;}
    if(phase==500u || phase==600u){l=cycle==0u?2.5f:-1.5f;r=cycle==0u?2.f:-1.f;}
    p.observe(4,t,t+63u,true,l,r);
  }
  const auto ambiguous=p.predict(3677u,13u);
  assert(ambiguous.ready() && ambiguous.confidence>.9f);
  assert(ambiguous.left_uncertainty>1.f && ambiguous.right_uncertainty>1.f);
  const auto older=p.predict(3750u);
  const auto fresh=p.predict(3677u);
  assert(older.left_uncertainty>fresh.left_uncertainty);
  std::cout<<"delayed frame, frozen projection, reversal, invalidation PASS\n";
}
'''

with tempfile.TemporaryDirectory(prefix="foot_phase_prediction_") as temporary:
    temporary = Path(temporary)
    cpp, exe = temporary / "test.cpp", temporary / "test.exe"
    cpp.write_text(SOURCE, encoding="utf8")
    subprocess.run([
        "g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I" + str(ROOT / "src"), str(cpp), "-o", str(exe)
    ], check=True)
    subprocess.run([str(exe)], check=True)
