"""Bounded first-reversal and amount-dependent width trial, using real headers."""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include "foot_support_pose.h"
#include "foot_support_assist_v13.h"
#include <cassert>
#include <iostream>
namespace v=foot_support_assist_v13;
static foot_local_prediction::Projection local(){
  foot_local_prediction::Predictor p;
  p.observe(2,28487,28549,true,-.64947f,1.76019f);
  p.observe(2,28588,28650,true,-.63555f,7.01348f);
  p.observe(2,28687,28749,true,-.58236f,6.83693f);
  return p.projection();
}
static foot_phase_prediction::Projection history(){
  foot_phase_prediction::Projection p;p.run_id=2;p.frame_ms=28687;p.received_ms=28749;
  p.reason=foot_phase_prediction::Reason::Ready;p.confidence=0;return p;
}
static v::Input input(){
  auto s=foot_support_pose::select(history(),local(),28758,true);
  assert(s.usable && s.source==foot_support_pose::Source::LocalTurn);
  assert(s.local_turn_confirmed && !s.local_returning_confirmed && s.pose.confidence==0);
  v::Input in;in.active=true;in.run_id=2;in.foot_run_id=2;in.peak_id=28558;in.peak_side=-1;
  in.now_ms=28758;in.frame_ms=28687;in.pose_ready=true;in.pose_confident=true;
  in.left_deg=s.pose.left;in.right_deg=s.pose.right;
  in.left_uncertainty_deg=s.pose.left_uncertainty;in.right_uncertainty_deg=s.pose.right_uncertainty;
  in.prediction_source=v::PredictionSource::LocalTurn;in.local_turn_confirmed=true;
  in.body_angle_deg=-4.7f;in.body_rate_dps=30;
  in.normal_drive_selected=true;in.normal_width_ms=40;in.raw_normal_width_ms=40;
  in.raw_boost_ms=v::boostMs(in);in.boosted_guarded_width_ms=40+in.raw_boost_ms;
  return in;
}
int main(){
  assert(!foot_support_pose::select(history(),local(),28758).usable); // v12 unchanged
  assert(!foot_support_pose::select(history(),local(),28788,true).usable); // age101
  {auto l=local();l.run_id=3;assert(!foot_support_pose::select(history(),l,28758,true).usable);}
  {auto l=local();l.right_model.last_change=-.14f;assert(!foot_support_pose::select(history(),l,28758,true).usable);}
  {auto l=local();l.right_model.prior_change=.9f;assert(!foot_support_pose::select(history(),l,28758,true).usable);}
  {auto l=local();l.left_model.phase=foot_local_prediction::Phase::Forward;assert(!foot_support_pose::select(history(),l,28758,true).usable);}
  {auto l=local();auto temp=l.left_model;l.left_model=l.right_model;l.right_model=temp;
   auto s=foot_support_pose::select(history(),l,28758,true);assert(s.usable && s.source==foot_support_pose::Source::LocalTurn);}
  auto in=input();auto d=v::decide(in);assert(d.selected && d.width_ms==50 && d.raw_boost_ms==10);
  assert(!d.stall_route && v::widthPreserved(d,50) && !v::widthPreserved(d,49));
  for(float speed:{0.f,-10.f,11.9f,40.1f}){auto x=in;x.body_rate_dps=speed;assert(!v::decide(x).selected);}
  {auto x=in;x.body_angle_deg=-1.9f;assert(!v::decide(x).selected);}
  {auto x=in;x.body_angle_deg=-4.7f;x.body_rate_dps=12;assert(!v::decide(x).selected);} // no tentative stall
  {auto x=in;x.normal_drive_selected=false;x.normal_coast_selected=true;x.normal_width_ms=0;x.raw_normal_width_ms=0;x.raw_boost_ms=0;assert(!v::decide(x).selected);}
  {auto x=in;x.raw_boost_ms=12;assert(!v::decide(x).selected);}
  {auto x=in;x.peak_side=1;x.body_angle_deg=4.7f;x.body_rate_dps=-30;assert(v::decide(x).selected);}
  {auto x=in;x.raw_normal_width_ms=81;x.raw_boost_ms=0;x.boosted_guarded_width_ms=70;assert(!v::decide(x).selected);}
  {auto x=in;x.boosted_guarded_width_ms=39;assert(!v::decide(x).selected);}
  {auto x=in;x.body_angle_deg=-2.24f;assert(!v::decide(x).selected);} // ETA74.67 <50+25
  // Amount affects the added width and caps the raw command, without changing current.
  for(unsigned width=1;width<=100;++width)for(unsigned foot=0;foot<=20;++foot){
    auto x=in;x.raw_normal_width_ms=width;x.right_deg=foot;
    auto extra=v::boostMs(x);assert(extra<=12);
    if(width<80)assert(width+extra<=80);else assert(extra==0);
  }
  {auto x=in;x.right_deg=2.f;assert(v::boostMs(x)==0);}
  {auto x=in;x.right_deg=NAN;assert(v::boostMs(x)==0 && !v::decide(x).selected);}
  {auto x=in;x.prediction_source=v::PredictionSource::Local;x.local_turn_confirmed=false;x.local_returning_confirmed=true;
   x.normal_drive_selected=false;x.normal_coast_selected=true;x.raw_normal_width_ms=0;x.normal_width_ms=0;
   x.raw_boost_ms=0;x.boosted_guarded_width_ms=0;x.inward_decelerating=true;
   auto r=v::decide(x);assert(r.selected && r.width_ms==15 && r.raw_boost_ms==0);}
  {auto x=in;x.prediction_source=v::PredictionSource::Local;x.local_turn_confirmed=false;x.local_returning_confirmed=true;
   x.body_angle_deg=-3.f;x.body_rate_dps=10;x.inward_decelerating=true;x.anticipated_shape_need=true;
   auto r=v::decide(x);assert(r.selected && r.stall_route);}
  std::cout<<"v13 first-reversal/width policy PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='foot_v13_') as tmp:
    p=Path(tmp);(p/'test.cpp').write_text(CPP,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-I',str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
    subprocess.run([str(p/'test.exe')],check=True)
