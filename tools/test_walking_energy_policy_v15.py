"""Actual v15 energy-ceiling inverse and post-guard early admission contract."""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include "walking_energy_policy_v15.h"
#include <iostream>
#include <cstdlib>
#define CHECK(x) do {if(!(x)){std::cerr<<"check failed: "<<#x<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
namespace v15=walking_energy_policy_v15;
static bool near(float a,float b,float tolerance=2e-9f){return std::fabs(a-b)<=tolerance;}
static v15::Input input(){v15::Input in;in.side=1;in.rate_dps=55;in.body_angle_deg=0;
 in.target_deg=8;in.previous_peak_deg=8;in.same_side_peak_deg=6;
 in.initial_current_ma=0;in.goal_current_ma=329.547119f;return in;}
int main(){
 lateral_energy::Geometry geometry;lateral_energy::CurrentTable table;CHECK(table.prepare(.0316f));
 auto in=input();auto legacy=walking_energy_policy::decide(geometry,table,in);
 auto full=v15::decide(geometry,table,in);CHECK(full.valid && v15::earlyAdmissible(full));
 CHECK(full.width_ms>=legacy.width_ms && full.predicted_work_j>=full.request_j && full.unmet_j==0);
 CHECK(near(full.target_j,legacy.target_j) && near(full.available_j,legacy.available_j));
 CHECK(near(full.common_correction_j,legacy.common_correction_j));
 CHECK(near(full.redistribution_j,legacy.redistribution_j));
 auto shortened=v15::evaluateWidth(geometry,table,in,full.width_ms-1);
 CHECK(shortened.valid && shortened.predicted_work_j<shortened.request_j);
 CHECK(shortened.unmet_j>v15::kEnergyRoundoffJ && !v15::earlyAdmissible(shortened));
 auto blocked=v15::evaluateWidth(geometry,table,in,0);
 CHECK(!blocked.valid && !blocked.coast && blocked.unmet_j>0 && !v15::earlyAdmissible(blocked));
 auto cap=in;cap.cap_ms=15;auto limited=v15::decide(geometry,table,cap);
 CHECK(limited.valid && limited.width_ms==15 && limited.saturated && limited.unmet_j>0);
 CHECK(!limited.coast && !v15::earlyAdmissible(limited));
 CHECK(near(limited.predicted_arrival_j,limited.available_j+limited.predicted_work_j));
 auto fast=in;fast.rate_dps=80;auto coast=v15::decide(geometry,table,fast);
 CHECK(coast.valid && coast.coast && coast.width_ms==0 && coast.request_j==0);
 CHECK(!v15::earlyAdmissible(coast));
 CHECK(!v15::evaluateWidth(geometry,table,fast,1).valid);
 CHECK(!v15::evaluateWidth(geometry,table,in,full.width_ms+1).valid);
 // Positive requests smaller than one pulse quantum must choose a positive
 // ceiling width, even when v14's nearest-error objective would coast.
 bool found_tiny=false;
 for(unsigned speed=60000;speed<80000;++speed){auto x=in;x.rate_dps=speed*.001f;
  auto old=walking_energy_policy::decide(geometry,table,x);auto now=v15::decide(geometry,table,x);
  if(old.valid && old.request_j>0 && old.coast){
   CHECK(now.valid && !now.coast && now.width_ms>0 && now.predicted_work_j>=now.request_j);
   CHECK(v15::earlyAdmissible(now));found_tiny=true;break;}}
 CHECK(found_tiny);
 // A negative signed residual must not be turned into excitation by abs(Q)
 // or the quadratic term. A cap entirely before current reversal is invalid.
 auto reverse=in;reverse.initial_current_ma=-150;reverse.cap_ms=1;
 auto invalid=v15::decide(geometry,table,reverse);
 CHECK(!invalid.valid && invalid.q_mas<0 && !v15::earlyAdmissible(invalid));
 for(unsigned bound:{0u,81u}){auto x=in;x.cap_ms=bound;CHECK(!v15::decide(geometry,table,x).valid);}
 CHECK(!v15::evaluateWidth(geometry,table,in,81).valid);
 auto not_ready=table;not_ready.ready=false;CHECK(!v15::decide(geometry,not_ready,in).valid);
 {auto x=in;x.rate_dps=NAN;CHECK(!v15::decide(geometry,table,x).valid);}
 // Independent linear enumeration verifies minimum feasible width, not just
 // approximate target error, through both sides, all caps and signed I0.
 unsigned feasible=0,limited_count=0,negative=0;
 for(int side:{-1,1})for(float rate:{.1f,5.5f,20.f,35.f,55.f,80.f})
 for(float target:{4.5f,6.f,8.f})for(float i0:{-500.f,-150.f,-5.f,0.f,30.f})
 for(unsigned cap_ms=1;cap_ms<=80;++cap_ms){
  auto x=in;x.side=side;x.rate_dps=rate;x.target_deg=target;x.initial_current_ma=i0;x.cap_ms=cap_ms;
  const auto old=walking_energy_policy::decide(geometry,table,x);
  const auto result=v15::decide(geometry,table,x);CHECK(old.valid);
  const auto c=lateral_energy::coefficients(side);const float omega=rate*lateral_energy::kRadPerDeg;
  CHECK(near(result.available_j,old.available_j) && near(result.request_j,old.request_j));
  unsigned first=0;
  for(unsigned ms=1;ms<=cap_ms;++ms){
   float q=lateral_energy::charge(table,ms,i0,x.goal_current_ma);
   if(q>=0 && lateral_energy::work(c,omega,q)>=old.request_j){first=ms;break;}}
  float qcap=lateral_energy::charge(table,cap_ms,i0,x.goal_current_ma);
  if(old.request_j==0){CHECK(result.valid && result.coast && result.width_ms==0);continue;}
  CHECK(!result.coast);
  if(qcap<0){++negative;CHECK(!result.valid && result.q_mas<0);continue;}
  CHECK(result.valid && result.width_ms<=cap_ms);
  CHECK(near(result.q_mas,lateral_energy::charge(table,result.width_ms,i0,x.goal_current_ma)));
  if(first){++feasible;CHECK(result.width_ms==first && result.predicted_work_j>=result.request_j);
   CHECK(result.unmet_j==0 && !result.saturated && v15::earlyAdmissible(result));}
  else{++limited_count;CHECK(result.width_ms==cap_ms && result.unmet_j>0);
   CHECK(near(result.unmet_j,result.request_j-result.predicted_work_j));
   if(result.unmet_j>v15::kEnergyRoundoffJ)CHECK(result.saturated && !v15::earlyAdmissible(result));}
  const auto actual=v15::evaluateWidth(geometry,table,x,result.width_ms);
  CHECK(actual.valid && near(actual.predicted_work_j,result.predicted_work_j));
 }
 CHECK(feasible>0 && limited_count>0 && negative>0);
 // Admission must audit flags and the actual post-guard energy; no caller
 // can bypass a saturated, zero-width, invalid or fabricated negative-Q row.
 for(unsigned field=0;field<6;++field){auto d=full;
  if(field==0)d.valid=false;
  if(field==1)d.selected=false;
  if(field==2)d.saturated=true;
  if(field==3)d.width_ms=0;
  if(field==4)d.q_mas=-1;
  if(field==5)d.request_j=0;
  CHECK(!v15::earlyAdmissible(d));}
 CHECK(!v15::earlyAdmissible(full,NAN) && !v15::earlyAdmissible(full,-1));
 CHECK(!v15::earlyAdmissible(full,1e-3f));
 std::cout<<"v15 energy ceiling: exhaustive widths/signed-I0/cap deficit/guard admission PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='walking_energy_v15_') as temporary:
    tmp=Path(temporary)
    (tmp/'test.cpp').write_text(CPP,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-I'+str(ROOT/'src'),
                    str(tmp/'test.cpp'),'-o',str(tmp/'test.exe')],check=True)
    subprocess.run([str(tmp/'test.exe')],check=True)
