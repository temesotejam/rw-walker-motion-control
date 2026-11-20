"""Native v18 return-aid/main budget and explicit return-energy selection tests.

These verify software accounting and empirical model use, not physical walking.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CPP = r'''
#include "half_cycle_energy_budget.h"
#include "energy_pulse_completion.h"
#include <cassert>
#include <iostream>
namespace b=half_cycle_energy_budget;
bool close(float a,float c){return std::fabs(a-c)<1e-9f;}
walking_return_energy::Input input(){
 walking_return_energy::Input in;in.side=-1;in.rate_dps=15;in.body_angle_deg=9.4f;
 in.target_deg=8;in.previous_peak_deg=10.6f;in.same_side_peak_deg=7;
 in.initial_current_ma=-25;in.goal_current_ma=328;in.return_work_j=.0001f;return in;
}
int main(){
 {b::Ledger l;assert(!l.allowAid(0) && !l.allowMain(900) && l.cap(900,80)==0);
  assert(!l.sourcePeak(0));assert(!l.accepted(b::Kind::Main,0));assert(!l.complete(1));
  assert(l.sourcePeak(900));assert(l.remaining()==80 && l.allowAid(900) && l.allowMain(900));
  assert(l.cap(900,100)==80 && l.cap(901,80)==0 && l.cap(900,17)==17);
  // A rejected candidate or failed transport does not call accepted().
  assert(!l.accepted(b::Kind::ReturnAid,0));assert(!l.accepted(b::Kind::ReturnAid,81));
  assert(!l.accepted(static_cast<b::Kind>(9),1));assert(l.remaining()==80 && !l.aidUsed());
  assert(l.accepted(b::Kind::ReturnAid,43));assert(l.aidUsed() && !l.mainUsed() && l.active());
  assert(l.reservedMs()==43 && l.spentMs()==0 && l.remaining()==37);
  assert(!l.allowAid(900) && !l.allowMain(900));assert(l.sourcePeak(900));
  assert(!l.sourcePeak(901) && !l.accepted(b::Kind::Main,1));
  assert(l.complete(44));assert(l.spentMs()==44 && l.remaining()==36 && !l.active());
  assert(!l.complete(44));assert(!l.allowAid(900) && l.allowMain(900));
  assert(!l.sourcePeak(899));assert(!l.accepted(b::Kind::Main,37));
  assert(l.accepted(b::Kind::Main,36));assert(l.mainUsed() && l.remaining()==0);
  assert(l.complete(37));assert(l.spentMs()==81 && l.remaining()==0);
  assert(!l.allowAid(900) && !l.allowMain(900));assert(l.sourcePeak(901));
  assert(l.remaining()==80 && !l.aidUsed() && !l.mainUsed());}
 // Check every accepted aid width, a shortened output and a one-ms late stop.
 for(unsigned width=1;width<=80;++width)for(unsigned elapsed:{0u,width-1,width,width+1}){
  b::Ledger l;assert(l.sourcePeak(1000));assert(l.accepted(b::Kind::ReturnAid,width));
  assert(l.remaining()==80-width);assert(l.complete(elapsed));
  const unsigned rem=elapsed>=80?0:80-elapsed;assert(l.remaining()==rem);
  assert(l.allowMain(1000) && !l.allowAid(1000));
  assert(!l.accepted(b::Kind::ReturnAid,1));
  if(rem){assert(!l.accepted(b::Kind::Main,rem+1));assert(l.accepted(b::Kind::Main,rem));
   assert(l.complete(rem));assert(l.remaining()==0 && l.spentMs()==elapsed+rem);}
  else {assert(l.accepted(b::Kind::Main,0));assert(!l.active() && !l.complete(0));}
  assert(!l.allowMain(1000) && !l.allowAid(1000));}
 {b::Ledger l;assert(l.sourcePeak(100));assert(l.accepted(b::Kind::Main,0));
  assert(l.mainUsed() && !l.aidUsed() && l.remaining()==80 && !l.active());
  assert(!l.accepted(b::Kind::ReturnAid,1) && !l.accepted(b::Kind::Main,1));
  l.reset();assert(l.peakMs()==0 && !l.aidUsed() && !l.mainUsed());}
 {b::Ledger l;assert(l.sourcePeak(100));assert(l.accepted(b::Kind::ReturnAid,20));
  assert(l.complete(UINT32_MAX));assert(l.remaining()==0 && l.spentMs()==UINT32_MAX);
  assert(!l.accepted(b::Kind::Main,1));assert(l.accepted(b::Kind::Main,0));}
 lateral_energy::Geometry g;lateral_energy::CurrentTable table;assert(table.prepare(.031652216f));
 auto in=input();const auto aid=b::decideReturnAid(g,table,in);
 assert(aid.valid && !aid.coast && aid.extrapolated && aid.width_ms>15);
 assert(aid.available_j==0 && close(aid.request_j,in.return_work_j));
 assert(aid.predicted_work_j+1e-9f>=aid.request_j && close(aid.predicted_arrival_j,aid.predicted_work_j));
 // Nominal U at the current angle and peak feedback never fund a return aid.
 for(float angle:{-12.f,0.f,1.f,9.4f,12.f})for(float bias:{-.0002f,0.f,.0002f}){
  auto x=in;x.body_angle_deg=angle;x.arrival_bias_j=bias;x.same_side_peak_deg=1;
  const auto a=b::decideReturnAid(g,table,x);assert(a.valid);
  assert(a.width_ms==aid.width_ms && close(a.request_j,aid.request_j));}
 for(int side:{-1,1})for(float i0:{-500.f,-25.f,0.f,100.f})for(unsigned cap=1;cap<=80;++cap){
  auto x=in;x.side=side;x.initial_current_ma=i0;x.cap_ms=cap;
  const auto a=b::decideReturnAid(g,table,x);
  if(!a.valid)continue;
  assert(a.width_ms>0 && a.width_ms<=cap);
  assert(close(a.request_j,in.return_work_j) && a.available_j==0);
  if(!a.saturated && a.width_ms>1){const auto shorter=b::evaluateReturnAidWidth(g,table,x,a.width_ms-1);
   assert(!shorter.valid || shorter.predicted_work_j<a.request_j);}
  const auto blocked=b::evaluateReturnAidWidth(g,table,x,0);assert(!blocked.valid);
  assert(!b::evaluateReturnAidWidth(g,table,x,a.width_ms+1).valid);
  assert(!b::evaluateReturnAidWidth(g,table,x,UINT16_MAX).valid);
 }
 {auto x=in;x.cap_ms=15;const auto short_aid=b::decideReturnAid(g,table,x);
  assert(short_aid.valid && short_aid.saturated && short_aid.unmet_j>0 && short_aid.width_ms==15);}
 for(float invalid:{0.f,-.0001f,NAN,INFINITY}){
  auto x=in;x.return_work_j=invalid;assert(!b::decideReturnAid(g,table,x).valid);}
 for(unsigned invalid_cap:{0u,81u,65535u}){
  auto x=in;x.cap_ms=invalid_cap;assert(!b::decideReturnAid(g,table,x).valid);}
 {auto x=in;x.return_work_j=1;auto limited=b::decideReturnAid(g,table,x);
  assert(limited.valid && close(limited.request_j,.5f*limited.target_j));}
 // Residual current retains its sign. Splitting the software allowance does
 // not create a second charge credit or turn reverse net Q into positive W.
 {energy_pulse_completion::Input x;x.side=-1;x.rate_dps=in.rate_dps;
  x.initial_current_ma=-150;x.goal_current_ma=328;x.tau_s=table.tau;
  x.available_j=0;x.request_j=.0001f;x.planned_ms=43;x.actual_elapsed_ms=17;
  const auto c=energy_pulse_completion::evaluate(x);
  assert(c.valid && c.q_mas<0 && !c.work_valid && std::isnan(c.work_j));
  assert(c.current_end_ma>0);}
 std::cout<<"v18 half-cycle budget: one aid/main, reservations/actual elapsed, late/short stops, explicit-J inverse, signed current PASS\n";
}
'''

with tempfile.TemporaryDirectory(prefix='half_cycle_energy_budget_') as folder:
    temp = Path(folder)
    source = temp / 'test.cpp'
    source.write_text(CPP, encoding='utf8')
    exe = temp / 'test.exe'
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(ROOT / 'src'), str(source), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
