"""Energy objective, units, peak/rate feedback and exact guarded-width audit."""
from pathlib import Path
import subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include "walking_energy_policy.h"
#include <cassert>
#include <iostream>
#include <cstdlib>
#undef assert
#define assert(test) do {if(!(test)){std::cerr<<"assertion failed: "<<#test<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
namespace e=walking_energy_policy;
static bool near(float a,float b,float tol=2e-9f){return std::fabs(a-b)<=tol;}
static e::Input input(){
 e::Input in;in.side=1;in.rate_dps=55;in.body_angle_deg=0;in.target_deg=8;
 in.previous_peak_deg=8;in.same_side_peak_deg=6;
 in.initial_current_ma=0;in.goal_current_ma=329.547119f;return in;
}
int main(){
 lateral_energy::Geometry g;lateral_energy::CurrentTable table;assert(table.prepare(.0316f));
 auto in=input();auto d=e::decide(g,table,in);assert(d.valid && d.selected && !d.coast);
 assert(near(d.target_j,lateral_energy::potential(g,8)));
 const float paired_mean=.5f*(lateral_energy::potential(g,8)+lateral_energy::potential(g,6));
 const float paired_imbalance=.5f*(lateral_energy::potential(g,8)-lateral_energy::potential(g,6));
 assert(near(d.mean_peak_energy_j,paired_mean) && near(d.side_imbalance_j,paired_imbalance));
 assert(near(d.common_correction_j,.25f*(d.target_j-paired_mean)));
 assert(near(d.redistribution_j,.125f*paired_imbalance));
 assert(near(d.peak_correction_j,d.common_correction_j+d.redistribution_j));
 float w=55*lateral_energy::kRadPerDeg;
 assert(near(d.effective_kinetic_j,.0003345881499f*w*w));
 assert(near(d.fitted_offset_j,.0001630156686f));
 assert(near(d.available_j,d.effective_kinetic_j+d.fitted_offset_j));
 assert(near(d.predicted_arrival_j,d.available_j+d.predicted_work_j));
 assert(near(d.deficit_j,d.target_j+d.peak_correction_j-d.available_j));
 // The completed same-side peak and current angular rate both affect ENERGY,
 // and therefore actual selected width, not merely a diagnostic field.
 auto weak=in;weak.same_side_peak_deg=2;auto strong=in;strong.same_side_peak_deg=11;
 auto a=e::decide(g,table,weak),b=e::decide(g,table,strong);
 assert(a.peak_correction_j>b.peak_correction_j && a.width_ms>b.width_ms);
 // Equal weak peaks increase common excitation without redistribution.
 auto equal=in;equal.previous_peak_deg=equal.same_side_peak_deg=5;
 auto balanced=e::decide(g,table,equal);
 assert(balanced.common_correction_j>0 && balanced.redistribution_j==0);
 // Exchanging the same two completed observations preserves the common
 // term and reverses redistribution: no energy is created by the latter.
 auto left=in;left.same_side_peak_deg=5;left.previous_peak_deg=9;
 auto right=left;right.side=-1;right.same_side_peak_deg=9;right.previous_peak_deg=5;
 auto l=e::decide(g,table,left),r=e::decide(g,table,right);
 assert(l.redistribution_j>0 && r.redistribution_j<0);
 assert(near(l.redistribution_j+r.redistribution_j,0));
 assert(near(l.common_correction_j,r.common_correction_j));
 assert(near(l.mean_peak_energy_j,r.mean_peak_energy_j));
 assert(l.peak_correction_j>r.peak_correction_j);
 // The other-side observation also changes the actual selected width.
 auto opposite_weak=in;opposite_weak.previous_peak_deg=2;
 auto opposite_strong=in;opposite_strong.previous_peak_deg=11;
 a=e::decide(g,table,opposite_weak);b=e::decide(g,table,opposite_strong);
 assert(a.peak_correction_j>b.peak_correction_j && a.width_ms>b.width_ms);
 auto slow=in;slow.rate_dps=25;auto fast=in;fast.rate_dps=80;
 a=e::decide(g,table,slow);b=e::decide(g,table,fast);
 assert(a.available_j<b.available_j && a.width_ms>b.width_ms && b.coast);
 // Added obstacle work enters the joule deficit, never an additive ms rule.
 auto more=in;more.extra_energy_j=.00005f;a=e::decide(g,table,more);
 assert(a.width_ms>d.width_ms && near(a.deficit_j,d.deficit_j+.00005f));
 more.extra_energy_j=1;a=e::decide(g,table,more);assert(a.extra_energy_clamped && near(a.extra_energy_j,.5f*a.target_j));
 auto early=in;early.body_angle_deg=-4;a=e::decide(g,table,early);
 assert(a.early_state_transfer && a.extrapolated && a.available_j>d.available_j);
 assert(near(a.available_j,lateral_energy::potential(g,4)+d.available_j));
 // No incoming loss is subtracted a second time from the arrival surrogate.
 assert(near(a.inbound_loss_proxy_j,std::fmax(0.f,a.peak_energy_j-a.available_j)));
 auto unready=table;unready.ready=false;assert(!e::decide(g,unready,in).valid);
 for(unsigned field=0;field<8;++field){auto x=in;
  if(field==0)x.rate_dps=NAN;
  if(field==1)x.body_angle_deg=NAN;
  if(field==2)x.target_deg=NAN;
  if(field==3)x.previous_peak_deg=NAN;
  if(field==4)x.same_side_peak_deg=INFINITY;
  if(field==5)x.initial_current_ma=NAN;
  if(field==6)x.goal_current_ma=NAN;
  if(field==7)x.extra_energy_j=-1;
  assert(!e::decide(g,table,x).valid);}
 {auto x=in;x.same_side_peak_deg=NAN;auto y=e::decide(g,table,x);
  assert(y.valid && y.common_correction_j==0 && y.redistribution_j==0 && y.peak_correction_j==0);}
 for(unsigned cap:{0u,81u}){auto x=in;x.cap_ms=cap;assert(!e::decide(g,table,x).valid);}
 {auto x=in;x.side=0;assert(!e::decide(g,table,x).valid);}
 {auto x=in;x.target_deg=30;assert(!e::decide(g,table,x).valid);}
 // Enumerating independently proves the binary inverse chooses the minimum
 // energy error over every allowed integer width, including coast and I0<0.
 for(int side:{-1,1})for(float rate:{.1f,5.5f,6.f,20.f,35.f,50.f,80.f})
 for(float target:{4.5f,6.f,8.f})for(float i0:{-150.f,-5.f,0.f,30.f})for(unsigned cap:{15u,40u,80u}){
  auto x=in;x.side=side;x.rate_dps=rate;x.target_deg=target;x.initial_current_ma=i0;x.cap_ms=cap;
  auto result=e::decide(g,table,x);assert(result.valid && result.width_ms<=cap);
  float best=std::fabs(result.request_j);unsigned best_ms=0;
  const auto coef=lateral_energy::coefficients(side);const float omega=rate*lateral_energy::kRadPerDeg;
  for(unsigned ms=1;ms<=cap;++ms){float q=lateral_energy::charge(table,ms,i0,x.goal_current_ma);if(q<0)continue;
   float err=std::fabs(lateral_energy::work(coef,omega,q)-result.request_j);
   if(err<best){best=err;best_ms=ms;}}
  assert(std::fabs(std::fabs(result.predicted_work_j-result.request_j)-best)<2e-9f);
  assert(result.width_ms==best_ms);
  assert(near(result.coast_residual_q_mas,i0*.070f));
  assert(!result.coast_residual_modelled);
  auto actual=e::evaluateWidth(g,table,x,result.width_ms);
  assert(actual.valid && near(actual.predicted_work_j,result.predicted_work_j));
 }
 auto guarded=e::evaluateWidth(g,table,in,d.width_ms/2);assert(guarded.valid && guarded.predicted_work_j<d.predicted_work_j && guarded.unmet_j>d.unmet_j);
 assert(!e::evaluateWidth(g,table,in,81).valid);
 auto off=e::evaluateWidth(g,table,in,0);assert(off.valid && off.coast && off.q_mas==0 && off.predicted_work_j==0);
 for(float rate:{0.f,-.1f}){auto x=in;x.rate_dps=rate;assert(!e::decide(g,table,x).valid);}
 // Stall rescue confirms at low positive incoming rate. Preserve those calls,
 // with explicit unsupported-rate and static-end-rolloff diagnostics.
 for(int side:{-1,1}){auto x=in;x.side=side;x.rate_dps=5.5f;x.body_angle_deg=-side*3.f;
  x.initial_current_ma=-150.f;auto rescue=e::decide(g,table,x);
  assert(rescue.valid && rescue.passive_intercept_rolloff && rescue.extrapolated);
  assert(!rescue.passive_rate_supported && !rescue.work_rate_supported);
  assert(rescue.q_mas>=0 && near(rescue.coast_residual_q_mas,-10.5f));
  assert(near(rescue.fitted_offset_j,e::passiveCoefficients(side).offset_j*(5.5f/20.f)*(5.5f/20.f)));
  if(rescue.width_ms)assert(near(rescue.q_mas,lateral_energy::charge(table,rescue.width_ms,-150.f,x.goal_current_ma)));
 }
 for(int side:{-1,1}){float previous=-1;
  for(unsigned speed=10;speed<=9000;++speed){auto x=in;x.side=side;x.rate_dps=speed*.01f;
   auto y=e::decide(g,table,x);assert(y.valid && y.available_j>=previous);previous=y.available_j;
   assert(y.passive_intercept_rolloff==(x.rate_dps<20.f));}}
 std::cout<<"walking energy: U/omega units, peak feedback, energy inverse, cap/coast/I0/actual guard PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='walking_energy_') as tmp:
 p=Path(tmp);(p/'test.cpp').write_text(CPP,encoding='utf8')
 subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
 subprocess.run([str(p/'test.exe')],check=True)
