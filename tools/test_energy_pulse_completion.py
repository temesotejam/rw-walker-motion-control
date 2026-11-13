"""Actual-elapsed analytic current prediction and provisional completion work."""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include "energy_pulse_completion.h"
#include <iostream>
#include <cstdlib>
#define CHECK(x) do {if(!(x)){std::cerr<<"check failed: "<<#x<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
namespace completion=energy_pulse_completion;
static bool close(float a,float b,float tolerance=2e-6f){return std::fabs(a-b)<=tolerance;}
static completion::Input input(){completion::Input in;in.side=1;in.rate_dps=55;
 in.initial_current_ma=0;in.goal_current_ma=329.547119f;in.tau_s=.0316f;
 in.available_j=.0004f;in.request_j=.0007f;in.planned_ms=80;return in;}
// Integrate the instantaneous current numerically as an independent charge
// check, especially beyond the existing CurrentTable's 100ms coverage.
static double numericQ(const completion::Input& in){
 const unsigned steps=100000;const double dt=in.actual_elapsed_ms*.001/steps;
 double result=0;for(unsigned k=0;k<steps;++k){const double t=(k+.5)*dt;
  result+=(in.goal_current_ma+(in.initial_current_ma-in.goal_current_ma)*std::exp(-t/in.tau_s))*dt;}
 return result;
}
int main(){
 auto in=input();in.initial_current_ma=-150;auto zero=completion::evaluate(in);
 CHECK(zero.valid && zero.work_valid && !zero.duration_completed);
 CHECK(zero.q_mas==0 && zero.work_j==0 && zero.current_end_ma==-150);
 CHECK(zero.arrival_j==in.available_j && zero.unmet_j==in.request_j);
 in=input();in.actual_elapsed_ms=17;auto aborted=completion::evaluate(in);
 in.actual_elapsed_ms=80;auto full=completion::evaluate(in);
 in.actual_elapsed_ms=123;auto late=completion::evaluate(in);
 CHECK(aborted.valid && aborted.work_valid && !aborted.duration_completed && aborted.width_supported);
 CHECK(full.valid && full.work_valid && full.duration_completed && full.width_supported);
 CHECK(late.valid && late.work_valid && late.duration_completed && !late.width_supported && late.extrapolated);
 CHECK(aborted.q_mas<full.q_mas && full.q_mas<late.q_mas);
 CHECK(aborted.work_j<full.work_j && full.work_j<late.work_j);
 CHECK(aborted.unmet_j>full.unmet_j && full.unmet_j>=late.unmet_j);
 CHECK(std::fabs(numericQ(in)-late.q_mas)<2e-6);
 CHECK(late.actual_elapsed_ms==123 && close(late.elapsed_s,.123f));
 CHECK(close(late.arrival_j,in.available_j+late.work_j,1e-9f));
 CHECK(close(late.unmet_j,std::fmax(0.f,in.request_j-late.work_j),1e-9f));
 // Compare the new analytic formula with the existing actual current model
 // at every supported integer width, without changing either model's gains.
 lateral_energy::CurrentTable table;CHECK(table.prepare(in.tau_s));
 for(int side:{-1,1})for(float i0:{-500.f,-150.f,-5.f,0.f,30.f})
 for(unsigned ms=0;ms<=100;++ms){
  auto x=input();x.side=side;x.initial_current_ma=i0;x.actual_elapsed_ms=ms;
  auto d=completion::evaluate(x);CHECK(d.valid);
  const float q=lateral_energy::charge(table,ms,i0,x.goal_current_ma);
  CHECK(close(d.q_mas,q,5e-6f));
  if(d.q_mas<0){CHECK(!d.work_valid && !d.charge_supported && d.extrapolated);
   CHECK(std::isnan(d.work_j) && std::isnan(d.arrival_j) && std::isnan(d.unmet_j));}
  else {CHECK(d.work_valid);
   CHECK(close(d.work_j,lateral_energy::work(lateral_energy::coefficients(side),
           x.rate_dps*lateral_energy::kRadPerDeg,d.q_mas),1e-9f));}
 }
 auto negative=input();negative.initial_current_ma=-150;negative.actual_elapsed_ms=17;
 auto reverse=completion::evaluate(negative);
 CHECK(reverse.valid && reverse.q_mas<0 && reverse.current_end_ma>0);
 CHECK(!reverse.work_valid && std::isnan(reverse.work_j) && std::isnan(reverse.unmet_j));
 // Reporting never clips elapsed time to planned80, historical100 or any
 // other controller limit; even a long observation remains auditable.
 for(unsigned duration:{123u,1000u,UINT32_MAX}){
  auto x=input();x.actual_elapsed_ms=duration;auto d=completion::evaluate(x);
  CHECK(d.valid && d.work_valid && !d.width_supported && d.actual_elapsed_ms==duration);
  CHECK(d.q_mas>full.q_mas && d.work_j>full.work_j && d.duration_completed);
 }
 {auto x=input();x.actual_elapsed_ms=17;x.planned_ms=150;auto d=completion::evaluate(x);
  CHECK(d.valid && !d.width_supported && !d.duration_completed && d.extrapolated);}
 for(float rate:{0.f,5.5f,55.f,100.f}){auto x=input();x.rate_dps=rate;x.actual_elapsed_ms=17;
  auto d=completion::evaluate(x);auto c=lateral_energy::coefficients(x.side);
  CHECK(d.valid && d.work_valid);
  CHECK(d.rate_supported==(rate>=c.rate_min_dps && rate<=c.rate_max_dps));}
 for(unsigned field=0;field<8;++field){auto x=input();
  if(field==0)x.side=0;
  if(field==1)x.rate_dps=-1;
  if(field==2)x.initial_current_ma=NAN;
  if(field==3)x.goal_current_ma=0;
  if(field==4)x.tau_s=0;
  if(field==5)x.available_j=NAN;
  if(field==6)x.request_j=-1;
  if(field==7)x.rate_dps=INFINITY;
  auto d=completion::evaluate(x);CHECK(!d.valid && !d.work_valid);}
 std::cout<<"completion prediction: actual0/17/80/123ms, signedQ/current, no clamp, table/numeric integration PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='energy_completion_') as temporary:
    tmp=Path(temporary)
    (tmp/'test.cpp').write_text(CPP,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src'),
                    str(tmp/'test.cpp'),'-o',str(tmp/'test.exe')],check=True)
    subprocess.run([str(tmp/'test.exe')],check=True)
