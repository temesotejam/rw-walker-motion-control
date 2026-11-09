#include "foot_energy_balance.h"
#include <cassert>
#include <cmath>
#include <iostream>
using foot_energy_balance::Controller;
// Synthetic camera samples and alternating commands. They verify software
// transitions only; they do not model the real walker's response.
static void step(Controller& c,unsigned t,bool good=true,bool marker=true,float stage=5.5f,bool change_charge=true,bool reverse=false,bool balanced=false) {
  const float wave=.5f-.5f*std::cos(6.2831853f*t/800.f);
  const float b=c.bias();
  const float left=(balanced ? 10.f+200.f*b : 15.f+(reverse?-100.f:100.f)*b)*wave;
  c.frame(1,t,marker,marker,left,(balanced?10.f:5.f)*wave);
  if(t%800==0)c.peak(-1,t);
  if(t%400==200) {
    c.update(t,stage);const int side=(t%800==200)?1:-1;
    const float q=10.f+(change_charge?(side<0?40.f:-40.f)*c.bias():0.f);
    c.decision(t,side,good,q,side>0?30.f:300.f,8000.f);
  }
}
int main() {
  Controller c;c.reset(1);
  for(unsigned t=0;t<30000;t+=100)step(c,t,false);
  assert(c.phase()==0 && c.bias()==0 && c.valid());
  assert(c.leftMean()>13 && c.rightMean()>4);
  c.reset(1);for(unsigned t=0;t<35000;t+=100)step(c,t,true,true,5.5f,false);
  assert(c.phase()==5 && std::fabs(c.bias())<.011f); // identical commands: no identification
  c.reset(1);for(unsigned t=0;t<35000;t+=100)step(c,t,true);
  assert(c.phase()==4);assert(c.bias()<-.019f); // positive bias worsens left-right excess
  for(unsigned t=35000;t<40000;t+=100)step(c,t,false);
  assert(c.phase()==5 && std::fabs(c.bias())<.001f);
  c.reset(1);for(unsigned t=0;t<16000;t+=100)step(c,t,true);
  assert(c.phase()>=1 && c.phase()<=3);
  for(unsigned t=16000;t<21000;t+=100)step(c,t,true,true,6.f);
  assert(c.phase()==5 && std::fabs(c.bias())<.001f);
  c.reset(1);for(unsigned t=0;t<25000;t+=100)step(c,t,true,false);
  assert(c.phase()==0 && !c.valid() && c.bias()==0);
  c.reset(1);unsigned first=0;
  for(unsigned t=0;t<35000;t+=100) {step(c,t,true,true,5.5f,true,true);if(c.phase()==4){first=t;break;}}
  assert(first && c.bias()<0); // enters from the opposite candidate
  for(unsigned t=first+100;t<first+6000;t+=100){step(c,t,true,true,5.5f,true,true);assert(c.phase()==4);}
  assert(c.bias()>.019f); // wait for transition and settle before judging
  c.reset(1);for(unsigned t=0;t<35000;t+=100)step(c,t,true,true,5.5f,true,false,true);
  assert(c.phase()==5 && std::fabs(c.bias())<.001f); // neutral is better than either extreme
  c.reset(1);for(unsigned t=0;t<35000;t+=100)step(c,t,true);
  assert(c.phase()==4);
  for(unsigned t=35000;t<39000;t+=100)step(c,t,true,false);
  assert(c.phase()==5 && c.reason()==1 && std::fabs(c.bias())<.001f);
  std::cout<<"Foot cycle identification: unsupported, unchanged command, both response signs, stage reset, missing marker and fade PASS\n";
}
