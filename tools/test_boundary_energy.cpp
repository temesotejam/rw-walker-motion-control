#include "wheel_work_energy_control.h"
#include <cassert>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
using namespace wheel_work_energy_control;
static float goal(float command,float voltage) {
  const float sat=329.547119f+46.253815f*(voltage-7.5f);
  return command/std::pow(1.f+std::pow(command/sat,3.86f),1.f/3.86f);
}
static float tau(float command) {
  return (31.4f+(72.8f-31.4f)/(1.f+std::pow(command/372.f,4.35f)))*.001f;
}
int main() {
  const lateral_energy::Geometry g;
  lateral_energy::CurrentTable storage[3];
  const lateral_energy::CurrentTable* tables[]={&storage[0],&storage[1],&storage[2]};
  for(unsigned i=0;i<3;++i)assert(storage[i].prepare(tau(300+50*i)));
  // Energy predictions and applied residuals agree on both sides of each edge.
  for(float boundary : {44.3821f,48.3821f,72.7405f,76.7405f}) {
    const auto a=positiveBoundary(g,State{boundary-.0001f,250,7440,true,9},12,.04f,-.03f);
    const auto b=positiveBoundary(g,State{boundary+.0001f,250,7440,true,9},12,.04f,-.03f);
    assert(a.valid && b.valid && std::fabs(a.energy_mj-b.energy_mj)<.0001f);
    assert(std::fabs(a.applied_bias_mj-b.applied_bias_mj)<.00001f);
  }
  for(float rpm : {0.f,4.28f,575.79f,580.37f,600.79f}) {
    const float a=negativeBoundaryWeight(State{70,rpm-.0001f,7440,true,8});
    const float b=negativeBoundaryWeight(State{70,rpm+.0001f,7440,true,8});
    assert(std::fabs(a-b)<.0001f);
  }
  assert(!negativeBoundary(State{70,620,7440,true,8},25).valid);
  assert(!negativeBoundary(State{70,580,7440,false,8},25).valid);
  assert(!negativeBoundary(State{70,580,8200,true,8},25).valid);
  NegativeFeedback guard;
  for(unsigned cap=25;cap<50;cap+=5) {
    guard.observe(1.f,.98f,.73f,8,cap,true);
    guard.observe(1.f,.98f,.73f,8,cap,true);
  }
  assert(guard.balancedCapMs(g,8,0)==50);
  assert(guard.balancedCapMs(g,7.4f,0)>25 && guard.balancedCapMs(g,7.4f,0)<50);
  guard.observe(1.f,.98f,.73f,7.4f,25,true);
  assert(guard.balancedCapMs(g,8,0)==50); // Temporary weakness does not erase learned cap.
  assert(guard.balancedCapMs(g,6.99f,0)==0);
  uint16_t previous=0;
  for(float a=7;a<8.001f;a+=.001f) {
    const auto cap=guard.balancedCapMs(g,a,-.1f);
    assert(cap>=previous && cap<=50);previous=cap;
  }
  // Constructed states from the public model bounds, not experimental records.
  const float target=lateral_energy::potential(g,8.f)*1000.f;
  const float goals[]={goal(300,7.5f),goal(350,7.5f),goal(400,7.5f)};
  const State low_edge{48.3821f-.25f,250,7500,true,9};
  const auto positive_choice=choosePositive(g,low_edge,0,tables,goals,target,.03f,0.f);
  assert(positive_choice.valid && positive_choice.branch==7);
  assert(positive_choice.width_ms<=100 && positive_choice.current_ma<=400);
  const State high_wheel{70,575.79f+12.5f,7500,true,8};
  const auto negative_choice=chooseNegativeBoundary(high_wheel,40,target,.03f);
  assert(negative_choice.valid && negative_choice.branch==8);
  assert(negative_choice.width_ms>0 && negative_choice.width_ms<=20);
  assert(negative_choice.energy_mj<negativeBoundary(high_wheel,0,.03f).energy_mj);
  std::cout<<"synthetic boundary continuity, hard-domain rejection, energy-reserve guard and candidate limits PASS\n";
}
