#include "lateral_energy_control.h"
#include "walking_positive_cap.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  using namespace lateral_energy;
  assert(walking_positive_cap::fallback(true,false,1,8.f,80,70,100)==70);
  assert(walking_positive_cap::fallback(true,false,1,8.f,60,90,100)==60);
  assert(walking_positive_cap::fallback(true,false,1,8.f,0,90,100)==0);
  assert(walking_positive_cap::fallback(true,false,1,8.f,110,120,100)==100);
  assert(walking_positive_cap::fallback(true,true,1,8.f,60,70,100)==100);
  assert(walking_positive_cap::fallback(false,false,1,8.f,60,70,100)==100);
  assert(walking_positive_cap::fallback(true,false,-1,8.f,60,70,100)==100);
  assert(walking_positive_cap::fallback(true,false,1,10.f,60,70,100)==100);

  Geometry geometry; CurrentTable table;assert(table.prepare(.05f));
  const float target=potential(geometry,8.f);
  // State chosen to require more than 70 ms in the uncapped fallback.
  auto unrestricted=decideBridge(geometry,table,1,20.f,target,0.f,0.f,300.f);
  auto bridge=decideBridge(geometry,table,1,20.f,target,0.f,0.f,300.f,0.f,70);
  assert(unrestricted.valid && unrestricted.ms>70);
  assert(bridge.valid && bridge.ms<=70 && bridge.upper);
  assert(std::fabs(bridge.selected_q_mAs-charge(table,bridge.ms,0.f,300.f))<1e-4f);
  assert(std::fabs(bridge.predicted_j-bridgeArrivalEnergy(geometry,1,20.f,bridge.selected_q_mAs))<1e-6f);
  auto zero=decideBridge(geometry,table,1,20.f,target,0.f,0.f,300.f,0.f,0);
  assert(zero.valid && zero.ms==0);
  assert(!decideBridge(geometry,table,1,20.f,target,0.f,0.f,300.f,0.f,101).valid);

  // The non-bridge fallback uses the same positive cap; it leaves a
  // negative-torque/braking limit independent.
  auto generic=decide(geometry,table,1,20.f,target,0.f,0.f,300.f,kBrakeMaxMs,0.f,70);
  assert(generic.valid && generic.ms<=70);
  assert(std::fabs(generic.selected_q_mAs-charge(table,generic.ms,0.f,300.f))<1e-4f);
  auto no_drive=decide(geometry,table,1,20.f,target,0.f,0.f,300.f,kBrakeMaxMs,0.f,0);
  assert(no_drive.valid && no_drive.ms==0);
  auto braking=decide(geometry,table,1,60.f,potential(geometry,2.f),0.f,
                      0.f,300.f,kBrakeMaxMs,0.f,0);
  assert(braking.valid && braking.command_sign==-1 && braking.ms>0 && braking.ms<=kBrakeMaxMs);
  assert(!decide(geometry,table,1,20.f,target,0.f,0.f,300.f,kBrakeMaxMs,0.f,101).valid);
  std::cout << "positive fallback: gradual/wheel caps, solver saturation and telemetry PASS\n";
}
