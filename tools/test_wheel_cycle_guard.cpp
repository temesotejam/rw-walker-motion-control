#include "wheel_cycle_guard.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  wheel_cycle_guard::Controller c;
  // Initial alternating samples from run 464 are kept intact.
  assert(c.limit(-1,270.97f,true,417,59)==59);
  assert(c.limit(1,-287.44f,true,814,43)==43);
  // At the first modest drift only a small correction is allowed.
  c.reset();c.limit(1,-346.53f,true,2360,26);
  auto modest=c.limit(-1,4.18f,true,2748,45);
  assert(modest>=43 && modest<=45);
  // Later negative wheel bias limits only the negative-going command.
  c.reset();c.limit(1,-566.3f,true,18748,32);
  auto limited=c.limit(-1,-1.88f,true,19217,110);
  assert(limited<110 && limited>=83);
  assert(c.limit(1,-647.55f,true,19585,32)==32);
  // Symmetry, stale data, invalid speed, and coast do not create torque.
  c.reset();c.limit(-1,600,true,100,30);
  assert(c.limit(1,0,true,500,100)<100);
  c.reset();c.limit(1,-600,true,100,30);
  assert(c.limit(-1,0,true,2000,100)==100);
  c.reset();c.limit(1,-600,true,100,30);
  assert(c.limit(-1,NAN,false,500,100)==100);
  assert(c.limit(1,-600,true,600,0)==0);
  // Startup and a partial observation never qualify the early trial.
  c.reset();c.peak(-1,5.3f,100,true);c.peak(1,5.4f,450,true);
  c.limit(1,-270,true,600,27);assert(c.limit(-1,14,true,900,49)==49);
  assert(c.extraReduction()==0 && !c.trialEligible());
  c.peak(-1,5.51f,800,true);c.peak(1,5.23f,1150,true);
  c.limit(1,-270,true,1200,27);
  auto earlier=c.limit(-1,14,true,1500,49);
  assert(c.trialEligible() && earlier<49 && c.extraReduction()<=5);
  assert(c.legacyWidth()==49 && earlier==c.legacyWidth()-c.extraReduction());
  // A falling same-side peak rejects only the additional trial.
  c.peak(-1,4.9f,1600,true);c.limit(1,-270,true,1700,27);
  assert(c.limit(-1,14,true,2000,49)==49 && !c.trialEligible());
  // Once the bias exceeds 150rpm, unqualified state retains legacy guard.
  c.peak(1,6.5f,2100,false);c.limit(1,-400,true,2200,27);
  auto legacy=c.limit(-1,0,true,2500,60);
  assert(!c.trialEligible() && c.extraReduction()==0 && legacy==c.legacyWidth() && legacy<60);
  // Declining weak-side sway disables the trial even on an opposite-side input.
  c.reset();c.peak(-1,5.f,100,true);c.peak(1,6.f,400,true);
  c.peak(-1,4.5f,700,true);c.peak(1,6.1f,1000,true);
  c.limit(-1,300,true,1100,40);c.limit(1,0,true,1300,60);
  assert(!c.trialEligible() && c.extraReduction()==0);
  // A qualified sequence ages out, and zero-width coast cannot become output.
  c.reset();c.peak(-1,5.f,100,true);c.peak(1,5.f,400,true);
  c.peak(-1,5.f,700,true);c.peak(1,5.f,1000,true);
  c.limit(1,-300,true,1100,30);
  assert(c.limit(-1,0,true,1200,0)==0 && c.extraReduction()==0);
  c.limit(1,-300,true,3000,30);
  assert(c.limit(-1,0,true,3100,60)==60 && !c.trialEligible());
  // Stale, invalid, nonalternating and missing command matches cannot qualify.
  c.reset();c.peak(-1,5.f,100,true);c.peak(1,5.f,200,true);
  c.peak(1,5.f,300,true);c.peak(-1,5.f,400,true);
  c.peak(1,5.f,500,false);c.limit(1,-300,true,600,30);
  c.limit(-1,0,true,900,60);assert(!c.trialEligible());
  std::cout << "wheel cycle guard PASS\n";
}
