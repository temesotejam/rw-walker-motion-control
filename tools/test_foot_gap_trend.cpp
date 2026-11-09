#include "foot_gap_trend.h"
#include <cassert>
#include <cmath>
#include <iostream>
using foot_gap_trend::Window;
int main() {
  Window w;
  assert(w.accept(17127,17982,3.10016f,2.72944f,1200,1.f));
  assert(w.accept(17982,18834,3.84595f,.49921f,1200,1.f));
  w.skip(18834,19614);w.skip(19614,20511);w.skip(20511,21291);
  assert(w.accept(21291,22160,3.93578f,1.12996f,1200,1.f));
  assert(w.direction(22200)==0); // three usable cycles never enough
  assert(w.accept(22160,22939,2.53153f,.21827f,1200,1.f));
  assert(w.direction(23140)==1); // 343 observed-quality-only candidate
  assert(w.direction(24440)==0); // data too old for a new request
  w.skip(22939,23781);assert(w.direction(23781)==0); // no latest usable cycle
  assert(w.accept(23781,24585,3.f,2.9f,1200,1.f));
  assert(w.direction(24600)==0); // newest is balanced
  assert(w.accept(24585,25460,1.f,3.f,1200,1.f));
  assert(w.direction(25480)==0); // newest opposite, not a one-sample reversal
  w.reset();
  assert(w.accept(100,200,3,1,1200,1));
  w.skip(200,300);w.skip(300,400);w.skip(400,500);w.skip(500,600);
  assert(w.count()==0); // at most 3 misses in the window
  assert(w.accept(600,700,3,1,1200,1));
  assert(w.accept(700,800,3,1,1200,1));
  assert(w.accept(800,900,3,1,1200,1.06f)); // changed target resets
  assert(w.count()==1 && w.direction(900)==0);
  assert(!w.accept(900,1000,3,1,400,1.06f) && w.count()==0);
  assert(!w.accept(1000,1100,NAN,1,1200,1) && w.count()==0);
  assert(w.accept(1100,1200,3,1,1200,1));
  assert(!w.accept(1300,1400,3,1,1200,1) && w.count()==0); // unknown gap
  std::cout << "foot trend with bounded quality gaps PASS\n";
}
