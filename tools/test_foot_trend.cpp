#include "foot_trend.h"
#include <cassert>
#include <cmath>
#include <iostream>
using foot_trend::Window;
static bool eq(float a,float b){return std::fabs(a-b)<1e-5f;}
int main(){
  Window w;
  assert(!w.ready() && std::isnan(w.average().left));
  assert(w.cycle(100,200,3,1,5,true));
  assert(w.cycle(200,300,3,1,6,true));
  assert(w.cycle(300,400,1,3,7,true)); // one reversed cycle is retained
  assert(w.count()==3 && !w.ready());
  assert(w.cycle(400,500,3,1,8,true));
  auto a=w.average();assert(w.ready() && a.start_ms==100 && a.end_ms==500);
  assert(eq(a.left,2.5f) && eq(a.right,1.5f) && eq(a.negative_peak,6.5f));
  assert(eq(a.error,(2.f/4.1f)/2.f));
  assert(w.cycle(500,600,1,3,9,true));
  a=w.average();assert(a.start_ms==200 && a.end_ms==600 && eq(a.error,0.f));
  // A temporal gap discards older observations and starts a fresh window.
  assert(w.cycle(700,800,3,1,5,true));assert(w.count()==1 && !w.ready());
  assert(w.cycle(800,900,3,1,5,true));
  assert(!w.cycle(900,1000,3,1,5,false));assert(w.count()==0);
  assert(w.cycle(1000,1100,3,1,5,true));
  assert(!w.cycle(1100,1200,NAN,1,5,true));assert(w.count()==0);
  assert(!w.cycle(1200,1300,-1,1,5,true));
  assert(!w.cycle(1200,1300,.1f,.1f,5,true));
  assert(!w.cycle(1200,1300,1,1,0,true));
  assert(!w.cycle(1300,1300,1,1,5,true));
  w.reset();assert(w.count()==0 && !w.ready());
  std::cout << "four-cycle foot trend: rolling means, reversal and invalidation PASS\n";
}
