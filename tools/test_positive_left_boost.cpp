#include "positive_left_boost.h"
#include <cassert>
#include <cmath>
#include <iostream>
using positive_left_boost::Controller;
using positive_left_boost::Input;

static Input example() {
  Input x;x.now_ms=1000;x.opposite_peak_ms=800;x.opposite_peak_deg=5;
  x.preset_deg=8;x.side_target_deg=10;x.next_side=1;
  x.wheel_rpm=120;x.battery_mv=8000;x.hardware_healthy=true;x.speed_fresh=true;
  x.normal_valid=true;x.normal_motion_aligned=true;x.normal_ma=300;
  x.normal_width_ms=40;x.normal_cap_ms=100;x.hardware_cap_ms=150;
  return x;
}
static void normalCycle(Controller& c,Input& x,float pos,float neg) {
  assert(!c.propose(x).selected);
  c.confirmNormal(x,x.normal_ma,x.normal_width_ms,1,true);
  c.peak(1,x.now_ms+200,pos,true);
  c.peak(-1,x.now_ms+600,neg,true);
  x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;x.opposite_peak_deg=neg;
}
static void baseline(Controller& c,Input& x) {
  normalCycle(c,x,5.0f,5.0f);normalCycle(c,x,5.2f,5.0f);
  assert(c.baselineCount()==2);
}
static void trial(Controller& c,Input& x,int current,int width,float pos,float neg) {
  const auto b=c.propose(x);
  assert(b.selected && b.current_ma==current && b.width_ms==width);
  c.confirmTrial(x,current,width,1,true);
  assert(c.awaitingPeak());
  c.peak(1,x.now_ms+200,pos,true);
  assert(c.awaitingPeak());
  c.peak(-1,x.now_ms+600,neg,true);
  x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;x.opposite_peak_deg=neg;
}
int main() {
  // Every level is reached with an observed larger positive/negative difference.
  { Controller c;auto x=example();baseline(c,x);
    trial(c,x,500,100,5.6f,5.f);assert(c.accepted());
    trial(c,x,700,100,5.6f,5.f);assert(c.accepted());
    trial(c,x,900,100,5.6f,5.f);assert(c.accepted());
    trial(c,x,1200,100,5.6f,5.f);assert(c.accepted());
    trial(c,x,1200,125,5.6f,5.f);assert(c.accepted());
    trial(c,x,1200,150,5.6f,5.f);assert(c.accepted());
    assert(c.propose(x).width_ms==150);
  }
  // Equal common-mode rises do not count as a one-sided improvement, but
  // remain inconclusive and can progress to higher levels.
  { Controller c;auto x=example();baseline(c,x);
    trial(c,x,500,100,5.6f,5.5f);assert(!c.accepted());
    trial(c,x,700,100,5.7f,5.6f);assert(!c.accepted());
    trial(c,x,900,100,5.8f,5.7f);assert(!c.accepted());
    trial(c,x,1200,100,5.9f,5.8f);assert(!c.accepted());
    trial(c,x,1200,125,6.0f,5.9f);assert(!c.accepted());
    trial(c,x,1200,150,6.1f,6.0f);assert(!c.accepted());
    assert(!c.propose(x).selected); // exhausted: ordinary output resumes
  }
  // Worse differential after a good trial returns to the last good level.
  { Controller c;auto x=example();baseline(c,x);
    trial(c,x,500,100,5.6f,5.f);assert(c.accepted());
    trial(c,x,700,100,5.1f,5.1f);assert(c.accepted());
    auto b=c.propose(x);assert(b.selected&&b.current_ma==500&&b.width_ms==100);
    trial(c,x,500,100,5.6f,5.f);
    b=c.propose(x);assert(b.selected&&b.current_ma==500&&b.width_ms==100);
  }
  { Controller c;auto x=example();baseline(c,x);
    trial(c,x,500,100,5.8f,3.f);assert(!c.accepted());
    assert(!c.propose(x).selected); // opposite fell markedly: normal fallback
  }
  { Controller c;auto x=example();x.hardware_cap_ms=120;baseline(c,x);
    trial(c,x,500,100,5.6f,5.f);trial(c,x,700,100,5.6f,5.f);
    trial(c,x,900,100,5.6f,5.f);trial(c,x,1200,100,5.6f,5.f);
    trial(c,x,1200,120,5.6f,5.f);assert(c.propose(x).width_ms==120);
  }
  { Controller c;auto x=example();baseline(c,x);
    trial(c,x,500,100,9.7f,5.f);assert(c.accepted());
    assert(!c.propose(x).selected); // side target reached
  }
  { Controller c;auto x=example();baseline(c,x);
    trial(c,x,500,100,9.7f,9.6f);assert(!c.accepted());
    assert(!c.propose(x).selected); // shared rise at target is inconclusive
  }
  { Controller c;auto x=example();baseline(c,x);
    trial(c,x,500,100,7.f,8.1f);assert(!c.accepted());
    assert(!c.propose(x).selected); // opposite side reached its nominal target
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,false);assert(!c.awaitingPeak()&&!c.accepted());
    c.peak(1,x.now_ms+200,7,true);assert(c.baselineCount()==0);
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,700,100,1,true);assert(!c.awaitingPeak());
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,-1,true);assert(!c.awaitingPeak());
  }
  { Controller c;auto x=example();c.confirmNormal(x,300,40,1,true);
    c.peak(1,x.now_ms+200,5,true);assert(c.baselineCount()==0 && c.awaitingPeak());
    c.peak(-1,x.now_ms+600,5,true);assert(c.baselineCount()==1);
  }
  { Controller c;auto x=example();c.confirmNormal(x,300,40,1,true);
    c.peak(1,x.now_ms+20,5,true);assert(c.baselineCount()==0&&!c.awaitingPeak());
  }
  { Controller c;auto x=example();baseline(c,x);x.hardware_healthy=false;
    assert(!c.propose(x).selected);x.hardware_healthy=true;assert(!c.propose(x).selected);
  }
  { Controller c;auto x=example();baseline(c,x);x.speed_fresh=false;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.wheel_rpm=NAN;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.battery_mv=0;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.opposite_peak_ms=x.now_ms-1501;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.hardware_cap_ms=99;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.normal_motion_aligned=false;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.normal_valid=false;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,5.5f,true);
    c.peak(-1,x.now_ms+600,4.9f,false);assert(!c.accepted());
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,true);x.now_ms+=1801;x.opposite_peak_ms=x.now_ms-200;
    assert(!c.propose(x).selected&&!c.awaitingPeak());
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,16.f,true);
    assert(!c.awaitingPeak()&&!c.accepted());
  }
  std::cout<<"paired peak contrast and exploratory 1200mA/150ms levels PASS\n";
}
