#include "tilt_priority.h"
#include <cassert>
#include <cstdint>
#include <initializer_list>

using tilt_priority::Controller;
using tilt_priority::Input;

static void cycles(Controller& c, uint32_t& now, unsigned count,
                   float peak, bool accepted=true, bool alter=false,
                   bool learn=true) {
  for(unsigned i=0;i<count;++i) for(int side : {-1,1}) {
    Input in; in.now_ms=now;in.preset_deg=8.f;in.target_deg=8.f;
    in.crossing_rate_dps=48.f;in.next_side=side;in.hardware_cap_ms=150;in.hardware_healthy=true;
    auto out=c.propose(in);assert(out.selected && !out.coast);
    if(alter && out.width_ms>1)--out.width_ms;
    c.crossing(side,now);c.confirm(side,now,out,accepted,false,learn);
    c.peak(side,now+170,peak);now+=350;
  }
}
static void start(Controller& c,uint32_t& now){
  c.peak(1,100,3.f);now=300;
  cycles(c,now,14,3.f);
  assert(c.target()==8.f);
}
static void asymmetric(Controller& c,uint32_t& now,unsigned count){
  for(unsigned i=0;i<count;++i)for(int side:{-1,1}) {
    Input in;in.now_ms=now;in.preset_deg=8;in.target_deg=8;in.crossing_rate_dps=48;
    in.next_side=side;in.hardware_cap_ms=150;in.hardware_healthy=true;
    auto out=c.propose(in);assert(out.selected&&!out.coast);
    c.crossing(side,now);c.confirm(side,now,out,true);c.peak(side,now+170,side>0?7.f:5.f);now+=350;
  }
}
int main(){
  Controller good;uint32_t t;start(good,t);
  auto before=good.widthLimit(1);cycles(good,t,2,5.5f);
  assert(good.widthLimit(1)==before+5);
  Controller modified;start(modified,t);before=modified.widthLimit(1);
  cycles(modified,t,4,5.5f,true,true);assert(modified.widthLimit(1)==before);
  Controller disabled;start(disabled,t);before=disabled.widthLimit(1);
  cycles(disabled,t,4,5.5f,true,false,false);assert(disabled.widthLimit(1)==before);
  Controller near;start(near,t);before=near.widthLimit(1);
  cycles(near,t,4,7.2f);assert(near.widthLimit(1)==before);
  Controller interrupted;start(interrupted,t);before=interrupted.widthLimit(1);
  cycles(interrupted,t,1,5.5f);cycles(interrupted,t,1,7.2f);
  cycles(interrupted,t,1,5.5f);assert(interrupted.widthLimit(1)==before);
  Controller bounded;start(bounded,t);cycles(bounded,t,16,5.5f);
  assert(bounded.widthLimit(1)==80 && bounded.widthLimit(-1)==80);
  Controller rollback;start(rollback,t);before=rollback.widthLimit(1);
  cycles(rollback,t,4,5.5f);assert(rollback.widthLimit(1)==before+10);
  asymmetric(rollback,t,2);assert(rollback.widthLimit(1)==before && rollback.widthLimit(-1)==before);
  asymmetric(rollback,t,4);assert(rollback.widthLimit(1)==before && rollback.widthLimit(-1)==before);
  cycles(rollback,t,2,5.5f);assert(rollback.widthLimit(1)==before);
  cycles(rollback,t,4,5.5f);assert(rollback.widthLimit(1)>before);
}
