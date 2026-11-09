#include "target12_state_work.h"
#include <cassert>
#include <iostream>
using namespace target12_state_work;
int main() {
  lateral_energy::Geometry g;
  State s{80,100,7470,true,12};s.target_deg=12;s.absolute_reference=true;
  assert(weight(s,1)==1 && weight(s,-1)==1);
  for(int side:{-1,1}) {
    const auto zero=local(s,side,0);
    assert(zero.valid && zero.on_mj==0 && zero.change_mj==0);
    assert(std::fabs(zero.energy_mj-zero.free_mj)<1e-6f);
    float prev=-1;
    for(unsigned i=0;i<=100;++i) {
      float q=i*.1f;
      const auto p=local(s,side,q);
      assert(p.valid && p.energy_mj>=prev);prev=p.energy_mj;
      assert(std::fabs(p.work_mj-p.on_mj-p.after_mj)<1e-6f);
      assert(std::fabs(p.work_mj-zero.work_mj-p.change_mj)<1e-6f);
    }
  }
  const auto brake=local(s,-1,-1),assist=local(s,-1,1),coast=local(s,-1,0);
  assert(brake.energy_mj<coast.energy_mj && assist.energy_mj>coast.energy_mj);
  assert(brake.on_mj<0 && assist.on_mj>0);
  assert(std::fabs(local(s,-1,-.00001f).energy_mj-local(s,-1,.00001f).energy_mj)<1e-5f);
  auto changed=s;changed.wheel_rpm=300;
  assert(local(changed,-1,2).work_mj!=local(s,-1,2).work_mj);
  changed=s;changed.battery_mv=7380;
  assert(local(changed,-1,2).work_mj!=local(s,-1,2).work_mj);
  changed=s;changed.battery_mv=7380-.001f;
  auto boundary=changed;boundary.battery_mv=7380;
  assert(std::fabs(predict(g,changed,-1,1,25).energy_mj-predict(g,boundary,-1,1,25).energy_mj)<1e-4f);
  ContextCaps caps;
  assert(caps.capMs(0,7380)==35 && caps.capMs(0,7550)==40 && caps.capMs(330,7380)==25);
  caps.observe(g,11,0,7470,0,false,0,25);caps.observe(g,11,0,7470,0,false,0,25);
  assert(caps.allowed());
  for(unsigned i=0;i<6;++i)caps.observe(g,11,0,7470,0,true,25,25);
  const auto stopped=caps.capMs(0,7470);assert(stopped>25);
  caps.observe(g,13,330,7470,0,true,29,30);
  assert(caps.capMs(0,7470)==stopped); // high-speed excess cannot shrink stopped-rotor cap
  for(float rpm:{0.f,60.f,149.999f,150.001f,250.f,330.f}) {
    float w[6];contextWeights(rpm,7470,w);float sum=0;
    for(float x:w){assert(x>=0);sum+=x;}assert(std::fabs(sum-1)<1e-5f);
    assert(std::abs(int(caps.capMs(rpm,7470))-int(caps.capMs(rpm+.001f,7470)))<=1);
  }
  Controller ctl;
  const float target=1000*lateral_energy::potential(g,12);
  lateral_energy::CurrentTable table;assert(table.prepare(.061135f));
  const lateral_energy::CurrentTable* tables[]={&table,&table,&table};const float goals[]={260,280,295};
  auto c=ctl.choose(g,s,1,tables,goals,0,target,0,25);
  assert(c.valid && c.width_ms<=100 && c.q_mas<=25 && c.command_sign==1);
  for(unsigned i=0;i<6;++i)ctl.observe(g,-1,11,target,0,7470,0,0,true,25,25);
  c=ctl.choose(g,s,-1,tables,goals,0,target+.5f,0,25);
  assert(c.valid && c.branch==9 && c.command_sign==1 && c.width_ms<=100 && c.q_mas>0);
  c=ctl.choose(g,s,-1,tables,goals,0,.8f,0,25);
  assert(c.valid && c.command_sign==-1 && c.q_mas<=0 && c.branch!=9);
  s.wheel_fresh=false;assert(!ctl.choose(g,s,-1,tables,goals,0,target,0,25).valid);
  s.wheel_fresh=true;s.battery_mv=8200;assert(!ctl.choose(g,s,-1,tables,goals,0,target,0,25).valid);
  s.battery_mv=7470;s.target_deg=8;assert(!ctl.choose(g,s,-1,tables,goals,0,target,0,25).valid);
  ContextFeedback fb;
  fb.observe(g,-1,12,target+.2f,0,7470,0,0);
  assert(std::fabs(fb.bias(0,7470,0,-1))>std::fabs(fb.bias(330,7470,0,-1)));
  std::cout<<"target12 state/work/arrival identities, context caps, smooth transitions, signed selection and hard gates PASS\n";
}
