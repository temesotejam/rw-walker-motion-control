#include "walking_energy_loss.h"
#include <cassert>
#include <iostream>
int main(int argc,char** argv) {
  using namespace walking_energy_loss;
  using wheel_work_energy_control::State;
  const lateral_energy::Geometry g;
  lateral_energy::CurrentTable table;assert(table.prepare(.06f));
  const float target=1000.f*lateral_energy::potential(g,8.f);
  Feedback f;State s{32,560,7840,true,5.5f};s.absolute_reference=true;
  assert(supported(s));
  State alternate{30,520,7690,true,5.5f};alternate.absolute_reference=true;
  assert(supported(alternate));
  State low{35.5324f,514.35f,7610,true,4.30066f};low.absolute_reference=true;
  auto low_choice=choose(g,low,table,0,280,target,f,0);
  assert(low_choice.valid && low_choice.walking_loss_mj<0.f);
  State low_rpm{30,446.55f,7610,true,5.2f};low_rpm.absolute_reference=true;
  assert(choose(g,low_rpm,table,0,280,target,f,0).valid);
  assert(choose(g,alternate,table,0,280,target,f,0).valid);
  alternate.battery_mv=7000;assert(supported(alternate));
  alternate.battery_mv=8120;assert(supported(alternate));
  alternate.battery_mv=8121;assert(!supported(alternate));
  State new_coast{13.8f,277.8f,7510.f,true,7.37f};new_coast.absolute_reference=true;
  assert(supported(new_coast));assert(choose(g,new_coast,table,0,280,target,f,0).valid);
  auto c=choose(g,s,table,0,280,target,f,0);
  assert(c.valid&&c.branch==10&&c.width_ms==0&&c.walking_loss_mj>0);
  assert(c.cross_condition&&c.support_weight==0);
  assert(std::fabs(c.free_mj+c.walking_loss_mj-1000.f*lateral_energy::potential(g,s.previous_peak_deg))<1e-5f);
  const float raw=coastMj(s.rate_dps);
  assert(f.observe(raw-.03f,raw,target));assert(f.capMs()==0);
  assert(f.observe(raw-.03f,raw,target));assert(f.capMs()==25);
  c=choose(g,s,table,0,280,target,f,0);
  assert(c.valid&&c.width_ms==25&&c.current_ma==300&&c.command_sign==1);
  assert(c.target_above_reachable&&c.q_mas>0&&c.q_mas<=25);
  assert(std::fabs(c.free_mj-(raw-.03f))<1e-5f); // residual exactly once
  assert(std::fabs(c.energy_mj-(c.free_mj+.22502117220f*.06189960458f*c.q_mas*s.rate_dps*wheel_work_energy_control::kRadPerDeg))<1e-5f);
  assert(!f.observe(NAN,raw,target));assert(!f.observe(raw+1,raw,target));
  for(int i=0;i<20;++i)f.observe(raw,raw,target);
  assert(f.capMs()==25); // no unsupported escalation
  for(int i=0;i<9;++i) {
    State bad=s;
    if(i==0)bad.wheel_fresh=false;
    if(i==1)bad.rate_dps=13.f;
    if(i==2)bad.rate_dps=38.f;
    if(i==3)bad.wheel_rpm=620;
    if(i==4)bad.battery_mv=6900;
    if(i==5)bad.previous_peak_deg=7.7f;
    if(i==6)bad.target_deg=10;
    if(i==7)bad.absolute_reference=false;
    if(i==8)bad.rate_dps=NAN;
    assert(!choose(g,bad,table,0,280,target,f,0).valid);
  }
  assert(!choose(g,s,table,NAN,280,target,f,0).valid);
  assert(!choose(g,s,table,0,280,target,f,.1f).valid);
  Feedback over;assert(over.observe(target,target,target));
  assert(over.observe(target,target,target));assert(over.capMs()==0);
  assert(!f.observe(target+.5f,raw,target));assert(f.capMs()==0);
  Feedback adaptive;
  assert(adaptive.observe(raw,raw,target));assert(adaptive.observe(raw,raw,target));
  assert(adaptive.capMs()==25);
  assert(adaptive.observe(raw,raw,target,25));assert(adaptive.capMs()==25);
  assert(adaptive.observe(raw,raw,target,25));assert(adaptive.capMs()==30);
  for(int i=0;i<50;++i)assert(adaptive.observe(raw,raw,target,adaptive.capMs()));
  assert(adaptive.capMs()==60);
  assert(adaptive.observe(target,target,target,60));assert(adaptive.capMs()==0);
  assert(adaptive.observe(raw,raw,target));assert(adaptive.capMs()==55);
  // Extension requires fresh, positive arrival speed and three matched
  // capped arrivals. Missing or low speed returns the cap to 60 ms.
  Feedback speed;assert(speed.observe(raw,raw,target));assert(speed.observe(raw,raw,target));
  for(int i=0;i<60 && speed.capMs()<60;++i)assert(speed.observe(raw,raw,target,speed.capMs()));
  for(int i=0;i<3;++i)assert(speed.observe(raw,raw,target,60,40.f,true));
  assert(speed.capMs()==60);
  assert(speed.observe(raw,raw,target,60,40.f,true));
  assert(!speed.observe(NAN,raw,target,60,40.f,true));
  for(int i=0;i<10 && speed.capMs()==60;++i)assert(speed.observe(raw,raw,target,60,40.f,true));
  assert(speed.capMs()==65);
  assert(speed.observe(raw,raw,target,65,NAN,false));assert(speed.capMs()==60);
  for(int i=0;i<60 && speed.capMs()<75;++i)assert(speed.observe(raw,raw,target,speed.capMs(),40.f,true));
  assert(speed.capMs()==75);
  assert(speed.observe(raw,raw,target,75,29.f,true));assert(speed.capMs()==60);
  assert(speed.observe(target,target,target,60,40.f,true));assert(speed.capMs()==0);
  // A real reduction of arrival energy after increasing width rolls back and
  // locks the exploration ceiling for this run, even if target is missed.
  Feedback worse;assert(worse.observe(raw,raw,target));assert(worse.observe(raw,raw,target));
  for(int i=0;i<50 && worse.capMs()<50;++i)assert(worse.observe(raw,raw,target,worse.capMs(),100,true));
  assert(worse.capMs()==50);
  for(int i=0;i<3;++i)assert(worse.observe(raw-.05f,raw,target,50,100,true));
  assert(worse.capMs()==45);
  for(int i=0;i<12;++i)assert(worse.observe(raw,raw,target,45,100,true));
  assert(worse.capMs()==45);
  Feedback interrupted;assert(interrupted.observe(raw,raw,target));assert(interrupted.observe(raw,raw,target));
  for(int i=0;i<50 && interrupted.capMs()<50;++i)assert(interrupted.observe(raw,raw,target,interrupted.capMs(),100,true));
  interrupted.interrupt();assert(interrupted.capMs()==45);
  // The low-rpm domain starts with two matched coast peaks. Nominal-region
  // arrivals never count as low-rpm coasts, and evidence expires or is reset.
  State ls{30,260,7990,true,5.2f};ls.absolute_reference=true;
  LowSpeedGuard lowguard;
  assert(lowSpeedSupported(ls) && !supported(ls));
  auto lc=choose(g,ls,table,0,280,target,f,0,100,&lowguard,1000);
  assert(lc.valid && lc.branch==11 && lc.width_ms==0 && lc.cross_condition);
  const float lraw=lc.energy_mj;
  lowguard.observe(lraw,lraw,target,0,true,1200);
  assert(lowguard.capMs(1500,target)==0);
  lowguard.bridge(lraw,lraw,target,290,true,1800);
  lowguard.observe(lraw,lraw,target,0,true,2400);
  assert(lowguard.capMs(2500,target)==10);
  assert(lowguard.capMs(2500,target+.04f)==0);
  lc=choose(g,ls,table,0,280,target,f,0,100,&lowguard,2500);
  assert(lc.valid && lc.branch==11 && lc.width_ms<=10);
  lowguard.observe(lraw,lraw,target,10,true,3000);
  assert(lowguard.capMs(3100,target)==10);
  lowguard.observe(lraw,lraw,target,10,true,3800);
  assert(lowguard.capMs(3900,target)==15);
  assert(lowguard.capMs(8001,target)==0);
  lowguard.bridge(lraw,lraw+.07f,target,290,true,4000);
  assert(lowguard.capMs(4100,target)==0); // intervening model mismatch
  lowguard.observe(lraw,lraw,target,0,true,4300);
  lowguard.observe(lraw,lraw,target,0,true,5100);
  lowguard.observe(target,target,target,10,true,5900); // overshoot locks extension
  lowguard.observe(lraw,lraw,target,0,true,6700);
  assert(lowguard.capMs(6800,target)==0);
  LowSpeedGuard worse_low;
  worse_low.observe(lraw,lraw,target,0,true,1000);
  worse_low.observe(lraw,lraw,target,0,true,1800);
  worse_low.observe(lraw-.04f,lraw,target,10,true,2600);
  worse_low.observe(lraw-.04f,lraw,target,10,true,3400);
  assert(worse_low.capMs(3500,target)==0); // deteriorating matched residual
  LowSpeedGuard missing_cycle;missing_cycle.observe(lraw,lraw,target,0,true,1000);
  missing_cycle.bridge(lraw,lraw,target,290,true,3000);
  missing_cycle.observe(lraw,lraw,target,0,true,3800);
  assert(missing_cycle.capMs(3900,target)==0); // missed intermediate cycle
  LowSpeedGuard missing;missing.observe(lraw,lraw,target,0,true,1000);
  missing.interrupt();missing.observe(lraw,lraw,target,0,true,1800);
  assert(missing.capMs(1900,target)==0);
  if(argc>1) {
    float rate,rpm,battery,previous,peak;unsigned count=0;float squared=0;
    while(std::cin>>rate>>rpm>>battery>>previous>>peak) {
      State recorded{rate,rpm,battery,true,previous};recorded.absolute_reference=true;
      Feedback coast;auto pred=choose(g,recorded,table,0,280,target,coast,0);
      assert(pred.valid&&pred.width_ms==0);
      const float error=pred.energy_mj-1000.f*lateral_energy::potential(g,peak);
      squared+=error*error;++count;
    }
    if(argv[1][0]=='l') assert(count==26&&std::sqrt(squared/count)<.028f);
    else if(argv[1][0]=='f') assert(count==18&&std::sqrt(squared/count)<.043f);
    else if(argv[1][0]=='t') assert(count==31&&std::sqrt(squared/count)<.031f);
    else assert(count==32&&std::sqrt(squared/count)<.019f);
  }
  std::cout<<"Walking loss balance, one residual, strict domain, bounded assist and recorded coast replay PASS\n";
}
