#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include "wheel_work_energy_control.h"

static std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream s(line); std::string value;
  while (std::getline(s,value,',')) out.push_back(value);
  return out;
}
static float goal(float command,float voltage) {
  const float sat=329.547119f+46.253815f*(voltage-7.5f);
  return command/std::pow(1.f+std::pow(command/sat,3.86f),1.f/3.86f);
}
static float tau(float command) {
  return (31.4f+(72.8f-31.4f)/(1.f+std::pow(command/372.f,4.35f)))*.001f;
}
int main() {
  using namespace wheel_work_energy_control;
  const lateral_energy::Geometry geometry;
  const float target=lateral_energy::potential(geometry,8.f)*1000.f;
  lateral_energy::CurrentTable storage[3];
  const lateral_energy::CurrentTable* tables[]={&storage[0],&storage[1],&storage[2]};
  for (unsigned i=0;i<3;++i) assert(storage[i].prepare(tau(300.f+50.f*i)));
  std::ifstream input("tools/fixtures/positive_work_8runs.csv");
  assert(input); std::string line;std::getline(input,line);
  unsigned count=0,latest=0,higher=0;float squared=0;
  while (std::getline(input,line)) {
    const auto row=split(line);assert(row.size()==11);
    const State state{std::stof(row[2]),std::stof(row[3]),std::stof(row[4]),true,std::stof(row[5])};
    const float q=std::stof(row[6]),actual=std::stof(row[7]);
    const auto p=positiveExpanded(state,q);
    assert(p.valid && p.branch==6);
    assert(std::fabs(p.work_mj-p.on_mj-p.after_mj)<1e-5f);
    assert(std::fabs(p.energy_mj-p.free_mj-.2250211722f*p.change_mj)<1e-5f);
    assert(positiveExpanded(state,q+.1f).energy_mj>p.energy_mj);
    if (row[0]=="130765421") {
      squared+=std::pow(lateral_energy::angle(geometry,p.energy_mj*.001f)-actual,2);
      ++latest;
      const float goals[]={goal(300,state.battery_mv*.001f),goal(350,state.battery_mv*.001f),goal(400,state.battery_mv*.001f)};
      const auto c=choosePositive(geometry,state,0,tables,goals,target,0);
      assert(c.valid && c.width_ms<=100 && c.current_ma>=300 && c.current_ma<=400);
      if (c.current_ma>300) ++higher;
    }
    ++count;
  }
  assert(count==180 && latest==26 && std::sqrt(squared/latest)<.20f);
  assert(higher>0); // The recent voltage/RPM can actually select 350/400 mA.
  const float goals[]={goal(300,7.94f),goal(350,7.94f),goal(400,7.94f)};
  const State state{58.f,180.f,7940.f,true,10.f};
  const auto normal=choosePositive(geometry,state,0,tables,goals,target,0);
  const auto deficit=choosePositive(geometry,state,0,tables,goals,target,-.15f);
  assert(normal.valid && deficit.valid && deficit.q_mas>normal.q_mas);
  const State startup{30.f,50.f,7940.f,true,5.f};
  const auto fallback=choosePositive(geometry,startup,0,tables,goals,target,0);
  assert(fallback.valid && fallback.branch==5 && fallback.current_ma>300);
  assert(!choosePositive(geometry,state,0,tables,goals,NAN,0).valid);
  assert(!positiveExpanded(State{52,180,7940,false,10},10).valid);

  NegativeFeedback cap;
  for (unsigned width=25;width<40;width+=5) {
    cap.observe(1.f,.98f,target,8.f,width);cap.observe(1.f,.98f,target,8.f,width);
  }
  assert(cap.capMs(8.f)==40);
  for (unsigned i=0;i<6;++i) cap.observe(1.f,.98f,target,7.7f,40);
  assert(cap.capMs(8.f)==40); // Stronger pulses require a healthy positive peak.
  cap.observe(1.f,.98f,target,8.f,40);cap.observe(1.f,.98f,target,8.f,40);
  assert(cap.capMs(8.f)==45 && cap.capMs(7.7f)==40 && cap.capMs(7.4f)==25);
  cap.observe(1.f,.98f,target,7.7f,40);assert(cap.capMs(8.f)==40);

  std::ifstream neg("tools/fixtures/negative_130765421_states.csv");
  assert(neg);std::getline(neg,line);unsigned n=0,extended=0;
  while (std::getline(neg,line)) {
    const auto row=split(line);assert(row.size()==5);
    const State x{std::stof(row[0]),std::stof(row[1]),std::stof(row[3]),true,std::stof(row[2])};
    Choice original,expanded;
    for (uint16_t width=0;width<=70;width=width==0 ? 10 : width+1) {
      const auto p=negative(x,width,std::stof(row[4]));
      if (width>40 && p.valid) assert(p.cross_condition);
      if (width<=40) consider(original,300,width,NAN,p,target);
      consider(expanded,300,width,NAN,p,target);
    }
    assert(original.valid && expanded.valid);
    assert(std::fabs(expanded.energy_mj-target)<std::fabs(original.energy_mj-target));
    if (expanded.width_ms>40) ++extended;
    ++n;
  }
  assert(n==26 && extended==26);
  std::cout<<"8-run positive predictions, expanded currents, energy inverse and gated negative extension PASS; higher-current selections="<<higher<<"/"<<latest<<"\n";
}
