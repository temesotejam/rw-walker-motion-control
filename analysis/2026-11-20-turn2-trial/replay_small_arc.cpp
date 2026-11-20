// Exercise the firmware observer, work inverse and wheel limiter on recorded inputs.
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#define SUPPORT_TRANSFER_V39 1
#include "return_observation_policy.h"
#include "half_cycle_energy_budget.h"
#include "receiving_arc_aid.h"
#include "wheel_cycle_guard.h"
#include <iostream>
#include <iomanip>
#include <string>

static float number() { std::string s; std::cin>>s; return std::stof(s); }

int main() {
  return_observation_policy::Observer observer;
  receiving_arc_aid::MainReserve reserve;
  wheel_cycle_guard::Controller wheel;
  lateral_energy::Geometry geometry;
  char kind;
  while (std::cin>>kind) {
    if (kind=='K') {
      int side,matched; unsigned peak;
      std::cin>>side>>peak>>matched;
      wheel.peak(side,number(),peak,matched);
    } else if (kind=='M') {
      unsigned time,width; int side;
      std::cin>>time>>side>>width;
      wheel.limit(side,number(),true,time,width);
    } else if (kind=='R') {
      unsigned width; int side,assisted;
      std::cin>>side>>assisted>>width;
      reserve.observe(side,assisted,width);
    } else if (kind=='B') {
      return_observation_policy::Input in; in.run_id=1;
      std::cin>>in.peak_ms;
      observer.observe(in);
    } else if (kind=='P') {
      return_observation_policy::Input in;
      periodic_foot::Packet p;
      int active,side,valid;
      std::cin>>in.now>>in.peak_ms>>side>>active>>in.sample_us>>p.count>>p.epoch_us>>p.scheduled_us>>p.published_us
              >>p.left.sequence>>p.right.sequence>>p.left.image_us>>p.right.image_us;
      p.left.measured=number();p.right.measured=number();
      auto& f=in.foot;
      std::cin>>f.sequence>>f.image_us>>f.frame_ms>>f.ready_ms>>valid;
      f.left_deg=number();f.right_deg=number(); in.peak_deg=number();in.angle_deg=number();
      in.rate_dps=number();in.periodic_acceleration_dps2=number();in.fore_aft_deg=number();
      const float target=number(), i0=number(), goal=number(), rpm=number(), tau=number();
      unsigned recorded_cap; std::cin>>recorded_cap;
      in.side=side;in.active=active;in.periodic=&p;in.run_id=f.run_id=1;
      f.timestamp_valid=f.frame_valid=f.zero_ready=f.left_detected=f.right_detected=f.left_range_ok=f.right_range_ok=valid;
      in.target_j=lateral_energy::potential(geometry,target);
      auto choice=observer.observe(in);
      if (!choice.selected || choice.evidence!=return_observation_policy::Evidence::MeasuredArcShortfall) continue;
      const auto keep=reserve.forSide(-1);
      if(keep>=80) continue;
      auto full_guard=wheel;
      if (!std::isfinite(rpm) || full_guard.limit(-1,rpm,true,in.now,80)!=recorded_cap) return 5;
      walking_return_energy::Input aid;aid.side=-1;aid.rate_dps=-in.rate_dps;aid.body_angle_deg=in.angle_deg;
      aid.target_deg=target;aid.previous_peak_deg=in.peak_deg;aid.initial_current_ma=i0;aid.goal_current_ma=goal;
      aid.cap_ms=80-keep;aid.return_work_j=choice.request_j;
      lateral_energy::CurrentTable current;
      if (!current.prepare(tau)) return 2;
      auto inverse=half_cycle_energy_budget::decideReturnAid(geometry,current,aid);
      auto guard=wheel;
      auto width=guard.limit(-1,rpm,true,in.now,inverse.width_ms);
      auto actual=half_cycle_energy_budget::evaluateReturnAidWidth(geometry,current,aid,width);
      if (!inverse.valid || !actual.valid) return 3;
      std::cout<<std::setprecision(9)<<in.now<<' '<<in.peak_ms<<' '<<choice.request_j<<' '<<keep
               <<' '<<inverse.width_ms<<' '<<width<<' '<<actual.predicted_work_j<<' '<<choice.stopping_distance_deg<<'\n';
      observer.consume(1,in.peak_ms);
    } else return 1;
    if (!std::cin) return 4;
  }
}
