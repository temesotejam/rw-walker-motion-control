#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "lateral_energy_control.h"

// A separate, bounded launch from rest after a walking cycle has stalled.
// Real crossings and signed excursions are observations. A queued command
// alone never confirms a restart.
namespace settled_walking_restart {
constexpr uint32_t quiet_ms=500, inactive_ms=1000, retry_ms=1600;
constexpr uint16_t episode_ms=80;
constexpr uint8_t max_attempts=2; // per measurement, including failed writes
struct Input {
  uint32_t now_ms=0,sample_us=0;
  float angle_deg=NAN,rate_dps=NAN,fore_aft_deg=NAN,fore_aft_rate_dps=NAN;
  float left_deg=NAN,right_deg=NAN;
  bool run_active=false,guarded=false,feet_fresh=false,pulse_active=false,had_peak=false,crossed_zero=false;
};
struct Attempt {
  uint32_t start=0, cross=0, opposite_peak=0, elapsed=0, timeout=0;
  int8_t direction=0;
  uint16_t width=0;
  float angle=NAN, reference_speed=NAN, demand_j=NAN, predicted_j=NAN;
  bool queued=false;
};
class Detector {
 public:
  int8_t observe(const Input& in) {
    if(!in.run_active || !in.sample_us){quiet_start_=0;return 0;}
    if(in.sample_us==sample_)return 0;
    if(in.sample_us-sample_>30000 || !in.guarded) {
      quiet_start_=0;reference_candidate_=0;last_excursion_side_=0;
    }
    sample_=in.sample_us;
    if(pending_ && in.now_ms-records[count-1].start>retry_ms) {
      records[count-1].timeout=in.now_ms;
      pending_=0;
    }
    if(in.guarded && in.crossed_zero && std::isfinite(in.rate_dps)) {
      last_active_=in.now_ms;
      const int8_t side=in.rate_dps>0?1:-1;
      const float speed=std::fabs(in.rate_dps);
      reference_candidate_=last_excursion_side_==-side && speed>=6 && speed<=95 ? side : 0;
      candidate_speed_=speed;
      if(episode_side_ && side==episode_side_)bracket_=true;
    }
    if(in.guarded && std::fabs(in.angle_deg)>=.5f) {
      const int8_t excursion=in.angle_deg>0?1:-1;
      if(excursion==reference_candidate_ && excursion!=last_excursion_side_) {
        auto& history=reference_[excursion>0];
        history.speeds[history.next]=candidate_speed_;
        history.next=(history.next+1)%3;
        if(history.size<3)++history.size;
      }
      reference_candidate_=0;last_excursion_side_=excursion;
    }
    if(in.guarded && episode_side_ && bracket_ && in.angle_deg*episode_side_>=.5f) {
      if(pending_)records[count-1].cross=in.now_ms;
      pending_=episode_side_=0;bracket_=false;used_=0;
    }
    if(in.pulse_active)last_active_=in.now_ms;
    const float lean=std::fabs(in.angle_deg);
    if(!in.guarded || !in.feet_fresh || !in.had_peak || in.pulse_active ||
       !last_active_ || in.now_ms-last_active_<inactive_ms ||
       count>=max_attempts || write_failed_ || (count && in.now_ms-records[count-1].start<retry_ms) ||
       !(lean>=1.f && lean<=9.f && std::fabs(in.rate_dps)<=4.f &&
         std::fabs(in.fore_aft_deg)<=6.f && std::fabs(in.fore_aft_rate_dps)<=12.f &&
         std::fabs(in.left_deg)<=10.f && std::fabs(in.right_deg)<=10.f)) {
      quiet_start_=0;return 0;
    }
    const int8_t direction=in.angle_deg>0?-1:1;
    if(!quiet_start_ || std::fabs(in.angle_deg-quiet_angle_)>.45f ||
        std::fabs(in.left_deg-quiet_left_)>.45f || std::fabs(in.right_deg-quiet_right_)>.45f) {
      quiet_start_=in.now_ms;quiet_angle_=in.angle_deg;quiet_left_=in.left_deg;quiet_right_=in.right_deg;
      return 0;
    }
    return in.now_ms-quiet_start_>=quiet_ms && referenceSpeed(direction)>0 && remaining()>0 ? direction : 0;
  }
  float referenceSpeed(int8_t direction) const {
    const auto& h=reference_[direction>0];
    if(!h.size)return 0;
    if(h.size==1)return h.speeds[0];
    if(h.size==2)return .5f*(h.speeds[0]+h.speeds[1]);
    const float a=h.speeds[0],b=h.speeds[1],c=h.speeds[2];
    return std::max(std::min(a,b),std::min(std::max(a,b),c));
  }
  uint16_t remaining()const{return used_<episode_ms?episode_ms-used_:0;}
  int8_t pending()const{return pending_;}
  bool bracketed()const{return bracket_;}
  Attempt& attempt(uint32_t t,int8_t direction) {
    auto& a=records[count++];a.start=t;a.direction=direction;
    quiet_start_=0;last_active_=t;
    return a;
  }
  void accepted(uint16_t width) {
    auto& a=records[count-1];a.queued=true;a.width=width;
    pending_=episode_side_=a.direction;bracket_=false;reserved_=width;used_+=width;
  }
  void failed(){write_failed_=true;}
  void completed(uint32_t elapsed) {
    if(!reserved_)return;
    records[count-1].elapsed=elapsed;
    if(used_>=reserved_)used_=std::min<uint32_t>(episode_ms,used_-reserved_+elapsed);
    reserved_=0;
  }
  void peak(uint32_t t,int8_t direction) {
    if(!count)return;
    auto& a=records[count-1];
    if(a.cross && !a.opposite_peak && t>=a.cross && a.direction==direction)a.opposite_peak=t;
  }
  Attempt records[max_attempts]{};
  uint8_t count=0;
 private:
  uint32_t sample_=0,quiet_start_=0,last_active_=0;
  uint16_t used_=0,reserved_=0;
  int8_t pending_=0,episode_side_=0,reference_candidate_=0,last_excursion_side_=0;
  bool bracket_=false,write_failed_=false;
  float quiet_angle_=0,quiet_left_=0,quiet_right_=0,candidate_speed_=0;
  struct Reference {float speeds[3]{};uint8_t size=0,next=0;} reference_[2];
};
struct Plan { uint16_t width=0;float demand_j=0,predicted_j=0; };
inline float excitationAtWidth(const lateral_energy::CurrentTable& table,int8_t direction,float rate,
                               float initial_ma,float goal_ma,uint16_t width) {
  if(!table.ready || !width || width>episode_ms || (direction!=-1 && direction!=1))return NAN;
  const auto c=lateral_energy::coefficients(direction);
  const float q=lateral_energy::charge(table,width,initial_ma,goal_ma);
  if(!(q>0 && q<=c.charge_max_mAs))return NAN;
  const float work=lateral_energy::work(c,direction*rate*lateral_energy::kRadPerDeg,q);
  return std::isfinite(work) && work>0 ? work : NAN;
}
inline Plan plan(const lateral_energy::CurrentTable& table,int8_t direction,float reference_speed,
                 float rate,float initial_ma,float goal_ma,uint16_t cap) {
  Plan out;
  if(!table.ready || (direction!=-1 && direction!=1) || !(reference_speed>0 && reference_speed<=95 &&
      std::fabs(rate)<=4 && std::fabs(initial_ma)<=40 && goal_ma>40 && cap && cap<=episode_ms))return out;
  const auto c=lateral_energy::coefficients(direction);
  const float ref=reference_speed*lateral_energy::kRadPerDeg;
  // Empirical speed-energy coefficient; this extrapolation to near rest is
  // a trial prediction. The measured crossing/peak verifies its effect.
  const float aligned=direction*rate*lateral_energy::kRadPerDeg;
  out.demand_j=c.free_omega2*(ref*ref-aligned*aligned);
  if(!(out.demand_j>0))return out;
  for(uint16_t ms=1;ms<=cap;++ms) {
    const float q=lateral_energy::charge(table,ms,initial_ma,goal_ma);
    if(!std::isfinite(q) || q>c.charge_max_mAs)break;
    const float work=lateral_energy::work(c,aligned,q);
    if(q<=0 || !(work>out.predicted_j))continue;
    out.predicted_j=work;out.width=ms;
    if(work>=out.demand_j)break;
  }
  return out;
}
}
