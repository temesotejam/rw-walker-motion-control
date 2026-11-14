#pragma once
#include "walking_energy_policy_v15.h"

// Effective energy observer, not identification of inertia, moving-foot U,
// gravity-neutral angle or mechanical work. All constants are trial limits.
namespace walking_return_energy {
struct Input : walking_energy_policy::Input {
  float arrival_bias_j=0,return_work_j=0;
};
using Decision=walking_energy_policy::Decision;
inline Decision select(const lateral_energy::Geometry& g,const lateral_energy::CurrentTable& table,
    const Input& in,uint16_t forced=UINT16_MAX){
  auto out=walking_energy_policy_v15::decide(g,table,in);
  if(!out.valid || !std::isfinite(in.arrival_bias_j) || !std::isfinite(in.return_work_j) || in.return_work_j<0)
    {out.valid=out.selected=false;return out;}
  const float limit=.25f*out.target_j;
  out.available_j=std::fmax(0.f,out.available_j+std::fmax(-limit,std::fmin(limit,in.arrival_bias_j)));
  out.deficit_j=out.target_j+out.peak_correction_j+out.extra_energy_j-out.available_j;
  out.request_j=std::fmax(std::fmax(0.f,out.deficit_j),std::fmin(.5f*out.target_j,in.return_work_j));
  const auto c=lateral_energy::coefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  if(out.request_j==0){
    if(forced!=UINT16_MAX && forced!=0){out.valid=out.selected=false;return out;}
    return walking_energy_policy_v15::detail::atWidth(out,table,in,0);
  }
  uint16_t lo=0,hi=in.cap_ms;
  while(hi-lo>1){
    const uint16_t mid=(lo+hi)/2;
    const float q=lateral_energy::charge(table,mid,in.initial_current_ma,in.goal_current_ma);
    if(q<0 || lateral_energy::work(c,omega,q)<out.request_j)lo=mid;else hi=mid;
  }
  const auto chosen=walking_energy_policy_v15::detail::atWidth(out,table,in,hi);
  if(forced==UINT16_MAX)return chosen;
  if(forced>chosen.width_ms){out.valid=out.selected=false;return out;}
  return walking_energy_policy_v15::detail::atWidth(out,table,in,forced);
}
inline Decision decide(const lateral_energy::Geometry& g,const lateral_energy::CurrentTable& t,const Input& in){auto d=select(g,t,in);d.extrapolated=true;return d;}
inline Decision evaluateWidth(const lateral_energy::Geometry& g,const lateral_energy::CurrentTable& t,const Input& in,uint16_t width){auto d=select(g,t,in,width);d.extrapolated=true;return d;}
inline bool earlyAdmissible(const Decision& d){return walking_energy_policy_v15::earlyAdmissible(d);}

class ArrivalFeedback {
 public:
  void reset(){*this=ArrivalFeedback{};}
  float bias(int side,uint32_t now,float target_j)const{
    if((side!=1 && side!=-1) || !std::isfinite(target_j) || target_j<=0)return 0;
    const auto& s=state_[side>0];
    if(s.count<2 || now<s.ms || now-s.ms>3000)return 0;
    return std::fmax(-.25f*target_j,std::fmin(.25f*target_j,s.j));
  }
  void observe(int side,uint32_t now,float measured,float predicted,float applied,float target,bool matched){
    if(!matched || (side!=1 && side!=-1) || !std::isfinite(measured) || measured<0 ||
       !std::isfinite(predicted) || predicted<0 || !std::isfinite(applied) || !std::isfinite(target) || target<=0)return;
    auto& s=state_[side>0];
    if(s.ms && now<=s.ms)return;
    if(s.ms && now-s.ms>3000)s=State{};
    // Recover residual against the uncorrected model. Do not learn a residual
    // of the corrected model as though it were the full model bias.
    const float residual=std::fmax(-.5f*target,std::fmin(.5f*target,measured-predicted+applied));
    s.j=s.count?.75f*s.j+.25f*residual:residual;s.ms=now;if(s.count<255)++s.count;
  }
 private:
  struct State{float j=0;uint32_t ms=0;uint8_t count=0;};State state_[2];
};

struct ReturnInput {
  uint32_t now=0,peak_ms=0,foot_frame_ms=0,foot_ready_ms=0;
  int side=0;float peak_deg=0,angle_deg=0,rate_dps=0,left_deg=NAN,right_deg=NAN,target_j=0;
  bool active=false,foot_valid=false;
};
struct ReturnChoice {bool selected=false;float request_j=0,deceleration_dps2=0,stopping_distance_deg=0;};
class ReturnDetector {
 public:
  void reset(){*this=ReturnDetector{};}
  ReturnChoice observe(const ReturnInput& in){
    ReturnChoice out;
    const float distance=in.angle_deg*in.side,speed=-in.rate_dps*in.side;
    if(!in.active || (in.side!=1 && in.side!=-1) || !std::isfinite(distance) || !std::isfinite(speed) ||
       !std::isfinite(in.peak_deg) || !in.peak_ms || in.now<in.peak_ms || in.now-in.peak_ms<60 || in.now-in.peak_ms>800 ||
       distance<.35f || distance>12 || in.peak_deg-distance<.5f || speed<3 || speed>35 ||
       !in.foot_valid || !in.foot_frame_ms || !in.foot_ready_ms || in.foot_ready_ms<in.foot_frame_ms ||
       in.now<in.foot_ready_ms || in.now<in.foot_frame_ms || in.now-in.foot_frame_ms>160 ||
       !std::isfinite(in.left_deg) || !std::isfinite(in.right_deg) || std::fabs(in.left_deg-in.right_deg)<3 ||
       !std::isfinite(in.target_j) || in.target_j<=0){reset();return out;}
    if(peak_!=in.peak_ms || !anchor_ms_ || in.now<=last_ms_ || in.now-last_ms_>10){
      reset();peak_=in.peak_ms;anchor_ms_=last_ms_=in.now;anchor_speed_=speed;return out;}
    last_ms_=in.now;
    if(in.now-anchor_ms_>=20){
      const float dt=(in.now-anchor_ms_)*.001f;
      acceleration_=(anchor_speed_-speed)/dt;anchor_ms_=in.now;anchor_speed_=speed;
    }
    if(acceleration_<40){pending_=0;return out;}
    out.deceleration_dps2=acceleration_;out.stopping_distance_deg=speed*speed/(2*acceleration_);
    if(out.stopping_distance_deg+1.f>=distance){pending_=0;return out;}
    // Extend the locally observed loss of the effective rate-energy proxy
    // toward mechanical zero. This is a provisional reachability budget,
    // not a calibrated potential barrier or an inferred energy-neutral point.
    const float c=walking_energy_policy::passiveCoefficients(-in.side).omega2_j;
    const float rad=lateral_energy::kRadPerDeg;
    out.request_j=std::fmin(.5f*in.target_j,c*rad*rad*(2*acceleration_*distance-speed*speed));
    if(!pending_)pending_=in.now;
    out.selected=in.now-pending_>=6;
    return out;
  }
 private:
  uint32_t peak_=0,anchor_ms_=0,last_ms_=0,pending_=0;
  float anchor_speed_=0,acceleration_=0;
};
}
