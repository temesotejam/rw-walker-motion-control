#pragma once
#include "walking_energy_policy.h"

// Effective energy observer, not identification of inertia, moving-foot U,
// gravity-neutral angle or mechanical work. All constants are trial limits.
namespace walking_return_energy {
struct Input : walking_energy_policy::Input {
  float arrival_bias_j=0,return_work_j=0;
#if WALKING_TURN_OBSERVER
  // Signed change to the ordinary positive work request. Return aid has its own budget.
  float turn_adjustment_j=0;
#endif
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
#if WALKING_TURN_OBSERVER
  if (!std::isfinite(in.turn_adjustment_j)) {out.valid=out.selected=false;return out;}
  out.deficit_j+=in.turn_adjustment_j;
#endif
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

// v18 separates a pre-centre return aid from the main arrival-energy decision.
// The 80ms limit is the inherited experimental software envelope, not an
// identified physical bound. Elapsed time and current/work remain predictions
// based on software observations, not measurements of mechanical work.
namespace half_cycle_energy_budget {
constexpr char kRevision[]="return_aid_then_main_energy_v18";
constexpr uint16_t kEnvelopeMs=walking_energy_policy::kMaxPulseMs;
enum class Kind : uint8_t { ReturnAid,Main,PostAid,Recheck };

class Ledger {
 public:
  void reset(){*this=Ledger{};}
  // Call for an accepted source peak. Repeated calls for that peak preserve
  // its spending. A stale peak, or a new peak while output is unresolved,
  // cannot restore allowance. Run-relative peak timestamps are ordered.
  bool sourcePeak(uint32_t peak){
    if(!peak)return false;
    if(peak==peak_ms_)return true;
    if(active_ || (peak_ms_ && peak<peak_ms_))return false;
    reset();peak_ms_=peak;return true;
  }
  uint16_t cap(uint32_t peak,uint16_t requested_cap=kEnvelopeMs)const{
    if(!matches(peak))return 0;
    const uint16_t available=remaining();
    return requested_cap<available?requested_cap:available;
  }
  bool allowAid(uint32_t peak)const{
    return matches(peak) && !active_ && !aid_used_ && !main_used_ && remaining()>0;
  }
  bool allowPostAid(uint32_t peak)const{
    return matches(peak) && !active_ && aid_used_ && !post_aid_used_ && !main_used_ && remaining()>0;
  }
  bool allowRecheck(uint32_t peak)const{
    return matches(peak) && !active_ && post_aid_used_ && !recheck_used_ && !main_used_ && remaining()>0;
  }
  // A main coast may consume the real ZERO even if the aid used the entire
  // envelope. Positive main widths must still pass cap()/accepted().
  bool allowMain(uint32_t peak)const{
    return matches(peak) && !active_ && !main_used_;
  }
  // Call only after transport acceptance, or for a valid zero-width main
  // coast. A failed write has no reservation. Uncertain transport is handled
  // by the runner's existing unresolved-command stop, never by retrying here.
  bool accepted(Kind kind,uint16_t width){
    if(kind==Kind::ReturnAid){
      if(!allowAid(peak_ms_) || !width || width>remaining())return false;
      aid_used_=true;
    }else if(kind==Kind::PostAid){
      if(!allowPostAid(peak_ms_) || !width || width>remaining())return false;
      post_aid_used_=true;
    }else if(kind==Kind::Recheck){
      if(!allowRecheck(peak_ms_) || !width || width>remaining())return false;
      recheck_used_=true;
    }else if(kind==Kind::Main){
      if(!allowMain(peak_ms_) || width>remaining())return false;
      main_used_=true;
    }else return false;
    reserved_ms_=width;active_=width>0;return true;
  }
  // Completion is counted once. Replace the in-flight planned reservation by
  // all actual elapsed software time, including a late stop. Never clamp the
  // observation to the planned width or let uint32 accumulation wrap.
  bool complete(uint32_t actual_elapsed){
    if(!active_)return false;
    spent_ms_=actual_elapsed>UINT32_MAX-spent_ms_?UINT32_MAX:spent_ms_+actual_elapsed;
    active_=false;reserved_ms_=0;return true;
  }
  bool aidUsed()const{return aid_used_;}
  bool mainUsed()const{return main_used_;}
  bool active()const{return active_;}
  uint32_t peakMs()const{return peak_ms_;}
  uint32_t spentMs()const{return spent_ms_;}
  uint16_t reservedMs()const{return reserved_ms_;}
  uint16_t remaining()const{
    if(spent_ms_>=kEnvelopeMs)return 0;
    const uint16_t unspent=static_cast<uint16_t>(kEnvelopeMs-spent_ms_);
    return reserved_ms_>=unspent?0:static_cast<uint16_t>(unspent-reserved_ms_);
  }
 private:
  bool matches(uint32_t peak)const{return peak && peak==peak_ms_;}
  uint32_t peak_ms_=0,spent_ms_=0;
  uint16_t reserved_ms_=0;
  bool active_=false,aid_used_=false,post_aid_used_=false,recheck_used_=false,main_used_=false;
};

namespace detail {
inline walking_return_energy::Decision selectReturnAid(
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    const walking_return_energy::Input& in,uint16_t forced=UINT16_MAX){
  // Reuse current validation and the same empirical excitation response. Its
  // ordinary-arrival request is discarded: neither current-angle nominal U,
  // fitted passive reservoir nor arrival feedback pays for this return aid.
  auto out=walking_energy_policy_v15::decide(geometry,current,in);
  if(!out.valid || !std::isfinite(in.return_work_j) || in.return_work_j<=0){
    out.valid=out.selected=false;return out;
  }
  out.available_j=0;out.effective_kinetic_j=0;out.fitted_offset_j=0;
  out.common_correction_j=out.redistribution_j=out.peak_correction_j=0;
  out.extra_energy_j=0;out.inbound_loss_proxy_j=0;
  out.request_j=std::fmin(.5f*out.target_j,in.return_work_j);
  out.deficit_j=out.request_j;
  const auto c=lateral_energy::coefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  uint16_t lo=0,hi=in.cap_ms;
  while(hi-lo>1){
    const uint16_t mid=static_cast<uint16_t>((lo+hi)/2);
    const float q=lateral_energy::charge(current,mid,in.initial_current_ma,in.goal_current_ma);
    if(q<0 || lateral_energy::work(c,omega,q)<out.request_j)lo=mid;else hi=mid;
  }
  if(forced!=UINT16_MAX && forced>hi){out.valid=out.selected=false;return out;}
  out=walking_energy_policy_v15::detail::atWidth(out,current,in,forced==UINT16_MAX?hi:forced);
  out.extrapolated=true;return out;
}
}
inline walking_return_energy::Decision decideReturnAid(
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    const walking_return_energy::Input& in){
  return detail::selectReturnAid(geometry,current,in);
}
inline walking_return_energy::Decision evaluateReturnAidWidth(
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    const walking_return_energy::Input& in,uint16_t actual_width){
  // UINT16_MAX is the private selector's automatic-width sentinel. It is not
  // a valid externally guarded command, nor may any other oversized width
  // silently turn back into a fresh selection.
  if(actual_width>in.cap_ms || actual_width>kEnvelopeMs)return walking_return_energy::Decision{};
  return detail::selectReturnAid(geometry,current,in,actual_width);
}
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
// During a replan the measured starting current may still be driving left.
// Count just the modeled gain over continuing that current's zero-command
// tail for the same interval. Both predictions use the same observed rate.
inline float postAidCoastWork(float speed,float initial_current,uint32_t width_ms) {
  const float tau=walking_energy_policy::kCoastFallTauS;
  const float charge=-initial_current*tau*std::expm1(-.001f*width_ms/tau);
  return lateral_energy::work(lateral_energy::coefficients(-1),speed*lateral_energy::kRadPerDeg,charge);
}
inline walking_return_energy::Decision selectPostAid(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const walking_return_energy::Input& in,
    float gross_limit,uint16_t forced=UINT16_MAX) {
  using Decision=walking_return_energy::Decision;
  if (!(in.side==-1 && in.initial_current_ma>=0 && in.initial_current_ma<in.goal_current_ma &&
        gross_limit>0 && std::isfinite(gross_limit))) return Decision{};
  auto out=decideReturnAid(geometry,current,in);
  if (!out.valid) return Decision{};
  const auto c=lateral_energy::coefficients(-1);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  const auto charge=[&](uint16_t w){return lateral_energy::charge(current,w,in.initial_current_ma,in.goal_current_ma);};
  const auto supported=[&](uint16_t w){
    const float q=charge(w);
    return q>=0 && q<=c.charge_max_mAs && lateral_energy::work(c,omega,q)<=gross_limit;
  };
  uint16_t lo=0,hi=in.cap_ms;
  if (!supported(hi)) {
    while (hi-lo>1) {
      const uint16_t mid=(lo+hi)/2;
      if (supported(mid)) lo=mid; else hi=mid;
    }
    hi=lo;
  }
  if (!hi || (forced!=UINT16_MAX && (!forced || forced>hi))) return Decision{};
  if (forced==UINT16_MAX) {
    lo=0;
    while (hi-lo>1) {
      const uint16_t mid=(lo+hi)/2;
      const float gain=lateral_energy::work(c,omega,charge(mid))-postAidCoastWork(in.rate_dps,in.initial_current_ma,mid);
      if (gain<out.request_j) lo=mid; else hi=mid;
    }
  } else hi=forced;
  out=walking_energy_policy_v15::detail::atWidth(out,current,in,hi);
  const float extra=out.predicted_work_j-postAidCoastWork(in.rate_dps,in.initial_current_ma,hi);
  if (!(extra>0)) return Decision{};
  out.unmet_j=std::fmax(0.f,out.request_j-extra);
  out.saturated=out.unmet_j>walking_energy_policy_v15::kEnergyRoundoffJ;
  out.reason=out.saturated?walking_energy_policy::Reason::CapLimited:walking_energy_policy::Reason::Drive;
  return out;
}
#endif
// At real ZERO, recompute ordinary arrival-energy demand from its fresh state
// and signed residual current using the remaining cap. Do not subtract aid
// work again: its motion is already present in that observed state. Do not add
// aid work to the main prediction or feed an assisted cycle to arrival learning.
}
