#pragma once
#include "walking_return_energy.h"

// v18 separates a pre-centre return aid from the main arrival-energy decision.
// The 80ms limit is the inherited experimental software envelope, not an
// identified physical bound. Elapsed time and current/work remain predictions
// based on software observations, not measurements of mechanical work.
namespace half_cycle_energy_budget {
constexpr char kRevision[]="return_aid_then_main_energy_v18";
constexpr uint16_t kEnvelopeMs=walking_energy_policy::kMaxPulseMs;
enum class Kind : uint8_t { ReturnAid,Main };

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
  bool active_=false,aid_used_=false,main_used_=false;
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
// At real ZERO, recompute ordinary arrival-energy demand from its fresh state
// and signed residual current using the remaining cap. Do not subtract aid
// work again: its motion is already present in that observed state. Do not add
// aid work to the main prediction or feed an assisted cycle to arrival learning.
}
