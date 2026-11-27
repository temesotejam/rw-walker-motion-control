#pragma once

// 4. Required work: nominal peak energy and predicted arrival energy -> joules to add.
// A return that would stop short has a separate local work demand.

#include "walking/coast_motion.h"
#include "models/lateral_energy_control.h"

namespace walking_energy_policy_v15 {
// Numerical tolerance when comparing a predicted work with a request.
constexpr float kEnergyRoundoffJ=1e-9f;
}

namespace walking_energy_policy {
constexpr char kRevision[]="walking_peak_rate_energy_v14";
constexpr float kCommonPeakFeedbackGain=.25f;
constexpr float kRedistributionGain=.125f;
enum class Reason : uint8_t { Invalid,Coast,Drive,CapLimited };
struct Input {
  int8_t side=0; // Intended NEXT body peak side; rate is a magnitude.
  float rate_dps=NAN,body_angle_deg=NAN,target_deg=NAN;
  float previous_peak_deg=NAN,same_side_peak_deg=NAN; // Peak amplitudes >=0.
  // Signed relative to intended excitation. Never abs() a residual current.
  float initial_current_ma=NAN,goal_current_ma=NAN;
  float extra_energy_j=0;
  uint16_t cap_ms=kMaxPulseMs;
};
struct Decision {
  bool valid=false,selected=false,coast=false,saturated=false,extrapolated=false;
  bool rate_supported=false,charge_supported=false,early_state_transfer=false;
  bool passive_rate_supported=false,work_rate_supported=false,passive_intercept_rolloff=false;
  bool coast_residual_modelled=false,extra_energy_clamped=false;
  Reason reason=Reason::Invalid;
  uint16_t width_ms=0,evaluations=0;
  float peak_energy_j=NAN,same_side_peak_energy_j=NAN;
  float mean_peak_energy_j=NAN,side_imbalance_j=NAN;
  float common_correction_j=0,redistribution_j=0;
  float potential_j=NAN,effective_kinetic_j=NAN,fitted_offset_j=NAN;
  float passive_omega2_coefficient=NAN;
  float available_j=NAN,target_j=NAN,peak_correction_j=0,extra_energy_j=0;
  float deficit_j=NAN,request_j=NAN,q_mas=0,predicted_work_j=0;
  float predicted_arrival_j=NAN,unmet_j=NAN,maximum_work_j=NAN;
  float coast_residual_q_mas=NAN,inbound_loss_proxy_j=NAN;
};

inline bool computePeakWork(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current, const Input& in, Decision& out) {
  if((in.side!=1 && in.side!=-1) || !current.ready || !in.cap_ms || in.cap_ms>kMaxPulseMs ||
     !std::isfinite(in.rate_dps) || in.rate_dps<=0 || !std::isfinite(in.body_angle_deg) ||
     !std::isfinite(in.target_deg) || in.target_deg<=0 ||
     !std::isfinite(in.previous_peak_deg) || in.previous_peak_deg<0 ||
     (!std::isnan(in.same_side_peak_deg) && (!std::isfinite(in.same_side_peak_deg) || in.same_side_peak_deg<0)) ||
     !std::isfinite(in.initial_current_ma) || !std::isfinite(in.goal_current_ma) || in.goal_current_ma<=0 ||
     !std::isfinite(in.extra_energy_j) || in.extra_energy_j<0)return false;
  const auto c=lateral_energy::coefficients(in.side);
  const auto passive=passiveCoefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  out.target_j=lateral_energy::potential(geometry,in.target_deg);
  out.potential_j=lateral_energy::potential(geometry,in.body_angle_deg);
  out.peak_energy_j=lateral_energy::potential(geometry,in.previous_peak_deg);
  if(!std::isfinite(out.target_j) || out.target_j<=0 || !std::isfinite(out.potential_j) ||
     !std::isfinite(out.peak_energy_j))return false;
  // Positive-rate callers include slow stall-rescue proposals; their detector
  // floors belong to the caller. Low-speed predictions remain extrapolations.
  // Low-rate walking fit throughout the current range, with out-of-domain
  // flags. Do not blend to the inconsistent old passive fit at35..45dps:
  // doing so decreases predicted energy when angular speed increases.
  out.passive_omega2_coefficient=passive.omega2_j;
  out.effective_kinetic_j=passive.omega2_j*omega*omega;
  const float scale=std::fmin(1.f,in.rate_dps/20.f);
  out.fitted_offset_j=passive.offset_j*scale*scale;
  out.passive_intercept_rolloff=in.rate_dps<20.f; // Unvalidated static-end rolloff.
  out.available_j=out.potential_j+out.effective_kinetic_j+out.fitted_offset_j;
  out.inbound_loss_proxy_j=std::fmax(0.f,out.peak_energy_j-out.available_j);
  // This is an incoming-to-predicted-arrival discrepancy, not observed
  // mechanical dissipation. Do not subtract it again from the same surrogate.
  // Paired completed peaks describe cycle mean and side imbalance. They are
  // successive observations, not simultaneous independent energy reservoirs.
  // Common correction changes total excitation; redistribution changes sign
  // when the two side observations are exchanged and therefore sums to zero.
  if(std::isfinite(in.same_side_peak_deg)){
    out.same_side_peak_energy_j=lateral_energy::potential(geometry,in.same_side_peak_deg);
    if(!std::isfinite(out.same_side_peak_energy_j))return false;
    out.mean_peak_energy_j=.5f*(out.same_side_peak_energy_j+out.peak_energy_j);
    out.side_imbalance_j=.5f*(out.peak_energy_j-out.same_side_peak_energy_j);
    const float limit=.125f*out.target_j;
    const float common=kCommonPeakFeedbackGain*(out.target_j-out.mean_peak_energy_j);
    const float redistribution=kRedistributionGain*out.side_imbalance_j;
    out.common_correction_j=std::fmax(-limit,std::fmin(limit,common));
    out.redistribution_j=std::fmax(-limit,std::fmin(limit,redistribution));
    out.peak_correction_j=out.common_correction_j+out.redistribution_j;
  }
  out.extra_energy_j=std::fmin(in.extra_energy_j,.5f*out.target_j);
  out.extra_energy_clamped=out.extra_energy_j!=in.extra_energy_j;
  out.deficit_j=out.target_j+out.peak_correction_j+out.extra_energy_j-out.available_j;
  out.request_j=std::fmax(0.f,out.deficit_j);
  out.coast_residual_q_mas=in.initial_current_ma*kCoastFallTauS;
  // The inherited response was fitted to pulse-only Q, with pulse/coast effects
  // folded into its arrival response. Adding a fresh tail to its Q would
  // silently change that fit. Log the signed pre-existing coast residual as a
  // model gap; no zero-current or measured-work claim follows from width=0.
  out.coast_residual_modelled=false;
  out.passive_rate_supported=in.rate_dps>=passive.rate_min_dps && in.rate_dps<=passive.rate_max_dps;
  out.work_rate_supported=in.rate_dps>=c.rate_min_dps && in.rate_dps<=c.rate_max_dps;
  out.rate_supported=out.passive_rate_supported && out.work_rate_supported;
  out.early_state_transfer=std::fabs(in.body_angle_deg)>.01f;
  return true;
}
} // namespace walking_energy_policy

namespace walking_return_energy {
struct Input : walking_energy_policy::Input {
  float arrival_bias_j=0,return_work_j=0;
#if WALKING_TURN_OBSERVER
  // Signed change to the ordinary positive work request. Return aid has its own budget.
  float turn_adjustment_j=0;
#endif
};
using Decision=walking_energy_policy::Decision;

inline bool updateWorkRequest(Decision& out, const Input& in) {
  if(!out.valid || !std::isfinite(in.arrival_bias_j) || !std::isfinite(in.return_work_j) || in.return_work_j<0)
    {out.valid=out.selected=false;return false;}
  const float limit=.25f*out.target_j;
  out.available_j=std::fmax(0.f,out.available_j+std::fmax(-limit,std::fmin(limit,in.arrival_bias_j)));
  out.deficit_j=out.target_j+out.peak_correction_j+out.extra_energy_j-out.available_j;
#if WALKING_TURN_OBSERVER
  if (!std::isfinite(in.turn_adjustment_j)) {out.valid=out.selected=false;return false;}
  out.deficit_j+=in.turn_adjustment_j;
#endif
  out.request_j=std::fmax(std::fmax(0.f,out.deficit_j),std::fmin(.5f*out.target_j,in.return_work_j));
  return true;
}

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
} // namespace walking_return_energy

namespace return_crossing_work {
inline float energyShortfall(float angle,float terminal,float speed,float residual,float decel) {
  const float rad=lateral_energy::kRadPerDeg;
  return walking_energy_policy::passiveCoefficients(-1).omega2_j*rad*rad*
      (2.f*decel*(angle-terminal)+residual*residual-speed*speed);
}
} // namespace return_crossing_work

// Extension of an already accepted return-aid event. No timing or current
// command is generated here. The trial data cover only motion to the left.
namespace receiving_arc_aid {
enum class Reason : uint8_t {
  Inactive, NoFreshShape, OutsideModel, NotReceiving, ShapeReturning,
  Eligible, NoMainRoom, NoWidthGain, Applied, UncertainPivot, CommandVeto
};
struct Input {
  int side=0;
  bool observed_shape=false;
#if SUPPORT_TRANSFER_STALL_V3
  uint16_t base_width_ms=0;  // Already selected after the wheel-speed guard.
#endif
  float body_deg=NAN, speed_dps=NAN, decel_dps2=NAN, fore_aft_deg=NAN;
  float left_deg=NAN, right_deg=NAN, left_est=NAN, right_est=NAN;
  float left_rate=NAN, right_rate=NAN, base_j=NAN, target_j=NAN;
};
struct Result {
  Reason reason=Reason::Inactive;
  float extra_j=0, terminal_deg=NAN, horizon_s=NAN;
#if SUPPORT_TRANSFER_STALL_V3
  bool extension_pivot_only=false;
#endif
};
inline Result evaluate(const Input& in) {
  Result out;
  if (in.side!=-1) return out;
  if (!in.observed_shape) {out.reason=Reason::NoFreshShape;return out;}
  if (!(in.body_deg>0 && in.body_deg<=12 && in.speed_dps>=3 && in.speed_dps<=35 &&
        std::isfinite(in.decel_dps2) && in.decel_dps2>0 &&
        std::fabs(in.fore_aft_deg)<=10 && std::fabs(in.left_deg)<=20 && std::fabs(in.right_deg)<=20 &&
        std::fabs(in.left_est)<=20 && std::fabs(in.right_est)<=20 &&
        std::fabs(in.left_rate)<=180 && std::fabs(in.right_rate)<=180 &&
        std::isfinite(in.base_j) && in.base_j>0 && std::isfinite(in.target_j) && in.target_j>0)) {
    out.reason=Reason::OutsideModel;return out;
  }
  const float gap=support_transfer_plan::supportGap(in.left_deg,in.right_deg,in.fore_aft_deg,in.body_deg);
  if (gap>=0 || in.left_deg-in.right_deg<3.f) {out.reason=Reason::NotReceiving;return out;}
  // Only ask whether the observed shape persists locally. In the stopped
  // trials the nominal ZERO time was beyond what the feet can be extrapolated to.
  out.horizon_s=std::fmin(support_transfer_plan::kForecastLimitMs*.001f,in.body_deg/in.speed_dps);
  const float left=in.left_est+in.left_rate*out.horizon_s;
  const float right=in.right_est+in.right_rate*out.horizon_s;
  if (std::fabs(left)>20 || std::fabs(right)>20) {out.reason=Reason::OutsideModel;return out;}
  if (in.left_est-in.right_est<3.f || left-right<3.f) {out.reason=Reason::ShapeReturning;return out;}
  const auto clearance=[&](float seconds) {
    return -support_transfer_plan::supportGap(in.left_est+in.left_rate*seconds,
        in.right_est+in.right_rate*seconds,in.fore_aft_deg,in.body_deg-in.speed_dps*seconds);
  };
  // Moving towards the left increases its clearance over the right at fixed
  // foot pitches: the right sole's horizontal coordinate exceeds the left's
  // over this roll range. Each sole's height changes by at most 28 mm/rad
  // with pitch: for |pitch|<=30 degrees its toe/heel derivative is bounded
  // by the 28 mm longitudinal edge distance (z in [-83,-75] mm).
  // Certify both halves if the full-interval bound is inconclusive.
  const float loss=28.f*lateral_energy::kRadPerDeg*(std::fabs(in.left_rate)+std::fabs(in.right_rate))*out.horizon_s;
  const float start=clearance(0);
  const float end=clearance(out.horizon_s);
  bool proven=start>0 && end>0 &&
      (start>loss || (start>.5f*loss && clearance(.5f*out.horizon_s)>.5f*loss));
#if SUPPORT_TRANSFER_STALL_V3
  // Both commands are identical up to base_width_ms. A receiving pivot need
  // only be certified where extending that pulse can change the current. The
  // same one-sided clearance-loss bound must cover every instant, including
  // the full possible pulse tail. Four equal intervals bound the extra cost.
  if (!proven && end>0 && in.base_width_ms>0 &&
      in.base_width_ms<walking_energy_policy::kMaxPulseMs &&
      out.horizon_s>=walking_energy_policy::kMaxPulseMs*.001f) {
    const float begin=in.base_width_ms*.001f;
    const float step=(out.horizon_s-begin)*.25f;
    const float bound=loss*(step/out.horizon_s);
    proven=true;
    for (unsigned i=0;i<4;++i) {
      if (!(clearance(begin+i*step)>bound)) {proven=false;break;}
    }
    out.extension_pivot_only=proven;
  }
#endif
  if (!proven) {
    out.reason=Reason::UncertainPivot;return out;
  }
  out.terminal_deg=support_transfer_plan::arcEntryDeg(in.side,in.left_deg+in.fore_aft_deg);
  const float c=walking_energy_policy::passiveCoefficients(in.side).omega2_j;
  const float rad=lateral_energy::kRadPerDeg, carry=support_transfer_plan::kCarryDps;
  const float extra=c*rad*rad*(2*in.decel_dps2*std::fabs(out.terminal_deg)+carry*carry);
  out.extra_j=std::fmax(0.f,std::fmin(.5f*in.target_j-in.base_j,extra));
  out.reason=Reason::Eligible;
  return out;
}

class MainReserve {
 public:
  void observe(int side,bool assisted,uint32_t elapsed) {
    if ((side!=-1 && side!=1) || !elapsed) return;
    auto& state=side>0?right_:left_;
    const auto width=static_cast<uint16_t>(elapsed>walking_energy_policy::kMaxPulseMs ?
        walking_energy_policy::kMaxPulseMs:elapsed);
    if (width>state.any) state.any=width;
    if (assisted && width>state.assisted) state.assisted=width;
  }
  uint16_t forSide(int side) const {
    const auto& state=side>0?right_:left_;
    return state.assisted?state.assisted:(state.any?state.any:walking_energy_policy::kMaxPulseMs);
  }
 private:
  struct History { uint16_t assisted=0,any=0; };
  History left_,right_;
};

#if WALKING_RETURN_HISTORY
// Trial for a left return already certified by evaluate(). Each reference is
// a real aid plus its ordinary main and the measured following left peak.
struct ReturnState {
  uint32_t departure_ms=0, image_age_ms=0;
  float body_deg=NAN, speed_dps=NAN, left_deg=NAN, right_deg=NAN, fore_aft_deg=NAN;
  float deceleration=NAN, terminal_deg=NAN, target_j=NAN;
  ReturnState()=default;
  ReturnState(uint32_t departure,uint32_t age,float body,float speed,float left,float right,
              float fore_aft,float decel,float terminal,float target)
      : departure_ms(departure),image_age_ms(age),body_deg(body),speed_dps(speed),left_deg(left),
        right_deg(right),fore_aft_deg(fore_aft),deceleration(decel),terminal_deg(terminal),target_j(target) {}
  bool valid() const {
    return departure_ms && image_age_ms<=75 && body_deg>0 && body_deg<=12 &&
        speed_dps>=3 && speed_dps<=35 && std::fabs(left_deg)<=20 && std::fabs(right_deg)<=20 &&
        std::fabs(fore_aft_deg)<=10 && deceleration>0 && std::isfinite(deceleration) &&
        terminal_deg<0 && std::isfinite(terminal_deg) && target_j>0 && std::isfinite(target_j);
  }
};
struct HistoryChoice {
  uint32_t first_peak_ms=0, second_peak_ms=0;
  uint16_t limit_ms=0;
  float deceleration=0, request_j=0;
};
class ReturnHistory {
 public:
  // Called once at the end of each executed LEFT aid, including interrupted
  // or uncertified ones. Such a transfer breaks the consecutive pair.
  void completedAid(const ReturnState& state,uint32_t end,uint32_t elapsed,bool qualified) {
    if (!qualified || !state.valid() || end<=state.departure_ms ||
        end-state.departure_ms>1500 || elapsed==0 || elapsed>walking_energy_policy::kMaxPulseMs) {
      *this={};return;
    }
    if (pending_.end_ms || (recent_[1].peak_ms && state.departure_ms<=recent_[1].peak_ms))
      *this={};
    pending_.state=state;pending_.end_ms=end;
    pending_.peak_ms=0;pending_.elapsed_ms=static_cast<uint16_t>(elapsed);
  }
  void peak(int side,uint32_t t,float magnitude,uint32_t departure,uint32_t zero,bool matched_main) {
    if (!pending_.end_ms) return;
    const auto& state=pending_.state;
    if (!(matched_main && side==-1 && departure==state.departure_ms &&
          zero>=pending_.end_ms && t>zero && t-state.departure_ms<=1500 &&
          std::isfinite(magnitude) && magnitude>=-state.terminal_deg)) {
      *this={};return;
    }
    pending_.peak_ms=t;
    recent_[0]=recent_[1];recent_[1]=pending_;pending_={};
  }
  HistoryChoice compare(const ReturnState& state,uint32_t now) const {
    HistoryChoice out;
    if (!state.valid() || now<=state.departure_ms || now-state.departure_ms>1500 ||
        pending_.end_ms || !similar(recent_[0],state,now) || !similar(recent_[1],state,now)) return out;
    out.first_peak_ms=recent_[0].peak_ms;out.second_peak_ms=recent_[1].peak_ms;
    out.limit_ms=recent_[0].elapsed_ms<recent_[1].elapsed_ms?recent_[0].elapsed_ms:recent_[1].elapsed_ms;
    out.deceleration=std::fmin(recent_[0].state.deceleration,recent_[1].state.deceleration);
    if (out.deceleration<=state.deceleration) return out;
    const float rad=lateral_energy::kRadPerDeg, carry=support_transfer_plan::kCarryDps;
    const float work=walking_energy_policy::passiveCoefficients(-1).omega2_j*rad*rad*
        (2.f*out.deceleration*(state.body_deg-state.terminal_deg)-state.speed_dps*state.speed_dps+carry*carry);
    out.request_j=std::fmin(.5f*state.target_j,std::fmax(0.f,work));
    return out;
  }
 private:
  struct Entry { ReturnState state; uint32_t end_ms=0,peak_ms=0; uint16_t elapsed_ms=0; };
  static bool similar(const Entry& entry,const ReturnState& state,uint32_t now) {
    if (!entry.peak_ms || entry.peak_ms>=state.departure_ms || now-entry.peak_ms>4000) return false;
    const auto& ref=entry.state;
    return std::fabs(ref.body_deg-state.body_deg)<=1.f && std::fabs(ref.speed_dps-state.speed_dps)<=3.f &&
        std::fabs(ref.left_deg-state.left_deg)<=2.f && std::fabs(ref.right_deg-state.right_deg)<=2.f &&
        std::fabs(ref.fore_aft_deg-state.fore_aft_deg)<=1.f &&
        std::fabs(static_cast<float>(ref.image_age_ms)-state.image_age_ms)<=25.f &&
        std::fabs(ref.target_j-state.target_j)<1.e-9f;
  }
  Entry recent_[2]{},pending_{};
};
#endif

#if SUPPORT_TRANSFER_ARC_FEEDBACK
// Compare the endpoint of a completed leftward assisted transfer with its
// own predicted arc entry. A new real aid replaces the previous evidence.
class ArrivalFeedback {
 public:
  struct Credit {
    uint32_t peak_ms=0, elapsed_ms=0;
    float missing_j=0, previous_work_j=0;
  };
  void completedAid(uint32_t departure, uint32_t end, float terminal,
                    uint32_t elapsed, float work) {
    *this=ArrivalFeedback{};
    if (!(departure && end>departure && end-departure<=1500 && elapsed && elapsed<=80 &&
          terminal<0 && std::isfinite(terminal) && work>0 && std::isfinite(work))) return;
    departure_ms_=departure; aid_end_ms_=end; terminal_deg_=terminal;
    credit_.elapsed_ms=elapsed; credit_.previous_work_j=work;
  }
  void peak(const lateral_energy::Geometry& geometry, int side, uint32_t time,
            float magnitude, uint32_t main_departure, uint32_t zero, bool matched_main) {
    if (!departure_ms_) return;
    const bool matched=matched_main && side==-1 && departure_ms_==main_departure &&
        zero>=aid_end_ms_ && time>zero && time-departure_ms_<=1500;
    departure_ms_=0;
    if (!matched || !(magnitude>=0 && magnitude<std::fabs(terminal_deg_))) return;
    const float delta=lateral_energy::potential(geometry,terminal_deg_)-lateral_energy::potential(geometry,magnitude);
    if (std::isfinite(delta) && delta>0) {
      credit_.missing_j=std::fmin(credit_.previous_work_j,delta);
      credit_.peak_ms=time;
    }
  }
  const Credit* recent(uint32_t now) const {
    return credit_.peak_ms && now>=credit_.peak_ms && now-credit_.peak_ms<=3000 ? &credit_ : nullptr;
  }
 private:
  uint32_t departure_ms_=0,aid_end_ms_=0;
  float terminal_deg_=NAN;
  Credit credit_;
};
#endif
inline uint16_t boundWidth(uint16_t baseline,uint16_t candidate,uint16_t envelope,uint16_t reserve) {
  const uint16_t room=envelope>reserve ? envelope-reserve : 0;
  const uint16_t limited=candidate<room ? candidate : room;
  return baseline>limited ? baseline : limited;
}
}

#if WALKING_TURN_OBSERVER
#include "gait_turn_observer.h"

namespace gait_turn_energy {
// One-step identification trial on the observed right-foot/negative-yaw pattern.
// 0.20 deg is the size of the project's earlier side-target response trial.
// Matched historical commands were compared at differences of at least 5 ms.
// Stay inside that step and at most 10% of the already-guarded pulse time.
constexpr float equivalent_target_step_deg = .20f;
inline uint16_t widthStep(uint16_t baseline_ms) {
  const uint16_t tenth = baseline_ms / 10;
  return tenth < 5 ? tenth : 5;
}

inline float adjustment(const gait_turn::Status& state, int side, uint32_t now,
                        const walking_return_energy::Input& in,
                        const walking_return_energy::Decision& base,
                        const lateral_energy::Geometry& geometry,
                        const lateral_energy::CurrentTable& current) {
  const auto& cycle = state.latest;
  if (!state.enabled || cycle.reason != gait_turn::Reason::Confirmed || cycle.weak_side != 1 ||
      cycle.turn_sign != -1 || cycle.end_ms <= cycle.start_ms || now < cycle.end_ms ||
      now-cycle.end_ms > cycle.end_ms-cycle.start_ms || (side != -1 && side != 1) ||
      in.side != side || !base.valid || !base.selected || base.coast || !base.width_ms ||
      !base.work_rate_supported || !base.charge_supported ||
      !(base.request_j > 0 && base.predicted_work_j > 0) ||
      !(in.target_deg > equivalent_target_step_deg && in.target_deg <= 8.f) ||
      !std::isfinite(in.same_side_peak_deg)) return 0;
  // On the positive half only reduce stimulation if the previously observed
  // positive body peak already met its current target and the full request was covered.
  if (side == 1 && (!base.passive_rate_supported || in.same_side_peak_deg < in.target_deg ||
                   base.unmet_j > walking_energy_policy_v15::kEnergyRoundoffJ)) return 0;
  if (!(cycle.left_swing_deg > cycle.right_swing_deg && cycle.right_swing_deg >= 0)) return 0;
  const float asymmetry = std::isfinite(cycle.paired_swing_asymmetry) ? cycle.paired_swing_asymmetry :
      (cycle.left_swing_deg-cycle.right_swing_deg)/(cycle.left_swing_deg+cycle.right_swing_deg);
  if (!(asymmetry > 0 && asymmetry <= 1)) return 0;
  const float delta_u = lateral_energy::potential(geometry, in.target_deg - side*equivalent_target_step_deg)
                      - lateral_energy::potential(geometry, in.target_deg);
  if (!std::isfinite(delta_u) || side*delta_u >= 0) return 0;
  const uint16_t step = widthStep(base.width_ms);
  if (!step) return 0;
  const uint16_t edge = side > 0 ? base.width_ms-step :
      static_cast<uint16_t>(std::fmin(in.cap_ms, base.width_ms+step));
  const float q = lateral_energy::charge(current, edge, in.initial_current_ma, in.goal_current_ma);
  const float edge_work = lateral_energy::work(lateral_energy::coefficients(side),
      in.rate_dps*lateral_energy::kRadPerDeg, q);
  const float room = -side * (edge_work - base.predicted_work_j);
  if (!(q >= 0 && std::isfinite(edge_work) && room > 0)) return 0;
  // Anchor the allowed work movement at the command after the wheel guard.
  // An old clipped request may still exceed this whole interval; retain it.
  const float desired = base.predicted_work_j - side*asymmetry*std::fmin(std::fabs(delta_u), room);
  const float request_delta = desired - base.request_j;
  return -side*request_delta > 0 ? request_delta : 0;
}

inline bool admissible(const walking_return_energy::Decision& base,
                       const walking_return_energy::Decision& candidate, float adjustment_j) {
  if (!candidate.valid || !candidate.selected || candidate.coast || !candidate.width_ms ||
      !candidate.work_rate_supported || !candidate.charge_supported ||
      !(candidate.request_j > 0) || candidate.unmet_j > walking_energy_policy_v15::kEnergyRoundoffJ)
    return false;
  const float difference = candidate.predicted_work_j-base.predicted_work_j;
  const int change_ms = static_cast<int>(candidate.width_ms) - base.width_ms;
  return difference*adjustment_j > 0 &&
         (candidate.width_ms > base.width_ms) == (adjustment_j > 0) &&
         change_ms != 0 && std::abs(change_ms) <= widthStep(base.width_ms);
}
} // namespace gait_turn_energy
#endif
