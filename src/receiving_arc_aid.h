#pragma once

#include "support_transfer_plan.h"

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
