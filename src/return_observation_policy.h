#pragma once
#include "walking_energy_policy.h"
#if SUPPORT_TRANSFER_STALL_V3
#include "periodic_foot_state.h"
#include "support_transfer_plan.h"
#endif

// v18 trial observer. The IMU evidence and the latest completed camera frame
// have separate lifetimes. Missing markers never become measured foot angles.
// The energy is an effective rate-energy reachability proxy, not identified
// inertia, contact load, moving-foot potential, or measured mechanical work.
namespace return_observation_policy {
enum class Evidence : uint8_t {
  Unavailable=0, ObservedShape=1, RecognitionDropout=2, RangeRejected=3,
  ObservedUnseparated=4, MeasuredArcShortfall=5, PredictedArcShortfall=6, PostAidShortfall=7,
  ContinuationShortfall=8
};
inline bool isPostAid(Evidence reason) {
  return reason==Evidence::PostAidShortfall || reason==Evidence::ContinuationShortfall;
}
struct FootFrame {
  uint16_t run_id=0;
  uint32_t frame_ms=0,ready_ms=0;
#if SUPPORT_TRANSFER_CARRY2
  uint32_t sequence=0;
  uint64_t image_us=0,received_us=0;
#endif
  bool timestamp_valid=false,frame_valid=false,zero_ready=false;
  bool left_detected=false,right_detected=false;
  // These must include the separately configured 5 degree range allowance.
  // A missing marker is not itself evidence of a range violation.
  bool left_range_ok=false,right_range_ok=false;
  float left_deg=NAN,right_deg=NAN;
};
struct Input {
  uint16_t run_id=0;
  uint32_t now=0,peak_ms=0,sample_us=0,sample_age_us=0;
  int side=0;
  float peak_deg=NAN,angle_deg=NAN,rate_dps=NAN,target_j=NAN;
  bool active=false;
  FootFrame foot;
#if SUPPORT_TRANSFER_STALL_V3
  const periodic_foot::Packet* periodic=nullptr;
  float fore_aft_deg=NAN, periodic_acceleration_dps2=NAN;
  bool predict_left_return=false;
  float current_rise_tau_s=NAN;
#endif
};
struct Choice {
  bool selected=false;
  float request_j=0,deceleration_dps2=0,stopping_distance_deg=0;
  Evidence evidence=Evidence::Unavailable;
  uint32_t foot_age_ms=0,deceleration_span_ms=0;
  bool range_veto=false;
#if SUPPORT_TRANSFER_STALL_V3
  float predicted_transfer_ms=NAN,terminal_deg=NAN;
  uint16_t loss_cap_ms=0;
  float aligned_current_ma=NAN, gross_work_limit_j=NAN;
  float release_deceleration_dps2=NAN;
  float previous_arc_deceleration_dps2=NAN;
#endif
};

inline Evidence assessFoot(const Input& in) {
  const auto& f=in.foot;
  if(!in.run_id || f.run_id!=in.run_id || !f.timestamp_valid || !f.frame_valid || !f.zero_ready ||
     !f.frame_ms || !f.ready_ms || f.ready_ms<f.frame_ms || in.now<f.ready_ms ||
     in.now<f.frame_ms || in.now-f.frame_ms>160u)return Evidence::Unavailable;
  // Only an actually detected finite marker can establish a known range
  // rejection. Do not reinterpret an ambiguous marker's invalid range flag.
  if((f.left_detected && std::isfinite(f.left_deg) && !f.left_range_ok) ||
     (f.right_detected && std::isfinite(f.right_deg) && !f.right_range_ok))return Evidence::RangeRejected;
  if((f.left_detected && !std::isfinite(f.left_deg)) ||
     (f.right_detected && !std::isfinite(f.right_deg)))return Evidence::Unavailable;
  if(!f.left_detected || !f.right_detected)return Evidence::RecognitionDropout;
  if(!f.left_range_ok || !f.right_range_ok)return Evidence::Unavailable;
  return std::fabs(f.left_deg-f.right_deg)>=3.f?Evidence::ObservedShape:Evidence::ObservedUnseparated;
}

#if SUPPORT_TRANSFER_STALL_V3
// Use the deceleration observed before issuing the new command. A rise in
// speed while powered is evidence of momentum, not a new estimate of the
// resistance the body will encounter after release.
inline bool canCoastToArc(float angle,float signed_rate,float terminal,float decel) {
  const float carry=support_transfer_plan::kCarryDps;
  return angle>0 && signed_rate<0 && terminal<0 && decel>0 &&
      std::isfinite(angle) && std::isfinite(signed_rate) && std::isfinite(terminal) && std::isfinite(decel) &&
      signed_rate*signed_rate >= 2.f*decel*(angle-terminal)+carry*carry;
}

// A completed left aid is a new observation horizon. While the body is still
// moving towards ZERO, check whether the measured speed gain has dissipated.
// Each accepted aid gets at most one such decision, including a vetoed one.
class PostAidObserver {
 public:
  bool releaseOnCarry(uint32_t sample_us,uint32_t age_us,uint32_t elapsed_ms,float angle,
      float rate,float terminal,float decel) {
    // The driver's queue uses millis() and the gyro uses micros(). Allow a
    // full ms of rounding when proving acquisition followed command start.
    const bool enough=sample_us && age_us<=10000 && elapsed_ms>age_us/1000+1 &&
        canCoastToArc(angle,rate,terminal,decel);
    const uint32_t gap=sample_us-ready_sample_us_;
    const bool confirmed=enough && ready_sample_us_ && gap>0 && gap<=10000;
    ready_sample_us_=enough?sample_us:0;
    return confirmed;
  }
  void arm(uint32_t peak,uint32_t end,float onset_speed,float delivered_work) {
    *this=PostAidObserver{};
    if (peak && end>peak && std::isfinite(onset_speed) && onset_speed>0 &&
        std::isfinite(delivered_work) && delivered_work>0) {
      peak_=peak;end_=end;onset_speed_=onset_speed;first_work_=delivered_work;
    }
  }
  Choice observe(const Input& in,uint64_t now_us,float aligned_current_ma=NAN,bool continuation=false) {
    Choice out;
    if (!peak_) return out;
    if (in.peak_ms!=peak_ || in.side!=1 || in.now<end_ ||
        in.now-end_>support_transfer_plan::kForecastLimitMs ||
        (in.sample_us && in.sample_age_us<=10000 && (in.angle_deg<=0 || in.rate_dps>=0))) {
      peak_=0; return out;
    }
    const auto* p=in.periodic;
    if (!p || !p->count || p->count==packet_) return out;
    if (p->published_us<p->epoch_us+uint64_t(end_)*1000 || !std::isfinite(in.periodic_acceleration_dps2)) return out;
    const bool adjacent=packet_ && p->count-packet_==1 && p->scheduled_us==slot_+periodic_foot::period_us;
    const float last_decel=adjacent ? decel_ : 0.f, last_speed=speed_;
    const uint64_t last_image=image_;
    packet_=p->count;slot_=p->scheduled_us;decel_=0;speed_=0;image_=0;
    if (!adjacent) max_speed_=0;
    const auto& l=p->left;
    const auto& r=p->right;
    const float speed=-in.rate_dps;
    if (!in.active || !in.sample_us || in.sample_age_us>10000 || assessFoot(in)==Evidence::RangeRejected ||
        !(in.angle_deg>0 && in.angle_deg<=12 &&
        speed>=support_transfer_plan::kCarryDps && speed<=35 && in.target_j>0 && std::isfinite(in.target_j)) ||
        !periodic_foot::matchesPair(*p,now_us,l.sequence,l.image_us) || !l.sequence ||
        !(std::fabs(in.fore_aft_deg)<=10 && std::fabs(l.measured)<=20 && std::fabs(r.measured)<=20 &&
          std::fabs(l.estimated)<=20 && std::fabs(r.estimated)<=20 && l.measured>r.measured && l.estimated>r.estimated) ||
        !(support_transfer_plan::supportGap(l.measured,r.measured,in.fore_aft_deg,in.angle_deg)<0 &&
          support_transfer_plan::supportGap(l.estimated,r.estimated,in.fore_aft_deg,in.angle_deg)<0)) {
      max_speed_=0; return out;
    }
    image_=l.image_us;speed_=speed;max_speed_=std::fmax(max_speed_,speed);
    const float a=in.periodic_acceleration_dps2;
    if (!(std::isfinite(a) && a>0 && speed*speed<2.f*a*in.angle_deg)) return out;
    decel_=a;
    const float kinetic=walking_energy_policy::passiveCoefficients(-1).omega2_j*
                        lateral_energy::kRadPerDeg*lateral_energy::kRadPerDeg;
    if (!(last_decel>0 && speed<last_speed && last_image && image_>last_image &&
          image_>=p->epoch_us+uint64_t(end_)*1000 && std::isfinite(aligned_current_ma))) return out;
    // A passed work budget describes the command model. It need not all
    // appear as observable lateral kinetic energy, especially across impact.
    const float conservative=std::fmin(last_decel,a);
    const float stopping_distance=speed*speed/(2.f*conservative);
    const bool lost_first_work=speed<onset_speed_ && max_speed_>onset_speed_ &&
          kinetic*(max_speed_*max_speed_-speed*speed)>=first_work_;
    // A newly inferred lack of momentum inside one foot-observation slot
    // may resolve at the next sample. Preserve that time for the main input.
    const float one_slot_travel=speed*(periodic_foot::period_us*1.e-6f);
    if (!lost_first_work && (!continuation || in.angle_deg-stopping_distance<=one_slot_travel)) return out;
    const float higher_decel=std::fmax(last_decel,a);
    const float time_to_stop=speed/higher_decel;
    if (!(in.current_rise_tau_s>0 && time_to_stop>=in.current_rise_tau_s+.005f)) return out;
    const float carry=support_transfer_plan::kCarryDps;
    out.gross_work_limit_j=.5f*in.target_j-first_work_;
    const float terminal=continuation ? support_transfer_plan::arcEntryDeg(-1,l.measured+in.fore_aft_deg) : 0.f;
    out.request_j=std::fmin(out.gross_work_limit_j,kinetic*(2.f*conservative*(in.angle_deg-terminal)+carry*carry-speed*speed));
    if (!(out.request_j>0)) return out;
    // Keep the command inside the observed inward interval even allowing
    // the same five ms of dispatch latency as the normal freshness gate.
    out.loss_cap_ms=static_cast<uint16_t>(std::fmin(1000.f*time_to_stop-5.f,walking_energy_policy::kMaxPulseMs));
    out.selected=true;out.evidence=continuation?Evidence::ContinuationShortfall:Evidence::PostAidShortfall;
    out.terminal_deg=terminal;
    // Both frames must predict enough residual momentum before releasing
    // the new input. The higher loss is the stricter coast projection.
    if (continuation) out.release_deceleration_dps2=higher_decel;
    out.foot_age_ms=static_cast<uint32_t>((now_us-image_)/1000);
    out.deceleration_dps2=conservative;out.stopping_distance_deg=stopping_distance;
    out.aligned_current_ma=aligned_current_ma;peak_=0;
    return out;
  }
 private:
  uint32_t peak_=0,end_=0,packet_=0,ready_sample_us_=0;
  uint64_t slot_=0,image_=0;
  float onset_speed_=0, first_work_=0, max_speed_=0, speed_=0, decel_=0;
};
#endif

class Observer {
 public:
  void reset(){*this=Observer{};}
  // Call only when the aid is accepted for output. A gap, stale image or
  // inactive sample must not rearm the same peak after an accepted aid.
  void consume(uint16_t run_id,uint32_t peak_ms) {
    if(!run_id || !peak_ms)return;
    if(run_!=run_id || peak_!=peak_ms){reset();run_=run_id;peak_=peak_ms;}
    consumed_=true;clearMotion();
  }
  Choice observe(const Input& in) {
    Choice out;
    // Missing identifiers cannot undo an accepted aid for the previous key.
    // An explicit reset or a new nonzero run/peak is required to rearm it.
    if(!in.run_id || !in.peak_ms){
#if SUPPORT_TRANSFER_STALL_V3
      arc_confirming_=false;
      approaching_gap_=NAN;
#endif
      clearMotion();return out;
    }
    if(run_!=in.run_id || peak_!=in.peak_ms){reset();run_=in.run_id;peak_=in.peak_ms;}
    out.evidence=assessFoot(in);
    if(out.evidence!=Evidence::Unavailable)out.foot_age_ms=in.now-in.foot.frame_ms;
    if(out.evidence==Evidence::RangeRejected)range_rejected_=true;
    else if(out.evidence==Evidence::ObservedShape || out.evidence==Evidence::ObservedUnseparated)range_rejected_=false;
    out.range_veto=range_rejected_;
#if SUPPORT_TRANSFER_STALL_V3
    const auto arc=observeSmallArc(in,out.evidence);
    if (arc.selected) return arc;
#endif

    const float distance=in.angle_deg*in.side,speed=-in.rate_dps*in.side;
    if(consumed_ || !in.active || (in.side!=1 && in.side!=-1) ||
       !std::isfinite(distance) || !std::isfinite(speed) || !std::isfinite(in.peak_deg) ||
       !std::isfinite(in.target_j) || in.target_j<=0 || !in.sample_us || in.sample_age_us>10000u ||
       in.now<in.peak_ms || in.now-in.peak_ms<60u || in.now-in.peak_ms>800u ||
       distance<.35f || distance>12.f || in.peak_deg-distance<.5f || speed<3.f || speed>35.f) {
      clearMotion();return out;
    }
    // Unique fresh IMU samples only. Both delivery and acquisition gaps reset
    // the derivative; a camera dropout alone does not reset this history.
    if(!anchor_sample_us_ || in.now<=last_ms_ || in.now-last_ms_>10u ||
       in.sample_us<=last_sample_us_ || in.sample_us-last_sample_us_>10000u) {
      clearMotion();anchor_sample_us_=last_sample_us_=in.sample_us;
      last_ms_=in.now;anchor_speed_=speed;return out;
    }
    last_ms_=in.now;last_sample_us_=in.sample_us;
    if(in.sample_us-anchor_sample_us_>=20000u) {
      const uint32_t span=in.sample_us-anchor_sample_us_;
      const float loss=anchor_speed_-speed;
      acceleration_=loss/(span*.000001f);
      anchor_sample_us_=in.sample_us;anchor_speed_=speed;
      if(acceleration_>=60.f) {
        strong_span_us_+=span;strong_speed_loss_+=loss;
        if(strong_windows_<255)++strong_windows_;
      }else {strong_span_us_=0;strong_speed_loss_=0;strong_windows_=0;}
    }
    out.deceleration_dps2=acceleration_;
    out.deceleration_span_ms=strong_span_us_/1000u;
    const bool dropout=out.evidence==Evidence::RecognitionDropout;
    const bool observed=out.evidence==Evidence::ObservedShape;
    if((!dropout && !observed) || (dropout && range_rejected_) ||
       acceleration_<(dropout?60.f:40.f)) {clearPending();return out;}
    out.stopping_distance_deg=speed*speed/(2.f*acceleration_);
    if(out.stopping_distance_deg+1.f>=distance ||
       (dropout && (strong_windows_<2 || strong_span_us_<40000u || strong_speed_loss_<4.f))) {
      clearPending();return out;
    }
    if(pending_evidence_!=out.evidence || !pending_ms_) {
      pending_evidence_=out.evidence;pending_ms_=in.now;
    }
    const float c=walking_energy_policy::passiveCoefficients(-in.side).omega2_j;
    const float rad=lateral_energy::kRadPerDeg;
    const float requested=c*rad*rad*(2.f*acceleration_*distance-speed*speed);
    out.request_j=std::fmin((dropout?.25f:.5f)*in.target_j,std::fmax(0.f,requested));
    out.selected=out.request_j>0 && in.now-pending_ms_>=(dropout?12u:6u);
    return out;
  }
 private:
#if SUPPORT_TRANSFER_STALL_V3
  // A 25 ms pair must show the same receiving sole, decreasing inward speed,
  // and a projected stop on the departing side of mechanical ZERO twice.
  Choice observeSmallArc(const Input& in, Evidence evidence) {
    Choice out;
    const auto* p=in.periodic;
    const float speed=-in.rate_dps;
    const bool approaching=!consumed_ && in.active && in.side==1 && in.sample_us &&
        in.sample_age_us<=10000 && in.angle_deg>=.35f && in.angle_deg<=12.f &&
        in.peak_deg-in.angle_deg>=.5f && in.now>=in.peak_ms && in.now-in.peak_ms>=60 &&
        in.now-in.peak_ms<=800 && speed>=3.f && speed<=35.f &&
        evidence==Evidence::ObservedUnseparated && in.foot.left_deg>in.foot.right_deg;
    if (!approaching) {
      arc_confirming_=false;
      // A measured change back towards the departing side after a forecast
      // sample ends this approach. A later gyro fluctuation is not a new step.
      if (in.predict_left_return && approaching_image_ &&
          in.sample_us && in.sample_age_us<=10000 && speed<=0) approach_reversed_=true;
      approaching_gap_=NAN;
    }
    if (!p || !p->count || !p->epoch_us || p->count==arc_packet_) return out;
    const uint32_t dt=in.sample_us-arc_sample_us_;
    const bool adjacent=arc_packet_ && p->count-arc_packet_==1 && dt>=15000 && dt<=50000 &&
        p->scheduled_us-arc_slot_us_==periodic_foot::period_us;
    // The control core already differentiates live gyro rates at adjacent
    // periodic frames. It passes that result only on the originating IMU tick.
    const float deceleration=in.periodic_acceleration_dps2;
    arc_packet_=p->count; arc_sample_us_=in.sample_us; arc_slot_us_=p->scheduled_us;
    const bool previous=arc_confirming_;
    const float previous_deceleration=arc_deceleration_;
    const uint64_t previous_image=arc_image_us_;
    const float previous_gap=approaching_gap_;
    const uint64_t previous_gap_image=approaching_image_;
    arc_confirming_=false;
    approaching_gap_=NAN;
    const auto& l=p->left;
    const auto& r=p->right;
    const uint32_t now_us=in.sample_us+in.sample_age_us;
    if (!approaching || !std::isfinite(deceleration) || deceleration<60.f || !l.sequence ||
        l.sequence!=r.sequence || l.sequence!=in.foot.sequence || !l.image_us ||
        l.image_us!=r.image_us || l.image_us!=in.foot.image_us || l.image_us<p->epoch_us ||
        p->published_us<l.image_us || p->published_us-l.image_us>periodic_foot::maximum_age_us ||
        now_us-static_cast<uint32_t>(p->published_us)>periodic_foot::period_us ||
        now_us-static_cast<uint32_t>(l.image_us)>periodic_foot::maximum_age_us ||
        !(std::fabs(in.fore_aft_deg)<=10 && std::fabs(l.measured)<=20 && std::fabs(r.measured)<=20 &&
          l.measured>r.measured && l.measured-r.measured<3 && in.target_j>0 && std::isfinite(in.target_j))) return out;
    const float stop=speed*speed/(2.f*deceleration);
    if (stop>=in.angle_deg) return out;
    const float gap=support_transfer_plan::supportGap(l.measured,r.measured,in.fore_aft_deg,in.angle_deg);
    Evidence reason=Evidence::Unavailable;
    if (gap<0 && l.image_us>previous_image) {
      arc_confirming_=true; arc_image_us_=l.image_us;arc_deceleration_=deceleration;
      if (adjacent && previous) {
        reason=Evidence::MeasuredArcShortfall;
        out.previous_arc_deceleration_dps2=previous_deceleration;
      }
    }
    // During current rise a still-higher receiving sole can reach the ground.
    // Forecast its gap from consecutive measured poses, only while the body
    // is still moving left and both snapshots predict a stop before ZERO.
    if (in.predict_left_return && !approach_reversed_ && speed>=support_transfer_plan::kCarryDps &&
        std::isfinite(in.current_rise_tau_s) && in.current_rise_tau_s>0 && std::isfinite(gap) &&
        l.image_us>previous_gap_image) {
      approaching_gap_=gap;
      approaching_image_=l.image_us;
      if (adjacent && previous_gap>0 && previous_gap>gap && speed<approaching_speed_) {
        const float until_stop=speed/deceleration;
        const float lead=std::fmin(in.current_rise_tau_s,2.e-6f*periodic_foot::period_us);
        const float until_transfer=std::fmax(0.f,gap)*float(dt)*1.e-6f/(previous_gap-gap);
        if (until_transfer<=lead && until_transfer<until_stop) {
          reason=Evidence::PredictedArcShortfall;
          out.predicted_transfer_ms=1000.f*until_transfer;
        }
      }
      approaching_speed_=speed;
    }
    if (reason==Evidence::Unavailable) return out;
    const float entry=support_transfer_plan::arcEntryDeg(-1,l.measured+in.fore_aft_deg);
    if (reason==Evidence::PredictedArcShortfall) out.terminal_deg=entry;
    const float c=walking_energy_policy::passiveCoefficients(-1).omega2_j;
    const float rad=lateral_energy::kRadPerDeg, carry=support_transfer_plan::kCarryDps;
    const float needed=c*rad*rad*(2*deceleration*(in.angle_deg-entry)-speed*speed+carry*carry);
    out.request_j=std::fmin(.25f*in.target_j,needed);
    out.selected=out.request_j>0; out.evidence=reason;
    out.deceleration_dps2=deceleration; out.stopping_distance_deg=stop;
    out.foot_age_ms=in.now-in.foot.frame_ms;
    return out;
  }
  uint32_t arc_packet_=0,arc_sample_us_=0;
  uint64_t arc_image_us_=0,arc_slot_us_=0;
  bool arc_confirming_=false;
  float arc_deceleration_=NAN;
  float approaching_gap_=NAN,approaching_speed_=0;
  uint64_t approaching_image_=0;
  bool approach_reversed_=false;
#endif
  void clearPending(){pending_ms_=0;pending_evidence_=Evidence::Unavailable;}
  void clearMotion() {
    anchor_sample_us_=last_sample_us_=last_ms_=0;anchor_speed_=acceleration_=0;
    strong_span_us_=0;strong_windows_=0;strong_speed_loss_=0;clearPending();
  }
  uint16_t run_=0;
  uint32_t peak_=0,anchor_sample_us_=0,last_sample_us_=0,last_ms_=0,pending_ms_=0,strong_span_us_=0;
  float anchor_speed_=0,acceleration_=0,strong_speed_loss_=0;
  uint8_t strong_windows_=0;
  Evidence pending_evidence_=Evidence::Unavailable;
  bool consumed_=false,range_rejected_=false;
};
}
