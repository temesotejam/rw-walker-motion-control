#pragma once
#include <cmath>
#include <cstdint>

// Experimental, observation-driven startup. The first isolated peak is not a
// completed walking cycle. Neither the requested current nor a peak comparison
// is proof of torque, ground stride, contact, or physical left/right mapping.
// Sensor, actuator, attitude and emergency protections belong to the runner.
namespace tilt_priority {
struct Input {
  uint32_t now_ms=0;
  float preset_deg=NAN;
  float target_deg=NAN;
  // Absolute angular rate at the detected crossing, before this input. This
  // does not measure subsequent acceleration or the speed of either foot.
  float crossing_rate_dps=NAN;
  int8_t next_side=0;
  uint16_t hardware_cap_ms=0;
  uint16_t negative_cap_extra_ms=0;
  uint16_t positive_cap_extra_ms=0;
  bool hardware_healthy=false;
};
struct Choice {
  bool selected=false,coast=false,speed_tapered=false;
  int16_t current_ma=0;
  uint16_t width_ms=0;
  // Width before downstream wheel and foot intervention; used only to avoid
  // learning from an input that was changed after the proposal.
  uint16_t proposed_width_ms=0;
  int8_t direction=0;
  float target_deg=NAN;
};

class Controller {
 public:
  void reset() {*this=Controller{};}
  float target()const{return target_deg_;}
  uint16_t widthLimit(int side)const{return history_[index(side)].width_ms;}
  // Absolute trial cap: retain the hardware limit and the previously
  // observed time-to-peak guard. The caller may explore around the ordinary
  // ramp/velocity-selected width only while staying under this cap.
  __attribute__((noinline)) uint16_t timingCap(int side,uint32_t now,uint16_t hardware) const {
    if((side!=1 && side!=-1)||!now)return 0;
    uint16_t cap=hardware<150 ? hardware : 150;
    const History& h=history_[index(side)];
    const bool fresh=h.peak_ms && now>=h.peak_ms && now-h.peak_ms<=1500;
    if(fresh && h.paired_ms==h.peak_ms && h.elapsed_ms) {
      if(h.elapsed_ms<=30)return 0;
      if(cap>h.elapsed_ms-30)cap=static_cast<uint16_t>(h.elapsed_ms-30);
    }
    return cap;
  }

  // Completed camera-relative full-cycle spans are a comparative signal,
  // not ground displacement. Never infer polarity from one frame or a first
  // half step. Invalid/stale input clears the experimental timing request.
  __attribute__((noinline)) void footCycle(uint32_t start,uint32_t end,float left,float right,bool usable) {
    if(!end || end<=foot_end_)return;
    const bool continuous=foot_end_ && start==foot_end_;
    foot_end_=end;
    if(!usable || start>=end || !std::isfinite(left) || !std::isfinite(right) ||
       left<1.f || right<0.f || right>=left*.6f || left-right<.5f) {
      foot_weak_count_=0;return;
    }
    foot_weak_count_=continuous ? (foot_weak_count_<2 ? foot_weak_count_+1 : 2) : 1;
  }
  // Trial: as the negative excursion weakens, move the inward trigger
  // gradually up to three degrees before zero. The negative excursion was
  // weak in runs 590 and 305. The threshold is bounded both by the latest
  // positive peak and by the measured inter-side gap. This is an input
  // timing experiment, not a measured ground step or guarantee.
  __attribute__((noinline)) float negativeEarlyThreshold(uint32_t now,int peak_side) const {
    if(peak_side!=1 || !now)return 0.f;
    const History& plus=history_[1];const History& minus=history_[0];
    if(!plus.peak_ms || !minus.peak_ms || now<plus.peak_ms || now<minus.peak_ms ||
       now-plus.peak_ms>1500 || now-minus.peak_ms>2500 ||
       !std::isfinite(plus.peak_deg) || !std::isfinite(minus.peak_deg))return 0.f;
    const bool observed_foot=foot_weak_count_>=2 && now>=foot_end_ && now-foot_end_<=2500;
    const float gap=plus.peak_deg-minus.peak_deg;
    if(!(gap>=2.5f || (observed_foot && gap>=1.f)))return 0.f;
    return std::fmin(3.f,std::fmin(plus.peak_deg*.4f,std::fmax(1.f,gap*.6f)));
  }

  // The crossing and its accepted result have run-relative timestamps. A
  // result is usable only after confirm and a subsequently paired peak.
  void crossing(int8_t side,uint32_t ms) {
    if((side!=1 && side!=-1)||!ms)return;
    History& h=history_[index(side)];h.cross_ms=ms;h.confirmed=false;
  }
  void confirm(int8_t side,uint32_t ms,const Choice& choice,bool executed,bool temporary_cap_trial=false,bool allow_taper_learning=true) {
    if((side!=1 && side!=-1)||!ms)return;
    History& h=history_[index(side)];
    h.confirmed=executed && choice.selected && choice.direction==side &&
      h.cross_ms==ms && std::isfinite(choice.target_deg) &&
      (choice.coast ? choice.width_ms==0 && choice.current_ma==0 :
        choice.width_ms>0 && choice.current_ma==1200);
    if(h.confirmed){
      h.command_ms=choice.width_ms;h.was_drive=!choice.coast;h.command_target=choice.target_deg;
      h.temporary_cap_trial=temporary_cap_trial;
      h.taper_learning=allow_taper_learning && !temporary_cap_trial && !choice.coast && choice.speed_tapered &&
          choice.proposed_width_ms && choice.width_ms==choice.proposed_width_ms;
      if(choice.coast) { last_coast_ms_=ms;last_coast_side_=side; }
      else { last_coast_ms_=0;last_coast_side_=0; }
    } else { last_coast_ms_=0;last_coast_side_=0; }
  }

  __attribute__((noinline)) void peak(int8_t side,uint32_t ms,float angle) {
    if((side!=1 && side!=-1)||!ms||!std::isfinite(angle)||angle<0.f)return;
    History& h=history_[index(side)];
    if(h.peak_ms && ms<=h.peak_ms)return;
    const bool paired=h.confirmed && h.cross_ms && ms>h.cross_ms &&
      ms-h.cross_ms<=1500 && (!h.was_drive || ms-h.cross_ms>=h.command_ms);
    if(initial_peak_seen_ && paired && last_pair_side_==side)clearPair();
    h.peak_deg=angle;h.peak_ms=ms;
    if(paired && ms-h.cross_ms<=600){h.elapsed_ms=static_cast<uint16_t>(ms-h.cross_ms);h.paired_ms=ms;}
    else {h.elapsed_ms=0;h.paired_ms=0;}
    // Preserve evidence only for an actually sent, time-matched drive. It is
    // applied after both halves of the same completed cycle are observed.
    h.evidence=0;
    if(initial_peak_seen_ && paired && h.was_drive && std::isfinite(h.command_target)) {
      if(angle>h.command_target+1.5f)h.evidence=2;
      else if(!h.temporary_cap_trial && angle<h.command_target-.5f && h.command_ms==h.width_ms)h.evidence=1;
      else if(h.taper_learning && h.command_target==8.f && angle<h.command_target-1.f &&
              h.command_ms<h.width_ms && h.width_ms<80)h.evidence=4;
      else h.evidence=3;
    }
    h.confirmed=false;h.cross_ms=0;
    if(!initial_peak_seen_) {initial_peak_seen_=true;return;}
    // A first, isolated peak never forms half of an established cycle. Only
    // paired post-start peaks from two alternating sides can advance a stage.
    if(!paired) {clearPair();return;}
    last_pair_side_=side;
    const unsigned bit=side>0?2u:1u;
    pair_mask_|=bit;
    // A consistently weak but valid oscillation must not trap startup below
    // the requested goal. Raise the stage after two complete observed cycles
    // even if the measured peak is low; a large overshoot holds the stage.
    if(angle>target_deg_+1.5f)pair_good_=false;
    if(pair_mask_==3u) {
      // Side-cap exploration must not run away while the two rocking peaks
      // diverge. Only undo cap increases made by the tapered-recovery rule;
      // ordinary controller and its protections remain in force.
      if(target_deg_==8.f && history_[0].paired_ms==history_[0].peak_ms &&
         history_[1].paired_ms==history_[1].peak_ms) {
        const float gap=std::fabs(history_[1].peak_deg-history_[0].peak_deg);
        if(gap>=1.5f) {
          balanced_pairs_=0;
          if(unequal_pairs_<2)++unequal_pairs_;
          if(unequal_pairs_>=2 && !recovery_locked_) {
            recovery_locked_=true;
            for(auto& h:history_) {
              if(h.recovery_extra_ms) {
                h.width_ms=h.width_ms>h.recovery_extra_ms+20 ?
                    static_cast<uint16_t>(h.width_ms-h.recovery_extra_ms) : 20;
              }
              h.recovery_extra_ms=0;h.under_count=0;
              if(h.evidence==4)h.evidence=3;
            }
          }
        } else if(recovery_locked_ && gap<=.8f) {
          unequal_pairs_=0;
          if(++balanced_pairs_>=3) {recovery_locked_=false;balanced_pairs_=0;}
        } else {unequal_pairs_=0;balanced_pairs_=0;}
      } else {unequal_pairs_=0;balanced_pairs_=0;}
      if(recovery_locked_)for(auto& h:history_)if(h.evidence==4)h.evidence=3;
      updateWidth(history_[0]);updateWidth(history_[1]);
      if(pair_good_)++good_pairs_;else good_pairs_=0;
      if(good_pairs_>=2) {target_deg_=std::fmin(8.f,target_deg_+.5f);good_pairs_=0;}
      pair_mask_=0;pair_good_=true;
    }
  }

  __attribute__((noinline)) Choice propose(const Input& in) const {
    Choice out;
    if(in.preset_deg!=8.f || !in.hardware_healthy ||
       (in.next_side!=1 && in.next_side!=-1) || !std::isfinite(in.target_deg) ||
       in.target_deg<=0.f || !std::isfinite(in.crossing_rate_dps) ||
       in.crossing_rate_dps<=0.f || !in.now_ms || !in.hardware_cap_ms)return out;
    const History& h=history_[index(in.next_side)];
    const History& opposite=history_[index(-in.next_side)];
    if((h.peak_ms && in.now_ms<h.peak_ms) ||
       (opposite.peak_ms && in.now_ms<opposite.peak_ms))return out;
    const bool fresh=h.peak_ms && in.now_ms>=h.peak_ms && in.now_ms-h.peak_ms<=1500 && std::isfinite(h.peak_deg);
    const bool opposite_fresh=opposite.peak_ms && in.now_ms>=opposite.peak_ms &&
        in.now_ms-opposite.peak_ms<=1500 && std::isfinite(opposite.peak_deg);
    // Once startup has a peak, a missing/stale preceding opposite observation
    // cannot justify a new experimental pulse. Return to ordinary control.
    if(initial_peak_seen_ && !opposite_fresh)return out;
    const float target=std::fmin(in.target_deg,target_deg_);
    out.target_deg=target;out.direction=in.next_side;
    // Trial heuristic, not an identified energy or foot-speed limit. A fast
    // body crossing may already have momentum to swing the foot. Rate alone
    // reduces pulse width, but does not force coast or command a stop.
    const float low_rate=target*4.f, high_rate=target*12.f;
    // A large incoming opposite peak can carry considerable energy to the
    // next side. Do not add another full input merely because the earlier
    // same-side peak was a small startup half step. This is not a stop.
    const bool overshoot=(opposite_fresh && opposite.peak_deg>target+1.5f) ||
                         (fresh && h.peak_deg>target+1.5f);
    // A preceding coast followed by a small returning peak can leave the
    // first oscillation without input for two halves. In this narrow startup
    // case try only 15 ms, retaining hardware and time-to-peak constraints.
    // This is a trial, not a guarantee of the next physical amplitude.
    const bool recovery=overshoot && target<=5.5f && last_coast_ms_ &&
        last_coast_side_==-in.next_side && in.now_ms>last_coast_ms_ &&
        in.now_ms-last_coast_ms_<=1500 && fresh && opposite_fresh &&
        opposite.peak_deg<=target+.5f && h.peak_deg<=target+2.5f &&
        in.crossing_rate_dps<target*10.f;
    if(overshoot && !recovery) {out.selected=true;out.coast=true;return out;}
    uint16_t cap=in.hardware_cap_ms<150 ? in.hardware_cap_ms : 150;
    uint16_t side_cap=h.width_ms;
    if(in.next_side>0 && in.target_deg==8.f && target_deg_==8.f && in.positive_cap_extra_ms)
      side_cap=static_cast<uint16_t>(side_cap+(in.positive_cap_extra_ms<5 ? in.positive_cap_extra_ms : 5));
    if(in.next_side<0 && in.target_deg==8.f && target_deg_==8.f && in.negative_cap_extra_ms)
      side_cap=static_cast<uint16_t>(side_cap+(in.negative_cap_extra_ms<5 ? in.negative_cap_extra_ms : 5));
    if(side_cap<cap)cap=side_cap;
    if(fresh && h.paired_ms==h.peak_ms && h.elapsed_ms) {
      // The previous timing is evidence, not a guarantee about a new pulse.
      if(h.elapsed_ms<=30) {out.selected=true;out.coast=true;return out;}
      if(cap>h.elapsed_ms-30)cap=static_cast<uint16_t>(h.elapsed_ms-30);
    }
    if(recovery && cap>15)cap=15;
    const bool uncapped_side_limit=(cap==h.width_ms && !recovery);
    if(in.crossing_rate_dps>low_rate && cap>20) {
      const uint16_t before_taper=cap;
      const float fraction=std::fmax(0.f,(high_rate-in.crossing_rate_dps)/(high_rate-low_rate));
      cap=static_cast<uint16_t>(20.f+(cap-20)*fraction);
      out.speed_tapered=uncapped_side_limit && cap>20 && cap<before_taper;
    }
    if(!cap)return Choice{};
    out.selected=true;out.current_ma=1200;out.width_ms=cap;out.proposed_width_ms=cap;return out;
  }

 private:
  struct History {
    float peak_deg=NAN,command_target=NAN;
    uint32_t peak_ms=0,cross_ms=0,paired_ms=0;
    uint16_t elapsed_ms=0,command_ms=0,width_ms=60,recovery_extra_ms=0;
    uint8_t under_count=0,evidence=0;
    bool confirmed=false,was_drive=false,temporary_cap_trial=false,taper_learning=false;
  };
  static unsigned index(int side){return side>0?1u:0u;}
  void clearPair(){pair_mask_=0;good_pairs_=0;pair_good_=true;unequal_pairs_=balanced_pairs_=0;history_[0].evidence=history_[1].evidence=0;}
  static void updateWidth(History& h) {
    if(h.evidence==2) {
      h.under_count=0;
      h.recovery_extra_ms=0;
      const uint16_t used=h.command_ms<h.width_ms ? h.command_ms : h.width_ms;
      h.width_ms=used>30 ? static_cast<uint16_t>(used-10) : 20;
    } else if(h.evidence==1) {
      if(++h.under_count>=2) {
        h.under_count=0;
        h.width_ms=h.width_ms<140 ? static_cast<uint16_t>(h.width_ms+10) : 150;
      }
    } else if(h.evidence==4) {
      // At the final 8-degree target, two paired, unmodified, tapered
      // commands with >1 degree undershoot allow a small cap increase.
      // This does not claim the pulse produced the observed peak.
      if(++h.under_count>=2) {
        h.under_count=0;
        const uint16_t previous_width=h.width_ms;
        h.width_ms=static_cast<uint16_t>(h.width_ms+5);
        if(h.width_ms>80)h.width_ms=80;
        h.recovery_extra_ms=static_cast<uint16_t>(h.recovery_extra_ms+h.width_ms-previous_width);
      }
    } else h.under_count=0;
    h.evidence=0;
  }
  History history_[2];
  float target_deg_=4.5f;
  uint8_t pair_mask_=0,good_pairs_=0;
  uint8_t unequal_pairs_=0,balanced_pairs_=0;
  bool recovery_locked_=false;
  uint8_t foot_weak_count_=0;
  uint32_t foot_end_=0;
  uint32_t last_coast_ms_=0;
  int8_t last_coast_side_=0;
  int8_t last_pair_side_=0;
  bool initial_peak_seen_=false,pair_good_=true;
};
}
