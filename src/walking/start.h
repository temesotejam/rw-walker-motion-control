#pragma once
#include <cstdint>
#include <cmath>

namespace walking_trial {
enum class Profile : uint8_t { None, Turn3, SmallStep, RightArrival, CarryHistory, ReturnPrediction, LeftTransferCarryV1, LeftTransferCarryV2, LeftTransferCarryV3, LeftTransferCarryV4, LeftTransferCarryV5, LeftTransferCarryV6, LeftTransferCarryV7 };

inline bool hasRecovery(Profile profile) {
  return profile == Profile::LeftTransferCarryV6 || profile == Profile::LeftTransferCarryV7;
}
// The accepted run settings determine the logged and executed policy. Earlier
// names remain available for interpreting archived measurements.
inline Profile forRun(bool wheel_work, float target_deg, float input_percent) {
  if (!wheel_work || target_deg!=8.f || input_percent!=0.f) return Profile::None;
#if WALKING_SMALL_STEP_AB && WALKING_RETURN_HISTORY && WALKING_RECOVERY_V6
  return Profile::LeftTransferCarryV7;
#elif WALKING_SMALL_STEP_AB && WALKING_RETURN_HISTORY
  return Profile::LeftTransferCarryV5;
#elif WALKING_SMALL_STEP_AB && WALKING_RIGHT_ARRIVAL
  return Profile::RightArrival;
#elif WALKING_SMALL_STEP_AB
  return Profile::Turn3;
#else
  return Profile::None;
#endif
}
inline const char* name(Profile profile) {
  switch (profile) {
    case Profile::Turn3: return "turn3";
    case Profile::SmallStep: return "v14_small_step";
    case Profile::RightArrival: return "right_arrival_v1";
    case Profile::CarryHistory: return "left_return_history_v1";
    case Profile::ReturnPrediction: return "left_return_prediction_v1";
    case Profile::LeftTransferCarryV1: return "left_transfer_carry_v1";
    case Profile::LeftTransferCarryV2: return "left_transfer_carry_v2";
    case Profile::LeftTransferCarryV3: return "left_transfer_carry_v3";
    case Profile::LeftTransferCarryV4: return "left_transfer_carry_v4";
    case Profile::LeftTransferCarryV6: return "left_transfer_carry_v6";
    case Profile::LeftTransferCarryV7: return "left_transfer_carry_v7";
    case Profile::LeftTransferCarryV5: return "left_transfer_carry_v5";
    default: return "none";
  }
}
}

// Experimental 8-degree walking startup. Two successive, fresh opposite-side
// peak pairs with usable wheel speed advance the requested target. A measured
// peak need not have reached the current target: otherwise a saturated pulse
// cap and an under-target peak can hold one another indefinitely. Invalid,
// stale, non-alternating or high-speed measurements never advance it. The
// caller remains responsible for physical angle/current/STOP guards.
namespace gradual_start {
// Provisional mounting hypothesis: positive pitch is physical left. Keep the
// 8-degree preset and start ramp, adding up to 2 degrees only to that side.
// This is a requested target, not a prediction or an observed amplitude.
inline __attribute__((noinline)) float sideTargetDeg(float ramp_deg,int side) {
  const float progress=std::fmax(0.f,std::fmin(1.f,(ramp_deg-4.5f)/3.5f));
  return ramp_deg+(side>0?2.f*progress:0.f);
}
class Controller {
 public:
  void reset() { *this=Controller{}; }
  float target() const {return target_;}
  bool __attribute__((noinline)) positivePeakAtLeast(float target_deg,uint32_t now_ms) const {
    return std::isfinite(target_deg) && std::isfinite(positive_peak_) && positive_peak_ms_ &&
        now_ms>=positive_peak_ms_ && now_ms-positive_peak_ms_<=1500 && positive_peak_>=target_deg;
  }
  uint16_t __attribute__((noinline)) positiveCapMs() const {
    const float steps=std::floor((target_-4.5f)/.875f+1e-4f);
    return static_cast<uint16_t>(80+5*steps);
  }
  void __attribute__((noinline)) peak(int side,float angle,float rpm,bool fresh,uint32_t ms) {
    if(!fresh || !std::isfinite(angle) || !std::isfinite(rpm) ||
       angle<0 || (side!=1 && side!=-1)) {
      paired_=false;good_=0;if(side>0){positive_peak_=NAN;positive_peak_ms_=0;}return;
    }
    if(side>0) {positive_peak_=angle;positive_peak_ms_=ms;}
    if(!paired_) {if(good_ && (ms<=ms_ || ms-ms_>1500))good_=0;side_=side;angle_=angle;rpm_=rpm;ms_=ms;paired_=true;return;}
    if(side==side_ || ms<=ms_ || ms-ms_>1500) {
      side_=side;angle_=angle;rpm_=rpm;ms_=ms;good_=0;return;
    }
    // Valid under-target peaks are evidence that the mechanism is still
    // oscillating; do not confuse model/goal error with sensor failure.
    const bool good=std::fabs(rpm_)<=500.f && std::fabs(rpm)<=500.f;
    paired_=false;ms_=ms;
    if(!good){good_=0;return;}
    // The first detected alternating pair can include the small kick/initial
    // half movement rather than established oscillation. It initializes the
    // history but cannot itself speed up startup.
    if(!have_initial_pair_) {have_initial_pair_=true;good_=0;return;}
    if(++good_>=2) {target_=std::fmin(8.f,target_+.5f);good_=0;}
  }
 private:
  float target_=4.5f,angle_=NAN,rpm_=NAN,positive_peak_=NAN;
  uint32_t ms_=0,positive_peak_ms_=0;int side_=0;unsigned good_=0;bool paired_=false,have_initial_pair_=false;
};
}

namespace early_clock {
// Maximum bounded run-relative time; refuse negative or wrapped clocks.
constexpr uint64_t kMaxRunUs=600000000ULL;
inline bool relativeMs(uint64_t absolute_us,uint64_t epoch_us,uint32_t& out) {
  if(!epoch_us || absolute_us<epoch_us || absolute_us-epoch_us>kMaxRunUs)return false;
  out=static_cast<uint32_t>((absolute_us-epoch_us)/1000ULL);return true;
}
inline bool epochSupported(uint64_t epoch_us) {
  return epoch_us && epoch_us/1000ULL<=UINT32_MAX;
}
}

// Diagnostic experiment: geometry-derived angle, never a contact assertion.
// All times use the same run-relative monotonic millisecond clock.
namespace early_motion_gate {
constexpr uint32_t kMaxFrameAgeMs=120, kMinEstimatedLeadMs=20;
struct Gate {
  uint16_t run=0; uint32_t peak=0,frame=0,ready=0; float threshold=0; bool valid=false;
  void clear(){valid=false;}
  bool offer(uint16_t r,uint32_t p,uint32_t f,uint32_t t,float a,uint32_t now){
    clear();
    if(!r || !p || !f || f>t || t>now || now-f>kMaxFrameAgeMs ||
       !std::isfinite(a) || std::fabs(a)>1.f || std::fabs(a)<.25f) return false;
    run=r;peak=p;frame=f;ready=t;threshold=a;valid=true;return true;
  }
  bool fire(uint16_t r,uint32_t p,uint32_t now,float a,float v) {
    if(!valid)return false;
    if(run!=r || peak!=p || now<ready || now<frame || now-frame>kMaxFrameAgeMs){clear();return false;}
    if(!std::isfinite(a) || !std::isfinite(v) || v==0.f)return false;
    const float side=threshold>0.f?1.f:-1.f;
    if(a*side<=0.f){clear();return false;}
    if(v*side>=0.f || a*side>threshold*side)return false;
    // Screening estimate only: a future actuator deadline is not guaranteed.
    // A tiny conservative margin avoids permitting a rounded borderline
    // value after replacing the former float division with multiplication.
    return (50.f*std::fabs(a)*0.999999f)>std::fabs(v);
  }
};
}
