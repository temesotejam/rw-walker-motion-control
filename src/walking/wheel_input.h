#pragma once

// 5. Wheel input: choose the commanded pulse duration for the requested work.
// Enforce rotation, posture, half-cycle and current-response limits before admission.

#include "walking/required_work.h"
#include "walking/transfer_speed.h"
#include "mekf6.hpp"

namespace wheel_input {
// Positive work request, previously validated cap. If the cap cannot cover the
// request, its residual is reported by the caller after evaluating this width.
inline uint16_t widthForWork(const lateral_energy::CurrentTable& current,
    const walking_energy_policy::Input& in, float requested_j) {
  const auto coefficients = lateral_energy::coefficients(in.side);
  const float omega = in.rate_dps * lateral_energy::kRadPerDeg;
  uint16_t low = 0, high = in.cap_ms;
  while (high - low > 1) {
    const uint16_t mid = (low + high) / 2;
    const float q = lateral_energy::charge(current, mid, in.initial_current_ma, in.goal_current_ma);
    if (q < 0 || lateral_energy::work(coefficients, omega, q) < requested_j) low = mid;
    else high = mid;
  }
  return high;
}
} // namespace wheel_input

// Both factors are ±1, so this factor converts logical excitation to the
// Roller register and converts its signed current reading back to excitation.
constexpr int walkingCurrentPolarity(int normal_output_sign, int driver_polarity) {
  return normal_output_sign * driver_polarity;
}

namespace walking_positive_cap {
// Scope: only the otherwise-uncapped positive fallback of target-8 work mode.
// All other selectors retain their own established limits.
inline uint16_t fallback(bool work_mode,bool work_choice_valid,int side,float target_deg,
                         uint16_t gradual_cap,uint16_t wheel_cap,uint16_t maximum) {
  if(!work_mode || work_choice_valid || side<=0 || target_deg!=8.f)return maximum;
  uint16_t result=gradual_cap<wheel_cap ? gradual_cap : wheel_cap;
  return result<maximum ? result : maximum;
}
}

// Post-command prediction using actual elapsed SOFTWARE time. The current
// rise and inherited pulse-Q arrival response remain empirical models, not a
// measurement of delivered charge/work or identified physical body inertia.
namespace energy_pulse_completion {
constexpr char kRevision[]="actual_elapsed_pulse_completion_v16";
struct Input {
  int8_t side=0;
  float rate_dps=NAN,initial_current_ma=NAN,goal_current_ma=NAN,tau_s=NAN;
  float available_j=NAN,request_j=NAN;
  uint32_t actual_elapsed_ms=0;
  uint16_t planned_ms=0;
};
struct Result {
  // valid covers inputs and finite current/charge arithmetic. A negative net
  // signed Q is retained for audit but does NOT have a valid work prediction.
  bool valid=false,work_valid=false,duration_completed=false;
  bool width_supported=false,rate_supported=false,charge_supported=false,extrapolated=false;
  uint32_t actual_elapsed_ms=0;
  uint16_t planned_ms=0;
  float elapsed_s=NAN,q_mas=NAN,current_end_ma=NAN;
  float work_j=NAN,arrival_j=NAN,unmet_j=NAN;
};
inline Result evaluate(const Input& in){
  Result out;out.actual_elapsed_ms=in.actual_elapsed_ms;out.planned_ms=in.planned_ms;
  out.duration_completed=in.planned_ms>0 && in.actual_elapsed_ms>=in.planned_ms;
  // This flag describes model coverage, never an integration clamp. A planned
  // or actual duration above the historical 100ms range remains fully counted.
  out.width_supported=in.planned_ms>0 && in.planned_ms<=lateral_energy::kMaxMs &&
      in.actual_elapsed_ms<=lateral_energy::kMaxMs;
  if((in.side!=1 && in.side!=-1) || !std::isfinite(in.rate_dps) || in.rate_dps<0 ||
      !std::isfinite(in.initial_current_ma) || !std::isfinite(in.goal_current_ma) || in.goal_current_ma<=0 ||
      !std::isfinite(in.tau_s) || in.tau_s<=0 || !std::isfinite(in.available_j) || in.available_j<0 ||
      !std::isfinite(in.request_j) || in.request_j<0)return out;
  const auto c=lateral_energy::coefficients(in.side);
  out.rate_supported=in.rate_dps>=c.rate_min_dps && in.rate_dps<=c.rate_max_dps;
  const double seconds=static_cast<double>(in.actual_elapsed_ms)*.001;
  const double tail=std::exp(-seconds/in.tau_s);
  const double rise=-std::expm1(-seconds/in.tau_s);
  const double q=in.goal_current_ma*seconds+
      (static_cast<double>(in.initial_current_ma)-in.goal_current_ma)*in.tau_s*rise;
  out.elapsed_s=static_cast<float>(seconds);
  out.q_mas=in.actual_elapsed_ms?static_cast<float>(q):0.f;
  out.current_end_ma=in.actual_elapsed_ms?static_cast<float>(in.goal_current_ma+
      (static_cast<double>(in.initial_current_ma)-in.goal_current_ma)*tail):in.initial_current_ma;
  if(!std::isfinite(out.q_mas) || !std::isfinite(out.current_end_ma))return out;
  out.valid=true;
  out.charge_supported=out.q_mas>=0 && out.q_mas<=c.charge_max_mAs;
  out.extrapolated=!out.width_supported || !out.rate_supported || !out.charge_supported;
  if(out.q_mas<0){out.extrapolated=true;return out;}
  out.work_j=in.actual_elapsed_ms?
      lateral_energy::work(c,in.rate_dps*lateral_energy::kRadPerDeg,out.q_mas):0.f;
  if(!std::isfinite(out.work_j) || out.work_j<0){out.work_j=NAN;out.extrapolated=true;return out;}
  out.arrival_j=in.available_j+out.work_j;
  out.unmet_j=std::fmax(0.f,in.request_j-out.work_j);
  if(!std::isfinite(out.arrival_j) || !std::isfinite(out.unmet_j)){
    out.work_j=out.arrival_j=out.unmet_j=NAN;out.extrapolated=true;return out;
  }
  out.work_valid=true;return out;
}
}

// Experimental: preserve the early alternating wheel motion. A pair of
// fresh pre-input observations estimates its midpoint, not ground motion.
// Only shorten the pulse that would push an already biased wheel farther in
// the same direction; do not add torque, reverse torque or force a stop.
namespace wheel_cycle_guard {
class Controller {
 public:
  void reset() { *this=Controller{}; }
  // Four alternating, command-matched peaks form two observed pairs. Any
  // invalid, stale, weak or falling peak immediately removes extra action.
  void __attribute__((noinline)) peak(int side,float angle,uint32_t ms,bool matched) {
    if(!matched || !std::isfinite(angle) || angle<0.f || !ms || (side!=1 && side!=-1)) {
      clearPeaks();return;
    }
    if(peak_count_ && (side==peak_side_[peak_count_-1] || ms<=peak_ms_ || ms-peak_ms_>1500))
      clearPeaks();
    if(peak_count_==4) {
      for(int i=0;i<3;++i){peak_side_[i]=peak_side_[i+1];peak_angle_[i]=peak_angle_[i+1];}
      peak_count_=3;
    }
    peak_side_[peak_count_]=side;peak_angle_[peak_count_]=angle;++peak_count_;peak_ms_=ms;
  }
  uint16_t legacyWidth() const {return legacy_width_;}
  uint16_t extraReduction() const {return extra_reduction_;}
  bool trialEligible() const {return trial_eligible_;}
  uint16_t limit(int side, float rpm, bool valid, uint32_t ms, uint16_t width) {
    legacy_width_=width;extra_reduction_=0;trial_eligible_=false;
    if (!valid || !std::isfinite(rpm) || !ms || (side!=1 && side!=-1)) {
      reset(); return width;
    }
    float midpoint=0.f; bool paired=false;
    if (last_side_==-side && ms>last_ms_ && ms-last_ms_<=1500) {
      midpoint=.5f*(last_rpm_+rpm); paired=true;
    }
    last_side_=side; last_rpm_=rpm; last_ms_=ms;
    if (!paired || !width) return width;
    const float bias=midpoint*side;
    if(bias>150.f) {
      const float strength=std::fmin(1.f,(bias-150.f)/250.f);
      legacy_width_=static_cast<uint16_t>(std::ceil(width*(1.f-.25f*strength)));
    }
    trial_eligible_=qualified(side,ms);
    if(!trial_eligible_ || bias<=100.f) return legacy_width_;
    const float early=std::fmin(1.f,(bias-100.f)/250.f);
    const uint16_t candidate=static_cast<uint16_t>(std::ceil(width*(1.f-.25f*early)));
    uint16_t reduced=candidate<legacy_width_ ? candidate : legacy_width_;
    if(legacy_width_-reduced>5)reduced=legacy_width_-5;
    extra_reduction_=legacy_width_-reduced;
    return reduced;
  }
 private:
  void clearPeaks(){peak_count_=0;peak_ms_=0;}
  bool qualified(int side,uint32_t now) const {
    if(peak_count_!=4 || !peak_ms_ || now<peak_ms_ || now-peak_ms_>1500)return false;
    for(int i=0;i<4;++i)if(peak_angle_[i]<4.5f)return false;
    if(std::fabs(peak_angle_[0]-peak_angle_[1])>2.f ||
       std::fabs(peak_angle_[2]-peak_angle_[3])>2.f)return false;
    // A trial is disqualified if either side has recently fallen.
    if(peak_angle_[2]<peak_angle_[0]-.4f || peak_angle_[3]<peak_angle_[1]-.4f)return false;
    int last=-1,opposite=-1,previous=-1;
    for(int i=3;i>=0;--i) {
      if(peak_side_[i]==side){if(last<0)last=i;else if(previous<0)previous=i;}
      else if(opposite<0)opposite=i;
    }
    if(last<0 || previous<0 || opposite<0)return false;
    // Do not add shortening to the already weaker or declining body side.
    return peak_angle_[last]>=peak_angle_[opposite]-.3f &&
           peak_angle_[last]>=peak_angle_[previous]-.4f;
  }
  float last_rpm_=NAN;
  uint32_t last_ms_=0;
  int last_side_=0;
  float peak_angle_[4]={}; int8_t peak_side_[4]={};uint8_t peak_count_=0;
  uint32_t peak_ms_=0;uint16_t legacy_width_=0,extra_reduction_=0;
  bool trial_eligible_=false;
};
}

// Experimental bias guard: compare successive, fresh pre-input speeds.
// This limits an existing excitation; it never commands reverse torque.
namespace wheel_speed_balance {
class Controller {
 public:
  void reset() { *this=Controller{}; }
  void __attribute__((noinline)) peak(int side,float angle) {
    if(!std::isfinite(angle)) return;
    if(side>0) positive_peak_=angle; else if(side<0) negative_peak_=angle;
  }
  void __attribute__((noinline)) crossing(int side,float rpm,bool fresh,uint32_t ms) {
    if(!fresh || !std::isfinite(rpm)) { reset(); return; }
    if(side<0) { negative_rpm_=rpm;negative_ms_=ms;have_negative_=true;return; }
    if(side!=1) return;
    if(!have_negative_ || ms<=negative_ms_ || ms-negative_ms_>1500) { cap_=100;biased_=0;have_negative_=false;return; }
    // Both crossings rotating in the same direction indicates sustained
    // momentum, including states beyond the negative-side model's domain.
    // Reduce only the positive excitation, slowly; never invent brake torque.
    const bool sustained=negative_rpm_>=400.f && rpm>=150.f;
    if(sustained) {
      if(++biased_>=2) {if(cap_>60)cap_-=5;biased_=0;}
      have_negative_=false;return;
    }
    if(!std::isfinite(positive_peak_) || positive_peak_<5.f ||
       !std::isfinite(negative_peak_) || negative_peak_<4.f) { cap_=100;biased_=0;have_negative_=false;return; }
    const bool biased=negative_rpm_>=250.f && rpm>=-20.f && rpm<=150.f && negative_rpm_-rpm>=250.f;
    if(biased) { if(++biased_>=2) {if(cap_>80)cap_-=5;biased_=0;} }
    else {biased_=0;if(cap_<100)cap_+=5;}
    have_negative_=false;
  }
  uint16_t capMs() const {return cap_;}
 private:
  float negative_rpm_=NAN,positive_peak_=NAN,negative_peak_=NAN;
  uint32_t negative_ms_=0;uint16_t cap_=100;unsigned biased_=0;bool have_negative_=false;
};
}

// A provisional motion envelope, not an energy/stability/contact model. No J
// or corrective pulse is inferred. Keep the independent absolute tilt stop.
namespace fore_aft_motion_guard {
static constexpr const char* kRevision="fore_aft_motion_guard_v19";
namespace limits {
// Trial motion envelope only; these are not identified stability/contact or
// energy limits. The constant-rate forecast is a veto diagnostic, not MPC.
static constexpr float kCandidateAngleDeg=6.f,kMinimumOutwardRateDps=20.f;
static constexpr float kPredictedBoundaryDeg=20.f,kMaximumTimeToBoundaryMs=350.f;
static constexpr float kMinimumOutwardGainDeg=1.f;
static constexpr uint32_t kConfirmationUs=40000,kMaximumSampleAgeUs=10000;
}
enum class Reason : uint8_t {
  Healthy=0, Reference=1, Unobservable=2, Candidate=3, Diverging=4,
  Invalid=5, Stale=6
};
inline const char* reasonName(Reason reason) {
  switch(reason){
    case Reason::Reference:return "reference";
    case Reason::Unobservable:return "fore_aft_unobservable";
    case Reason::Candidate:return "fore_aft_outward_candidate";
    case Reason::Diverging:return "fore_aft_outward_divergence";
    case Reason::Invalid:return "fore_aft_invalid_sample";
    case Reason::Stale:return "fore_aft_stale_sample";
    default:return "healthy";
  }
}
struct Config {
  float candidate_angle_deg=limits::kCandidateAngleDeg;
  float minimum_outward_rate_dps=limits::kMinimumOutwardRateDps;
  float predicted_boundary_deg=limits::kPredictedBoundaryDeg;
  float maximum_time_to_boundary_ms=limits::kMaximumTimeToBoundaryMs;
  float minimum_outward_gain_deg=limits::kMinimumOutwardGainDeg;
  uint32_t confirmation_us=limits::kConfirmationUs;
  uint32_t maximum_sample_age_us=limits::kMaximumSampleAgeUs;
  uint32_t maximum_continuity_gap_us=10000;
  // Below this YZ gravity magnitude, lateral tilt makes fore/aft projection
  // ill-conditioned. Do not fabricate a fore/aft angle or a divergence event.
  float minimum_yz_projection_sq=.04f;
};
struct Input {
  // Same uint32 micros clock; valid includes acquisition health. Quaternion,
  // gyro and bias use the MEKF body frame (not raw IMU axes). Call for every
  // fresh sample, including while a motor pulse is active.
  uint32_t now_us=0,sample_us=0;
  bool valid=false;
  mekf6::Quaternion quaternion;
  mekf6::Vec3 gyro_rad_s,bias_rad_s;
};
struct Result {
  bool stop=false,allow_output=false,valid=false,initialized=false,reference_ready=false,sample_advanced=false;
  Reason reason=Reason::Healthy;
  float angle_deg=NAN,rate_dps=NAN,time_to_boundary_ms=NAN;
  uint32_t candidate_us=0,sample_us=0;
  uint32_t start_sample_us=0;
  float start_roll_deg=NAN;
};
class Guard {
 public:
  explicit Guard(const Config& config=Config{}):config_(config){}
  void reset(){
    ready_=stopped_=candidate_=false;
    reference_y_=0;reference_z_=previous_up_=1.f;previous_forward_=0.f;
    last_sample_us_=candidate_start_us_=start_sample_us_=0;
    candidate_angle_=0;start_roll_deg_=NAN;last_=Result{};
  }
  const Config& config()const{return config_;}
  const Result& result()const{return last_;}
  Result observe(const Input& in){
    if(stopped_){last_.sample_advanced=false;return last_;} // New run resets latch.
    const auto& q=in.quaternion;
    const float n=q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
    const auto finite=[](const mekf6::Vec3& v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
    if(!in.valid || !in.sample_us || !std::isfinite(n) || n<.9f || n>1.1f ||
       !finite(in.gyro_rad_s) || !finite(in.bias_rad_s))return stop(Reason::Invalid,in.sample_us);
    if(static_cast<uint32_t>(in.now_us-in.sample_us)>config_.maximum_sample_age_us)
      return stop(Reason::Stale,in.sample_us);
    if(ready_ && in.sample_us==last_sample_us_){last_.sample_advanced=false;return last_;}
    const float gx=2.f*(q.x*q.z-q.w*q.y)/n;
    const float gy=2.f*(q.y*q.z+q.w*q.x)/n;
    const float gz=(q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z)/n;
    const float yz=gy*gy+gz*gz;
    Result out;out.sample_us=in.sample_us;out.reference_ready=out.initialized=ready_;out.sample_advanced=true;
    out.start_sample_us=start_sample_us_;out.start_roll_deg=start_roll_deg_;
    if(yz<config_.minimum_yz_projection_sq){
      candidate_=false;out.reason=Reason::Unobservable;last_sample_us_=in.sample_us;
      last_=out;return out;
    }
    const float length=std::sqrt(yz);
    if(!ready_){
      reference_y_=gy/length;reference_z_=gz/length;ready_=true;
      start_sample_us_=in.sample_us;start_roll_deg_=std::atan2(gy,gz)*57.29577951308232f;
      previous_up_=1;previous_forward_=0;last_sample_us_=in.sample_us;
      out.valid=out.reference_ready=out.initialized=out.allow_output=true;out.reason=Reason::Reference;
      out.start_sample_us=start_sample_us_;out.start_roll_deg=start_roll_deg_;
      out.angle_deg=0;out.rate_dps=0;last_=out;return out;
    }
    if(static_cast<uint32_t>(in.sample_us-last_sample_us_)>config_.maximum_continuity_gap_us)
      candidate_=false;
    last_sample_us_=in.sample_us;
    float up=(gy*reference_y_+gz*reference_z_)/length;
    float forward=(gy*reference_z_-gz*reference_y_)/length;
    // A lateral +/-90 passage reverses both projected components. Preserve
    // their branch, assuming less than 90 deg fore/aft change per sample.
    if(up*previous_up_+forward*previous_forward_<0){up=-up;forward=-forward;}
    previous_up_=up;previous_forward_=forward;
    out.valid=out.reference_ready=out.initialized=out.allow_output=true;
    out.angle_deg=std::atan2(forward,up)*57.29577951308232f;
    const float wx=in.gyro_rad_s.x-in.bias_rad_s.x;
    const float wy=in.gyro_rad_s.y-in.bias_rad_s.y;
    const float wz=in.gyro_rad_s.z-in.bias_rad_s.z;
    // dg/dt = g cross omega. Body-X gyro alone is not d(Euler roll)/dt.
    out.rate_dps=(wx-gx*(gz*wz+gy*wy)/yz)*57.29577951308232f;
    const float distance=std::fabs(out.angle_deg);
    const float outward_rate=out.angle_deg>=0?out.rate_dps:-out.rate_dps;
    if(outward_rate>0)
      out.time_to_boundary_ms=1000.f*std::fmax(0.f,config_.predicted_boundary_deg-distance)/outward_rate;
    const bool qualifies=distance>=config_.candidate_angle_deg &&
      outward_rate>=config_.minimum_outward_rate_dps &&
      std::isfinite(out.time_to_boundary_ms) &&
      out.time_to_boundary_ms<=config_.maximum_time_to_boundary_ms;
    if(!qualifies){candidate_=false;last_=out;return out;}
    if(!candidate_){candidate_=true;candidate_start_us_=in.sample_us;candidate_angle_=out.angle_deg;}
    // A sign reversal is a returning/oscillating observation, not continuation.
    if(candidate_angle_*out.angle_deg<0){candidate_start_us_=in.sample_us;candidate_angle_=out.angle_deg;}
    out.reason=Reason::Candidate;
    out.allow_output=false;
    out.candidate_us=static_cast<uint32_t>(in.sample_us-candidate_start_us_);
    const float gain=std::fabs(out.angle_deg)-std::fabs(candidate_angle_);
    if(out.candidate_us>=config_.confirmation_us && gain>=config_.minimum_outward_gain_deg){
      out.stop=true;out.reason=Reason::Diverging;stopped_=true;
    }
    last_=out;return out;
  }
 private:
  Result stop(Reason reason,uint32_t sample_us){
    Result out;out.stop=true;out.reference_ready=out.initialized=ready_;out.reason=reason;out.sample_us=sample_us;
    out.start_sample_us=start_sample_us_;out.start_roll_deg=start_roll_deg_;
    stopped_=true;last_=out;return out;
  }
  Config config_;Result last_;
  bool ready_=false,stopped_=false,candidate_=false;
  float reference_y_=0,reference_z_=1,previous_up_=1,previous_forward_=0,candidate_angle_=0,start_roll_deg_=NAN;
  uint32_t last_sample_us_=0,candidate_start_us_=0,start_sample_us_=0;
};
}

namespace tilt_stop {
// Track the continuous fore/aft roll branch from an upright run start.
// In the calibrated body frame, gravity projected onto the YZ plane is
// cos(lateral_pitch) * [sin(fore_aft_roll), cos(fore_aft_roll)]. Sideways
// passage through +/-90 reverses BOTH components, not the fore/aft angle.
// Choose the sign continuous with the preceding sample to reject that reversal.
// This assumes <90 degrees of fore/aft motion between observable samples.
// No Euler extraction, inverse trig, allocation or extra estimator is used.
class ForeAftGuard {
 public:
  void reset() { previous_up_ = 1.0f; previous_forward_ = 0.0f; }
  const char* reason(const mekf6::Quaternion& q, bool valid, uint32_t age_us,
                     const mekf6::Vec3& upright) {
    const float n = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (!valid || !std::isfinite(n) || n < 0.9f || n > 1.1f) return "tilt_invalid_mekf";
    if (age_us > 10000) return "tilt_stale_mekf";
    const float gx = 2.0f * (q.x*q.z - q.w*q.y);
    const float gy = 2.0f * (q.y*q.z + q.w*q.x);
    const float gz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
    float up = gx*upright.x + gy*upright.y + gz*upright.z;
    // The calibrated Y axis is proportional to {0, upright.z, -upright.y}.
    // Its constant positive scale does not affect the zero boundary.
    float forward = gy*upright.z - gz*upright.y;
    const float projection_sq = up*up + forward*forward;
    // At exact sideways +/-90 the fore/aft gravity projection is unobservable.
    // Retain the previous branch; sideways posture itself must not cause STOP.
    if (projection_sq <= 1.0e-8f*n*n) return nullptr;
    if (up*previous_up_ + forward*previous_forward_ < 0.0f) {
      up = -up; forward = -forward;
    }
    previous_up_ = up; previous_forward_ = forward;
    // Relative tolerance keeps the 90-degree edge independent of lateral tilt.
    return up <= 0.0f || up*up <= 1.0e-12f*projection_sq ? "fore_aft_tilt_90deg" : nullptr;
  }
 private:
  float previous_up_ = 1.0f, previous_forward_ = 0.0f;
};
}

namespace walking_energy_policy {
inline Decision decide(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const Input& in){
  Decision out;
  if (!computePeakWork(geometry, current, in, out)) return out;
  const auto c=lateral_energy::coefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  const float maximum_q=lateral_energy::charge(current,in.cap_ms,in.initial_current_ma,in.goal_current_ma);
  if(!std::isfinite(out.available_j) || !std::isfinite(out.request_j) || !std::isfinite(maximum_q))return out;
  out.maximum_work_j=maximum_q>0?lateral_energy::work(c,omega,maximum_q):0.f;
  if(!std::isfinite(out.maximum_work_j) || out.maximum_work_j<0)return out;
  out.predicted_arrival_j=out.available_j;
  float error=out.request_j; // The zero-width candidate: no new pulse work.
  // W(Q) is monotone for Q>=0. Find the crossing then compare nearby integer
  // widths by ENERGY error. No fixed +/-ms rule sets the selected command.
  uint16_t lo=0,hi=in.cap_ms;
  if(out.request_j>0){
    while(hi-lo>1){
      const uint16_t mid=static_cast<uint16_t>((lo+hi)/2);
      const float q=lateral_energy::charge(current,mid,in.initial_current_ma,in.goal_current_ma);
      ++out.evaluations;
      if(q<0 || lateral_energy::work(c,omega,q)<out.request_j)lo=mid;else hi=mid;
    }
    const uint16_t candidates[3]={lo,hi,in.cap_ms};
    for(uint16_t width:candidates){
      const float q=lateral_energy::charge(current,width,in.initial_current_ma,in.goal_current_ma);
      ++out.evaluations;
      if(q<0 || !std::isfinite(q))continue;
      const float work=lateral_energy::work(c,omega,q);
      if(!std::isfinite(work) || work<0)continue;
      const float candidate_error=std::fabs(work-out.request_j);
      if(candidate_error<error || (candidate_error==error && width<out.width_ms)){
        error=candidate_error;out.width_ms=width;out.q_mas=q;
        out.predicted_work_j=work;out.predicted_arrival_j=out.available_j+work;
      }
    }
  }
  out.unmet_j=std::fmax(0.f,out.request_j-out.predicted_work_j);
  out.saturated=out.request_j>out.maximum_work_j+1e-9f;
  out.charge_supported=out.q_mas>=0 && out.q_mas<=c.charge_max_mAs;
  out.extrapolated=!out.rate_supported || !out.charge_supported || out.early_state_transfer || out.extra_energy_j>0;
  out.valid=out.selected=true;out.coast=out.width_ms==0;
  out.reason=out.coast?Reason::Coast:out.saturated?Reason::CapLimited:Reason::Drive;
  return out;
}
// Re-evaluate the *actual* post-guard width with the identical energy model.
// This does not authorize a different command or relax any guard.
inline Decision evaluateWidth(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const Input& in,uint16_t actual_width_ms){
  auto out=decide(geometry,current,in);
  if(!out.valid || actual_width_ms>in.cap_ms){out.valid=out.selected=false;out.reason=Reason::Invalid;return out;}
  const auto c=lateral_energy::coefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  const float q=actual_width_ms?lateral_energy::charge(current,actual_width_ms,in.initial_current_ma,in.goal_current_ma):0.f;
  const float work=actual_width_ms?lateral_energy::work(c,omega,q):0.f;
  if(!std::isfinite(q) || q<0 || !std::isfinite(work) || work<0){
    out.valid=out.selected=false;out.reason=Reason::Invalid;return out;
  }
  out.width_ms=actual_width_ms;out.q_mas=q;out.predicted_work_j=work;
  out.predicted_arrival_j=out.available_j+work;
  out.unmet_j=std::fmax(0.f,out.request_j-work);
  out.saturated=out.unmet_j>1e-9f;out.coast=actual_width_ms==0;
  out.charge_supported=q<=c.charge_max_mAs;
  out.extrapolated=!out.rate_supported || !out.charge_supported || out.early_state_transfer || out.extra_energy_j>0;
  out.reason=out.coast?Reason::Coast:out.saturated?Reason::CapLimited:Reason::Drive;
  return out;
}
}

// v15 keeps the v14 potential, paired-peak corrections and empirical response
// coefficients. Only integer-width selection and early admission change.
// Reaching the model's energy request is not a physical walking guarantee.
namespace walking_energy_policy_v15 {
using Input=walking_energy_policy::Input;
using Decision=walking_energy_policy::Decision;
using Reason=walking_energy_policy::Reason;
constexpr char kRevision[]="walking_early_energy_main_v15";
constexpr uint16_t kMaxPulseMs=walking_energy_policy::kMaxPulseMs;

namespace detail {
inline Decision atWidth(Decision out,const lateral_energy::CurrentTable& current,
    const Input& in,uint16_t width){
  out.width_ms=width;out.q_mas=0;out.predicted_work_j=0;
  out.predicted_arrival_j=out.available_j;out.unmet_j=out.request_j;
  out.coast=out.request_j==0.f;
  if(width){
    const auto c=lateral_energy::coefficients(in.side);
    out.q_mas=lateral_energy::charge(current,width,in.initial_current_ma,in.goal_current_ma);
    // Never count reverse residual charge as positive added work via abs(Q)
    // or via the quadratic term outside its identified excitation branch.
    if(!std::isfinite(out.q_mas) || out.q_mas<0){
      out.valid=out.selected=false;out.saturated=true;out.reason=Reason::Invalid;
      out.predicted_work_j=out.predicted_arrival_j=NAN;return out;
    }
    out.predicted_work_j=lateral_energy::work(c,in.rate_dps*lateral_energy::kRadPerDeg,out.q_mas);
    if(!std::isfinite(out.predicted_work_j) || out.predicted_work_j<0){
      out.valid=out.selected=false;out.saturated=true;out.reason=Reason::Invalid;return out;
    }
    out.predicted_arrival_j=out.available_j+out.predicted_work_j;
    out.unmet_j=std::fmax(0.f,out.request_j-out.predicted_work_j);
  }
  out.saturated=out.unmet_j>kEnergyRoundoffJ;
  const auto c=lateral_energy::coefficients(in.side);
  out.charge_supported=out.q_mas>=0 && out.q_mas<=c.charge_max_mAs;
  out.extrapolated=!out.rate_supported || !out.charge_supported || out.early_state_transfer || out.extra_energy_j>0;
  out.reason=out.coast?Reason::Coast:out.saturated?Reason::CapLimited:Reason::Drive;
  // A guard blocking all input while energy is missing is not surplus coast
  // and cannot become a zero-width drive. Retain its energy audit, fail closed.
  if(!width && !out.coast){out.valid=out.selected=false;out.reason=Reason::CapLimited;}
  return out;
}
}

inline Decision decide(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const Input& in){
  // Reuse the frozen v14 energy accounting and validity checks (including
  // cap0 invalid). Its nearest-width choice is discarded below.
  auto out=walking_energy_policy::decide(geometry,current,in);
  if(!out.valid)return out;
  out.evaluations=0;
  if(out.request_j==0.f)return detail::atWidth(out,current,in,0);
  const auto c=lateral_energy::coefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  const float maximum_q=lateral_energy::charge(current,in.cap_ms,in.initial_current_ma,in.goal_current_ma);
  if(!std::isfinite(maximum_q) || maximum_q<0){
    return detail::atWidth(out,current,in,in.cap_ms);
  }
  // 80ms is the inherited engineering envelope, not a physics-derived bound.
  // If even the cap cannot cover the request, report its remaining deficit.
  if(out.maximum_work_j<out.request_j)return detail::atWidth(out,current,in,in.cap_ms);
  uint16_t lo=0,hi=in.cap_ms;
  while(hi-lo>1){
    const uint16_t mid=static_cast<uint16_t>((lo+hi)/2);
    const float q=lateral_energy::charge(current,mid,in.initial_current_ma,in.goal_current_ma);
    ++out.evaluations;
    if(!std::isfinite(q)){
      out.valid=out.selected=false;out.reason=Reason::Invalid;return out;
    }
    if(q<0 || lateral_energy::work(c,omega,q)<out.request_j)lo=mid;
    else hi=mid;
  }
  // Unlike nearest-energy rounding, this is the least integer width whose
  // predicted work reaches a positive request. Tiny requests never round down
  // to surplus coast.
  return detail::atWidth(out,current,in,hi);
}

inline Decision evaluateWidth(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const Input& in,uint16_t actual_width_ms){
  auto out=walking_energy_policy_v15::decide(geometry,current,in);
  // A post-selection guard may only shorten the selected command. In
  // particular, a zero-request coast cannot acquire a positive pulse here.
  if(!out.valid || actual_width_ms>in.cap_ms || actual_width_ms>out.width_ms){
    out.valid=out.selected=false;out.reason=Reason::Invalid;return out;
  }
  return detail::atWidth(out,current,in,actual_width_ms);
}

inline bool earlyAdmissible(const Decision& post_guard,float epsilon_j=kEnergyRoundoffJ){
  return std::isfinite(epsilon_j) && epsilon_j>=0 && epsilon_j<=kEnergyRoundoffJ &&
      post_guard.valid && post_guard.selected && post_guard.width_ms>0 && !post_guard.coast &&
      !post_guard.saturated && std::isfinite(post_guard.request_j) && post_guard.request_j>0 &&
      std::isfinite(post_guard.predicted_work_j) && post_guard.predicted_work_j>=0 &&
      std::isfinite(post_guard.q_mas) && post_guard.q_mas>=0 &&
      post_guard.predicted_work_j+epsilon_j>=post_guard.request_j;
}
}

// Admission only: keep the selected J/width/current unchanged. A relocated
// main input needs both inherited fits' recorded rate support and excitation
// charge support, as well as its existing post-guard positive energy budget.
// These coverage flags are not proof of physical prediction accuracy.
namespace energy_admission_v19 {
constexpr char kRevision[]="optional_main_empirical_support_v19";
enum class Scope : uint8_t { OrdinaryZero=0,OptionalMain=1,ExplicitReturnAid=2,SmallStepEarly=3 };
enum class Reason : uint8_t {
  Allowed=0,OrdinaryZeroExempt=1,ExplicitReturnAidExempt=2,InvalidScope=3,
  InvalidDecision=4,NoPositiveCoveredBudget=5,PassiveRateUnsupported=6,
  WorkRateUnsupported=7,ChargeUnsupported=8,NoPositiveSmallStepWork=9
};
enum Unsupported : uint8_t { PassiveRate=1u,WorkRate=2u,Charge=4u };
struct Choice {
  bool applies=false,allowed=false;
  uint8_t unsupported_mask=0;
  Reason reason=Reason::InvalidScope;
};
inline Choice assess(Scope scope,const walking_energy_policy::Decision& post_guard){
  Choice out;
  if(!post_guard.passive_rate_supported)out.unsupported_mask|=PassiveRate;
  if(!post_guard.work_rate_supported)out.unsupported_mask|=WorkRate;
  if(!post_guard.charge_supported)out.unsupported_mask|=Charge;
  // Exempt means this additional optional-main gate has no veto. It does not
  // authorize output: the runner retains ordinary energy validity, explicit
  // return-J evidence, health, freshness, wheel guard and shared80ms checks.
  if(scope==Scope::OrdinaryZero){out.allowed=true;out.reason=Reason::OrdinaryZeroExempt;return out;}
  if(scope==Scope::ExplicitReturnAid){out.allowed=true;out.reason=Reason::ExplicitReturnAidExempt;return out;}
  out.applies=true;
  if(scope==Scope::SmallStepEarly){
    // The observed v14 gait used a bounded partial contribution. Report fit
    // coverage and residual J even when only part of the request can be met.
    if(!post_guard.valid || !post_guard.selected || post_guard.width_ms>15){out.reason=Reason::InvalidDecision;return out;}
    if(post_guard.coast || !post_guard.width_ms || !(post_guard.request_j>0 && post_guard.q_mas>0 &&
       post_guard.predicted_work_j>0)){out.reason=Reason::NoPositiveSmallStepWork;return out;}
    if(!post_guard.charge_supported){out.reason=Reason::ChargeUnsupported;return out;}
    out.allowed=true;out.reason=Reason::Allowed;return out;
  }
  if(scope!=Scope::OptionalMain)return out;
  if(!post_guard.valid || !post_guard.selected){out.reason=Reason::InvalidDecision;return out;}
  if(!walking_energy_policy_v15::earlyAdmissible(post_guard)){
    out.reason=Reason::NoPositiveCoveredBudget;return out;
  }
  if(out.unsupported_mask&PassiveRate){out.reason=Reason::PassiveRateUnsupported;return out;}
  if(out.unsupported_mask&WorkRate){out.reason=Reason::WorkRateUnsupported;return out;}
  if(out.unsupported_mask&Charge){out.reason=Reason::ChargeUnsupported;return out;}
  out.allowed=true;out.reason=Reason::Allowed;return out;
}
inline bool optionalMainAdmissible(const walking_energy_policy::Decision& post_guard){
  return assess(Scope::OptionalMain,post_guard).allowed;
}
// On rejection, send nothing and leave the real ZERO opportunity unconsumed.
// Do not reinterpret a narrower supported Q as a newly authorized command;
// the width remains the existing energy inverse result or is not relocated.
}

namespace walking_return_energy {
inline Decision select(const lateral_energy::Geometry& g,const lateral_energy::CurrentTable& table,
    const Input& in,uint16_t forced=UINT16_MAX){
  auto out=walking_energy_policy_v15::decide(g,table,in);
  if (!updateWorkRequest(out, in)) return out;
  if(out.request_j==0){
    if(forced!=UINT16_MAX && forced!=0){out.valid=out.selected=false;return out;}
    return walking_energy_policy_v15::detail::atWidth(out,table,in,0);
  }
  const uint16_t hi=wheel_input::widthForWork(table,in,out.request_j);
  const auto chosen=walking_energy_policy_v15::detail::atWidth(out,table,in,hi);
  if(forced==UINT16_MAX)return chosen;
  if(forced>chosen.width_ms){out.valid=out.selected=false;return out;}
  return walking_energy_policy_v15::detail::atWidth(out,table,in,forced);
}
inline Decision decide(const lateral_energy::Geometry& g,const lateral_energy::CurrentTable& t,const Input& in){auto d=select(g,t,in);d.extrapolated=true;return d;}
inline Decision evaluateWidth(const lateral_energy::Geometry& g,const lateral_energy::CurrentTable& t,const Input& in,uint16_t width){auto d=select(g,t,in,width);d.extrapolated=true;return d;}
inline bool earlyAdmissible(const Decision& d){return walking_energy_policy_v15::earlyAdmissible(d);}
} // namespace walking_return_energy

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
  const uint16_t hi=wheel_input::widthForWork(current,in,out.request_j);
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

namespace return_crossing_work {
struct Audit {
  uint32_t zero_ms=0, peak_ms=0;
  float speed_dps=0, request_j=0;
  uint16_t base_ms=0, final_ms=0;
  float transfer_ms=NAN, terminal_deg=NAN, arc_request_j=NAN;
  uint16_t passage_main_ms=0, passage_reserve_ms=0;
  float passage_main_j=NAN, passage_zero_dps=NAN;
};

// Called only after current-foot geometry already qualified an actual arc
// extension. The source speed was measured before its ordinary ZERO pulse.
inline Audit apply(const Reference& source,uint32_t departure,uint32_t now,float decel,
    uint16_t envelope,uint16_t reserve,float rpm,const wheel_cycle_guard::Controller& wheel,
    wheel_cycle_guard::Controller& chosen_guard,
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    walking_return_energy::Input& in,walking_return_energy::Decision& selected,uint16_t& width,
    uint16_t history_limit=UINT16_MAX) {
  Audit audit;
  if (!usableReference(source,departure,now,selected.target_j) || in.side!=-1 ||
      !(in.body_angle_deg>0 && in.body_angle_deg<=12 && in.rate_dps>=3 && in.rate_dps<=35 &&
        in.previous_peak_deg>in.target_deg && in.same_side_peak_deg>0 && in.same_side_peak_deg<in.target_deg &&
        std::isfinite(decel) && decel>0 && std::isfinite(rpm)) ||
      !selected.valid || !width || width>=selected.width_ms || reserve>=envelope) return audit;
  audit.zero_ms=source.zero_ms;audit.peak_ms=source.peak_ms;audit.speed_dps=source.speed_dps;
  audit.base_ms=audit.final_ms=width;
  // Independent centre-speed boundary. Take the larger requirement for
  // reaching ZERO or the arc endpoint, since both cover the same approach.
  const float centre=energyShortfall(in.body_angle_deg,0,in.rate_dps,source.speed_dps,decel);
  audit.request_j=std::fmin(.5f*selected.target_j,std::fmax(0.f,centre));
  if (audit.request_j<=selected.request_j) return audit;
  auto proposed=in;
  proposed.return_work_j=audit.request_j;
  const auto inverse=half_cycle_energy_budget::decideReturnAid(geometry,current,proposed);
  if (!inverse.valid || inverse.coast || inverse.width_ms<=selected.width_ms) return audit;
  auto fresh_guard=wheel;
  auto allowed=fresh_guard.limit(-1,rpm,true,now,inverse.width_ms);
  if (allowed>envelope-reserve) allowed=envelope-reserve;
  // One observed speed is a reference, not an identified optimum. Admit at
  // most the unguarded width already requested for today's certified arc.
  if (allowed>selected.width_ms) allowed=selected.width_ms;
  if (allowed>history_limit) allowed=history_limit;
  if (allowed<=width) return audit;
  const auto result=half_cycle_energy_budget::evaluateReturnAidWidth(geometry,current,proposed,allowed);
  if (!result.valid || !result.charge_supported || !(result.predicted_work_j>0 &&
      result.predicted_work_j<=.5f*result.target_j)) return audit;
  selected=inverse;in=proposed;chosen_guard=fresh_guard;width=allowed;audit.final_ms=width;
  return audit;
}

#if SUPPORT_TRANSFER_STALL_V3
struct MeasuredArcPriority {
  uint16_t base_ms=0, cap_ms=0;
  float base_work_j=NAN, zero_work_j=NAN;
};

// A post-ZERO reservation is only usable if the body can first reach ZERO.
// The caller has already observed the receiving sole and a stopping shortfall
// on two consecutive frames. Compare the reserved plan with the lower of
// those two decelerations before giving the first event priority.
inline MeasuredArcPriority prioritizeMeasuredArc(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const walking_return_energy::Input& in,
    float previous_decel,float current_decel,uint16_t main_reserve,float rpm,uint32_t now,
    const wheel_cycle_guard::Controller& wheel) {
  MeasuredArcPriority audit;
  if (!(in.side==-1 && in.cap_ms>0 && in.cap_ms<=half_cycle_energy_budget::kEnvelopeMs &&
        main_reserve>0 && current.ready && in.body_angle_deg>0 && in.body_angle_deg<=12 &&
        in.rate_dps>=3 && in.rate_dps<=35 && previous_decel>0 && current_decel>0 &&
        std::isfinite(previous_decel) && std::isfinite(current_decel) && std::isfinite(rpm))) return audit;
  audit.zero_work_j=std::fmax(0.f,energyShortfall(in.body_angle_deg,0,in.rate_dps,0,
                                                 std::fmin(previous_decel,current_decel)));
  const auto demand=half_cycle_energy_budget::decideReturnAid(geometry,current,in);
  if (!demand.valid || !demand.width_ms) return audit;
  const uint16_t available=in.cap_ms>main_reserve ? in.cap_ms-main_reserve : 0;
  auto guard=wheel;
  audit.base_ms=guard.limit(-1,rpm,true,now,available<demand.width_ms ? available : demand.width_ms);
  const auto coeff=lateral_energy::coefficients(-1);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  const float base_q=lateral_energy::charge(current,audit.base_ms,in.initial_current_ma,in.goal_current_ma);
  audit.base_work_j=lateral_energy::work(coeff,omega,base_q);
  if (!(demand.width_ms>available && audit.zero_work_j>0 && base_q>=0 &&
        audit.base_work_j+walking_energy_policy_v15::kEnergyRoundoffJ<audit.zero_work_j)) return audit;
  const float stop_s=in.rate_dps/std::fmax(previous_decel,current_decel);
  if (!(current.tau>0 && stop_s>current.tau+.005f)) return audit;
  const auto cap=static_cast<uint16_t>(std::fmin(demand.width_ms,1000.f*stop_s-5.f));
  guard=wheel;
  const uint16_t width=guard.limit(-1,rpm,true,now,cap);
  const float q=lateral_energy::charge(current,width,in.initial_current_ma,in.goal_current_ma);
  const float work=lateral_energy::work(coeff,omega,q);
  if (width>audit.base_ms && q>0 && q<=coeff.charge_max_mAs && work>audit.base_work_j &&
      work<=.5f*demand.target_j) audit.cap_ms=cap;
  return audit;
}

// At the estimated crossing, reserve enough time to deliver the second
// stage's work with the current battery model. Do not credit the aid's
// unmeasured current tail. The real ZERO still re-solves from its fresh state.
inline uint16_t mainDuration(const lateral_energy::CurrentTable& current,
    const walking_return_energy::Input& in,float speed,float work,uint16_t cap) {
  const auto c=lateral_energy::coefficients(-1);
  const float q=lateral_energy::requiredCharge(c,speed*lateral_energy::kRadPerDeg,work);
  const float initial=std::fmin(0.f,in.initial_current_ma);
  if (!cap || !(q>0 && q<=c.charge_max_mAs) ||
      lateral_energy::charge(current,cap,initial,in.goal_current_ma)<q) return 0;
  uint16_t lo=0, hi=cap;
  while (hi-lo>1) {
    const uint16_t mid=(lo+hi)/2;
    if (lateral_energy::charge(current,mid,initial,in.goal_current_ma)<q) lo=mid;
    else hi=mid;
  }
  return lateral_energy::charge(current,hi,initial,in.goal_current_ma)<=c.charge_max_mAs ? hi : 0;
}

#if WALKING_RETURN_HISTORY
// Replan an already wheel-limited continuation. A shorter pulse must first
// pay for reaching ZERO at a speed that previously led into the receiving
// arc. Only the gain over the present zero-command current tail is available.
inline Audit planPostAidPassage(const MainPassages& passages,uint32_t departure,uint32_t now,
    uint16_t envelope,const lateral_energy::CurrentTable& current,const walking_return_energy::Input& in,
    float target_j,float gross_limit_j,float conservative_decel,uint16_t guarded_width) {
  Audit best;
  if (!(departure && now>departure && now-departure<=1500 && in.side==-1 &&
        in.body_angle_deg>0 && in.body_angle_deg<=12 && in.rate_dps>=support_transfer_plan::kCarryDps &&
        in.rate_dps<=35 && conservative_decel>0 && std::isfinite(conservative_decel) &&
        target_j>0 && std::isfinite(target_j) && gross_limit_j>0 && std::isfinite(gross_limit_j) &&
        current.ready && in.initial_current_ma>=0 && in.initial_current_ma<in.goal_current_ma &&
        std::isfinite(in.goal_current_ma) && guarded_width>1 && guarded_width<=in.cap_ms &&
        guarded_width<=envelope && envelope<=half_cycle_energy_budget::kEnvelopeMs)) return best;

  const auto coeff=lateral_energy::coefficients(-1);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  const auto net_work=[&](uint16_t ms) {
    const float q=lateral_energy::charge(current,ms,in.initial_current_ma,in.goal_current_ma);
    return lateral_energy::work(coeff,omega,q)-
        half_cycle_energy_budget::postAidCoastWork(in.rate_dps,in.initial_current_ma,ms);
  };
  const float q=lateral_energy::charge(current,guarded_width,in.initial_current_ma,in.goal_current_ma);
  if (!(q>0 && q<=coeff.charge_max_mAs && lateral_energy::work(coeff,omega,q)<=gross_limit_j)) return best;
  const float maximum_gain=net_work(guarded_width);
  uint16_t best_total=envelope+1;
  for (const auto& pass:passages.recent) {
    if (!(pass.zero_ms && pass.peak_ms>pass.zero_ms && pass.peak_ms-pass.zero_ms<=1500 &&
          pass.peak_ms<departure && now-pass.peak_ms<=4000 && pass.target_j>0 && pass.target_j<=target_j+1.e-9f &&
          pass.speed_dps>=support_transfer_plan::kCarryDps &&
          pass.speed_dps<=walking_energy_policy::passiveCoefficients(-1).rate_max_dps &&
          pass.main_ms>0 && pass.main_ms<=half_cycle_energy_budget::kEnvelopeMs &&
          pass.main_j>0 && std::isfinite(pass.main_j))) continue;
    const float needed=energyShortfall(in.body_angle_deg,0,in.rate_dps,pass.speed_dps,conservative_decel);
    if (!(needed>0 && maximum_gain>=needed)) continue;

    // Model the later main at the reference speed, starting with no current
    // credit from this pulse. The real main is solved again at measured ZERO.
    const float main_work=pass.main_j+std::fmax(0.f,target_j-pass.target_j);
    const uint16_t reserve=mainDuration(current,in,pass.speed_dps,main_work,envelope);
    if (!reserve) continue;
    if (guarded_width+reserve<=envelope) return {};  // Both events already fit.
    const uint16_t available=envelope-reserve;
    if (!available || net_work(available)<needed) continue;
    // A nonnegative initial current below the goal makes incremental work
    // relative to the decaying coast tail monotone over this interval.
    uint16_t lo=0,hi=available;
    while (hi-lo>1) {
      const uint16_t mid=(lo+hi)/2;
      if (net_work(mid)<needed) lo=mid; else hi=mid;
    }
    const uint16_t total=hi+reserve;
    if (total>best_total || (total==best_total && hi>=best.final_ms)) continue;
    best_total=total;
    best.zero_ms=pass.zero_ms;best.peak_ms=pass.peak_ms;best.speed_dps=pass.speed_dps;
    best.request_j=needed;best.base_ms=guarded_width;best.final_ms=hi;
    best.passage_main_ms=pass.main_ms;best.passage_reserve_ms=reserve;
    best.passage_main_j=main_work;best.passage_zero_dps=pass.speed_dps;
  }
  return best;
}
#endif

// Fallback for an accepted left aid while the receiving sole is still
// higher. A fixed-foot path must change supporting side before body zero,
// in both the measured and delay-compensated poses. Its exact impact time
// is unknown. Use the necessary centre passage as the intermediate event.
inline Audit applyPassage(const receiving_arc_aid::Input& arc,const MainPassages& passages,
    uint32_t departure,uint32_t now,uint16_t envelope,float rpm,
    const wheel_cycle_guard::Controller& wheel,wheel_cycle_guard::Controller& chosen_guard,
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    walking_return_energy::Input& in,walking_return_energy::Decision& selected,uint16_t& width) {
  Audit best;
  if (!(arc.observed_shape && arc.side==-1 && in.side==-1 && now>departure && now-departure<=1500 &&
        in.previous_peak_deg>in.target_deg && in.same_side_peak_deg>0 && in.same_side_peak_deg<in.target_deg &&
        arc.body_deg>0 && arc.body_deg<=12 && arc.speed_dps>=support_transfer_plan::kCarryDps && arc.speed_dps<=35 &&
        std::isfinite(arc.decel_dps2) && arc.decel_dps2>0 && std::fabs(arc.fore_aft_deg)<=10 &&
        arc.left_deg-arc.right_deg>=3 && std::fabs(arc.left_deg)<=20 && std::fabs(arc.right_deg)<=20 &&
        arc.left_est>arc.right_est && std::fabs(arc.left_est)<=20 && std::fabs(arc.right_est)<=20 &&
        std::fabs(arc.left_rate)<=80 && std::fabs(arc.right_rate)<=80 && selected.valid && !selected.coast &&
        current.ready && std::isfinite(rpm) && width>0 && width<envelope && envelope<=in.cap_ms)) return best;
  const auto changes_side=[&](float left,float right) {
    return support_transfer_plan::supportGap(left,right,arc.fore_aft_deg,arc.body_deg)>0 &&
           support_transfer_plan::supportGap(left,right,arc.fore_aft_deg,0)<0;
  };
  if (!changes_side(arc.left_deg,arc.right_deg) || !changes_side(arc.left_est,arc.right_est)) return best;

  const float to_zero=energyShortfall(arc.body_deg,0,arc.speed_dps,0,arc.decel_dps2);
  const float rad=lateral_energy::kRadPerDeg;
  const float kinetic=walking_energy_policy::passiveCoefficients(-1).omega2_j*rad*rad;
  const float cap=.5f*selected.target_j;
  if (!(to_zero>0 && to_zero<cap)) return best;
  uint16_t best_total=envelope+1;
  auto best_guard=chosen_guard;
  auto best_inverse=selected;
  for (const auto& pass : passages.recent) {
    if (!pass.zero_ms || pass.peak_ms>=departure || now-pass.peak_ms>4000 ||
        pass.target_j>selected.target_j+1.e-9f) continue;
    const float reference_kinetic=kinetic*pass.speed_dps*pass.speed_dps;
    auto proposed=in;
    proposed.return_work_j=to_zero+reference_kinetic;
    if (!(proposed.return_work_j>selected.request_j && proposed.return_work_j<=cap)) continue;
    const auto inverse=half_cycle_energy_budget::decideReturnAid(geometry,current,proposed);
    if (!inverse.valid || inverse.coast) continue;
    auto trial_guard=wheel;
    const uint16_t aid=trial_guard.limit(-1,rpm,true,now,inverse.width_ms);
    if (aid<=width || aid>=envelope) continue;
    const auto delivered=half_cycle_energy_budget::evaluateReturnAidWidth(geometry,current,proposed,aid);
    if (!delivered.valid || !delivered.charge_supported || !(delivered.predicted_work_j<=cap)) continue;
    const float crossing_kinetic=delivered.predicted_work_j-to_zero;
    if (!(crossing_kinetic>=kinetic*support_transfer_plan::kCarryDps*support_transfer_plan::kCarryDps)) continue;
    const float crossing_speed=std::sqrt(crossing_kinetic/kinetic);
    // The reference main started at its own observed speed and target. Add
    // only changes to that starting boundary, not the work spent before it.
    const float main_work=pass.main_j+std::fmax(0.f,selected.target_j-pass.target_j)+
                          std::fmax(0.f,reference_kinetic-crossing_kinetic);
    const uint16_t reserve=mainDuration(current,in,crossing_speed,main_work,envelope-aid);
    if (!reserve) continue;
    const uint16_t total=aid+reserve;
    if (total>best_total || (total==best_total && aid>=best.final_ms)) continue;
    best_total=total;
    best.base_ms=width; best.final_ms=aid; best.zero_ms=pass.zero_ms; best.peak_ms=pass.peak_ms;
    best.speed_dps=pass.speed_dps; best.request_j=proposed.return_work_j;
    best.passage_main_ms=pass.main_ms; best.passage_reserve_ms=reserve;
    best.passage_main_j=main_work; best.passage_zero_dps=crossing_speed;
    best_inverse=inverse; best_guard=trial_guard;
  }
  if (best.final_ms) {
    in.return_work_j=best.request_j; selected=best_inverse; chosen_guard=best_guard; width=best.final_ms;
  }
  return best;
}

// Forecast using the deceleration that qualified the existing aid. Require a
// closing gap over both halves of the interval, ending on the receiving sole.
// Interpolate the time within the bracketing half; this is a predicted CAD
// height equality, not an observed impact or load measurement.
inline float closingTime(const receiving_arc_aid::Input& arc,float left,float right,float horizon,float fore_rate) {
  const auto gap=[&](float t) {
    const float l=left+arc.left_rate*t,r=right+arc.right_rate*t;
    const float fore=arc.fore_aft_deg+fore_rate*t;
    if (!(std::fabs(l)<=20 && std::fabs(r)<=20 && std::fabs(fore)<=10 && l>r)) return float(NAN);
    const float body=arc.body_deg-t*(arc.speed_dps-.5f*arc.decel_dps2*t);
    return support_transfer_plan::supportGap(l,r,fore,body);
  };
  const float first=gap(0),half=.5f*horizon,middle=gap(half),end=gap(horizon);
  if (!(first>middle && middle>end && end<0)) return NAN;
  if (first<=0) return 0;
  if (middle<=0) return half*first/(first-middle);
  return half+half*middle/(middle-end);
}

// Only an already selected, fresh left aid can use this forecast. Changes
// to the waveform begin at the old cutoff, so search for an arriving pivot
// during that original width plus one current-rise time constant. Never
// extend the assumed direction beyond the estimated passive stopping time.
inline Audit applyApproach(const receiving_arc_aid::Input& arc,const Reference& source,
    uint32_t departure,uint32_t now,uint16_t envelope,uint16_t reserve,float rpm,
    const wheel_cycle_guard::Controller& wheel,wheel_cycle_guard::Controller& chosen_guard,
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    walking_return_energy::Input& in,walking_return_energy::Decision& selected,uint16_t& width,
    float fore_rate_dps) {
  Audit audit;
  if (!(arc.observed_shape && arc.side==-1 && in.side==-1 && now>departure && now-departure<=1500 &&
        in.previous_peak_deg>in.target_deg && in.same_side_peak_deg>0 && in.same_side_peak_deg<in.target_deg &&
        arc.body_deg>0 && arc.body_deg<=12 && arc.speed_dps>=support_transfer_plan::kCarryDps &&
        arc.speed_dps<=35 && std::isfinite(arc.decel_dps2) && arc.decel_dps2>0 &&
        std::fabs(arc.fore_aft_deg)<=10 && std::isfinite(fore_rate_dps) &&
        std::fabs(arc.left_rate)<=80 && std::fabs(arc.right_rate)<=80 &&
        arc.left_deg-arc.right_deg>=3 && selected.valid && !selected.coast && width>0 &&
        current.ready && current.tau>0 && std::isfinite(current.tau) && std::isfinite(rpm) &&
        reserve<envelope && width<envelope-reserve)) return audit;
  if (!(support_transfer_plan::supportGap(arc.left_deg,arc.right_deg,arc.fore_aft_deg,arc.body_deg)>=0)) return audit;
  const float stop_s=arc.speed_dps/arc.decel_dps2;
  const float horizon=std::fmin(.001f*support_transfer_plan::kForecastLimitMs,
      std::fmin(stop_s,.001f*width+current.tau));
  const float measured=closingTime(arc,arc.left_deg,arc.right_deg,horizon,0);
  const float estimated=closingTime(arc,arc.left_est,arc.right_est,horizon,0);
  if (!(std::isfinite(measured) && std::isfinite(estimated))) return audit;
  // A changing fore-aft pitch can lift the receiving foot even while the
  // roll closes the fixed-pitch gap. Retain the later plausible boundary.
  const float pitched=closingTime(arc,arc.left_est,arc.right_est,horizon,fore_rate_dps);
  if (!std::isfinite(pitched)) return audit;
  const float transfer=std::fmax(pitched,std::fmax(measured,estimated));
  if (!(transfer<stop_s)) return audit;
  audit.transfer_ms=1000.f*transfer;
  // Of the frozen and moving-pitch forecasts, the receiving sole closest
  // to horizontal needs the greatest leftward travel to reach its arc.
  const float fixed_pitch=arc.left_est+transfer*arc.left_rate+arc.fore_aft_deg;
  const float moving_pitch=fixed_pitch+transfer*fore_rate_dps;
  const float closest_pitch=fixed_pitch*moving_pitch<=0 ? 0.f :
      std::fmin(std::fabs(fixed_pitch),std::fabs(moving_pitch));
  audit.terminal_deg=support_transfer_plan::arcEntryDeg(-1,closest_pitch);
  audit.base_ms=audit.final_ms=width;
  const float cap=.5f*selected.target_j;
  audit.arc_request_j=std::fmin(cap,std::fmax(0.f,energyShortfall(arc.body_deg,audit.terminal_deg,
      arc.speed_dps,support_transfer_plan::kCarryDps,arc.decel_dps2)));
  audit.request_j=audit.arc_request_j;
  if (usableReference(source,departure,now,selected.target_j)) {
    audit.zero_ms=source.zero_ms;audit.peak_ms=source.peak_ms;audit.speed_dps=source.speed_dps;
    const float centre=energyShortfall(arc.body_deg,0,arc.speed_dps,source.speed_dps,arc.decel_dps2);
    audit.request_j=std::fmin(cap,std::fmax(audit.arc_request_j,centre));
  }
  // The crossing and arc endpoint share the same approach. Use the larger
  // requirement once, then apply the actual wheel and main-reserve limits.
  if (!(audit.request_j>selected.request_j)) return audit;
  auto proposed=in;
  proposed.return_work_j=audit.request_j;
  const auto inverse=half_cycle_energy_budget::decideReturnAid(geometry,current,proposed);
  if (!inverse.valid || inverse.coast || inverse.width_ms<=width) return audit;
  auto candidate_guard=wheel;
  uint16_t allowed=candidate_guard.limit(-1,rpm,true,now,inverse.width_ms);
  if (allowed>envelope-reserve) allowed=envelope-reserve;
  if (allowed<=width) return audit;
  const auto result=half_cycle_energy_budget::evaluateReturnAidWidth(geometry,current,proposed,allowed);
  if (!result.valid || !result.charge_supported || !(result.predicted_work_j>0 && result.predicted_work_j<=cap))
    return audit;
  selected=inverse;in=proposed;chosen_guard=candidate_guard;width=allowed;audit.final_ms=width;
  return audit;
}
#endif
}

namespace settled_walking_restart {
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
