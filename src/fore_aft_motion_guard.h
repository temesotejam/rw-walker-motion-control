#pragma once
#include <cmath>
#include <cstdint>
#include "mekf6.hpp"

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
