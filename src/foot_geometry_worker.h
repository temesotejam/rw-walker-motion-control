#pragma once
#include <cmath>
#include <cstdint>
#ifndef FOOT_GEOMETRY_PROVISIONAL_EXPOSURE
#define FOOT_GEOMETRY_PROVISIONAL_EXPOSURE 0
#endif

// Worker-owned, allocation-free model. All timestamps share one monotonic
// run-relative millisecond clock. The caller owns cross-task synchronization.
namespace foot_geometry_worker {
struct Pose { uint32_t ms=0; float fore_aft_deg=NAN; };

// Call on an immutable snapshot of the IMU ring, not concurrently with a writer.
// Reject an unbracketed time or a gap too large to justify interpolation.
inline bool interpolate(const Pose* p,unsigned n,uint32_t ms,uint32_t max_gap_ms,float& out) {
  if(!p || n<2 || !max_gap_ms) return false;
  for(unsigned i=1;i<n;++i) {
    const Pose &a=p[i-1], &b=p[i];
    if(b.ms<=a.ms) return false;
    if(a.ms<=ms && ms<=b.ms) {
      if(b.ms-a.ms>max_gap_ms || !std::isfinite(a.fore_aft_deg) || !std::isfinite(b.fore_aft_deg)) return false;
      const float q=float(ms-a.ms)/float(b.ms-a.ms);
      out=a.fore_aft_deg+q*(b.fore_aft_deg-a.fore_aft_deg);
      return std::isfinite(out);
    }
  }
  return false;
}
// Single-owner worker-side history. Control supplies sampled poses through a
// nonblocking SPSC queue. Failed queue sends create gaps; never fill or guess
// them. Queue clearing and history.reset() are required on every run change.
template<unsigned Capacity> class PoseHistory {
 public:
  static_assert(Capacity>=2,"pose history needs at least two samples");
  void reset(){next_=count_=0;}
  bool push(const Pose& p) {
    if(!std::isfinite(p.fore_aft_deg))return false;
    if(count_ && p.ms<=samples_[(next_+Capacity-1)%Capacity].ms) return false;
    samples_[next_]=p;next_=(next_+1)%Capacity;if(count_<Capacity)++count_;return true;
  }
  bool at(uint32_t ms,uint32_t max_gap_ms,float& out) const {
    if(count_<2 || !max_gap_ms)return false;
    const unsigned start=(next_+Capacity-count_)%Capacity;
    for(unsigned i=1;i<count_;++i) {
      const Pose& a=samples_[(start+i-1)%Capacity];
      const Pose& b=samples_[(start+i)%Capacity];
      if(a.ms<=ms && ms<=b.ms) {
        if(b.ms-a.ms>max_gap_ms)return false;
        const float f=float(ms-a.ms)/float(b.ms-a.ms);
        out=a.fore_aft_deg+f*(b.fore_aft_deg-a.fore_aft_deg);return std::isfinite(out);
      }
    }
    return false;
  }
  // Range includes every stored interior sample, not just the endpoints.
  // The bound assumes linear interpolation between consecutive samples.
  bool range(uint32_t from,uint32_t to,uint32_t max_gap_ms,float& lo,float& hi) const {
    if(from>to)return false;
    float a,b;if(!at(from,max_gap_ms,a)||!at(to,max_gap_ms,b))return false;
    lo=std::fmin(a,b);hi=std::fmax(a,b);
    const unsigned start=(next_+Capacity-count_)%Capacity;
    for(unsigned i=0;i<count_;++i) {
      const Pose& p=samples_[(start+i)%Capacity];
      if(p.ms>=from && p.ms<=to){lo=std::fmin(lo,p.fore_aft_deg);hi=std::fmax(hi,p.fore_aft_deg);}
      if(i) {
        const Pose& previous=samples_[(start+i-1)%Capacity];
        if(previous.ms<to && p.ms>from && p.ms-previous.ms>max_gap_ms)return false;
      }
    }
    return true;
  }
 private: Pose samples_[Capacity]{};unsigned next_=0,count_=0;
};
struct Input {
  uint32_t run_id=0, peak_id=0, sequence=0, generation=0;
  uint32_t frame_ms=0, observed_ms=0, expiry_ms=0;
  // Absolute table-reference angles, not angles tared to the current boot.
  float left_deg=NAN, right_deg=NAN;
  // Fore-aft body angle minimum and maximum over the exposure interval,
  // including all interior samples. Include camera/IMU clock uncertainty.
  float pitch_lo_deg=NAN, pitch_hi_deg=NAN;
  float origin_uncertainty_deg=1.f;
  // The initial body pose is not known to be level. This trial bound is an
  // assumption, not a measured error or a guarantee of containing the truth.
  float pitch_origin_uncertainty_deg=2.f;
  float max_spread_deg=1.5f, zero_margin_deg=.25f;
  int8_t side=0;
  enum class Exposure : uint8_t { Unknown=0, Verified=1, Provisional=2 };
  Exposure exposure=Exposure::Unknown;
  uint32_t exposure_earliest_ms=0, exposure_latest_ms=0;
  bool pose_interval_bracketed=false;
};
struct Result {
  uint32_t run_id=0, peak_id=0, sequence=0, generation=0;
  uint32_t frame_ms=0, ready_ms=0, expiry_ms=0;
  uint32_t exposure_earliest_ms=0, exposure_latest_ms=0;
  Input::Exposure exposure=Input::Exposure::Unknown;
  float threshold_deg=NAN, root_min_deg=NAN, root_max_deg=NAN;
};
enum class Status : uint8_t { Idle, Working, Rejected, Ready };
enum class Reason : uint8_t { None, UnknownExposure, ProvisionalDisabled, BadExposureInterval,
  PoseUnavailable, BadIdentity, BadTime, BadInput, StaleGeneration, Expired, NoRoot, Disagree };
struct Diagnostic {
  Status status=Status::Idle;Reason reason=Reason::None;
  Input::Exposure exposure=Input::Exposure::Unknown;
  uint32_t run_id=0, peak_id=0, sequence=0, generation=0, frame_ms=0;
  uint32_t exposure_earliest_ms=0, exposure_latest_ms=0, expiry_ms=0;
  uint32_t observed_ms=0;
  float origin_uncertainty_deg=NAN,pitch_origin_uncertainty_deg=NAN,max_spread_deg=NAN,zero_margin_deg=NAN;
  unsigned evaluated=0;
};

inline float height(int side,float sway,float pitch,float leg,float H) {
  constexpr float rad=.01745329251994329577f,R=150.f;
  sway*=rad;pitch*=rad;leg*=rad;
  const float c=std::cos(pitch+leg), cs=std::cos(sway);
  if(c<=0 || cs<=0) return NAN;
  const float k=std::tan(sway)/c;
  float x=R*k/std::sqrt(1.f+k*k);
  const float lo=side<0?-45.f:5.f, hi=side<0?-5.f:45.f;
  x=std::fmax(lo,std::fmin(hi,x));
  const float z=std::sqrt(R*R-25.f)-std::sqrt(R*R-x*x);
  return -std::sin(sway)*x+cs*(H*std::cos(pitch)-28.f*std::fabs(std::sin(pitch+leg))+c*(z-H));
}
inline float gap(float sway,float pitch,float left,float right,float H) {
  return height(-1,sway,pitch,left,H)-height(1,sway,pitch,right,H);
}
inline bool root(float pitch,float left,float right,float H,float& value) {
  float lo=-15.f,hi=15.f,fl=gap(lo,pitch,left,right,H),fh=gap(hi,pitch,left,right,H);
  if(!std::isfinite(fl)||!std::isfinite(fh)||fl*fh>0.f) return false;
  for(unsigned i=0;i<20;++i) {
    float mid=(lo+hi)*.5f, fm=gap(mid,pitch,left,right,H);
    if(!std::isfinite(fm))return false;
    if(fl*fm<=0.f)hi=mid;else {lo=mid;fl=fm;}
  }
  value=(lo+hi)*.5f;return true;
}
class Worker {
 public:
  // Single-slot replacement; the worker owns this object. A separate task
  // mailbox may replace not-yet-consumed Inputs under a short lock.
  bool submit(const Input& in,uint32_t now) {
    cancel();
    in_=in;
    if(in.exposure==Input::Exposure::Unknown)return reject(Reason::UnknownExposure);
    if(in.exposure==Input::Exposure::Provisional && !FOOT_GEOMETRY_PROVISIONAL_EXPOSURE)
      return reject(Reason::ProvisionalDisabled);
    if(in.exposure!=Input::Exposure::Verified && in.exposure!=Input::Exposure::Provisional)
      return reject(Reason::UnknownExposure);
    if(in.exposure_earliest_ms>in.exposure_latest_ms ||
       in.exposure_earliest_ms>in.frame_ms || in.exposure_latest_ms<in.frame_ms ||
       in.exposure_latest_ms>in.observed_ms) return reject(Reason::BadExposureInterval);
    if(!in.pose_interval_bracketed)return reject(Reason::PoseUnavailable);
    if(!in.run_id || !in.peak_id || !in.generation || (in.side!=1 && in.side!=-1))
      return reject(Reason::BadIdentity);
    if(in.frame_ms>in.observed_ms || in.observed_ms>now || now>=in.expiry_ms)
      return reject(Reason::BadTime);
    if(
       !std::isfinite(in.left_deg)||!std::isfinite(in.right_deg)||
       !std::isfinite(in.pitch_lo_deg)||!std::isfinite(in.pitch_hi_deg)||
       in.pitch_lo_deg>in.pitch_hi_deg ||
       !std::isfinite(in.origin_uncertainty_deg)||in.origin_uncertainty_deg<0 ||
       !std::isfinite(in.pitch_origin_uncertainty_deg)||in.pitch_origin_uncertainty_deg<0 ||
       !std::isfinite(in.max_spread_deg)||in.max_spread_deg<0 ||
       !std::isfinite(in.zero_margin_deg)||in.zero_margin_deg<0) {
      return reject(Reason::BadInput);
    }
    state_=Status::Working;return true;
  }
  void cancel() {state_=Status::Idle;reason_=Reason::None;index_=0;lo_=INFINITY;hi_=-INFINITY;}
  // At most one hypothesis per call. A dedicated low-priority task must call
  // this again with a fresh clock and current run/peak generation each time.
  Status step(uint32_t now,uint32_t run,uint32_t peak,uint32_t generation) {
    if(state_!=Status::Working) return state_;
    if(run!=in_.run_id || peak!=in_.peak_id || generation!=in_.generation) {reject(Reason::StaleGeneration);return state_;}
    if(now>=in_.expiry_ms || now<in_.observed_ms) {reject(Reason::Expired);return state_;}
    // 3 hip heights x 2 pitch signs x 2 foot-axis signs x 2 origin errors
    // per foot x 2 exposure endpoints = 96. Correlation is not assumed.
    unsigned i=index_; const unsigned h=i%3;i/=3;const unsigned sign=i%2;i/=2;
    const unsigned l=i%2;i/=2;const unsigned r=i%2;i/=2;const unsigned t=i%2;i/=2;
    const unsigned foot_sign=i%2;
    const float heights[]={55.33f,82.83f,120.f};
    const float pitch=(t?in_.pitch_hi_deg+in_.pitch_origin_uncertainty_deg:
                         in_.pitch_lo_deg-in_.pitch_origin_uncertainty_deg)*(sign?1.f:-1.f);
    const float ls=in_.left_deg+(l?in_.origin_uncertainty_deg:-in_.origin_uncertainty_deg);
    const float rs=in_.right_deg+(r?in_.origin_uncertainty_deg:-in_.origin_uncertainty_deg);
    const float left=foot_sign?ls:-ls, right=foot_sign?rs:-rs;
    float v;
    if(!root(pitch,left,right,heights[h],v)) {reject(Reason::NoRoot);return state_;}
    const float oriented=in_.side*v;
    lo_=std::fmin(lo_,oriented);hi_=std::fmax(hi_,oriented);
    if(lo_<=in_.zero_margin_deg || hi_-lo_>in_.max_spread_deg) {reject(Reason::Disagree);return state_;}
    if(++index_==96) {
      result_.run_id=in_.run_id;result_.peak_id=in_.peak_id;result_.frame_ms=in_.frame_ms;
      result_.sequence=in_.sequence;result_.generation=in_.generation;
      result_.exposure=in_.exposure;result_.exposure_earliest_ms=in_.exposure_earliest_ms;
      result_.exposure_latest_ms=in_.exposure_latest_ms;
      result_.expiry_ms=in_.expiry_ms;result_.root_min_deg=lo_;result_.root_max_deg=hi_;
      result_.threshold_deg=in_.side*lo_;state_=Status::Ready;
    }
    return state_;
  }
  // Re-read the actual completion clock after the last step. Never publish a
  // result after expiry or across a run/peak transition.
  bool result(uint32_t now,uint32_t run,uint32_t peak,uint32_t generation,Result& out) {
    if(state_!=Status::Ready)return false;
    if(run!=in_.run_id || peak!=in_.peak_id || generation!=in_.generation)return reject(Reason::StaleGeneration);
    if(now>=in_.expiry_ms || now<in_.observed_ms)return reject(Reason::Expired);
    result_.ready_ms=now;out=result_;state_=Status::Idle;return true;
  }
  Status status()const{return state_;}
  Diagnostic diagnostic()const {
    Diagnostic d;d.status=state_;d.reason=reason_;d.exposure=in_.exposure;
    d.run_id=in_.run_id;d.peak_id=in_.peak_id;d.sequence=in_.sequence;d.generation=in_.generation;
    d.frame_ms=in_.frame_ms;d.exposure_earliest_ms=in_.exposure_earliest_ms;
    d.exposure_latest_ms=in_.exposure_latest_ms;d.expiry_ms=in_.expiry_ms;d.observed_ms=in_.observed_ms;
    d.origin_uncertainty_deg=in_.origin_uncertainty_deg;d.max_spread_deg=in_.max_spread_deg;
    d.pitch_origin_uncertainty_deg=in_.pitch_origin_uncertainty_deg;
    d.zero_margin_deg=in_.zero_margin_deg;d.evaluated=index_;return d;
  }
 private:
  bool reject(Reason reason){reason_=reason;state_=Status::Rejected;return false;}
  Input in_{};Result result_{};Status state_=Status::Idle;unsigned index_=0;
  Reason reason_=Reason::None;
  float lo_=INFINITY,hi_=-INFINITY;
};
}
