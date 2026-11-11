#pragma once
#include <cmath>
#include <cstdint>

// Camera-relative angles only. This predictor never authorizes motor output.
namespace foot_pose_prediction {
enum class Reason : uint8_t { Ready, Warmup, Invalid, Timestamp, Gap, Jump, Stale, Horizon };
struct Result {
  Reason reason=Reason::Warmup;
  float left=NAN,right=NAN,left_rate=NAN,right_rate=NAN;
  uint32_t age_ms=0,horizon_ms=0;
  bool ready()const{return reason==Reason::Ready;}
};
class Predictor {
 public:
  void reset(){*this=Predictor{};}
  Result observe(uint16_t run,uint32_t frame,uint32_t now,bool valid,float l,float r,uint32_t lead=30) {
    Result out;
    if(run!=run_){reset();run_=run;}
    if(!run || !valid || !std::isfinite(l) || !std::isfinite(r)) {have_=false;out.reason=Reason::Invalid;return out;}
    if(frame>now || (have_ && frame<=frame_)){have_=false;out.reason=Reason::Timestamp;return out;}
    out.age_ms=now-frame;out.horizon_ms=out.age_ms+lead;
    if(out.age_ms>180){have_=false;out.reason=Reason::Stale;return out;}
    if(lead>50 || out.horizon_ms>200){have_=false;out.reason=Reason::Horizon;return out;}
    const bool prior=have_;const uint32_t dt=frame-frame_;
    const float dl=l-left_,dr=r-right_;
    frame_=frame;left_=l;right_=r;have_=true;
    if(!prior)return out;
    if(dt<40 || dt>200){out.reason=Reason::Gap;return out;}
    out.left_rate=dl*1000.f/dt;out.right_rate=dr*1000.f/dt;
    if(std::fabs(dl)>8 || std::fabs(dr)>8 || std::fabs(out.left_rate)>80 || std::fabs(out.right_rate)>80) {
      have_=false;out.reason=Reason::Jump;return out;
    }
    out.left=l+out.left_rate*out.horizon_ms*.001f;
    out.right=r+out.right_rate*out.horizon_ms*.001f;
    out.reason=Reason::Ready;return out;
  }
 private:
  uint16_t run_=0;uint32_t frame_=0;float left_=0,right_=0;bool have_=false;
};
}
