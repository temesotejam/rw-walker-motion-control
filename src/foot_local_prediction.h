#pragma once
#include <cmath>
#include <cstdint>

// Local evidence complements the repeated-orbit predictor; it never changes
// that predictor's confidence. Camera-relative angles are delayed observations,
// not contact/load measurements. Margins below are uncalibrated heuristics,
// NOT physical error bounds or a guarantee that a torque pulse is appropriate.
namespace foot_local_prediction {
enum class Reason : uint8_t { Ready, Warmup, Invalid, Timestamp, Gap, Jump, Stale, Horizon };
enum class Phase : uint8_t { Unknown, Stationary, Forward, Turning, Returning };
enum class Method : uint8_t { Hold=0, Local=2 };
struct Result {
  Reason reason=Reason::Warmup;
  Method method=Method::Hold;
  float left=NAN,right=NAN,left_rate=NAN,right_rate=NAN;
  float left_uncertainty=INFINITY,right_uncertainty=INFINITY;
  // Fraction of feet with qualifying three-frame local evidence, reduced
  // when reused after 100 ms. This is NOT the history match score/probability.
  float confidence=0;
  Phase phase_left=Phase::Unknown,phase_right=Phase::Unknown;
  uint32_t age_ms=0,horizon_ms=0;
  bool ready()const{return reason==Reason::Ready;}
};

struct LocalSide {
  float observed=NAN,trend=0,latest_rate=0,rate_difference=0,span=0,net_change=0;
  Phase phase=Phase::Unknown;
  bool qualified=false;
};

// Prepared by Core0. A query on Core1 uses two small local models; no search,
// allocation, regression or camera work occurs at control time.
struct Projection {
  uint16_t run_id=0;
  uint32_t frame_ms=0,received_ms=0;
  Reason reason=Reason::Warmup;
  LocalSide left_model,right_model;
  static void estimate(const LocalSide& model,float seconds,float& angle,float& rate,float& margin){
    angle=model.observed;rate=model.trend;margin=1.25f+.01f*seconds*1000.f;
    if(model.phase==Phase::Stationary){
      margin=.35f+.5f*model.span+seconds;
      rate=0;
    }else if(model.phase==Phase::Forward){
      // Even two increasing intervals can end at an unseen maximum. Do not
      // extrapolate beyond it merely because an old camera slope was positive.
      margin=.35f+.5f*std::fabs(model.latest_rate)*seconds+.25f*model.rate_difference*seconds;
    }else if(model.phase==Phase::Returning){
      // Use only a small part of the slower observed inward foot trend.
      // Full age-times-velocity extrapolation overshot the v10 reversals.
      const float duration=seconds<.025f?seconds:.025f;
      float correction=std::fabs(model.trend)*duration;
      const float local_cap=.25f*std::fabs(model.net_change);
      if(correction>local_cap)correction=local_cap;
      if(correction>.5f)correction=.5f;
      angle-=correction;
      margin=.35f+.15f*std::fabs(model.latest_rate)*seconds+.25f*model.rate_difference*seconds;
    }
  }
  Result predict(uint32_t now_ms,uint32_t lead_ms=0)const{
    Result out;out.reason=reason;
    if(reason!=Reason::Ready)return out;
    if(now_ms<received_ms || now_ms<frame_ms){out.reason=Reason::Timestamp;return out;}
    out.age_ms=now_ms-frame_ms;out.horizon_ms=out.age_ms;
    if(out.age_ms>160u){out.reason=Reason::Stale;return out;}
    // This local model is for delayed-observation -> current-state support.
    // A motor response lead is not identified and is not silently assumed.
    if(lead_ms!=0u){out.reason=Reason::Horizon;return out;}
    const float seconds=out.age_ms*.001f;
    estimate(left_model,seconds,out.left,out.left_rate,out.left_uncertainty);
    estimate(right_model,seconds,out.right,out.right_rate,out.right_uncertainty);
    out.phase_left=left_model.phase;out.phase_right=right_model.phase;
    out.method=Method::Local;
    const float age_factor=out.age_ms<=100u?1.f:(160.f-out.age_ms)/60.f;
    out.confidence=.5f*(static_cast<unsigned>(left_model.qualified)+static_cast<unsigned>(right_model.qualified))*age_factor;
    return out;
  }
};

class Predictor {
 public:
  void reset(){*this=Predictor{};}
  Reason observe(uint16_t run,uint32_t frame,uint32_t ready,bool valid,float left,float right){
    if(run!=run_){reset();run_=run;}
    if(!run){reset();return reason_=Reason::Invalid;}
    if(frame>ready || (seen_ && frame<=last_seen_)){
      count_=0;return reason_=Reason::Timestamp;
    }
    seen_=true;last_seen_=frame;ready_=ready;
    if(!valid || !std::isfinite(left) || !std::isfinite(right)){
      count_=0;return reason_=Reason::Invalid;
    }
    if(ready-frame>160u){count_=0;return reason_=Reason::Stale;}
    if(count_){
      const Point& prior=points_[count_-1];const uint32_t dt=frame-prior.ms;
      if(dt<40u || dt>200u){
        points_[0]=Point(frame,left,right);count_=1;return reason_=Reason::Gap;
      }
      if(std::fabs(left-prior.left)>8.f || std::fabs(right-prior.right)>8.f ||
         std::fabs(left-prior.left)*1000.f/dt>80.f || std::fabs(right-prior.right)*1000.f/dt>80.f){
        count_=0;return reason_=Reason::Jump;
      }
    }
    if(count_==4u){for(unsigned i=1;i<4;++i)points_[i-1]=points_[i];--count_;}
    points_[count_++]=Point(frame,left,right);
    return reason_=count_>=3u?Reason::Ready:Reason::Warmup;
  }
  Projection projection()const{
    Projection out;out.run_id=run_;out.received_ms=ready_;out.reason=reason_;
    if(!count_)return out;
    out.frame_ms=points_[count_-1].ms;
    if(reason_!=Reason::Ready)return out;
    out.left_model=side(true);out.right_model=side(false);return out;
  }
  Result predict(uint32_t now_ms,uint32_t lead_ms=0)const{return projection().predict(now_ms,lead_ms);}

 private:
  struct Point {
    uint32_t ms=0;float left=0,right=0;
    Point()=default;Point(uint32_t t,float l,float r):ms(t),left(l),right(r){}
  };
  Point points_[4]{};
  unsigned count_=0;uint16_t run_=0;uint32_t last_seen_=0,ready_=0;
  bool seen_=false;Reason reason_=Reason::Warmup;
  LocalSide side(bool left)const{
    const Point& a=points_[count_-3];const Point& b=points_[count_-2];const Point& c=points_[count_-1];
    const float x=left?a.left:a.right,y=left?b.left:b.right,z=left?c.left:c.right;
    const float first=y-x,second=z-y;
    const float rate_a=first*1000.f/(b.ms-a.ms),rate_b=second*1000.f/(c.ms-b.ms);
    LocalSide out;out.observed=z;out.latest_rate=rate_b;
    out.rate_difference=std::fabs(rate_b-rate_a);out.net_change=z-x;
    const float low=std::fmin(x,std::fmin(y,z)),high=std::fmax(x,std::fmax(y,z));out.span=high-low;
    if(out.span<=.3f){out.phase=Phase::Stationary;out.qualified=true;return out;}
    // Two same-sign intervals are required; one reversal never immediately
    // receives a directional straight-line forecast. Tiny differences alone
    // do not establish a useful local direction.
    if(first*second>0.f && std::fabs(first)>=.12f && std::fabs(second)>=.12f && std::fabs(out.net_change)>=.4f){
      const float slower=std::fmin(std::fabs(rate_a),std::fabs(rate_b));
      out.phase=second>0.f?Phase::Forward:Phase::Returning;
      out.trend=second>0.f?slower:-slower;out.qualified=true;return out;
    }
    if(first*second<0.f && (std::fabs(first)>=.12f || std::fabs(second)>=.12f))out.phase=Phase::Turning;
    return out;
  }
};
}
