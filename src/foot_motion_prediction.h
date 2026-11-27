#pragma once
#include <stdint.h>
#include <cmath>
#include <cstdint>

namespace foot_timing_policy {
#if defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
// Sensor frames arrive near 20ms; allow timestamp quantization/jitter while
// predictors still reject duplicates, reverse time, stale data and jumps.
constexpr uint32_t kMinFrameIntervalMs=15;
#elif defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
constexpr uint32_t kMinFrameIntervalMs=25;
#else
constexpr uint32_t kMinFrameIntervalMs=40;
#endif
}

// Delayed camera-relative observations. Confidence is a history-match score,
// not a calibrated contact probability. No part of this class authorizes drive.
namespace foot_phase_prediction {
enum class Reason : uint8_t { Ready, Warmup, Invalid, Timestamp, Gap, Jump, Stale, Horizon };
enum class Phase : uint8_t { Unknown, Stationary, Forward, Turning, Returning };
enum class Method : uint8_t { Hold, History };
struct Result {
  Reason reason=Reason::Warmup;
  Method method=Method::Hold;
  float left=NAN,right=NAN,left_rate=NAN,right_rate=NAN,confidence=0;
  // Heuristic disagreement margin in degrees; NOT an error bound calibrated
  // against angle truth or a guarantee that the foot is clear of the floor.
  float left_uncertainty=INFINITY,right_uncertainty=INFINITY;
  Phase phase_left=Phase::Unknown,phase_right=Phase::Unknown;
  uint32_t age_ms=0,horizon_ms=0;
  bool ready()const{return reason==Reason::Ready;}
};

// Four forecasts are prepared on the camera task. A control-task query only
// copies/interpolates these fixed values; it never scans the history.
struct Projection {
  uint16_t run_id=0;
  uint32_t frame_ms=0,received_ms=0;
  Reason reason=Reason::Warmup;
  float left[4]={NAN,NAN,NAN,NAN},right[4]={NAN,NAN,NAN,NAN};
  float left_uncertainty[4]={1.25f,1.25f,1.25f,1.25f};
  float right_uncertainty[4]={1.25f,1.25f,1.25f,1.25f};
  float confidence=0;
  static uint32_t knot(unsigned i){return i==0?0u:i==1?66u:i==2?133u:200u;}
  static Phase phase(float rate){return std::fabs(rate)<.8f?Phase::Stationary:rate>0?Phase::Forward:Phase::Returning;}
  Result predict(uint32_t now_ms,uint32_t lead_ms=0) const {
    Result out;out.reason=reason;
    if(reason!=Reason::Ready)return out;
    if(now_ms<received_ms || now_ms<frame_ms){out.reason=Reason::Timestamp;return out;}
    out.age_ms=now_ms-frame_ms;
    if(out.age_ms>180u){out.reason=Reason::Stale;return out;}
    if(lead_ms>50u || out.age_ms+lead_ms>200u){out.reason=Reason::Horizon;return out;}
    out.horizon_ms=out.age_ms+lead_ms;
    unsigned i=0;while(i<2 && out.horizon_ms>knot(i+1))++i;
    const float duration=static_cast<float>(knot(i+1)-knot(i));
    const float mix=(out.horizon_ms-knot(i))/duration;
    out.left=left[i]+mix*(left[i+1]-left[i]);
    out.right=right[i]+mix*(right[i+1]-right[i]);
    out.left_rate=(left[i+1]-left[i])*1000.f/duration;
    out.right_rate=(right[i+1]-right[i])*1000.f/duration;
    const float age_margin=out.age_ms<=100u?0.f:(out.age_ms-100u)*.005f;
    out.left_uncertainty=left_uncertainty[i]+mix*(left_uncertainty[i+1]-left_uncertainty[i])+age_margin;
    out.right_uncertainty=right_uncertainty[i]+mix*(right_uncertainty[i+1]-right_uncertainty[i])+age_margin;
    const float age_factor=out.age_ms<=100u?1.f:(180.f-out.age_ms)/80.f;
    out.confidence=confidence*age_factor;
    out.method=confidence>0.f?Method::History:Method::Hold;
    if(out.confidence>=.2f){
      out.phase_left=phase(out.left_rate);out.phase_right=phase(out.right_rate);
      // A nearby forecast knot with an actual sign reversal is a turning zone.
      for(unsigned j=1;j<3;++j){
        const uint32_t d=out.horizon_ms>knot(j)?out.horizon_ms-knot(j):knot(j)-out.horizon_ms;
        if(d>20u)continue;
        if((left[j]-left[j-1])*(left[j+1]-left[j])<0.f)out.phase_left=Phase::Turning;
        if((right[j]-right[j-1])*(right[j+1]-right[j])<0.f)out.phase_right=Phase::Turning;
      }
    }
    return out;
  }
};

class Predictor {
 public:
  void reset(){*this=Predictor{};}
  Reason observe(uint16_t run,uint32_t frame,uint32_t received,bool valid,float l,float r){
    if(run!=run_){reset();run_=run;}
    if(!run){reset();return reason_=Reason::Invalid;}
    if(frame>received || (seen_ && frame<=last_seen_)){
      continuous_=0;return reason_=Reason::Timestamp;
    }
    seen_=true;last_seen_=frame;received_=received;
    Reason next=Reason::Ready;
    if(!valid || !std::isfinite(l) || !std::isfinite(r))next=Reason::Invalid;
    else if(received-frame>180u)next=Reason::Stale;
    else if(count_ && points_[count_-1].valid){
      const Point& prev=points_[count_-1];const uint32_t dt=frame-prev.ms;
      if(dt<foot_timing_policy::kMinFrameIntervalMs || dt>200u)next=Reason::Gap;
      else if(std::fabs(l-prev.l)>8.f || std::fabs(r-prev.r)>8.f ||
          std::fabs(l-prev.l)*1000.f/dt>80.f || std::fabs(r-prev.r)*1000.f/dt>80.f)next=Reason::Jump;
    }
    if(count_==capacity){for(unsigned i=1;i<capacity;++i)points_[i-1]=points_[i];--count_;}
    // Gap is a usable new seed, never a velocity interval across the gap.
    const bool usable=next==Reason::Ready || next==Reason::Gap;
    points_[count_++]=Point{frame,l,r,usable};
    if(next==Reason::Gap){continuous_=1;return reason_=next;}
    if(!usable){continuous_=0;return reason_=next;}
    if(continuous_<3u)++continuous_;
    reason_=continuous_>=2u?Reason::Ready:Reason::Warmup;
    return reason_;
  }

  Projection projection() const {
    Projection out;out.run_id=run_;out.received_ms=received_;out.reason=reason_;
    if(!count_)return out;
    const Point& current=points_[count_-1];out.frame_ms=current.ms;
    for(unsigned j=0;j<4;++j){out.left[j]=current.l;out.right[j]=current.r;}
    if(reason_!=Reason::Ready || continuous_<3u || count_<3u)return out;
    // Match complete *past* three-frame trajectories, including both feet.
    // Repetition interval is deliberately not hard-coded to the current run.
    float score[2]={.5f,.5f};unsigned candidate[2]={0,0};unsigned found=0;
    for(unsigned k=2;k+1<count_;++k){
      if(current.ms<points_[k].ms)continue;
      const uint32_t lag=current.ms-points_[k].ms;
      if(lag<800u || lag>4000u)continue;
      bool usable=true;float distance=0;
      for(unsigned j=0;j<3;++j){
        const Point& a=points_[count_-3+j];const Point& b=points_[k-2+j];
        const uint32_t da=current.ms-a.ms,db=points_[k].ms-b.ms;
        if(!a.valid || !b.valid || (da>db?da-db:db-da)>35u){usable=false;break;}
        distance+=(a.l-b.l)*(a.l-b.l)+(a.r-b.r)*(a.r-b.r);
      }
      if(!usable)continue;
      float future_l,future_r;
      if(!pastValue(points_[k].ms+200u,k,future_l,future_r))continue;
      distance/=6.f;
      if(distance>=score[1])continue;
      unsigned slot=distance<score[0]?0u:1u;
      if(slot==0){score[1]=score[0];candidate[1]=candidate[0];}
      score[slot]=distance;candidate[slot]=k;if(found<2u)++found;
    }
    if(!found)return out;
    // Divide by two even if only one match exists: single-cycle evidence is
    // deliberately shrunk toward holding the last observation.
    for(unsigned c=0;c<found;++c){
      const unsigned k=candidate[c];const float weight=(1.f-score[c]/.5f)*.5f;
      out.confidence+=weight;
      for(unsigned j=1;j<4;++j){
        float l,r;
        if(!pastValue(points_[k].ms+Projection::knot(j),k,l,r))continue;
        out.left[j]+=weight*clamp(l-points_[k].l,2.f);
        out.right[j]+=weight*clamp(r-points_[k].r,2.f);
      }
    }
    if(found==2u){
      // An identical recent trajectory can still precede different futures.
      // Keep this disagreement separate from the past-match confidence and
      // from the forecast itself: it must never improve a replay error score.
      out.left_uncertainty[0]=out.right_uncertainty[0]=.35f;
      for(unsigned j=1;j<4;++j){
        float al,ar,bl,br;
        const unsigned a=candidate[0],b=candidate[1];
        if(pastValue(points_[a].ms+Projection::knot(j),a,al,ar) &&
           pastValue(points_[b].ms+Projection::knot(j),b,bl,br)){
          out.left_uncertainty[j]=.35f+.5f*std::fabs((al-points_[a].l)-(bl-points_[b].l));
          out.right_uncertainty[j]=.35f+.5f*std::fabs((ar-points_[a].r)-(br-points_[b].r));
        }
      }
    }
    return out;
  }
  Result predict(uint32_t now_ms,uint32_t lead_ms=0) const{return projection().predict(now_ms,lead_ms);}

 private:
  struct Point {uint32_t ms=0;float l=0,r=0;bool valid=false;
    Point()=default;Point(uint32_t t,float a,float b,bool v):ms(t),l(a),r(b),valid(v){}
  };
  static constexpr unsigned capacity=48;
  Point points_[capacity]{};
  unsigned count_=0,continuous_=0;
  uint16_t run_=0;uint32_t last_seen_=0,received_=0;bool seen_=false;
  Reason reason_=Reason::Warmup;
  static float clamp(float x,float a){return x>a?a:x<-a?-a:x;}
  bool pastValue(uint32_t target,unsigned start,float& l,float& r) const {
    // Reject intervening invalid frames rather than interpolate through a loss.
    if(start>=count_ || !points_[start].valid || target<points_[start].ms)return false;
    if(target==points_[start].ms){l=points_[start].l;r=points_[start].r;return true;}
    for(unsigned j=start+1;j<count_;++j){
      const Point& a=points_[j-1];const Point& b=points_[j];
      if(!b.valid || b.ms<=a.ms || b.ms-a.ms>200u)return false;
      if(target<=b.ms){
        const float f=static_cast<float>(target-a.ms)/(b.ms-a.ms);
        l=a.l+f*(b.l-a.l);r=a.r+f*(b.r-a.r);return true;
      }
    }
    return false;
  }
};
}

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
  // Raw adjacent differences are retained for the separately gated v13
  // first-reversal experiment; v12 interpretation remains unchanged.
  float prior_change=0,last_change=0;
  uint32_t last_interval_ms=0;
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
      if(dt<foot_timing_policy::kMinFrameIntervalMs || dt>200u){
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
    out.prior_change=first;out.last_change=second;out.last_interval_ms=c.ms-b.ms;
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

// One source-selection function shared by firmware and causal replay. Local
// evidence never increases the history-match score or makes it a probability.
namespace foot_support_pose {
enum class Source : uint8_t { Unknown,History,Local,LocalTurn };
struct Selection {
  foot_phase_prediction::Result pose;
  Source source=Source::Unknown;
  bool usable=false,local_returning_confirmed=false;
  bool local_turn_confirmed=false;
  float local_evidence=0;
};
inline foot_phase_prediction::Phase phase(foot_local_prediction::Phase p){
  using L=foot_local_prediction::Phase;using H=foot_phase_prediction::Phase;
  switch(p){case L::Stationary:return H::Stationary;case L::Forward:return H::Forward;
    case L::Turning:return H::Turning;case L::Returning:return H::Returning;default:return H::Unknown;}
}
inline bool margins(float l,float r){return std::isfinite(l)&&std::isfinite(r)&&l>=0&&r>=0&&l<=.75f&&r<=.75f;}
inline Selection select(const foot_phase_prediction::Projection& history,
    const foot_local_prediction::Projection& local,uint32_t now,bool allow_first_turn=false){
  Selection out;out.pose=history.predict(now,0);
  if(out.pose.ready() && out.pose.age_ms<=160 && out.pose.confidence>=.5f &&
     out.pose.method==foot_phase_prediction::Method::History &&
     margins(out.pose.left_uncertainty,out.pose.right_uncertainty)){
    out.source=Source::History;out.usable=true;return out;
  }
  // Both projections must describe the same run and image, including losses.
  if(!history.run_id || history.run_id!=local.run_id || history.frame_ms!=local.frame_ms)return out;
  const auto q=local.predict(now,0);
  if(!q.ready() || q.age_ms>160 ||
     !std::isfinite(q.left)||!std::isfinite(q.right))return out;
  auto adopted=q;
  // A first small drop after a substantial rise is *tentative*, not confirmed
  // Returning. Use an observed angle (no extrapolation) for at most 100ms.
  // This new source is opt-in: v12 calls keep their original behavior.
  bool turn=false;
  if(allow_first_turn && q.age_ms<=100){
    const bool ld=q.left>=q.right;
    const auto& m=ld?local.left_model:local.right_model;
    const auto& other=ld?local.right_model:local.left_model;
    const float drop=-m.last_change;
    if(m.phase==foot_local_prediction::Phase::Turning && m.prior_change>=1.f &&
       drop>=.15f && drop<=1.5f && m.latest_rate<=-1.5f && m.latest_rate>=-20.f &&
       m.last_interval_ms>=60 && m.last_interval_ms<=150 &&
       other.phase==foot_local_prediction::Phase::Stationary && other.qualified){
      const float margin=.45f+.5f*drop+.002f*q.age_ms;
      const float other_margin=ld?q.right_uncertainty:q.left_uncertainty;
      if(m.observed-margin>=3.f && std::fabs(q.left-q.right)-margin-other_margin>=2.f){
        if(ld){adopted.left=m.observed;adopted.left_uncertainty=margin;}
        else {adopted.right=m.observed;adopted.right_uncertainty=margin;}
        turn=true;
      }
    }
  }
  if(!margins(adopted.left_uncertainty,adopted.right_uncertainty))return out;
  const bool left=adopted.left-adopted.left_uncertainty>=adopted.right-adopted.right_uncertainty;
  if(!turn && (left?q.phase_left:q.phase_right)!=foot_local_prediction::Phase::Returning)return out;
  // Preserve the historical score for logging; local evidence is separate.
  out.pose.reason=foot_phase_prediction::Reason::Ready;
  out.pose.left=adopted.left;out.pose.right=adopted.right;out.pose.left_rate=q.left_rate;out.pose.right_rate=q.right_rate;
  out.pose.left_uncertainty=adopted.left_uncertainty;out.pose.right_uncertainty=adopted.right_uncertainty;
  out.pose.phase_left=phase(q.phase_left);out.pose.phase_right=phase(q.phase_right);
  out.pose.age_ms=q.age_ms;out.pose.horizon_ms=q.horizon_ms;
  out.source=turn?Source::LocalTurn:Source::Local;out.usable=true;
  out.local_returning_confirmed=!turn;out.local_turn_confirmed=turn;out.local_evidence=q.confidence;
  return out;
}
}
