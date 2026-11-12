#pragma once
#include <cmath>
#include <cstdint>

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
      if(dt<40u || dt>200u)next=Reason::Gap;
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
