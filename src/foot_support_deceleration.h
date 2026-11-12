#pragma once
#include <cmath>
#include <cstdint>

// Confirm loss of *inward* body speed using only samples already received.
// This is neither a contact detector nor a decision to apply motor current.
// Intended for the runner's 5 ms candidate check; fixed storage, no allocation.
namespace foot_support_deceleration {
class Tracker {
 public:
  void reset(){*this=Tracker{};}
  bool update(uint16_t run,uint32_t peak,uint32_t now,float inward_speed){
    if(run!=run_ || peak!=peak_){reset();run_=run;peak_=peak;}
    if(!run || !peak || !std::isfinite(inward_speed) || inward_speed<=0.f){reset();return false;}
    if(have_time_ && now<=last_ms_){reset();return false;}
    if(have_time_ && now-last_ms_>15u){
      // The current valid sample can seed a new observation window, but no
      // speed maximum or confirmation survives an interruption.
      reset();run_=run;peak_=peak;
    }
    have_time_=true;last_ms_=now;
    raw_[raw_next_]=inward_speed;raw_next_=(raw_next_+1u)%3u;
    if(raw_count_<3u)++raw_count_;
    if(raw_count_<3u)return false;
    const float filtered=median(raw_[0],raw_[1],raw_[2]);
    history_[next_].ms=now;history_[next_].speed=filtered;
    next_=(next_+1u)%capacity;
    if(count_<capacity)++count_;
    float highest=filtered;
    for(unsigned i=0;i<count_;++i){
      // The window expires from actual sample times, never sample count alone.
      if(now-history_[i].ms<=100u && history_[i].speed>highest)highest=history_[i].speed;
    }
    if(highest-filtered<4.f){pending_=false;pending_count_=0;return false;}
    if(!pending_){pending_=true;pending_since_ms_=now;pending_count_=1;}
    else if(pending_count_<3u)++pending_count_;
    return pending_count_>=3u && now-pending_since_ms_>=10u;
  }

 private:
  struct Sample {uint32_t ms=0;float speed=0;};
  static constexpr unsigned capacity=24;
  Sample history_[capacity]{};
  float raw_[3]={};
  unsigned next_=0,count_=0,raw_next_=0,raw_count_=0,pending_count_=0;
  uint16_t run_=0;uint32_t peak_=0,last_ms_=0,pending_since_ms_=0;
  bool have_time_=false,pending_=false;
  static float median(float a,float b,float c){
    if(a>b){const float x=a;a=b;b=x;}
    if(b>c){const float x=b;b=c;c=x;}
    if(a>b)b=a;
    return b;
  }
};
}
