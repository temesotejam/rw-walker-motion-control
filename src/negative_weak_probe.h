#pragma once
#include <cmath>
#include <cstdint>

// A bounded, one-pulse experiment. Peaks are body angles, not foot travel.
// The caller retains all actuator, timing and wheel protections and only
// confirms a candidate after the increased command was actually sent.
namespace negative_weak_probe {
class Controller {
 public:
  void reset() {*this=Controller{};}
  void command(int side,uint32_t ms,uint16_t actual_ms,uint16_t baseline_cap_ms,bool executed) {
    if(attempted_)return;
    if(!ms || (side!=1 && side!=-1) || !executed || !actual_ms) {clear();return;}
    if(side<0) {negative_command_ms_=ms;negative_width_=actual_ms;negative_cap_=baseline_cap_ms;}
  }
  void peak(int side,uint32_t ms,float angle,float target,bool matched,bool healthy) {
    if(attempted_)return;
    if(!healthy || !matched || !ms || (side!=1 && side!=-1) || target!=8.f ||
       !std::isfinite(angle) || angle<0.f) {clear();return;}
    if(last_peak_ms_ && (ms<=last_peak_ms_ || ms-last_peak_ms_>1500 || side==last_side_))clear();
    last_peak_ms_=ms;last_side_=static_cast<int8_t>(side);
    if(side>0) {
      if(angle<5.f || angle>9.f) {clear();return;}
      plus_peak_=angle;plus_ms_=ms;return;
    }
    if(!plus_ms_ || ms<=plus_ms_ || ms-plus_ms_>1000 || angle<2.5f || angle>5.f ||
       plus_peak_-angle<1.f || !negative_command_ms_ || ms<=negative_command_ms_ ||
       ms-negative_command_ms_>1500 || negative_command_ms_<=plus_ms_ ||
       negative_cap_!=60 || negative_width_<55 || negative_width_>60) {clear();return;}
    if(weak_pairs_<4)++weak_pairs_;
    negative_command_ms_=0;
  }
  bool candidate(uint32_t now,float target,uint16_t baseline_cap,bool healthy) const {
    return !attempted_ && healthy && target==8.f && baseline_cap==60 && weak_pairs_>=4 &&
      last_peak_ms_ && now>=last_peak_ms_ && now-last_peak_ms_<=1500;
  }
  void attempt(uint32_t ms,bool executed,uint16_t guarded_baseline,uint16_t guarded_actual) {
    if(!attempted_ && ms && executed && guarded_actual>guarded_baseline &&
       guarded_actual<=guarded_baseline+5) {attempted_=true;applied_=true;}
  }
  bool attempted() const{return attempted_;}
  bool applied() const{return applied_;}
 private:
  void clear() {last_peak_ms_=plus_ms_=negative_command_ms_=0;last_side_=0;
    plus_peak_=0.f;negative_width_=negative_cap_=0;weak_pairs_=0;}
  uint32_t last_peak_ms_=0,plus_ms_=0,negative_command_ms_=0;
  float plus_peak_=0.f;
  uint16_t negative_width_=0,negative_cap_=0;
  uint8_t weak_pairs_=0;
  int8_t last_side_=0;
  bool attempted_=false,applied_=false;
};
}
