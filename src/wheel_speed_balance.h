#pragma once
#include <cmath>
#include <cstdint>
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
