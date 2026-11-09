#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "wheel_work_energy_control.h"

// 8-degree walking, positive branch 5 only. Bias is measured arrival energy
// minus the same model's uncorrected prediction. This is an experimental
// output correction, not a learned heading or foot-dynamics model.
namespace low_peak_energy {
class Feedback {
 public:
  void reset(){*this=Feedback{};}
  void command(uint32_t ms,int side,bool output,int current,uint16_t width,uint8_t branch,float rate,float rpm,float vbat) {
    if(side!=1)return;
    if(!output || current!=400 || width<60 || width>100 || branch!=5 || !std::isfinite(rate) ||
       rate<12.f || rate>40.f || !std::isfinite(rpm) ||
       rpm< -3.f || rpm>200.f || !std::isfinite(vbat) || vbat<7400.f || vbat>8060.f) {
      command_valid_=false;invalidate();return;
    }
    command_ms_=ms;width_=width;command_valid_=true;
  }
  __attribute__((noinline)) void negative(uint32_t ms,float amplitude,bool fresh) {
    if(!fresh || !std::isfinite(amplitude)) {negative_ms_=0;invalidate();return;}
    negative_=amplitude;negative_ms_=ms;
    if(amplitude<3.2f) {invalidate();return;}
    if(bias_>0 && changed_>0 && (amplitude<3.f || amplitude<negative_baseline_-.5f)) {
      if(++weak_>=2) {disabled_=true;release();}
    } else weak_=0;
  }
  __attribute__((noinline)) void positive(uint32_t ms,float actual,float prediction,float applied_own_bias,float target,float amplitude,
      bool fresh,bool foot_valid,float left,float right) {
    if(!command_valid_ || ms<=command_ms_ || ms-command_ms_>500 || !fresh ||
       !std::isfinite(actual) || !std::isfinite(prediction) || !std::isfinite(applied_own_bias) || !std::isfinite(target) ||
       actual<0 || prediction<0 || target<=0 || !std::isfinite(amplitude) || amplitude<2.5f || amplitude>=6.f ||
       !negative_ms_ || ms<=negative_ms_ || ms-negative_ms_>1500 || negative_<3.2f ||
       !foot_valid || !std::isfinite(left) || !std::isfinite(right) || left<right+4.f || left+right<5.f) {
      invalidate();return;
    }
    if(last_target_>0 && std::fabs(target-last_target_)>.04f) invalidate();
    last_target_=target;
    // Remove this learner's contribution only; retain the original bridge bias.
    const float residual=actual-(prediction-applied_own_bias);
    if(!std::isfinite(residual) || residual<.015f || residual>.20f) {invalidate();return;}
    last_observed_ms_=ms;
    const float error=std::fabs(left-right)/(left+right+2.f);
    residuals_[next_]=residual;foot_errors_[next_]=error;next_=(next_+1)%3;if(samples_<3)++samples_;
    if(samples_<3 || disabled_) {if(disabled_)release();return;}
    float lo=residuals_[0],hi=lo,sum=0;
    for(unsigned i=0;i<3;++i){lo=std::fmin(lo,residuals_[i]);hi=std::fmax(hi,residuals_[i]);sum+=residuals_[i];}
    if(hi-lo>.06f) {invalidate();return;}
    const float mean_error=(foot_errors_[0]+foot_errors_[1]+foot_errors_[2])/3.f;
    if(!active_) {active_=true;foot_baseline_=mean_error;negative_baseline_=negative_;initial_width_=width_;changed_=0;}
    if(width_<initial_width_)++changed_;
    if(changed_>=8 && mean_error>foot_baseline_-.01f) {disabled_=true;release();return;}
    if(changed_>=3 && mean_error>foot_baseline_+.08f) {disabled_=true;release();return;}
    const float desired=std::fmin(.12f,std::fmax(0.f,sum/3.f));
    bias_+=std::fmax(-.01f,std::fmin(.01f,desired-bias_));
  }
  bool eligibleForCorrection(uint32_t ms,uint16_t cap,float target,bool foot_valid,float left,float right) const {
    return !disabled_ && command_valid_ && ms>=command_ms_ && ms-command_ms_<=1500 && cap>=60 &&
       last_observed_ms_ && ms>=last_observed_ms_ && ms-last_observed_ms_<=1500 &&
       negative_ms_ && ms>=negative_ms_ && ms-negative_ms_<=1500 && negative_>=3.2f &&
       std::isfinite(target) && last_target_>0 && std::fabs(target-last_target_)<=.04f &&
       foot_valid && std::isfinite(left) && std::isfinite(right) && left>=right+4.f && left+right>=5.f &&
       (bias_>0.f || (active_ && width_<cap));
  }
  // If correction becomes unavailable, approach the ordinary selector in
  // 5ms steps using zero extra bias, provided the original 400mA choice is safe.
  bool recover(uint32_t ms,uint16_t cap)const {
    return command_valid_ && ms>=command_ms_ && ms-command_ms_<=1500 && cap>=60 && width_>=60 && width_<cap;
  }
  float bias()const{return bias_;}
  uint16_t lastWidth()const{return width_;}
  bool active()const{return active_ && !disabled_;}
 private:
  void release(){bias_=std::fmax(0.f,bias_-.01f);}
  void invalidate(){samples_=0;active_=false;bias_=0;last_observed_ms_=0;last_target_=0;changed_=weak_=0;}
  float residuals_[3]={},foot_errors_[3]={},bias_=0,last_target_=0,negative_=NAN,negative_baseline_=0,foot_baseline_=0;
  uint32_t command_ms_=0,negative_ms_=0,last_observed_ms_=0;uint16_t width_=0,initial_width_=0;
  unsigned samples_=0,next_=0,weak_=0,changed_=0;bool command_valid_=false,active_=false,disabled_=false;
};

// Fix current to the already selected 400 mA, and move width no more than 5ms.
// Baseline safety caps take precedence; no 0ms action is converted to a pulse.
__attribute__((noinline)) inline wheel_work_energy_control::Choice choose(
    const lateral_energy::Geometry& geometry,const wheel_work_energy_control::State& state,
    const lateral_energy::CurrentTable& table,float goal,float initial,float target,
    float work_bias,float bridge_bias,uint16_t cap,uint16_t previous) {
  using namespace wheel_work_energy_control;
  Choice c;const uint16_t lo=previous>65?previous-5:60;
  const uint16_t hi=std::min<uint16_t>(cap,previous+5);
  if(lo>hi || !std::isfinite(goal) || goal<=0.f)return c;
  for(uint16_t width=lo;width<=hi;++width) {
    const float q=lateral_energy::charge(table,width,initial,goal);
    const Prediction p=positiveBoundary(geometry,state,q,work_bias,bridge_bias);
    if(p.branch==5)consider(c,400,width,q,p,target);
  }
  c.allowed_cap_ms=cap;finish(c,target);return c;
}
}
