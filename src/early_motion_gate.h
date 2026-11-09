#pragma once
#include <cmath>
#include <cstdint>
// Diagnostic experiment: geometry-derived angle, never a contact assertion.
// All times use the same run-relative monotonic millisecond clock.
namespace early_motion_gate {
constexpr uint32_t kMaxFrameAgeMs=120, kMinEstimatedLeadMs=20;
struct Gate {
  uint16_t run=0; uint32_t peak=0,frame=0,ready=0; float threshold=0; bool valid=false;
  void clear(){valid=false;}
  bool offer(uint16_t r,uint32_t p,uint32_t f,uint32_t t,float a,uint32_t now){
    clear();
    if(!r || !p || !f || f>t || t>now || now-f>kMaxFrameAgeMs ||
       !std::isfinite(a) || std::fabs(a)>1.f || std::fabs(a)<.25f) return false;
    run=r;peak=p;frame=f;ready=t;threshold=a;valid=true;return true;
  }
  bool fire(uint16_t r,uint32_t p,uint32_t now,float a,float v) {
    if(!valid)return false;
    if(run!=r || peak!=p || now<ready || now<frame || now-frame>kMaxFrameAgeMs){clear();return false;}
    if(!std::isfinite(a) || !std::isfinite(v) || v==0.f)return false;
    const float side=threshold>0.f?1.f:-1.f;
    if(a*side<=0.f){clear();return false;}
    if(v*side>=0.f || a*side>threshold*side)return false;
    // Screening estimate only: a future actuator deadline is not guaranteed.
    // A tiny conservative margin avoids permitting a rounded borderline
    // value after replacing the former float division with multiplication.
    return (50.f*std::fabs(a)*0.999999f)>std::fabs(v);
  }
};
}
