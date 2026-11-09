#pragma once
#include "wheel_work_energy_control.h"

// Fixed-leg target-12 calibration. Runtime features are exclusively the
// pre-command state and the candidate current integral. Reaction work and
// retained arrival energy are different empirical predictions, both in mJ.
namespace target12_state_work {
using wheel_work_energy_control::State;
using wheel_work_energy_control::Prediction;
using wheel_work_energy_control::Choice;
constexpr char kRevision[]="target12_state_work_20261027";
inline float clamp(float x,float lo,float hi) { return std::fmax(lo,std::fmin(hi,x)); }
inline void contextWeights(float rpm,float voltage,float w[6]) {
  const float r=clamp(rpm,0.f,330.f);
  const float a=wheel_work_energy_control::ramp(r/150.f);
  const float b=wheel_work_energy_control::ramp((r-150.f)/180.f);
  const float rw[]={1.f-a,a-b,b};
  const float v=wheel_work_energy_control::ramp((voltage-7380.f)/170.f);
  for(unsigned i=0;i<3;++i) { w[i*2]=rw[i]*(1.f-v);w[i*2+1]=rw[i]*v; }
}
inline float weight(const State& s,int side) {
  using wheel_work_energy_control::boxWeight;
  if(s.target_deg!=12.f || !s.absolute_reference || !wheel_work_energy_control::freshFinite(s) ||
      !std::isfinite(s.previous_peak_deg)) return 0.f;
  return boxWeight(s.rate_dps,side>0 ? 66.266f : 70.0554f,side>0 ? 89.0184f : 89.6155f,4.f,4.f)*
      boxWeight(s.wheel_rpm,side>0 ? -18.01f : 0.f,side>0 ? 230.94f : 339.47f,20.f,40.f)*
      boxWeight(s.battery_mv,7380.f,side>0 ? 7560.f : 7550.f,150.f,150.f)*
      boxWeight(s.previous_peak_deg,side>0 ? 9.31816f : 10.24767f,side>0 ? 12.96732f : 13.85487f,1.f,1.f);
}
inline Prediction local(const State& s,int side,float signed_q) {
  Prediction p;
  if(!wheel_work_energy_control::freshFinite(s) || !std::isfinite(signed_q) ||
      std::fabs(signed_q)>25.f || (side!=1 && side!=-1)) return p;
  const float omega=s.rate_dps*wheel_work_energy_control::kRadPerDeg;
  const float x[]={1.f,omega*omega-1.8f,s.wheel_rpm/200.f,(s.battery_mv-7470.f)/100.f,s.previous_peak_deg-12.f};
  const float positive_work[]={.06495765078f,.05670368411f,.00065973430f,.04409678615f,.00655686205f};
  const float negative_work[]={.05774250858f,-.04010128325f,.28719547764f,.01764314989f,-.00098205826f};
  const float positive_energy[]={1.17989186647f,.85474509630f,.04232319621f,.02081009773f,-.07510864409f};
  const float negative_energy[]={1.13109614121f,.52297048567f,.12763007965f,.05708735823f,-.02834127311f};
  const float* wc=side>0 ? positive_work : negative_work;
  const float* ec=side>0 ? positive_energy : negative_energy;
  float zero_work=0.f;
  p.free_mj=0.f;
  for(unsigned i=0;i<5;++i) {zero_work+=wc[i]*x[i];p.free_mj+=ec[i]*x[i];}
  const bool assist=side>0 || signed_q>=0.f;
  const float gain=side>0 ? .09562070337f-.01169490267f*x[2]-.00357421794f*x[3] :
      assist ? .11313805070f+.01481461148f*x[2]-.01466338968f*x[3] :
      .16463515404f+.10123550962f*x[2]+.00260921637f*x[3];
  const float on_gain=side>0 ? .11077397423f-.00992843138f*x[2]+.00291289191f*x[3] :
      assist ? .18255593405f+.03606718983f*x[2]-.01138727712f*x[3] :
      .20951999460f-.00858806089f*x[2]-.00247415662f*x[3];
  // Positive slopes only in the bounded calibration/transition envelope.
  // This is an empirical dose response, not identified motor efficiency.
  const float dose=signed_q*omega;
  p.change_mj=clamp(gain,.02f,.45f)*dose;
  p.work_mj=zero_work+p.change_mj;
  p.on_mj=clamp(on_gain,.02f,.45f)*dose;
  p.after_mj=p.work_mj-p.on_mj;
  const float retained=side>0 ? .31786294189f : assist ? .28058382254f : .27965969606f;
  p.energy_mj=p.free_mj+retained*p.change_mj;
  p.valid=std::isfinite(p.energy_mj) && p.energy_mj>=0.f;
  p.branch=side>0 ? 6 : assist && signed_q>0.f ? 9 : 4;
  p.cross_condition=true;
  return p;
}

inline Prediction predict(const lateral_energy::Geometry& g,const State& s,int side,
    float signed_q,uint16_t width) {
  using namespace wheel_work_energy_control;
  auto old=side>0 ? positiveBoundary(g,s,signed_q,0.f,0.f) :
      signed_q>0.f ? negativeAssistRaw(s,signed_q,0.f,0.f) : negativeBoundary(s,width,0.f);
  const float w=weight(s,side);
  if(w<=0.f) return old;
  auto p=local(s,side,signed_q);
  if(!p.valid || !old.valid) return p.valid && w>=1.f ? p : old;
  p.energy_mj=w*p.energy_mj+(1.f-w)*old.energy_mj;
  p.free_mj=w*p.free_mj+(1.f-w)*old.free_mj;
  // The historical positive bridge supplies no physical-work prediction.
  // Its transition remains energy-only rather than inventing work labels.
  if(w<1.f && (!std::isfinite(old.work_mj) || !std::isfinite(old.on_mj) || !std::isfinite(old.after_mj))) {
    p.work_mj=p.on_mj=p.after_mj=p.change_mj=NAN;
  } else if(w<1.f) {
    p.work_mj=w*p.work_mj+(1.f-w)*old.work_mj;
    p.on_mj=w*p.on_mj+(1.f-w)*old.on_mj;
    p.after_mj=p.work_mj-p.on_mj;
    p.change_mj=w*p.change_mj+(1.f-w)*old.change_mj;
  }
  p.support_weight=side>0 ? w : negativeBoundaryWeight(s);
  p.valid=std::isfinite(p.energy_mj) && p.energy_mj>=0.f;
  return p;
}

// Keep causal residuals with their pre-input context. Shrink sparse estimates
// toward zero; never carry a high-rpm residual wholesale into a stopped rotor.
class ContextFeedback {
 public:
  unsigned count() const {return count_;}
  void observe(const lateral_energy::Geometry& g,int side,float amplitude,float raw,
      float rpm,float voltage,float applied_bias,float cg) {
    if(!std::isfinite(amplitude) || amplitude<6.f || !std::isfinite(raw) ||
        !std::isfinite(rpm) || !std::isfinite(voltage)) return;
    const float residual=1000.f*lateral_energy::potential(g,amplitude)-raw;
    const float limit=.5f*lateral_energy::potential(g,12.f)/lateral_energy::potential(g,8.f);
    coast_only_=amplitude>15.f || !std::isfinite(residual) || std::fabs(residual)>limit;
    if(coast_only_) return;
    Row r{residual,g.mass_g*std::sin(side*amplitude*lateral_energy::kRadPerDeg),rpm,voltage};
    history_[next_]=r;next_=(next_+1)%8;if(count_<8) ++count_;
    latest_=r;
    guard_=side>0 && residual+r.coefficient*cg-applied_bias>
        1000.f*(lateral_energy::potential(g,12.5f)-lateral_energy::potential(g,12.f));
  }
  float bias(float rpm,float voltage,float cg,int side) const {
    float sum=0.f,n=0.f;
    for(unsigned i=0;i<count_;++i) {
      const auto& r=history_[i];const float dr=(rpm-r.rpm)/100.f,dv=(voltage-r.voltage)/150.f;
      const float w=1.f/(1.f+dr*dr+dv*dv);
      sum+=w*(r.residual+r.coefficient*cg);n+=w;
    }
    float b=sum/(n+(side>0 ? 4.f : 2.f));
    if(side>0 && guard_) {
      const float dr=(rpm-latest_.rpm)/100.f,dv=(voltage-latest_.voltage)/150.f;
      b=std::fmax(b,(latest_.residual+latest_.coefficient*cg)/(1.f+dr*dr+dv*dv));
    }
    return clamp(b,-.30f,.30f);
  }
  bool coastOnly() const {return coast_only_;}
 private:
  struct Row {float residual=0,coefficient=0,rpm=0,voltage=0;
    Row()=default;Row(float a,float b,float c,float d):residual(a),coefficient(b),rpm(c),voltage(d) {}};
  Row history_[8],latest_;
  unsigned count_=0,next_=0;
  bool guard_=false,coast_only_=false;
};

class ContextCaps {
 public:
  bool allowed() const {return armed_;}
  uint16_t capMs(float rpm,float voltage) const {
    float w[6];contextWeights(rpm,voltage,w);float cap=0;
    for(unsigned i=0;i<6;++i)cap+=w[i]*caps_[i];
    return static_cast<uint16_t>(std::floor(clamp(cap,25.f,100.f)+1e-4f));
  }
  void observe(const lateral_energy::Geometry& g,float amplitude,float rpm,float voltage,
      float cg,bool assisted,uint16_t width,uint16_t applied_cap) {
    if(!std::isfinite(amplitude) || amplitude<6.f || amplitude>15.f ||
        !std::isfinite(rpm) || !std::isfinite(voltage)) return;
    const float actual=1000.f*lateral_cg::potential(g,-amplitude,cg);
    const float target=1000.f*lateral_cg::potential(g,-12.f,cg);
    const float low=1000.f*lateral_cg::potential(g,-11.7f,cg);
    const float high=1000.f*lateral_cg::potential(g,-12.3f,cg);
    if(actual<low) {if(low_count_<2)++low_count_;} else low_count_=0;
    if(low_count_>=2)armed_=true;
    if(!assisted) return;
    float w[6];contextWeights(rpm,voltage,w);
    for(unsigned i=0;i<6;++i) {
      if(actual<low && width>=applied_cap && applied_cap>=10) {
        evidence_[i]+=w[i];
        if(evidence_[i]>=1.5f) {caps_[i]=std::fmin(100.f,caps_[i]+5.f);evidence_[i]-=1.5f;}
      } else if(actual>high && actual>target) {
        caps_[i]=std::fmax(25.f,caps_[i]-5.f*w[i]);evidence_[i]*=1.f-w[i];
      }
    }
  }
 private:
  // Seed from the recent measured duration envelope: near-stopped 35/40 ms,
  // middle-speed 35 ms, fast rotor 25 ms. Further extension needs matching
  // completed low responses; these are exploration caps, not forced pulses.
  float caps_[6]={35,40,35,35,25,25},evidence_[6]={};
  uint8_t low_count_=0;bool armed_=false;
};

class Controller {
 public:
  unsigned count(int side) const {return feedback_[side>0 ? 0 : 1].count();}
  Prediction forecast(const lateral_energy::Geometry& g,const State& s,int side,
      float signed_q,uint16_t width,float cg) const {
    auto p=predict(g,s,side,signed_q,width);
    const float b=feedback_[side>0 ? 0 : 1].bias(s.wheel_rpm,s.battery_mv,cg,side);
    p.energy_mj+=b;p.applied_bias_mj=b;
    p.valid=p.valid && std::isfinite(p.energy_mj) && p.energy_mj>=0.f;
    return p;
  }
  void observe(const lateral_energy::Geometry& g,int side,float amplitude,float raw,
      float rpm,float voltage,float bias,float cg,bool assisted,uint16_t width,uint16_t cap,
      bool calibrated=true) {
    if(calibrated || amplitude>15.f)
      feedback_[side>0 ? 0 : 1].observe(g,side,amplitude,raw,rpm,voltage,bias,cg);
    if(side<0)caps_.observe(g,amplitude,rpm,voltage,cg,assisted,width,cap);
  }
  __attribute__((noinline)) Choice choose(const lateral_energy::Geometry& g,const State& s,int side,
      const lateral_energy::CurrentTable* const tables[3],const float goals[3],float aligned_initial,
      float target_mj,float cg,uint16_t brake_cap) const {
    using namespace wheel_work_energy_control;
    Choice c;
    if(s.target_deg!=12.f || !s.absolute_reference || !freshFinite(s) ||
        !std::isfinite(s.previous_peak_deg) || !std::isfinite(target_mj) || target_mj<=0.f ||
        s.battery_mv<(side>0 ? 7000.f : 6210.f) || s.battery_mv>8120.f || (side!=1 && side!=-1) ||
        (side<0 && negativeBoundaryWeight(s)<=0.f)) return c;
    const auto& fb=feedback_[side>0 ? 0 : 1];
    const float bias=fb.bias(s.wheel_rpm,s.battery_mv,cg,side);
    const auto add=[&](int16_t ma,uint16_t ms,float signed_q,int sign,uint16_t cap) {
      auto p=predict(g,s,side,signed_q,ms);
      p.energy_mj+=bias;p.applied_bias_mj=bias;
      p.valid=p.valid && std::isfinite(p.energy_mj) && p.energy_mj>=0.f;
      const auto before=c.evaluations;const float old_error=c.valid ? std::fabs(c.energy_mj-target_mj) : INFINITY;
      consider(c,ma,ms,signed_q,p,target_mj);
      if(c.evaluations>before && std::fabs(c.energy_mj-target_mj)<old_error) {
        c.command_sign=sign;c.allowed_cap_ms=cap;
      }
    };
    if(side>0) {
      const int16_t currents[]={300,350,400};
      for(unsigned i=0;i<3;++i) {
        for(uint16_t ms=0;ms<=100;ms=ms==0 ? 10 : ms+1) {
          if(ms>0 && fb.coastOnly())break;
          add(currents[i],ms,chargeSafe(*tables[i],ms,aligned_initial,goals[i]),1,100);
        }
        if(c.valid && std::fabs(c.energy_mj-target_mj)<=.005f)break;
      }
    } else {
      const float boundary=negativeBoundaryWeight(s);
      const uint16_t brake=static_cast<uint16_t>(std::floor(std::fmin<float>(70,brake_cap)*boundary));
      for(uint16_t ms=0;ms<=brake;ms=ms==0 ? 10 : ms+1)
        add(300,ms,-chargeSafe(*tables[0],ms,-aligned_initial,goals[0]),-1,brake);
      const auto coast=predict(g,s,side,0,0);
      if(caps_.allowed() && s.battery_mv>=7000.f && coast.valid && target_mj>coast.energy_mj+bias+.03f) {
        const uint16_t cap=static_cast<uint16_t>(std::floor(caps_.capMs(s.wheel_rpm,s.battery_mv)*boundary));
        for(uint16_t ms=10;ms<=cap;++ms)
          add(300,ms,chargeSafe(*tables[0],ms,aligned_initial,goals[0]),1,cap);
      }
    }
    finish(c,target_mj);return c;
  }
 private:
  static float chargeSafe(const lateral_energy::CurrentTable& t,uint16_t ms,float i,float goal) {
    return ms ? lateral_energy::charge(t,ms,i,goal) : 0.f;
  }
  ContextFeedback feedback_[2];ContextCaps caps_;
};
} // namespace target12_state_work
