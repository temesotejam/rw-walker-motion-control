#pragma once
#include "lateral_cg_energy.h"
#include "work_target_policy.h"
namespace wheel_work_energy_control {
// Store U0(observed)-Eraw and its signed CG coefficient. Re-express the
// residual using the CURRENT CG estimate, so a paired CG update is not added
// a second time to an already-corrected model prediction. All values are mJ.
class SideEnergyFeedback {
 public:
  unsigned count() const { return count_; }
  __attribute__((noinline)) bool observe(const lateral_energy::Geometry& g, int side,
      float amplitude_deg, float raw_mj,float target_deg=8.f,
      float observed_cg_mm=0.f,float applied_bias_mj=0.f) {
    if ((side!=1 && side!=-1) || !std::isfinite(amplitude_deg) ||
        amplitude_deg<6.f || !std::isfinite(raw_mj) || raw_mj<0.f ||
        !work_target_policy::valid(target_deg)) return false;
    const float limit=.5f*std::fmax(1.f,lateral_energy::potential(g,target_deg)/lateral_energy::potential(g,8.f));
    const float residual=1000.f*lateral_energy::potential(g,amplitude_deg)-raw_mj;
    if(side==1 && target_deg==12.f && std::isfinite(residual) &&
        std::isfinite(observed_cg_mm) && std::isfinite(applied_bias_mj)) {
      coast_only_=amplitude_deg>work_target_policy::observationCeiling(target_deg) || std::fabs(residual)>limit;
      const float coefficient=g.mass_g*std::sin(amplitude_deg*lateral_energy::kRadPerDeg);
      const float error=residual+coefficient*observed_cg_mm-applied_bias_mj;
      const float tolerance=1000.f*(lateral_energy::potential(g,target_deg+.5f)-lateral_energy::potential(g,target_deg));
      underestimated_=error>tolerance;
      guard_residual_=residual;guard_coefficient_=coefficient;
    }
    if(amplitude_deg>work_target_policy::observationCeiling(target_deg)) return false;
    if (!std::isfinite(residual) || std::fabs(residual)>limit) return false;
    limit_mj_=limit;
    residual_[next_]=residual;
    coefficient_[next_]=g.mass_g*std::sin(side*amplitude_deg*lateral_energy::kRadPerDeg);
    next_=(next_+1)%3;if(count_<3) ++count_;
    return true;
  }
  __attribute__((noinline)) float biasMj(float cg_mm,float fallback_mj) const {
    if (count_<2 || !std::isfinite(cg_mm)) return fallback_mj;
    float sum=0;
    for(unsigned i=0;i<count_;++i) sum+=residual_[i]+coefficient_[i]*cg_mm;
    return std::fmax(-limit_mj_,std::fmin(limit_mj_,sum/count_));
  }
  // Use the latest same-model residual immediately after a large positive
  // underprediction. MAX avoids adding it a second time to the running mean.
  float positiveBiasMj(float cg_mm,float fallback_mj,float target_deg) const {
    const float average=biasMj(cg_mm,fallback_mj);
    if(target_deg!=12.f || !underestimated_ || !std::isfinite(cg_mm)) return average;
    const float latest=std::fmax(-limit_mj_,std::fmin(limit_mj_,guard_residual_+guard_coefficient_*cg_mm));
    return std::fmax(average,latest);
  }
  bool coastOnly() const { return coast_only_; }
 private:
  float residual_[3]={},coefficient_[3]={};
  unsigned count_=0,next_=0;
  float limit_mj_=.5f;
  float guard_residual_=0.f,guard_coefficient_=0.f;
  bool underestimated_=false,coast_only_=false;
};
}
