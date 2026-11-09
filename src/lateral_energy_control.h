#pragma once
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include "realtime_code.h"

// The quadratic model converts logged Q (mA*s) to A*s and rate to rad/s.
// The historical angle bridge below uses logged Q in mA*s and rate in deg/s.
// Both coefficient sets are empirical, not measured inertia/torque constants.
namespace lateral_energy {
constexpr float kRadPerDeg = 0.01745329251994329577f;
constexpr uint16_t kMaxMs = 100;
// Reverse-current response has not been identified on this mechanism yet.
// Limit the first braking trials to half of the excitation time budget.
constexpr uint16_t kBrakeMaxMs = 50; // Absolute limit; trial choices are 0/20/50 ms.
constexpr float kIntegralGain = .10f;
constexpr char kRevision[] = "lateral_energy_angle_bridge_v8_20261015";
constexpr char kQuadraticRevision[] = "lateral_energy_saved_upright_v7_20261014";
struct Geometry {
  float mass_g, height, radius, inner_x, center, inner_angle, outer_angle;
  Geometry(float m=.1997f, float gravity=9.80665f, float h=.120f,
           float r=.150f, float x=.005f, float outer=.045f)
      : mass_g(m*gravity),height(h),radius(r),inner_x(x),
        center(std::sqrt(r*r-x*x)),inner_angle(std::asin(x/r)),
        outer_angle(std::asin(outer/r)) {}
};
// Stable height differences; shape-only constants are prepared once.
static inline RW_SPEED_CODE float potential(const Geometry& g, float degrees) {
  const float raw=std::fabs(degrees)*kRadPerDeg;
  if (!std::isfinite(raw) || raw>g.outer_angle+1e-6f || !(g.mass_g>0)) return NAN;
  const float a=std::fmin(raw,g.outer_angle);
  const float half_sine=std::sin(.5f*a);
  const float dh=a<=g.inner_angle
      ? -2.f*g.height*half_sine*half_sine+g.inner_x*std::sin(a)
      : (g.radius-g.center)+2.f*(g.center-g.height)*half_sine*half_sine;
  return g.mass_g*dh;
}
static inline RW_SPEED_CODE float angle(const Geometry& g, float energy) {
  if (!std::isfinite(energy) || energy<0 || !(g.mass_g>0)) return NAN;
  if (energy==0) return 0;
  const float boundary=potential(g,g.inner_angle/kRadPerDeg);
  const float y=energy/g.mass_g;
  float a;
  if (energy<=boundary) {
    const float disc=g.inner_x*g.inner_x-(2.f*g.height+y)*y;
    if (disc<0) return NAN;
    const float t=y/(g.inner_x+std::sqrt(disc));
    a=2.f*std::atan(t);
  } else {
    const float term=(y-(g.radius-g.center))/(2.f*(g.center-g.height));
    if (!(term>=0 && term<=1)) return NAN;
    a=2.f*std::asin(std::sqrt(term));
  }
  return a<=g.outer_angle+1e-6f ? a/kRadPerDeg : NAN;
}
struct Coefficients {
  float free_offset_j, free_omega2, work_omega_q, work_q2;
  float rate_min_dps, rate_max_dps, charge_max_mAs;
};
static inline RW_SPEED_CODE Coefficients coefficients(int8_t side) {
  return side>0
    ? Coefficients{-.0002019364753f,.0007317420955f,.01259279858f,.5522071518f,17.9508f,92.2064f,13.27138f}
    : Coefficients{-.0002180822115f,.0009189896078f,.03889674250f,.2578004825f,35.7375f,91.4938f,12.83679f};
}
static inline RW_SPEED_CODE float freeEnergy(const Coefficients& c,float omega) {
  return std::fmax(0.f,c.free_omega2*omega*omega+c.free_offset_j);
}
// Q is signed in the intended body excitation direction. Negative residual
// current cannot be counted as positive added energy via abs(Q).
static inline RW_SPEED_CODE float work(const Coefficients& c,float omega,float q_mAs) {
  const float q=q_mAs*.001f;
  return c.work_omega_q*omega*q+c.work_q2*q*q;
}
static inline RW_SPEED_CODE float requiredCharge(const Coefficients& c,float omega,float energy) {
  if (!std::isfinite(omega) || omega<0 || !std::isfinite(energy) ||
      c.work_omega_q<0 || c.work_q2<0) return NAN;
  if (energy<=0) return 0;
  const float a=c.work_omega_q*omega;
  const float root=std::sqrt(a*a+4.f*c.work_q2*energy);
  const float denominator=a+root;
  return denominator>0 ? 2000.f*energy/denominator : NAN;
}
// Select the root nearest zero. Negative work is used only within the
// reachable, limited reverse-charge interval, below the quadratic vertex.
static inline RW_SPEED_CODE float requiredSignedCharge(const Coefficients& c,float omega,float energy) {
  if (energy>=0) return requiredCharge(c,omega,energy);
  if (!std::isfinite(omega) || omega<0 || !std::isfinite(energy) ||
      !(c.work_omega_q>0) || !(c.work_q2>0)) return NAN;
  const float a=c.work_omega_q*omega;
  const float discriminant=a*a+4.f*c.work_q2*energy;
  if (discriminant<0) return NAN;
  return 2000.f*energy/(a+std::sqrt(discriminant));
}
struct CurrentTable {
  float a[kMaxMs+1]{},b[kMaxMs+1]{};
  float tau=0;
  bool ready=false;
  // Called before START_SYNC, never from the zero-cross decision.
  bool prepare(float seconds) {
    ready=false;
    if (!std::isfinite(seconds) || seconds<=0) return false;
    tau=seconds;
    for (uint16_t n=0;n<=kMaxMs;++n) {
      const float t=static_cast<float>(n)*.001f;
      b[n]=tau*(-std::expm1(-t/tau));a[n]=t-b[n];
    }
    ready=true;return true;
  }
};
static inline RW_SPEED_CODE float charge(const CurrentTable& t,uint16_t ms,float initial,float goal) {
  return ms<=kMaxMs && t.ready ? goal*t.a[ms]+initial*t.b[ms] : NAN;
}
struct Decision {
  bool valid=false, upper=false, lower=false, extrapolated=false;
  // +1 adds energy in the zero-cross motion direction; -1 removes it.
  int8_t command_sign=0;
  uint16_t ms=0, evaluations=0;
  float kinetic_j=NAN, loss_j=NAN, free_j=NAN, target_j=NAN, deficit_j=NAN;
  float commanded_deficit_j=NAN, corrected_target_j=NAN;
  float requested_q_mAs=NAN, available_q_mAs=NAN, selected_q_mAs=0;
  float predicted_j=NAN, energy_error_j=NAN;
  // Unrestricted inverse is diagnostic; an invalid root does not change output.
  float unclamped_deficit_j=NAN, unclamped_q_mAs=NAN;
  float maximum_input_work_j=NAN;
};
static inline RW_SPEED_CODE Decision decide(const Geometry& geometry,const CurrentTable& table,
      int8_t side,float rate_dps,float target_j,float integral_j,
      float initial_current_mA,float goal_current_mA,
      uint16_t brake_max_ms=kBrakeMaxMs,float prediction_bias_j=0.f,
      uint16_t excitation_max_ms=kMaxMs) {
  Decision d;
  if ((side!=1 && side!=-1) || !table.ready || !std::isfinite(rate_dps) || rate_dps<0 ||
      !std::isfinite(target_j) || target_j<=0 || !std::isfinite(integral_j) ||
      !std::isfinite(prediction_bias_j) ||
      !std::isfinite(initial_current_mA) || !std::isfinite(goal_current_mA) || goal_current_mA<=0 ||
      brake_max_ms>kBrakeMaxMs || excitation_max_ms>kMaxMs)
    return d;
  const auto c=coefficients(side);
  const float omega=rate_dps*kRadPerDeg;
  d.kinetic_j=c.free_omega2*omega*omega;
  d.loss_j=-c.free_offset_j; // effective fitted loss, not a measured collision loss
  d.free_j=freeEnergy(c,omega);d.target_j=target_j;
  const float support_energy=potential(geometry,geometry.outer_angle/kRadPerDeg);
  if (!std::isfinite(d.free_j) || !std::isfinite(support_energy) || target_j>support_energy || d.free_j>support_energy)
    return d;
  d.deficit_j=target_j-d.free_j;
  const float correction_limit=.5f*target_j;
  const float correction=std::fmax(-correction_limit,std::fmin(correction_limit,integral_j));
  // A positive prediction bias means that the measured peak previously had
  // more energy than the uncorrected model predicted for this same state.
  const float raw=d.deficit_j+correction-prediction_bias_j;
  d.unclamped_deficit_j=raw;
  d.unclamped_q_mAs=requiredSignedCharge(c,omega,raw);
  const bool braking=raw<0;
  const uint16_t max_ms=braking?brake_max_ms:excitation_max_ms;
  const float signed_goal=braking?-goal_current_mA:goal_current_mA;
  const float limit_q=charge(table,max_ms,initial_current_mA,signed_goal);
  if (!std::isfinite(limit_q)) return d;
  if (braking) {
    // The fitted work curve is monotone over the deliberately small reverse
    // charge interval. Cap it at the quadratic vertex for high current or
    // low rate, so the model never interprets stronger reverse current as
    // adding energy again.
    const float vertex_q=-500.f*c.work_omega_q*omega/c.work_q2;
    const float min_q=std::fmax(vertex_q,std::fmin(0.f,limit_q));
    const float min_work=std::fmin(0.f,work(c,omega,min_q));
    d.maximum_input_work_j=min_work;
    const float desired=std::fmax(raw,-d.free_j);
    d.commanded_deficit_j=std::fmax(desired,min_work);
    d.lower=raw<-d.free_j || desired<min_work;
    d.command_sign=-1;
    d.available_q_mAs=-min_q;
  } else {
    d.commanded_deficit_j=std::fmin(raw,support_energy-d.free_j);
    d.upper=raw>support_energy-d.free_j;
    d.command_sign=raw>0?1:0;
    d.available_q_mAs=std::fmax(0.f,limit_q);
    d.maximum_input_work_j=work(c,omega,d.available_q_mAs);
  }
  d.requested_q_mAs=requiredSignedCharge(c,omega,d.commanded_deficit_j);
  if (!std::isfinite(d.requested_q_mAs) || !std::isfinite(d.available_q_mAs)) return d;
  d.upper=d.upper || (!braking && d.requested_q_mAs>d.available_q_mAs);
  const float requested=braking?std::fmax(d.requested_q_mAs,-d.available_q_mAs):
      std::fmin(d.requested_q_mAs,d.available_q_mAs);
  d.corrected_target_j=d.free_j+work(c,omega,requested);
  d.predicted_j=d.free_j;d.energy_error_j=std::fabs(d.corrected_target_j-d.free_j);
  d.extrapolated=rate_dps<c.rate_min_dps || rate_dps>c.rate_max_dps ||
      requested<0 || requested>c.charge_max_mAs;
  d.valid=true;
  if (requested==0) { d.command_sign=0; return d; }
  // Once signed charge crosses the requested sign, it is monotone toward the
  // commanded current even when residual current initially points the other way.
  uint16_t lo=0,hi=max_ms;
  while (hi-lo>1) {
    const uint16_t mid=static_cast<uint16_t>((lo+hi)/2);
    ++d.evaluations;
    const float q_mid=charge(table,mid,initial_current_mA,signed_goal);
    if (braking ? q_mid>requested : q_mid<requested) lo=mid;
    else hi=mid;
  }
  const uint16_t candidates[3]={lo,hi,max_ms};
  for (uint16_t n:candidates) {
    ++d.evaluations;
    const float q=charge(table,n,initial_current_mA,signed_goal);
    if (braking ? (q>0 || q< -d.available_q_mAs) : q<0) continue;
    const float predicted=d.free_j+work(c,omega,q);
    if (!std::isfinite(predicted) || predicted<0 || predicted>support_energy) continue;
    const float error=std::fabs(predicted-d.corrected_target_j);
    if (error<d.energy_error_j || (error==d.energy_error_j && n<d.ms)) {
      d.ms=n;d.selected_q_mAs=q;d.predicted_j=predicted;d.energy_error_j=error;
    }
  }
  d.extrapolated=d.extrapolated || d.selected_q_mAs<0 || d.selected_q_mAs>c.charge_max_mAs;
  if (d.ms==0) d.command_sign=0;
  return d;
}

// 0.47.31 measured a next-peak angle map, not motor work.  Mapping that
// predicted turning point through gravitational potential gives a conditional
// *arrival energy*.  Its difference from the no-command arrival energy is an
// effective response, not a measurement of electrical or shaft work.
struct BridgeCoefficients {
  float at_65_deg, per_dps, gain_deg_per_mAs, rate_min_dps, rate_max_dps;
};
static inline RW_SPEED_CODE BridgeCoefficients bridgeCoefficients(int8_t side) {
  return side>0
      ? BridgeCoefficients{6.730502640f,.187441879f,.29032f,17.9508f,92.2064f}
      : BridgeCoefficients{7.691200794f,.194970900f,.25455f,35.7375f,91.4938f};
}
static inline RW_SPEED_CODE float bridgeFreeAngle(int8_t side,float rate_dps) {
  if ((side!=1 && side!=-1) || !std::isfinite(rate_dps) || rate_dps<0) return NAN;
  const auto c=bridgeCoefficients(side);
  return std::fmax(0.f,c.at_65_deg+c.per_dps*(rate_dps-65.f));
}
static inline RW_SPEED_CODE float bridgeArrivalEnergy(const Geometry& geometry,int8_t side,
      float rate_dps,float q_mAs) {
  if (!std::isfinite(q_mAs) || q_mAs<0) return NAN;
  const auto c=bridgeCoefficients(side);
  return potential(geometry,bridgeFreeAngle(side,rate_dps)+c.gain_deg_per_mAs*q_mAs);
}
// The inverse operates in arrival-energy coordinates.  The historical
// positive-input map is preserved; a negative Q is deliberately unavailable
// because the 0.47.31 gain did not identify reverse-command response.
static inline RW_SPEED_CODE Decision decideBridge(const Geometry& geometry,const CurrentTable& table,
      int8_t side,float rate_dps,float target_j,float integral_j,
      float initial_current_mA,float goal_current_mA,float prediction_bias_j=0.f,
      uint16_t excitation_max_ms=kMaxMs) {
  Decision d;
  if ((side!=1 && side!=-1) || !table.ready || !std::isfinite(rate_dps) || rate_dps<0 ||
      !std::isfinite(target_j) || target_j<=0 || !std::isfinite(integral_j) ||
      !std::isfinite(prediction_bias_j) || !std::isfinite(initial_current_mA) ||
      !std::isfinite(goal_current_mA) || goal_current_mA<=0 || excitation_max_ms>kMaxMs) return d;
  const auto c=bridgeCoefficients(side);
  const float free_angle=bridgeFreeAngle(side,rate_dps);
  const float outer_angle=geometry.outer_angle/kRadPerDeg;
  const float support_energy=potential(geometry,outer_angle);
  d.free_j=potential(geometry,free_angle);
  if (!std::isfinite(d.free_j) || !std::isfinite(support_energy) ||
      target_j>support_energy || free_angle>outer_angle || !(c.gain_deg_per_mAs>0)) return d;
  d.target_j=target_j;
  d.deficit_j=target_j-d.free_j;
  const float correction_limit=.5f*target_j;
  const float correction=std::fmax(-correction_limit,std::fmin(correction_limit,integral_j));
  const float raw=d.deficit_j+correction-prediction_bias_j;
  d.unclamped_deficit_j=raw;
  d.lower=raw<0;
  d.upper=target_j+correction-prediction_bias_j>support_energy;
  d.corrected_target_j=std::fmax(d.free_j,
      std::fmin(support_energy,target_j+correction-prediction_bias_j));
  d.commanded_deficit_j=d.corrected_target_j-d.free_j;
  const float desired_angle=angle(geometry,d.corrected_target_j);
  if (!std::isfinite(desired_angle)) return d;
  d.requested_q_mAs=std::fmax(0.f,(desired_angle-free_angle)/c.gain_deg_per_mAs);
  const float raw_target=target_j+correction-prediction_bias_j;
  d.unclamped_q_mAs=raw_target>=d.free_j && raw_target<=support_energy
      ? (angle(geometry,raw_target)-free_angle)/c.gain_deg_per_mAs : NAN;
  const float limit_q=charge(table,excitation_max_ms,initial_current_mA,goal_current_mA);
  if (!std::isfinite(limit_q)) return d;
  d.available_q_mAs=std::fmax(0.f,std::fmin(limit_q,
      (outer_angle-free_angle)/c.gain_deg_per_mAs));
  const float max_angle=free_angle+c.gain_deg_per_mAs*d.available_q_mAs;
  d.maximum_input_work_j=potential(geometry,max_angle)-d.free_j;
  d.upper=d.upper || d.requested_q_mAs>d.available_q_mAs;
  d.extrapolated=rate_dps<c.rate_min_dps || rate_dps>c.rate_max_dps ||
      d.requested_q_mAs>7.3f; // The 0.47.31 steady-state validation reached 7.21 mA*s.
  d.predicted_j=d.free_j;
  d.energy_error_j=std::fabs(d.corrected_target_j-d.free_j);
  d.valid=true;
  const float requested=std::fmin(d.requested_q_mAs,d.available_q_mAs);
  if (requested<=0) return d;
  uint16_t lo=0,hi=excitation_max_ms;
  while (hi-lo>1) {
    const uint16_t mid=static_cast<uint16_t>((lo+hi)/2);
    ++d.evaluations;
    if (charge(table,mid,initial_current_mA,goal_current_mA)<requested) lo=mid;
    else hi=mid;
  }
  for (uint16_t n : {lo,hi,excitation_max_ms}) {
    ++d.evaluations;
    const float q=charge(table,n,initial_current_mA,goal_current_mA);
    if (q<0 || q>d.available_q_mAs) continue;
    const float predicted=potential(geometry,free_angle+c.gain_deg_per_mAs*q);
    if (!std::isfinite(predicted)) continue;
    const float error=std::fabs(predicted-d.corrected_target_j);
    if (error<d.energy_error_j || (error==d.energy_error_j && n<d.ms)) {
      d.ms=n;d.selected_q_mAs=q;d.predicted_j=predicted;d.energy_error_j=error;
    }
  }
  if (d.ms>0) d.command_sign=1;
  return d;
}
static inline RW_SPEED_CODE float updateBridgeIntegral(float value,float error,float target,
      int8_t side,bool upper,bool lower,bool matched) {
  if (!matched || !std::isfinite(value) || !std::isfinite(error) ||
      !std::isfinite(target) || target<=0 || (side!=1 && side!=-1))
    return std::isfinite(value)?value:0;
  if ((upper && error>0) || (lower && error<0)) return value;
  // Near the target, dE/dA converts the old 0.1 mA*s/degree integral to
  // dE feedback with dimensionless gain 0.1*gain_deg_per_mAs.
  const float gain=.10f*bridgeCoefficients(side).gain_deg_per_mAs;
  const float limit=.5f*target;
  return std::fmax(-limit,std::fmin(limit,value+gain*error));
}
static inline RW_SPEED_CODE float updateIntegral(float value,float error,float target,
      bool upper,bool lower,bool matched) {
  if (!matched || !std::isfinite(value) || !std::isfinite(error) || !std::isfinite(target) || target<=0)
    return std::isfinite(value)?value:0;
  if ((upper && error>0) || (lower && error<0)) return value;
  const float limit=.5f*target;
  return std::fmax(-limit,std::fmin(limit,value+kIntegralGain*error));
}
} // namespace lateral_energy
