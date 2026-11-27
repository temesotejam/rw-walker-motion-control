#pragma once

#include "walking/walking_energy_policy.h"

// Research model. Positive lateral motion is towards the CAD right foot.
// Camera-positive -> CAD-positive, absolute foot zeros and body alignment
// are hypotheses. Equality of sole heights does not measure load or slip.
namespace support_transfer_plan {
enum class Reason : uint8_t { Invalid, OutsideApproach, Coast, Wait, Drive, Insufficient, Complete };
enum class Phase : uint8_t { None, Approach, Receiving, Complete };
enum class Anchor : uint8_t { None, Bracketed, FirstReceiving };
constexpr float kRad = lateral_energy::kRadPerDeg;
constexpr float kMassKg = .1997f;
constexpr float kGravity = 9.80665f;
constexpr float kCgAboveHingeMm = 37.174236f;
constexpr float kExitDeg = 1.f;  // Travel into the receiving sole after equality.
constexpr float kCarryDps = 5.f;  // Trial residual forward-speed target at that event.
constexpr uint16_t kForecastLimitMs = 150;  // Validity of local motion extrapolation.
constexpr uint16_t kReplanMs = 25;

struct Input {
  int side = 0;  // Next body peak, NOT the departing peak or raw motor polarity.
  float angle_deg = NAN, rate_dps = NAN, accel_dps2 = NAN;
  float fore_aft_deg = NAN, left_deg = NAN, right_deg = NAN;
  float left_rate_dps = NAN, right_rate_dps = NAN;
  float initial_current_ma = NAN, goal_current_ma = NAN;  // Aligned to side.
  uint16_t cap_ms = 0;
};

struct Decision {
  Reason reason = Reason::Invalid;
  int side = 0;
  uint16_t delay_ms = 0, width_ms = 0;
  float transfer_ms = NAN, transfer_deg = NAN;
  float coast_margin_j = NAN, margin_j = NAN, work_j = NAN;
  bool work_supported = false;  // Rate and charge ranges only; transfer physics uncalibrated.
  Phase phase = Phase::None;
  Anchor anchor = Anchor::None;
  float terminal_deg = NAN, terminal_ms = NAN;
};

namespace detail {
struct Contact {float x_mm, z_mm;};
struct Pose {Contact left, right; float cg_x_mm, cg_z_mm;};

// Exact bounded minimum of the circular arc reconstructed in
// tools/analyze_energy_neutral_v14.py. Minimize z_world analytically over
// the 150 mm circle and the two longitudinal sole edges, skipping mesh scans.
inline Contact contact(bool right, float foot_pitch_deg, float s, float c) {
  const float cx = right ? .997904f : -1.002092f;
  const float a = foot_pitch_deg * kRad, ca = std::cos(a), sa = std::sin(a);
  const float cy = sa >= 0 ? -28.f : 27.f;
  const float tangent = 150.f * s / std::sqrt(s*s+c*c*ca*ca);
  const float u = right ? std::fmax(5.f,std::fmin(45.f,tangent))
                        : std::fmax(-45.f,std::fmin(-5.f,tangent));
  const float x = cx + u;
  const float z = sa*cy + ca*(67.09089f-std::sqrt(22500.f-u*u));
  return {c*x+s*z, -s*x+c*z};
}

inline Pose pose(float left, float right, float pitch, float roll) {
  const float s = std::sin(roll*kRad), c = std::cos(roll*kRad);
  const float cg = kCgAboveHingeMm*std::cos(pitch*kRad);
  return {contact(false,left+pitch,s,c), contact(true,right+pitch,s,c), s*cg, c*cg};
}

inline float gap(const Pose& p) {return p.left.z_mm-p.right.z_mm;}
inline float height(const Pose& p) {return p.cg_z_mm-std::fmin(p.left.z_mm,p.right.z_mm);}

inline Pose forecast(const Input& in, float ms) {
  const float t = ms*.001f;
  return pose(in.left_deg+in.left_rate_dps*t, in.right_deg+in.right_rate_dps*t,
              in.fore_aft_deg, in.angle_deg+in.rate_dps*t);
}

inline float withinAngle(float angle, float rate, float bound, float limit_ms) {
  if (rate == 0) return limit_ms;
  const float edge = std::copysign(bound,rate);
  return std::fmin(limit_ms,1000.f*(edge-angle)/rate);
}

// Nominal event clock uses current body rate and observed foot velocities.
// Energy feasibility uses observed acceleration separately; even a predicted
// coast stall can therefore have an event to plan towards. Candidate work
// does not retime the nominal foot path within this single shooting step.
inline bool findTransfer(const Input& in, float& crossing_ms, Pose& at, Pose& beyond) {
  const float exit_ms = 1000.f*kExitDeg/std::fabs(in.rate_dps);
  float limit = withinAngle(in.angle_deg,in.rate_dps,15.f,kForecastLimitMs);
  limit = withinAngle(in.left_deg,in.left_rate_dps,20.f,limit);
  limit = withinAngle(in.right_deg,in.right_rate_dps,20.f,limit);
  const float last = limit-exit_ms;
  if (!(last>0) || in.side*gap(forecast(in,0)) >= 0) return false;
  float lo = 0;
  while (lo < last) {
    float hi = std::fmin(lo+kReplanMs,last);
    if (in.side*gap(forecast(in,hi)) >= 0) {
      // 25 ms maximum bracket, uncertainty below 0.1 ms for the assumed path.
      for (unsigned i=0;i<8;++i) {
        const float mid = .5f*(lo+hi);
        if (in.side*gap(forecast(in,mid)) >= 0) hi=mid; else lo=mid;
      }
      crossing_ms=hi;
      at=forecast(in,hi);
      beyond=forecast(in,hi+exit_ms);
      return in.side*gap(beyond)>0;
    }
    lo=hi;
  }
  return false;
}

inline bool validGeometry(const Input& in) {
  return (in.side==1 || in.side==-1) && std::fabs(in.angle_deg)<=15 &&
    std::fabs(in.fore_aft_deg)<=10 && std::fabs(in.left_deg)<=20 && std::fabs(in.right_deg)<=20;
}

inline bool valid(const Input& in, const lateral_energy::CurrentTable& table) {
  return table.ready && std::isfinite(table.tau) && table.tau>0 && validGeometry(in) &&
    std::fabs(in.left_rate_dps)<=180 && std::fabs(in.right_rate_dps)<=180 &&
    std::isfinite(in.rate_dps) && std::isfinite(in.accel_dps2) &&
    std::isfinite(in.initial_current_ma) && std::isfinite(in.goal_current_ma) &&
    in.goal_current_ma>0 && in.cap_ms<=walking_energy_policy::kMaxPulseMs;
}
}  // namespace detail

inline float supportGap(float left_deg,float right_deg,float fore_aft_deg,float angle_deg) {
  return detail::gap(detail::pose(left_deg,right_deg,fore_aft_deg,angle_deg));
}

// At the inner lateral edge the unconstrained tangent of the circular sole
// reaches |u|=5 mm (radius 150 mm). This event is defined by the receiving
// foot's CAD arc and pitch, independently of body ZERO and load sensing.
inline float arcEntryDeg(int side, float receiving_pitch_deg) {
  const float ca=std::cos(receiving_pitch_deg*kRad);
  return side*std::atan2(5.f*ca,std::sqrt(22500.f-25.f))/kRad;
}

struct Progress {
  void observe(uint32_t peak_ms, uint32_t packet, const Input& in, bool geometry_valid) {
    if (side_!=in.side || peak_ms_!=peak_ms) {
      *this=Progress{};
      side_=in.side;
      peak_ms_=peak_ms;
    }
    if (complete) return;  // Latched to this half-cycle; there is no later anchor to find.
    if (!geometry_valid || !detail::validGeometry(in)) {
      previous_valid_=false;
      return;
    }
    const float gap=in.side*supportGap(in.left_deg,in.right_deg,in.fore_aft_deg,in.angle_deg);
    const float pitch=(in.side>0 ? in.right_deg : in.left_deg)+in.fore_aft_deg;
    if (anchor==Anchor::None && gap>=0) {
      float entry_pitch=pitch;
      // Only adjacent causal observations can bracket an arriving sole.
      if (previous_valid_ && packet-previous_packet_==1 && previous_gap_<0) {
        const float fraction=-previous_gap_/(gap-previous_gap_);
        entry_pitch=previous_pitch_+fraction*(pitch-previous_pitch_);
        anchor=Anchor::Bracketed;
      } else {
        anchor=Anchor::FirstReceiving;
      }
      terminal_deg=arcEntryDeg(in.side,entry_pitch);
    }
    if (anchor!=Anchor::None && gap>=0 && in.side*(in.angle_deg-terminal_deg)>=0)
      complete=true;
    previous_valid_=true;
    previous_gap_=gap;
    previous_pitch_=pitch;
    previous_packet_=packet;
  }

  Anchor anchor=Anchor::None;
  float terminal_deg=NAN;
  bool complete=false;
  bool matches(int side) const {return side==side_;}
 private:
  int side_=0;
  uint32_t peak_ms_=0, previous_packet_=0;
  float previous_gap_=0, previous_pitch_=0;
  bool previous_valid_=false;
};

namespace detail {
// Search the same calibrated current response for either phase. Energy up to
// the end of a candidate uses measured motion. Only a future impact attenuates
// its work; receiving-side motion has already undergone that transition.
inline Decision choosePulse(const lateral_energy::CurrentTable& current,const Input& in,
                            Decision out,float kinetic_scale,float retain,uint16_t deadline) {
  const float speed=in.side*in.rate_dps, accel=in.side*in.accel_dps2;
  out.margin_j=out.coast_margin_j;
  out.work_j=0;
  if (out.coast_margin_j>0) {out.reason=Reason::Coast;return out;}
  if (!in.cap_ms || !deadline) {out.reason=Reason::Insufficient;return out;}

  const auto coeff=lateral_energy::coefficients(in.side);
  const float fall_step=std::exp(-.001f/walking_energy_policy::kCoastFallTauS);
  bool sufficient=false;
  for (uint16_t delay=0; delay<deadline; delay+=kReplanMs) {
    const float speed2=speed*speed+2.f*accel*speed*.001f*delay;
    if (!(speed2>0)) break; // A later pulse cannot wake a stopped trajectory.
    const float start_speed=std::sqrt(speed2);
    const float i0=in.initial_current_ma*std::exp(-.001f*delay/walking_energy_policy::kCoastFallTauS);
    uint16_t limit=in.cap_ms<deadline-delay ? in.cap_ms : deadline-delay;
    // Once a sufficient pulse exists, only equal or shorter widths can win.
    if (sufficient && out.width_ms<limit) limit=out.width_ms;
    const float omega=start_speed*kRad;
    float coast_q=0, next_coast_q=i0*walking_energy_policy::kCoastFallTauS*(1.f-fall_step);
    for (uint16_t width=1;width<=limit;++width) {
      const float q=lateral_energy::charge(current,width,i0,in.goal_current_ma);
      coast_q+=next_coast_q; next_coast_q*=fall_step;
      // Acceleration already includes residual-current effects. Compare a
      // command with the decay of that same signed current if left unpowered.
      // No post-command tail is added to the empirical pulse-Q response.
      const float candidate=lateral_energy::work(coeff,omega,q);
      const float passive=lateral_energy::work(coeff,omega,coast_q);
      const float work=(q>=0 ? candidate : std::fmin(0.f,candidate))-
                       (coast_q>=0 ? passive : std::fmin(0.f,passive));
      const float prefix_j=kinetic_scale*(speed2+2.f*accel*speed*.001f*width)+work;
      if (!(prefix_j>0)) break; // The rising current must prevent an earlier stall.
      if (!(q>0)) continue;
      const float margin=out.coast_margin_j+retain*work;
      if (!std::isfinite(margin)) continue;
      const bool reaches=margin>0;
      // Minimum sufficient width, then prefer the later replan slot.
      // With no sufficient candidate retain the largest positive improvement.
      if (sufficient ? reaches && width<=out.width_ms : reaches || margin>out.margin_j) {
        sufficient=reaches; out.margin_j=margin; out.work_j=work;
        out.delay_ms=delay; out.width_ms=width;
        out.work_supported=start_speed>=coeff.rate_min_dps && start_speed<=coeff.rate_max_dps &&
                           q<=coeff.charge_max_mAs;
      }
      if (reaches) break;
    }
  }
  out.reason=sufficient ? (out.delay_ms ? Reason::Wait : Reason::Drive) : Reason::Insufficient;
  return out;
}

inline bool keepsReceivingPivot(const Input& in,float local_ms,const Pose& start,const Pose& end) {
  if (in.side*gap(end)<0) return false;
  if (in.left_rate_dps==0 && in.right_rate_dps==0) return true;
  // With fixed feet, rolling toward the receiving side only increases the
  // signed gap: its angular derivative is the horizontal right-left pivot
  // separation, positive throughout the model's +/-15 deg lateral range.
  // Pitch motion of either sole can lower its world height at most
  // (28 + 83*sin(30 deg)) mm/rad. This Lipschitz bound also holds at the
  // longitudinal edge switch and where the circle tangent hits an endpoint.
  const float fall_mm_per_ms=70.f*kRad*.001f*(std::fabs(in.left_rate_dps)+std::fabs(in.right_rate_dps));
  constexpr float roundoff_mm=.002f;
  const float initial_gap=in.side*gap(start);
  if (initial_gap>fall_mm_per_ms*local_ms+roundoff_mm) return true;
  // A returning foot can switch from toe to heel and back into load between
  // two same-sign endpoint gaps. Probe both exact edge-change times and the
  // smooth pieces at a finer cadence than the 25 ms camera publication.
  const auto changes_edge_on_other_pivot=[&](float foot,float rate) {
    if (rate==0) return false;
    const float ms=-1000.f*(foot+in.fore_aft_deg)/rate;
    return ms>0 && ms<local_ms && in.side*gap(forecast(in,ms))<0;
  };
  if (changes_edge_on_other_pivot(in.left_deg,in.left_rate_dps) ||
      changes_edge_on_other_pivot(in.right_deg,in.right_rate_dps)) return false;
  float last_gap=initial_gap;
  unsigned last_ms=0;
  for (unsigned ms=5;ms<local_ms;ms+=5) {
    // Skip only probes whose sign is already known. Retain the old 5 ms
    // sample positions near a possible reversal, including on the boundary.
    if (last_gap>fall_mm_per_ms*(ms-last_ms)+roundoff_mm) continue;
    last_gap=in.side*gap(forecast(in,ms));
    if (last_gap<0) return false;
    last_ms=ms;
  }
  return true;
}
}  // namespace detail

inline Decision select(const lateral_energy::CurrentTable& current,const Input& in) {
  Decision out; out.side=in.side;
  if (!detail::valid(in,current)) return out;
  const float speed = in.side*in.rate_dps, accel = in.side*in.accel_dps2;
  detail::Pose at{}, beyond{};
  if (!(speed>0) || !detail::findTransfer(in,out.transfer_ms,at,beyond)) {
    out.reason=Reason::OutsideApproach;
    return out;
  }
  const float travel = speed*out.transfer_ms*.001f;
  out.transfer_deg = in.angle_deg+in.side*travel;
  const auto from = in.side>0 ? at.left : at.right;
  const auto to = in.side>0 ? at.right : at.left;
  const auto exit_foot = in.side>0 ? beyond.right : beyond.left;
  const float ax=at.cg_x_mm-from.x_mm, az=at.cg_z_mm-from.z_mm;
  const float bx=at.cg_x_mm-to.x_mm, bz=at.cg_z_mm-to.z_mm;
  const float pre_r2=ax*ax+az*az, post_r2=bx*bx+bz*bz;
  const float alignment=ax*bx+az*bz;
  if (!(alignment>0 && pre_r2>0 && post_r2>0)) return out;
  // Inelastic handoff of a point mass: project its incoming tangential
  // velocity onto the new pivot tangent. Feet/roller inertia and slip omitted.
  const float retain=std::fmin(1.f,alignment*alignment/(pre_r2*post_r2));
  const float pre_scale=.5f*kMassKg*pre_r2*1e-6f*kRad*kRad;
  const float ex=beyond.cg_x_mm-exit_foot.x_mm, ez=beyond.cg_z_mm-exit_foot.z_mm;
  const float minimum_exit_j=.5f*kMassKg*(ex*ex+ez*ez)*1e-6f*kRad*kRad*kCarryDps*kCarryDps;
  const float delta_u_j=kMassKg*kGravity*.001f*(detail::height(beyond)-detail::height(at));
  // Required pre-impact energy covers reaching the boundary AND directional
  // carry at exit; a downhill after the boundary cannot fund the approach.
  const float required_pre_j=std::fmax(0.f,delta_u_j+minimum_exit_j)/retain;
  // Spatial work/energy form, v^2=v0^2+2*a*s. A negative value carries
  // the deficit past a predicted stop. Gravity before contact is already
  // present in measured acceleration, so do not add a CAD height gain here.
  const float coast_pre_j=pre_scale*(speed*speed+2.f*accel*travel);
  out.coast_margin_j=retain*(coast_pre_j-required_pre_j);
  if (!std::isfinite(out.coast_margin_j)) return Decision{};
  out.phase=Phase::Approach;
  out.terminal_deg=out.transfer_deg+in.side*kExitDeg;
  out.terminal_ms=out.transfer_ms+1000.f*kExitDeg/speed;
  return detail::choosePulse(current,in,out,pre_scale,retain,static_cast<uint16_t>(out.transfer_ms));
}

inline Decision select(const lateral_energy::CurrentTable& current,const Input& in,const Progress& progress) {
  if (!progress.matches(in.side) || progress.anchor==Anchor::None) {
    auto out=select(current,in);
    if (out.reason==Reason::OutsideApproach && in.side*in.rate_dps>0) out.phase=Phase::Approach;
    return out;
  }
  Decision out;
  out.side=in.side;
  out.anchor=progress.anchor;
  out.terminal_deg=progress.terminal_deg;
  if (!detail::valid(in,current)) return out;
  if (progress.complete) {
    out.phase=Phase::Complete;
    out.reason=Reason::Complete;
    out.terminal_ms=0;
    return out;
  }
  out.phase=Phase::Receiving;
  const float speed=in.side*in.rate_dps;
  const float travel=in.side*(out.terminal_deg-in.angle_deg);
  const auto now=detail::forecast(in,0);
  if (!(speed>0 && travel>0 && in.side*detail::gap(now)>=0)) {
    out.reason=Reason::OutsideApproach;
    return out;
  }
  out.terminal_ms=1000.f*travel/speed;
  // The spatial target is fixed. Limit extrapolation of the current motion
  // and the pulse search to the local time and foot-angle limits.
  float local_ms=std::fmin(out.terminal_ms,kForecastLimitMs);
  local_ms=detail::withinAngle(in.left_deg,in.left_rate_dps,20.f,local_ms);
  local_ms=detail::withinAngle(in.right_deg,in.right_rate_dps,20.f,local_ms);
  const float local_travel=std::fmin(travel,speed*local_ms*.001f);
  const auto local=detail::forecast(in,local_ms);
  if (!(local_ms>0) || !detail::keepsReceivingPivot(in,local_ms,now,local)) {
    // A forecast hand-back invalidates the single receiving-pivot hypothesis.
    out.reason=Reason::OutsideApproach;
    return out;
  }
  const auto pivot=in.side>0 ? now.right : now.left;
  const float x=now.cg_x_mm-pivot.x_mm, z=now.cg_z_mm-pivot.z_mm;
  const float scale=.5f*kMassKg*(x*x+z*z)*1e-6f*kRad*kRad;
  const float local_kinetic=scale*(speed*speed+2.f*in.side*in.accel_dps2*local_travel);

  // Measured acceleration already contains gravity and foot motion in the
  // local segment. Beyond it only gravity is modelled, with both feet held
  // at the final locally predicted pose; no distant velocity extrapolation.
  const float coast_left=in.left_deg+in.left_rate_dps*local_ms*.001f;
  const float coast_right=in.right_deg+in.right_rate_dps*local_ms*.001f;
  const float h_local=detail::height(local);
  float barrier_mm=0;
  for (float s=local_travel+1.f;s<travel;s+=1.f) {
    const auto p=detail::pose(coast_left,coast_right,in.fore_aft_deg,in.angle_deg+in.side*s);
    if (in.side*detail::gap(p)<0) {out.reason=Reason::OutsideApproach;return out;}
    barrier_mm=std::fmax(barrier_mm,detail::height(p)-h_local);
  }
  const auto terminal=detail::pose(coast_left,coast_right,in.fore_aft_deg,out.terminal_deg);
  if (in.side*detail::gap(terminal)<0) {out.reason=Reason::OutsideApproach;return out;}
  const auto exit_foot=in.side>0 ? terminal.right : terminal.left;
  const float ex=terminal.cg_x_mm-exit_foot.x_mm, ez=terminal.cg_z_mm-exit_foot.z_mm;
  const float carry_j=.5f*kMassKg*(ex*ex+ez*ez)*1e-6f*kRad*kRad*kCarryDps*kCarryDps;
  const float mg_mm=kMassKg*kGravity*.001f;
  const float required=std::fmax(mg_mm*barrier_mm,mg_mm*(detail::height(terminal)-h_local)+carry_j);
  out.coast_margin_j=local_kinetic-required;
  if (!std::isfinite(out.coast_margin_j)) return Decision{};
  return detail::choosePulse(current,in,out,scale,1.f,static_cast<uint16_t>(local_ms));
}
}  // namespace support_transfer_plan
