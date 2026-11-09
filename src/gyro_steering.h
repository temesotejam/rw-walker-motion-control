#pragma once
#include <cmath>
#include <cstdint>

namespace steering {
constexpr float kRad = 0.017453292519943295f;
constexpr float kDeg = 57.29577951308232f;
constexpr uint32_t kSettleMs = 10000;
constexpr float kLimitDeg = 1.0f;
constexpr float kStepDeg = 0.08f;
constexpr float kYawGain = 0.12f; // desired peak difference / net yaw angle
constexpr float kTrackGain = 0.15f; // side target / peak-difference error per cycle
constexpr float kSmooth = 0.35f;
constexpr float kResponseDeltaDeg = 0.20f;
enum class Mode : uint8_t { Feedback=0, ResponseCheck=1 };
inline int responseFirstSign(uint16_t run_id) { return run_id == 0 || (run_id & 1U) ? 1 : -1; }
inline float clamp(float x, float lo, float hi) { return fmaxf(lo, fminf(hi, x)); }

// Independent body-to-world quaternion. Acceleration is used only at reset;
// fixed startup RAW gyro bias is supplied by the caller, never MEKF bias/yaw.
class GyroHeading {
 public:
  bool reset(float ax, float ay, float az, uint32_t us) {
    *this = GyroHeading{};
    const float n = ax*ax + ay*ay + az*az;
    if (!std::isfinite(n) || n < .25f || n > 2.25f) return false;
    const float roll = atan2f(ay, az), pitch = atan2f(-ax, sqrtf(ay*ay+az*az));
    const float cr=cosf(roll*.5f), sr=sinf(roll*.5f);
    const float cp=cosf(pitch*.5f), sp=sinf(pitch*.5f);
    w_=cr*cp; x_=sr*cp; y_=cr*sp; z_=-sr*sp;
    previous_us_=us; valid_=true;
    return true;
  }
  void update(float gx, float gy, float gz, uint32_t us) {
    if (!valid_ || us == previous_us_) return;
    const uint32_t dt = us-previous_us_; previous_us_=us;
    if (dt > 10000 || !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) {
      valid_=false; return; // latch until next run; no invented rotation across gaps
    }
    const float a=gx*kRad, b=gy*kRad, c=gz*kRad;
    const float h=static_cast<float>(dt)*.25e-6f;
    const float dx=(a+(have_rate_?px_:a))*h;
    const float dy=(b+(have_rate_?py_:b))*h;
    const float dz=(c+(have_rate_?pz_:c))*h;
    px_=a; py_=b; pz_=c; have_rate_=true;
    const float w=w_-x_*dx-y_*dy-z_*dz;
    const float x=x_+w_*dx+y_*dz-z_*dy;
    const float y=y_+w_*dy+z_*dx-x_*dz;
    const float z=z_+w_*dz+x_*dy-y_*dx;
    const float inv=1.0f/sqrtf(w*w+x*x+y*y+z*z);
    w_=w*inv; x_=x*inv; y_=y*inv; z_=z*inv;
    const float heading=atan2f(2*(w_*z_+x_*y_), 1-2*(y_*y_+z_*z_))*kDeg;
    if (!std::isfinite(heading)) { valid_=false; return; }
    float d=heading-last_wrapped_;
    if (d>180) d-=360;
    if (d< -180) d+=360;
    yaw_+=d; last_wrapped_=heading;
  }
  bool valid() const { return valid_; }
  float yaw() const { return valid_ ? yaw_ : NAN; }
 private:
  float w_=1,x_=0,y_=0,z_=0,px_=0,py_=0,pz_=0,yaw_=0,last_wrapped_=0;
  uint32_t previous_us_=0;
  bool valid_=false,have_rate_=false;
};

enum class Reason : uint8_t {
  Waiting=0, Settling=1, Active=2, Invalid=3, Saturated=4,
  ResponsePositive=5, ResponseNegative=6, ResponseReturn=7, Disabled=8
};
struct Snapshot {
  float yaw_deg=NAN, delta_deg=0, actual_difference_deg=NAN;
  float desired_difference_deg=NAN, cycle_yaw_rate_dps=NAN;
  uint16_t cycles=0;
  bool gyro_valid=false;
  Reason reason=Reason::Waiting;
};

// Keep the v53 diagnostic layout without claiming an unmeasured zero heading.
inline Snapshot disabledSnapshot() {
  Snapshot s;
  s.reason=Reason::Disabled;
  return s;
}

// Outer yaw loop requests a peak difference; the inner loop compares it with
// measured A+ - A-. A nonzero difference is allowed when it produces zero yaw.
class Controller {
 public:
  void reset(Mode mode=Mode::Feedback, uint16_t run_id=1) {
    *this=Controller{}; mode_=mode; first_sign_=responseFirstSign(run_id);
  }
  const Snapshot& state() const { return s_; }
  float target(float mean, int side) const { return mean+(side>0?s_.delta_deg:-s_.delta_deg); }
  void peak(int side, float amplitude, float yaw, uint32_t ms, bool upper, bool lower) {
    if (!std::isfinite(amplitude) || !std::isfinite(yaw) || (side!=1 && side!=-1)) {
      previous_side_=0; boundary_=false; filtered_=false;
      s_.desired_difference_deg=NAN; s_.reason=Reason::Invalid; return;
    }
    if (side==1) {
      if (boundary_ && previous_side_!= -1) {
        boundary_=false; filtered_=false;
        s_.desired_difference_deg=NAN; s_.reason=Reason::Invalid;
      }
      plus_=amplitude; plus_upper_=upper; plus_lower_=lower;
      plus_ms_=ms; previous_side_=1; return;
    }
    const uint32_t dt_ms=ms-boundary_ms_;
    const bool complete=boundary_ && previous_side_==1 && plus_ms_!=boundary_ms_ &&
        uint32_t(plus_ms_-boundary_ms_)<dt_ms && dt_ms>=300 && dt_ms<=3000;
    const float rotation=yaw-boundary_yaw_;
    boundary_ms_=ms; boundary_yaw_=yaw; boundary_=true; previous_side_=-1;
    if (!complete) {
      filtered_=false; s_.desired_difference_deg=NAN;
      s_.reason=Reason::Waiting; return;
    }
    const float dt=dt_ms*.001f;
    const float rate=rotation/dt, difference=plus_-amplitude;
    if (fabsf(rate)>90 || fabsf(difference)>15) {
      filtered_=false; s_.desired_difference_deg=NAN;
      s_.reason=Reason::Invalid; return;
    }
    ++s_.cycles;
    s_.cycle_yaw_rate_dps=filtered_?s_.cycle_yaw_rate_dps+kSmooth*(rate-s_.cycle_yaw_rate_dps):rate;
    s_.actual_difference_deg=filtered_?s_.actual_difference_deg+kSmooth*(difference-s_.actual_difference_deg):difference;
    filtered_=true;
    if (mode_==Mode::ResponseCheck) {
      // A bounded, predetermined input separates actuator response from yaw
      // feedback. Change only on a valid full cycle; keep the existing slew.
      const float requested = ms<kSettleMs || ms>=26000 ? 0.0f :
          (ms<18000 ? first_sign_ : -first_sign_)*kResponseDeltaDeg;
      s_.delta_deg=clamp(s_.delta_deg+clamp(requested-s_.delta_deg,-kStepDeg,kStepDeg),
                         -kResponseDeltaDeg,kResponseDeltaDeg);
      s_.desired_difference_deg=NAN; // no closed-loop D reference in this mode
      s_.reason=ms<kSettleMs ? Reason::Settling : ms>=26000 ? Reason::ResponseReturn :
          requested>0 ? Reason::ResponsePositive : Reason::ResponseNegative;
      return;
    }
    if (ms<kSettleMs || !std::isfinite(s_.desired_difference_deg)) {
      s_.desired_difference_deg=s_.actual_difference_deg;
      s_.reason=Reason::Settling; return;
    }
    // Observed sign: positive (left) yaw requires increasing A+ - A-.
    // Deadband is continuous; do not chase tiny net rotation or reverse abruptly.
    const float r=s_.cycle_yaw_rate_dps;
    const float effective_rate=r>0 ? fmaxf(0,r-.3f) : fminf(0,r+.3f);
    const float request=clamp(s_.desired_difference_deg+kYawGain*effective_rate*dt,-4,4);
    const float step=clamp(kTrackGain*(request-s_.actual_difference_deg),-kStepDeg,kStepDeg);
    // One side may still change the difference when the other side is limited.
    const bool blocked=(step>0 && ((plus_upper_ && lower) || s_.delta_deg>=kLimitDeg)) ||
        (step<0 && ((upper && plus_lower_) || s_.delta_deg<=-kLimitDeg));
    if (blocked) {
      // Tracking anti-windup: release immediately when rotation reverses.
      s_.desired_difference_deg=s_.actual_difference_deg;
      s_.reason=Reason::Saturated; return;
    }
    s_.delta_deg=clamp(s_.delta_deg+step,-kLimitDeg,kLimitDeg);
    // Bound outer-loop lead while the amplitude loop catches up.
    s_.desired_difference_deg=clamp(request,s_.actual_difference_deg-.5f,s_.actual_difference_deg+.5f);
    s_.reason=Reason::Active;
  }
 private:
  Snapshot s_;
  float plus_=0,boundary_yaw_=0;
  uint32_t boundary_ms_=0,plus_ms_=0;
  int previous_side_=0;
  bool boundary_=false,filtered_=false,plus_upper_=false,plus_lower_=false;
  Mode mode_=Mode::Feedback;
  int first_sign_=1;
};
}
