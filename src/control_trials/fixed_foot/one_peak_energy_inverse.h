#pragma once
#include <cmath>
#include <cstdint>

// Fixed-leg, negative-side, 300 mA model. Energies are mJ here because the
// regression was fitted in mJ; callers convert to J for the existing log.
namespace one_peak_energy_inverse {
constexpr char kRevision[] = "negative_one_peak_energy_20261021_fourteen_runs_v3_relative_reference";
constexpr uint16_t kWidthsMs[] = {0, 10, 15, 20, 25};
constexpr uint16_t kMaximumAdaptiveMs = 30;
constexpr float kLowOppositePeakDeg = 7.f;
constexpr float kPeakDeadbandDeg = .5f;

// peak_error_deg is target minus measured peak. A negative error means the
// negative-side swing is too large, so more braking is needed next cycle.
inline uint8_t nextAdaptiveExtraMs(uint8_t current_extra_ms, bool opposite_healthy,
                                   float measured_peak_deg, float peak_error_deg,
                                   uint16_t applied_width_ms) {
  if (!opposite_healthy || measured_peak_deg < 5.f) return 0;
  if (peak_error_deg < -kPeakDeadbandDeg && applied_width_ms >= 25 && current_extra_ms < 5)
    return current_extra_ms + 1;
  if (peak_error_deg > kPeakDeadbandDeg && current_extra_ms > 0)
    return current_extra_ms - 1;
  return current_extra_ms;
}

struct State {
  State() = default;
  State(float rate, float wheel, float previous, float battery, float reference)
      : rate_dps(rate), wheel_rpm(wheel), previous_peak_deg(previous),
        battery_mv(battery), reference_abs_pitch_deg(reference) {}
  float rate_dps = NAN;
  float wheel_rpm = NAN;
  float previous_peak_deg = NAN;
  float battery_mv = NAN;
  float reference_abs_pitch_deg = NAN;
};
struct Prediction {
  bool valid = false;
  float free_mj = NAN, on_mj = NAN, after_mj = NAN;
  float work_change_mj = NAN, energy_mj = NAN;
};
struct Choice {
  bool valid = false, below_reachable = false, above_reachable = false;
  bool extrapolated = false;
  uint16_t width_ms = 0;
  float free_mj = NAN, energy_mj = NAN, energy_error_mj = NAN;
};

inline bool supported(const State& s) {
  return std::isfinite(s.rate_dps) && s.rate_dps >= 57.2522f && s.rate_dps <= 76.2778f &&
      std::isfinite(s.wheel_rpm) && s.wheel_rpm >= 4.28f && s.wheel_rpm <= 550.04f &&
      std::isfinite(s.previous_peak_deg) && s.previous_peak_deg >= 5.0288f && s.previous_peak_deg <= 8.52741f &&
      std::isfinite(s.battery_mv) && s.battery_mv >= 7220.f && s.battery_mv <= 8000.f &&
      // Prediction uses angles relative to the saved upright pose. The MEKF
      // absolute pitch at that pose is not an input to the fitted equation.
      std::isfinite(s.reference_abs_pitch_deg);
}

inline Prediction predict(const State& s, uint16_t width_ms, float residual_mj = 0.f) {
  Prediction p;
  if (!supported(s) || width_ms > 25 || !std::isfinite(residual_mj)) return p;
  const float rate=(s.rate_dps-72.5f)/3.f;
  const float rpm=(s.wheel_rpm-80.f)/50.f;
  const float previous=(s.previous_peak_deg-8.f)/.5f;
  const float voltage=(s.battery_mv-7000.f)/1000.f;
  const float t=static_cast<float>(width_ms)/25.f;
  p.on_mj=t*(-.11374238610590627f) + t*t*(-.2137163629606094f) +
      t*rate*(-.0121297577897634f) + t*rpm*(.002750431639219765f);
  p.after_mj=.22670010497624232f + rate*.007915741915865374f +
      rpm*.01891249166239664f + t*(-.12244123666267166f) + t*t*.07027683583380318f;
  const float after_zero=.22670010497624232f + rate*.007915741915865374f +
      rpm*.01891249166239664f;
  p.work_change_mj=p.on_mj+p.after_mj-after_zero;
  p.free_mj=1.396542982188298f + rate*.0986473699053286f +
      rpm*(-.024852205530032204f) + previous*(-.04795626985252528f) +
      voltage*.004742852774672139f;
  p.energy_mj=p.free_mj+.42290164040184f*p.work_change_mj+residual_mj;
  p.valid=std::isfinite(p.energy_mj) && p.energy_mj >= 0.f;
  return p;
}

inline Choice choose(const State& s, float target_mj, float residual_mj,
                     uint8_t adaptive_extra_ms = 0, bool protect_opposite_peak = false) {
  Choice c;
  if (!supported(s) || !std::isfinite(target_mj) || target_mj <= 0.f ||
      !std::isfinite(residual_mj)) return c;
  float minimum=INFINITY, maximum=-INFINITY;
  for (uint16_t width : kWidthsMs) {
    const auto p=predict(s,width,residual_mj);
    if (!p.valid) return Choice{};
    minimum=std::fmin(minimum,p.energy_mj);
    maximum=std::fmax(maximum,p.energy_mj);
    const float error=std::fabs(p.energy_mj-target_mj);
    if (!c.valid || error<std::fabs(c.energy_error_mj) ||
        (error==std::fabs(c.energy_error_mj) && width<c.width_ms)) {
      c.valid=true;c.width_ms=width;c.free_mj=p.free_mj;
      c.energy_mj=p.energy_mj;c.energy_error_mj=p.energy_mj-target_mj;
    }
  }
  c.below_reachable=target_mj<minimum;
  c.above_reachable=target_mj>maximum;
  if (protect_opposite_peak) {
    const auto p=predict(s,0,residual_mj);
    c.width_ms=0;c.free_mj=p.free_mj;c.energy_mj=p.energy_mj;
    c.energy_error_mj=p.energy_mj-target_mj;
    return c;
  }
  if (c.below_reachable && c.width_ms==25 && adaptive_extra_ms>0) {
    const uint16_t extra=adaptive_extra_ms>5 ? 5 : adaptive_extra_ms;
    const auto p20=predict(s,20,residual_mj),p25=predict(s,25,residual_mj);
    // 25-30 ms has no calibrated work fit. The local slope is diagnostic; the
    // actual adjustment is bounded by completed-peak feedback in the runner.
    const float slope=std::fmin(0.f,(p25.energy_mj-p20.energy_mj)/5.f);
    c.width_ms=static_cast<uint16_t>(25+extra);
    c.energy_mj=p25.energy_mj+slope*extra;
    c.energy_error_mj=c.energy_mj-target_mj;
    c.extrapolated=true;
  }
  return c;
}
}  // namespace one_peak_energy_inverse
