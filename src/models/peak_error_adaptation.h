#pragma once

#include <cmath>
#include <cstdint>
#include "realtime_code.h"

// Online correction of the energy model's next-peak residual. No fixed
// wheel-speed coefficient is inferred from the three brake-cap trials:
// their speed ranges and commands are confounded. A correction is used only
// after four consistent observations in the same physical side, command
// mode, and signed wheel-speed range during the current run.
namespace peak_error_adaptation {
constexpr uint32_t kLearnAfterMs = 5000;
constexpr uint32_t kMaxSpeedAgeUs = 50000;
constexpr float kMaxAbsSpeedRpm = 1000.f;
constexpr float kMaxAbsCorrectionJ = .000070f;
constexpr float kMaxTrainingResidualJ = .000200f;
constexpr float kSignThresholdJ = .000010f;
constexpr float kUpdateFraction = .25f;
constexpr uint8_t kRequiredConsistentObservations = 4;

// Modes are relative to the body's zero-cross motion, not motor polarity.
enum Mode : int8_t { Brake = -1, Coast = 0, Excite = 1 };
enum SpeedBin : uint8_t { ReverseFast = 0, Slow = 1, Forward = 2, ForwardFast = 3,
                          Invalid = 255 };

static inline RW_SPEED_CODE bool speedUsable(float rpm, bool valid, uint32_t age_us) {
  return valid && age_us <= kMaxSpeedAgeUs && std::isfinite(rpm) &&
      std::fabs(rpm) <= kMaxAbsSpeedRpm;
}
static inline RW_SPEED_CODE SpeedBin speedBin(float rpm) {
  return rpm < -100.f ? ReverseFast :
      (rpm < 100.f ? Slow : (rpm < 300.f ? Forward : ForwardFast));
}
static inline RW_SPEED_CODE Mode modeForSignedCharge(float q_mAs) {
  return q_mAs > 0.f ? Excite : (q_mAs < 0.f ? Brake : Coast);
}

struct Cell {
  float residual_j = 0.f;
  uint8_t consistent = 0;
  int8_t last_sign = 0;
};

struct Model {
  Cell cells[2][2][4]{};
  void reset() { *this = Model{}; }

  static inline RW_SPEED_CODE int sideIndex(int8_t side) { return side > 0 ? 1 : 0; }
  static inline RW_SPEED_CODE int modeIndex(Mode mode) { return mode == Excite ? 1 : 0; }

  RW_SPEED_CODE float correction(int8_t side, Mode mode, float rpm,
                                  bool valid, uint32_t age_us,
                                  uint8_t* bin_out = nullptr,
                                  uint8_t* count_out = nullptr) const {
    if (bin_out) *bin_out = Invalid;
    if (count_out) *count_out = 0;
    if ((side != 1 && side != -1) || mode == Coast ||
        !speedUsable(rpm, valid, age_us)) return 0.f;
    const SpeedBin bin = speedBin(rpm);
    const Cell& cell = cells[sideIndex(side)][modeIndex(mode)][bin];
    if (bin_out) *bin_out = bin;
    if (count_out) *count_out = cell.consistent;
    return cell.consistent >= kRequiredConsistentObservations ? cell.residual_j : 0.f;
  }

  void observe(uint32_t zero_cross_ms, int8_t side, Mode mode, float rpm,
               bool valid, uint32_t age_us, float measured_minus_model_j) {
    if (zero_cross_ms < kLearnAfterMs || (side != 1 && side != -1) || mode == Coast ||
        !speedUsable(rpm, valid, age_us) || !std::isfinite(measured_minus_model_j) ||
        std::fabs(measured_minus_model_j) > kMaxTrainingResidualJ) return;
    Cell& cell = cells[sideIndex(side)][modeIndex(mode)][speedBin(rpm)];
    const int8_t sign = measured_minus_model_j > kSignThresholdJ ? 1 :
        (measured_minus_model_j < -kSignThresholdJ ? -1 : 0);
    if (sign == 0) {
      cell = Cell{};
      return;
    }
    const float bounded = std::fmax(-kMaxAbsCorrectionJ,
        std::fmin(kMaxAbsCorrectionJ, measured_minus_model_j));
    if (sign != cell.last_sign) {
      cell.residual_j = bounded;
      cell.consistent = 1;
      cell.last_sign = sign;
    } else {
      cell.residual_j += kUpdateFraction * (bounded - cell.residual_j);
      if (cell.consistent < 255) ++cell.consistent;
    }
  }
};
} // namespace peak_error_adaptation
