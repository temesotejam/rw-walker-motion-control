#pragma once
#include <stdint.h>

namespace control_math {

// Owned exclusively by ExperimentRunner's control task. Model coefficients are
// immutable; only exactly equal inputs may reuse a result. Voltage is not
// rounded here, so updatePulseModelPrediction's fresh voltage still takes effect.
class CurrentModelCache {
 public:
  template<class Compute>
  float goal(float absolute_command_mA, float voltage_v, Compute compute) {
    if (goal_valid_ && absolute_command_mA == goal_command_ && voltage_v == goal_voltage_)
      return goal_;
    goal_ = compute();
    goal_command_ = absolute_command_mA;
    goal_voltage_ = voltage_v;
    goal_valid_ = true;
    return goal_;
  }
  template<class Compute>
  float riseTime(float absolute_command_mA, Compute compute) {
    if (tau_valid_ && absolute_command_mA == tau_command_) return tau_;
    tau_ = compute();
    tau_command_ = absolute_command_mA;
    tau_valid_ = true;
    return tau_;
  }
 private:
  bool goal_valid_ = false, tau_valid_ = false;
  float goal_command_ = 0, goal_voltage_ = 0, goal_ = 0;
  float tau_command_ = 0, tau_ = 0;
};

struct WidthPrediction {
  float q_mA_s, energy_j;
};

// One instance per zero-cross decision. Target-independent charge and energy
// are shared by the two searches; target error and tie-breaking are not cached.
// Only the bitmap is initialized, not all 101 float pairs. No heap/PSRAM access.
template<unsigned MaxWidthMs>
class WidthPredictionCache {
 public:
  template<class Compute>
  WidthPrediction get(uint16_t width_ms, Compute compute) {
    if (width_ms > MaxWidthMs) return compute();
    const uint32_t mask = uint32_t{1} << (width_ms % 32U);
    auto& seen = present_[width_ms / 32U];
    if (!(seen & mask)) {
      values_[width_ms] = compute();
      seen |= mask;
    }
    return values_[width_ms];
  }
 private:
  uint32_t present_[(MaxWidthMs + 32U) / 32U] = {};
  WidthPrediction values_[MaxWidthMs + 1U];
};

}  // namespace control_math
