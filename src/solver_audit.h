#pragma once
#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include "direct_q_solver.h"
#include "compact_json_table.h"
#include <string.h>

// One writer (run-control), stopped-only reader (RWLOG export).
// No allocation, formatting or model evaluation in push()/clear().
namespace solver_audit {
static_assert(sizeof(float) == 4, "Replay requires float32");
struct Record {
  uint32_t index = 0;
  uint32_t t_test_ms = 0;
  uint32_t start_us = 0;
  uint32_t end_us = 0;
  uint32_t decision_us = 0;
  uint32_t free_model_us = 0;
  uint32_t solver_start_us = 0;
  uint32_t solver_end_us = 0;
  uint32_t solver_us = 0;
  uint32_t fast_solver_us = 0;
  uint32_t ff_search_us = 0;
  uint32_t selected_search_us = 0;
  uint32_t gyro_sequence = 0;
  uint32_t sample_time_us = 0;
  uint32_t entry_sample_age_us = 0;
  uint32_t exit_sample_age_us = 0;
  uint32_t pulse_id_before = 0;
  uint32_t pulse_id_after = 0;
  uint16_t model_vbat_mV = 0;
  uint16_t ff_width_ms = 65535;
  uint16_t fast_selected_width_ms = 65535;
  uint16_t selected_width_ms = 65535;
  uint16_t ff_eval_count = 0;
  uint16_t selected_eval_count = 0;
  uint16_t eval_count = 0;
  uint8_t stage = 0;
  uint8_t outcome_reason = 0;
  uint8_t state_at_exit = 0;
  uint8_t sample_time_valid = 0;
  uint8_t solver_started = 0;
  uint8_t solver_complete = 0;
  uint8_t output_executed = 0;
  uint8_t event_valid = 0;
  uint8_t ff_valid = 0;
  uint8_t selected_valid = 0;
  int8_t physical_side = 0;
  int8_t command_direction = 0;
  float i0_mA = NAN;
  float free_peak_deg = NAN;
  float target_peak_deg = NAN;
  float target_energy_j = NAN;
  float passive_energy_j = NAN;
  float q_available_mA_s = NAN;
  float integral_mA_s = NAN;
  float signed_target_current_mA = NAN;
  float tau_s = NAN;
  float base_gain = NAN;
  float correction_c = NAN;
  float correction_gain = NAN;
  float correction_limit = NAN;
  float ff_q_mA_s = NAN;
  float selected_q_mA_s = NAN;
  float corrected_target_energy_j = NAN;
};
inline uint32_t floatBits(float value) { uint32_t bits; memcpy(&bits, &value, 4); return bits; }
template<class F> class ScopeExit {
 public:
  explicit ScopeExit(F& fn) : fn_(fn) {}
  ~ScopeExit() { fn_(); }
  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;
 private:
  F& fn_;
};
template<unsigned Capacity> class Buffer {
 public:
  static_assert(Capacity > 0, "Nonzero capacity required");
  void clear() { count_ = next_ = total_ = 0; }
  void push(const Record& input) {
    records_[next_] = input;
    records_[next_].index = ++total_;
    next_ = (next_ + 1U) % Capacity;
    if (count_ < Capacity) ++count_;
  }
  uint32_t count() const { return count_; }
  uint32_t overwritten() const { return total_ - count_; }
  const Record& at(unsigned index) const {
    return records_[((count_ == Capacity ? next_ : 0U) + index) % Capacity];
  }
  template<class Output> bool appendJson(Output& json) const {
    json += "{\"schema_version\":2,\"available\":true,\"solver_revision\":\"" + String(direct_q::kRevision) + "\",";
    json += "\"policy\":\"diagnostic_only;elapsed_wall_time_includes_preemption;no_legacy_online;stopped_export\",";
    json += "\"capacity\":" + String(Capacity) + ",\"count\":" + String(count_);
    json += ",\"overwritten\":" + String(overwritten());
    json += ",\"stage_legend\":\"0=before_solver,1=ff_failed,2=corrected_target_failed,3=selected_failed,4=selected_complete\",";
    json += "\"timing_scope\":\"decision=start_to_return_before_audit_copy;solver=cached_setup_through_selection;ff=analytic_inverse;selected=charge_inverse_and_final_energy;sample_age=consumed_host_sample_not_sensor_timestamp\",";
    json += "\"inverse_policy\":\"continuous_ff_clipped_before_side_integral;minimum_absolute_Q_error;integer_ms;ties_shorter;both_residual_current_branches\",";
    json += "\"field_semantics\":\"ff_width_ms=65535_not_computed;ff_eval_count=0;eval_count=nonzero_width_charge_integral_calls;q_ff_energy_mA_s=continuous_clipped_inverse\",";
    json += "\"float_fields\":[\"i0_mA\",\"free_peak_deg\",\"target_peak_deg\",\"target_energy_j\",\"passive_energy_j\",\"q_available_mA_s\",\"integral_mA_s\",\"signed_target_current_mA\",\"tau_s\",\"base_gain\",\"correction_c\",\"correction_gain\",\"correction_limit\",\"ff_q_mA_s\",\"selected_q_mA_s\",\"corrected_target_energy_j\"],\"events\":";
    CompactJsonTable<Output> table(json);
    for (unsigned i = 0; i < count_; ++i) {
      const Record& e = at(i);
      String row;
      row += "{";
      row += "\"index\":" + String(e.index);
      row += ",\"t_test_ms\":" + String(e.t_test_ms);
      row += ",\"start_us\":" + String(e.start_us);
      row += ",\"end_us\":" + String(e.end_us);
      row += ",\"decision_us\":" + String(e.decision_us);
      row += ",\"free_model_us\":" + String(e.free_model_us);
      row += ",\"solver_start_us\":" + String(e.solver_start_us);
      row += ",\"solver_end_us\":" + String(e.solver_end_us);
      row += ",\"solver_us\":" + String(e.solver_us);
      row += ",\"fast_solver_us\":" + String(e.fast_solver_us);
      row += ",\"ff_search_us\":" + String(e.ff_search_us);
      row += ",\"selected_search_us\":" + String(e.selected_search_us);
      row += ",\"gyro_sequence\":" + String(e.gyro_sequence);
      row += ",\"sample_time_us\":" + String(e.sample_time_us);
      row += ",\"entry_sample_age_us\":" + String(e.entry_sample_age_us);
      row += ",\"exit_sample_age_us\":" + String(e.exit_sample_age_us);
      row += ",\"pulse_id_before\":" + String(e.pulse_id_before);
      row += ",\"pulse_id_after\":" + String(e.pulse_id_after);
      row += ",\"model_vbat_mV\":" + String(e.model_vbat_mV);
      row += ",\"ff_width_ms\":" + String(e.ff_width_ms);
      row += ",\"fast_selected_width_ms\":" + String(e.fast_selected_width_ms);
      row += ",\"selected_width_ms\":" + String(e.selected_width_ms);
      row += ",\"ff_eval_count\":" + String(e.ff_eval_count);
      row += ",\"selected_eval_count\":" + String(e.selected_eval_count);
      row += ",\"eval_count\":" + String(e.eval_count);
      row += ",\"stage\":" + String(e.stage);
      row += ",\"outcome_reason\":" + String(e.outcome_reason);
      row += ",\"state_at_exit\":" + String(e.state_at_exit);
      row += ",\"sample_time_valid\":" + String(e.sample_time_valid);
      row += ",\"solver_started\":" + String(e.solver_started);
      row += ",\"solver_complete\":" + String(e.solver_complete);
      row += ",\"output_executed\":" + String(e.output_executed);
      row += ",\"event_valid\":" + String(e.event_valid);
      row += ",\"ff_valid\":" + String(e.ff_valid);
      row += ",\"selected_valid\":" + String(e.selected_valid);
      row += ",\"physical_side\":" + String(e.physical_side);
      row += ",\"command_direction\":" + String(e.command_direction);
      row += ",\"f32_bits\":[";
      row += String(floatBits(e.i0_mA));
      row += "," + String(floatBits(e.free_peak_deg));
      row += "," + String(floatBits(e.target_peak_deg));
      row += "," + String(floatBits(e.target_energy_j));
      row += "," + String(floatBits(e.passive_energy_j));
      row += "," + String(floatBits(e.q_available_mA_s));
      row += "," + String(floatBits(e.integral_mA_s));
      row += "," + String(floatBits(e.signed_target_current_mA));
      row += "," + String(floatBits(e.tau_s));
      row += "," + String(floatBits(e.base_gain));
      row += "," + String(floatBits(e.correction_c));
      row += "," + String(floatBits(e.correction_gain));
      row += "," + String(floatBits(e.correction_limit));
      row += "," + String(floatBits(e.ff_q_mA_s));
      row += "," + String(floatBits(e.selected_q_mA_s));
      row += "," + String(floatBits(e.corrected_target_energy_j));
      row += "]}";
      if (!table.append(row)) return false;
    }
    const bool ok=table.finish();
    json += "}";
    return ok;
  }
 private:
  Record records_[Capacity];
  uint32_t count_ = 0, next_ = 0, total_ = 0;
};
}  // namespace solver_audit
