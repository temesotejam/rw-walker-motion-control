#include "compact_json_table.h"
#include "gyro_steering.h"
#include "psram_logger.h"
#include "negative_probe_batch.h"
#include "negative_probe_extended_batch.h"
#include "positive_current_batch.h"
#include "imu_manager.h"
extern ImuManager imu;
#include "run_control_worker.h"
extern RunControlWorker run_control;
#include "roller485_manager.h"
extern Roller485Manager roller;
#if defined(FOOT_EARLY_TRIAL) && FOOT_EARLY_TRIAL && !(defined(ANGLE_EARLY_TRIAL) && ANGLE_EARLY_TRIAL)
#include "early_geometry_diagnostics.h"
#endif
#include "foot_observer.h"
extern FootObserver feet;
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
#include "experiment_runner.h"
extern ExperimentRunner runner;
#endif

#include <string.h>
#include <new>
#include <errno.h>


#include "config.h"
#include "lateral_energy_control.h"
#include "one_peak_energy_inverse.h"
#include "wheel_work_energy_control.h"
#include "walking_energy_loss.h"

static constexpr uint16_t RWLOG_FORMAT_VERSION = 56;
static constexpr uint32_t RWLOG_FLAG_CRC32 = 1U << 0;
namespace {
String jsonFloatOrNull(float value, unsigned int decimals) {
  return isfinite(value) ? String(value, decimals) : String("null");
}
const char* e2ShadowInvalidReasonName(uint8_t reason) {
  switch (reason) {
    case Config::E2_SHADOW_INVALID_NONE: return "VALID";
    case Config::E2_SHADOW_INVALID_H_OUT_OF_CALIBRATION_RANGE:
      return "INVALID_OUT_OF_CALIBRATION_RANGE";
    case Config::E2_SHADOW_INVALID_BRAKING_NOT_IDENTIFIED:
      return "INVALID_BRAKING_NOT_IDENTIFIED";
    case Config::E2_SHADOW_INVALID_SIDE_MISMATCH: return "INVALID_SIDE_MISMATCH";
    case Config::E2_SHADOW_INVALID_Q_OUT_OF_CALIBRATION_RANGE:
      return "INVALID_OUT_OF_CALIBRATION_RANGE";
    case Config::E2_SHADOW_INVALID_ABSOLUTE_PEAK_ESTIMATOR_UNVALIDATED:
      return "INVALID_ABSOLUTE_PEAK_ESTIMATOR_UNVALIDATED";
    case Config::E2_SHADOW_INVALID_NONALTERNATING_TURN:
      return "INVALID_NONALTERNATING_TURN";
    case Config::E2_SHADOW_INVALID_INSUFFICIENT_TURNS:
      return "INVALID_INSUFFICIENT_TURNS";
    default: return "INVALID_NONFINITE_STATE";
  }
}
const char* q1ShadowInvalidReasonName(uint8_t reason) {
  switch (reason) {
    case Config::Q1_SHADOW_INVALID_NONE: return "VALID";
    case Config::Q1_SHADOW_INVALID_NONFINITE_STATE: return "INVALID_NONFINITE_STATE";
    case Config::Q1_SHADOW_INVALID_RATE_BELOW_SUPPORT: return "INVALID_RATE_BELOW_SUPPORT";
    case Config::Q1_SHADOW_INVALID_RATE_ABOVE_SUPPORT: return "INVALID_RATE_ABOVE_SUPPORT";
    case Config::Q1_SHADOW_INVALID_BRAKING_NOT_IDENTIFIED:
      return "INVALID_BRAKING_NOT_IDENTIFIED";
    case Config::Q1_SHADOW_INVALID_Q_BELOW_SUPPORT: return "INVALID_Q_BELOW_SUPPORT";
    case Config::Q1_SHADOW_INVALID_Q_ABOVE_SUPPORT: return "INVALID_Q_ABOVE_SUPPORT";
    default: return "INVALID_NONFINITE_STATE";
  }
}
const char* qIdentInvalidReasonName(uint8_t reason) {
  switch (reason) {
    case Config::Q_IDENT_INVALID_NONE: return "VALID";
    case Config::Q_IDENT_INVALID_ARM_WAITING: return "ARM_WAITING";
    case Config::Q_IDENT_INVALID_ARMED_EVENT_NO_OUTPUT: return "ARMED_EVENT_NO_OUTPUT";
    case Config::Q_IDENT_INVALID_RATE_BELOW_SUPPORT: return "INVALID_RATE_BELOW_SUPPORT";
    case Config::Q_IDENT_INVALID_RATE_ABOVE_SUPPORT: return "INVALID_RATE_ABOVE_SUPPORT";
    case Config::Q_IDENT_INVALID_SCHEDULE_EXHAUSTED: return "INVALID_SCHEDULE_EXHAUSTED";
    case Config::Q_IDENT_INVALID_Q_ZERO_NO_PULSE: return "VALID_Q_ZERO_NO_PULSE";
    case Config::Q_IDENT_INVALID_ROLLER_NOT_READY: return "INVALID_ROLLER_NOT_READY";
    case Config::Q_IDENT_INVALID_BATTERY_GUARD: return "INVALID_BATTERY_GUARD";
    case Config::Q_IDENT_INVALID_PULSE_WIDTH_GUARD: return "INVALID_PULSE_WIDTH_GUARD";
    case Config::Q_IDENT_INVALID_SOLVER_NONFINITE: return "INVALID_SOLVER_NONFINITE";
    case Config::Q_IDENT_INVALID_CURRENT_WRITE_FAILED: return "INVALID_CURRENT_WRITE_FAILED";
    case Config::Q_IDENT_INVALID_ESTOP_OR_STATE: return "INVALID_ESTOP_OR_STATE";
    default: return "INVALID_UNKNOWN";
  }
}
const char* energyControlV0ReasonName(uint8_t reason) {
  switch (reason) {
    case Config::ENERGY_CONTROL_V0_INVALID_NONE: return "VALID";
    case Config::ENERGY_CONTROL_V0_VALID_PASSIVE_NO_OUTPUT: return "VALID_PASSIVE_NO_OUTPUT";
    case Config::ENERGY_CONTROL_V0_INVALID_NONFINITE_STATE: return "INVALID_NONFINITE_STATE";
    case Config::ENERGY_CONTROL_V0_INVALID_POTENTIAL_DOMAIN: return "INVALID_POTENTIAL_DOMAIN";
    case Config::ENERGY_CONTROL_V0_INVALID_ROLLER_NOT_READY: return "INVALID_ROLLER_NOT_READY";
    case Config::ENERGY_CONTROL_V0_INVALID_BATTERY_GUARD: return "INVALID_BATTERY_GUARD";
    case Config::ENERGY_CONTROL_V0_INVALID_PULSE_WIDTH_GUARD: return "INVALID_PULSE_WIDTH_GUARD";
    case Config::ENERGY_CONTROL_V0_INVALID_CURRENT_WRITE_FAILED: return "INVALID_CURRENT_WRITE_FAILED";
    case Config::ENERGY_CONTROL_V0_INVALID_ESTOP_OR_STATE: return "INVALID_ESTOP_OR_STATE";
    case Config::ENERGY_CONTROL_V0_INVALID_WAIT_INITIAL_EXCURSION: return "WAIT_INITIAL_EXCURSION";
    case Config::ENERGY_CONTROL_V0_INVALID_WAIT_REARM_EXCURSION: return "WAIT_REARM_EXCURSION";
    case Config::ENERGY_CONTROL_V0_INVALID_NONALTERNATING_SIDE: return "NONALTERNATING_SIDE";
    case Config::ENERGY_CONTROL_V0_INVALID_EVENT_LOG_OVERFLOW: return "INVALID_EVENT_LOG_OVERFLOW";
    default: return "INVALID_UNKNOWN";
  }
}
const char* energyControlAutonomousReasonName(uint8_t reason) {
  switch (reason) {
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE: return "VALID";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT: return "VALID_NO_OUTPUT";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_STRONG_START_KICK_EXECUTED: return "STRONG_START_KICK_EXECUTED";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_EVENT_LOG_OVERFLOW: return "INVALID_EVENT_LOG_OVERFLOW";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE: return "INVALID_NONFINITE_STATE";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_POTENTIAL_DOMAIN: return "INVALID_POTENTIAL_DOMAIN";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY: return "INVALID_ROLLER_NOT_READY";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_BATTERY_GUARD: return "INVALID_BATTERY_GUARD";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_PULSE_WIDTH_GUARD: return "INVALID_PULSE_WIDTH_GUARD";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED: return "INVALID_CURRENT_WRITE_FAILED";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_Q_BELOW_MIN_PULSE: return "VALID_Q_BELOW_MIN_PULSE";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE: return "INVALID_ESTOP_OR_STATE";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_WORK_OUTSIDE_SUPPORT: return "WORK_OUTSIDE_MEASURED_SUPPORT";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_WORK_PROTECT_OPPOSITE: return "WORK_PROTECT_OPPOSITE_PEAK";
    default: return "INVALID_UNKNOWN";
  }
}
}  // namespace
bool PsramLogger::begin() {
  ready_ = false;
  if (!psramFound()) {
    last_error_ = "psram_not_found";
    return false;
  }

  if (!events_) {
    void* storage = ps_malloc(sizeof(EventStorage));
    if (storage) events_ = new (storage) EventStorage();
  }
  if (!events_) { last_error_ = "event_psram_allocation_failed"; return false; }
  sample_capacity_ = Config::LOG_BUFFER_BYTES / sizeof(LogSample);
  samples_ = static_cast<LogSample*>(ps_malloc(sample_capacity_ * sizeof(LogSample)));
  if (!samples_) {
    last_error_ = "psram_allocation_failed";
    return false;
  }

  // Optional diagnostics must not consume internal task/driver RAM. No
  // allocation occurs during a run. Failure is explicit in the export and
  // never changes the existing controller, IMU guards, or actuator authority.
  if (!solver_audit_) {
    void* audit_storage = ps_malloc(sizeof(solver_audit::Buffer<128>));
    if (audit_storage) solver_audit_ = new (audit_storage) solver_audit::Buffer<128>();
  }
  clear();
  ready_ = true;
  last_error_ = "";
  return true;
}

void PsramLogger::clear() {
  if (downloading_) return;
  sealed_ = true;
  sample_count_ = 0;
  current_run_id_ = 0;
  run_start_us_ = 0;
  run_current_mA_ = 0;
  run_pulse_width_ms_ = 0;
  run_input_interval_ms_ = 0;
  identification_mode_ = false;
  q_run_mode_ = 0;
  q_probe_schedule_id_ = 0;
  control_target_cdeg_ = 0;
  passive_capture_ = false;
  q1_shadow_target_peak_abs_deg_ = NAN;
  q_ident_mode_ = false;
  q_ident_run_schedule_id_ = 0;
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  autonomous_timing_compensation_us_ = 0;
  autonomous_input_peak_percent_ = 0;
  identification_event_count_ = 0;
  calibration_peak_event_count_ = 0;
  calibration_probe_event_count_ = 0;
  calibration_state_gate_event_count_ = 0;
  calibration_build_up_event_count_ = 0;
  e2_shadow_peak_event_count_ = 0;
  e2_shadow_peak_event_overflow_ = false;
  q1_shadow_event_count_ = 0;
  q1_shadow_event_overflow_ = false;
  q_ident_event_count_ = 0;
  q_ident_event_overflow_ = false;
  energy_control_v0_event_count_ = 0;
  energy_control_v0_event_overflow_ = false;
  energy_control_autonomous_peak_event_count_ = 0;
  energy_control_autonomous_zero_cross_event_count_ = 0;
  energy_control_autonomous_event_overflow_ = false;
  solver_shadow_event_count_ = 0;
  solver_shadow_event_overflow_ = false;
  if (solver_audit_) solver_audit_->clear();
  timing_probe_event_count_ = 0;
  timing_probe_event_overflow_ = false;
  calibration_result_ = CalibrationResult{};
  last_measurement_done_ = false;
}

void PsramLogger::startRun(uint16_t run_id, uint64_t run_start_us, int16_t current_mA, uint16_t pulse_width_ms,
                             uint16_t input_interval_ms, bool identification_mode,
                             uint8_t q_run_mode, int16_t control_target_cdeg,
                             uint8_t q_probe_schedule_id, bool passive_capture,
                             float q1_shadow_target_peak_abs_deg, bool q_ident_mode,
                             uint8_t q_ident_run_schedule_id, bool energy_control_v0_mode,
                             bool energy_control_autonomous_mode, float input_peak_percent,
                             uint16_t brake_max_ms, bool peak_adaptation_enabled,
                             bool energy_bridge_enabled, bool negative_probe_mode,
                             const upright_reference::Record& upright_reference,
                             uint32_t negative_probe_batch_seed) {
  if (!ready_ || downloading_) return;
  sealed_ = false;
  sample_count_ = 0;
  current_run_id_ = run_id;
  run_start_us_ = run_start_us;
  run_current_mA_ = current_mA;
  run_pulse_width_ms_ = pulse_width_ms;
  run_input_interval_ms_ = input_interval_ms;
  identification_mode_ = identification_mode;
  q_run_mode_ = q_run_mode;
  q_probe_schedule_id_ = q_probe_schedule_id;
  control_target_cdeg_ = control_target_cdeg;
  passive_capture_ = passive_capture;
  q1_shadow_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg;
  q_ident_mode_ = q_ident_mode;
  q_ident_run_schedule_id_ = q_ident_run_schedule_id;
  energy_control_v0_mode_ = energy_control_v0_mode;
  energy_control_autonomous_mode_ = energy_control_autonomous_mode;
  autonomous_timing_compensation_us_ = 0;
  autonomous_input_peak_percent_ = energy_control_autonomous_mode ? input_peak_percent : 0;
  autonomous_brake_max_ms_ = energy_control_autonomous_mode ? brake_max_ms : lateral_energy::kBrakeMaxMs;
  peak_adaptation_enabled_ = peak_adaptation_enabled;
  energy_bridge_enabled_ = energy_bridge_enabled;
  negative_probe_mode_ = negative_probe_mode;
  negative_probe_batch_seed_ = negative_probe_batch_seed;
  run_upright_reference_ = upright_reference;
  run_lateral_cg_ = lateral_cg::Record{};
  run_adaptive_cg_ = lateral_cg::AdaptiveSnapshot{};
  measurement_start_abs_deg_ = NAN;
  identification_event_count_ = 0;
  calibration_peak_event_count_ = 0;
  calibration_probe_event_count_ = 0;
  calibration_state_gate_event_count_ = 0;
  calibration_build_up_event_count_ = 0;
  e2_shadow_peak_event_count_ = 0;
  e2_shadow_peak_event_overflow_ = false;
  q1_shadow_event_count_ = 0;
  q1_shadow_event_overflow_ = false;
  q_ident_event_count_ = 0;
  q_ident_event_overflow_ = false;
  energy_control_v0_event_count_ = 0;
  energy_control_v0_event_overflow_ = false;
  energy_control_autonomous_peak_event_count_ = 0;
  energy_control_autonomous_zero_cross_event_count_ = 0;
  energy_control_autonomous_event_overflow_ = false;
  solver_shadow_event_count_ = 0;
  solver_shadow_event_overflow_ = false;
  if (solver_audit_) solver_audit_->clear();
  timing_probe_event_count_ = 0;
  timing_probe_event_overflow_ = false;
  calibration_result_ = CalibrationResult{};
  last_measurement_done_ = false;
}

uint16_t PsramLogger::beginIdentificationEvent(const IdentificationEvent& event) {
  if (sealed_ || !events_) return 0;
  if (!identification_mode_ || identification_event_count_ >= kMaxIdentificationEvents) return 0;
  IdentificationEvent stored = event;
  stored.event_id = identification_event_count_ + 1;
  events_->identification_events_[identification_event_count_++] = stored;
  return stored.event_id;
}

void PsramLogger::finishIdentificationEvent(uint16_t event_id, uint32_t peak_ms, int16_t theta_peak_cdeg) {
  if (sealed_ || !events_) return;
  if (event_id == 0 || event_id > identification_event_count_) return;
  IdentificationEvent& event = events_->identification_events_[event_id - 1];
  event.peak_ms = peak_ms;
  event.theta_peak_cdeg = theta_peak_cdeg;
  event.peak_detected = true;
}

void PsramLogger::addCalibrationPeakEvent(const CalibrationPeakEvent& event) {
  if (sealed_ || !events_) return;
  if (calibration_peak_event_count_ >= kMaxCalibrationPeakEvents) return;
  CalibrationPeakEvent stored = event;
  stored.peak_index = calibration_peak_event_count_ + 1;
  events_->calibration_peak_events_[calibration_peak_event_count_++] = stored;
}

void PsramLogger::addCalibrationProbeEvent(const CalibrationProbeEvent& event) {
  if (sealed_ || !events_) return;
  if (calibration_probe_event_count_ >= kMaxCalibrationProbeEvents) return;
  CalibrationProbeEvent stored = event;
  stored.probe_index = calibration_probe_event_count_ + 1;
  events_->calibration_probe_events_[calibration_probe_event_count_++] = stored;
}

void PsramLogger::addCalibrationStateGateEvent(const CalibrationStateGateEvent& event) {
  if (sealed_ || !events_) return;
  if (calibration_state_gate_event_count_ >= kMaxCalibrationStateGateEvents) return;
  CalibrationStateGateEvent stored = event;
  stored.event_index = calibration_state_gate_event_count_ + 1;
  events_->calibration_state_gate_events_[calibration_state_gate_event_count_++] = stored;
}

void PsramLogger::addCalibrationBuildUpEvent(const CalibrationBuildUpEvent& event) {
  if (sealed_ || !events_) return;
  if (calibration_build_up_event_count_ >= kMaxCalibrationBuildUpEvents) return;
  CalibrationBuildUpEvent stored = event;
  stored.pulse_index = calibration_build_up_event_count_ + 1;
  events_->calibration_build_up_events_[calibration_build_up_event_count_++] = stored;
}

void PsramLogger::addE2ShadowPeakEvent(const E2ShadowPeakEvent& event) {
  if (sealed_ || !events_) return;
  if (e2_shadow_peak_event_count_ >= kMaxE2ShadowPeakEvents) {
    e2_shadow_peak_event_overflow_ = true;
    return;
  }
  E2ShadowPeakEvent stored = event;
  stored.event_index = e2_shadow_peak_event_count_ + 1;
  events_->e2_shadow_peak_events_[e2_shadow_peak_event_count_++] = stored;
}

void PsramLogger::addQ1ShadowEvent(const Q1ShadowEvent& event) {
  if (sealed_ || !events_) return;
  if (q1_shadow_event_count_ >= kMaxQ1ShadowEvents) {
    q1_shadow_event_overflow_ = true;
    return;
  }
  Q1ShadowEvent stored = event;
  stored.q1_shadow_event_index = q1_shadow_event_count_ + 1;
  events_->q1_shadow_events_[q1_shadow_event_count_++] = stored;
}
void PsramLogger::addQIdentEvent(const QIdentEvent& event) {
  if (sealed_ || !events_) return;
  if (q_ident_event_count_ >= kMaxQIdentEvents) {
    q_ident_event_overflow_ = true;
    return;
  }
  QIdentEvent stored = event;
  stored.q_ident_event_index = q_ident_event_count_ + 1;
  events_->q_ident_events_[q_ident_event_count_++] = stored;
}
void PsramLogger::addEnergyControlV0Event(const EnergyControlV0Event& event) {
  if (sealed_ || !events_) return;
  if (energy_control_v0_event_count_ >= kMaxEnergyControlV0Events) {
    energy_control_v0_event_overflow_ = true;
    return;
  }
  EnergyControlV0Event stored = event;
  stored.event_index = energy_control_v0_event_count_ + 1;
  events_->energy_control_v0_events_[energy_control_v0_event_count_++] = stored;
}
void PsramLogger::addEnergyControlAutonomousPeakEvent(const EnergyControlAutonomousPeakEvent& event) {
  if (sealed_ || !events_) return;
  if (energy_control_autonomous_peak_event_count_ >= kMaxEnergyControlAutonomousEvents) {
    energy_control_autonomous_event_overflow_ = true;
    return;
  }
  EnergyControlAutonomousPeakEvent stored = event;
  stored.peak_index = energy_control_autonomous_peak_event_count_ + 1;
  events_->energy_control_autonomous_peak_events_[energy_control_autonomous_peak_event_count_++] = stored;
}
void PsramLogger::addEnergyControlAutonomousZeroCrossEvent(const EnergyControlAutonomousZeroCrossEvent& event) {
  if (sealed_ || !events_) return;
  if (energy_control_autonomous_zero_cross_event_count_ >= kMaxEnergyControlAutonomousEvents) {
    energy_control_autonomous_event_overflow_ = true;
    return;
  }
  EnergyControlAutonomousZeroCrossEvent stored = event;
  stored.event_index = energy_control_autonomous_zero_cross_event_count_ + 1;
  events_->energy_control_autonomous_zero_cross_events_[energy_control_autonomous_zero_cross_event_count_++] = stored;
}
void PsramLogger::addSolverShadowEvent(const SolverShadowEvent& event) {
  if (sealed_ || !events_) return;
  if (solver_shadow_event_count_ >= kMaxSolverShadowEvents) {
    solver_shadow_event_overflow_ = true;
    return;
  }
  SolverShadowEvent stored = event;
  stored.event_index = solver_shadow_event_count_ + 1;
  events_->solver_shadow_events_[solver_shadow_event_count_++] = stored;
}
void PsramLogger::addTimingProbeEvent(const TimingProbeEvent& event) {
  if (sealed_ || !events_) return;
  if (timing_probe_event_count_ >= kMaxTimingProbeEvents) {
    timing_probe_event_overflow_ = true;
    return;
  }
  TimingProbeEvent stored = event;
  stored.event_index = timing_probe_event_count_ + 1;
  events_->timing_probe_events_[timing_probe_event_count_++] = stored;
}
void PsramLogger::setCalibrationResult(const CalibrationResult& result) {
  if (sealed_ || !events_) return;
  calibration_result_ = result;
}
void PsramLogger::markMeasurementDone() {
  last_measurement_done_ = true;
}

bool PsramLogger::addSample(const LogSample& row) {
  if (!ready_ || sealed_ || downloading_ || sample_count_ >= sample_capacity_) return false;
  samples_[sample_count_++] = row;
  return true;
}

uint8_t PsramLogger::usagePercent() const {
  if (sample_capacity_ == 0) return 0;
  return static_cast<uint8_t>((sample_count_ * 100ULL) / sample_capacity_);
}

bool PsramLogger::warningLevel() const {
  return usagePercent() >= Config::BUFFER_WARNING_PERCENT;
}

bool PsramLogger::rwlogDownloadable() const {
  return ready_ && sealed_ && last_measurement_done_ && run_start_us_ != 0 && sample_count_ > 0;
}

void PsramLogger::downloadFilename(char* out, size_t out_len) const {
  if (!out || out_len == 0) return;
  const char* prefix = energy_control_autonomous_mode_ ? "energy_control_autonomous_run" :
      (energy_control_v0_mode_ ? "energy_control_v0_run" :
      (q_ident_mode_ ? "q_ident_fixed_schedule_run" :
       (passive_capture_ ? "passive_absolute_roll_run" : "dynamic_beta_holdcompare_zerocross_run")));
  snprintf(out, out_len, "%s_%u_%llu.rwlog", prefix, current_run_id_,
           static_cast<unsigned long long>(run_start_us_));
}

size_t PsramLogger::psramTotal() const {
  return ESP.getPsramSize();
}

size_t PsramLogger::psramFree() const {
  return ESP.getFreePsram();
}

bool PsramLogger::buildMetadataJson(PsramString& json) const {

  // Stopped-only compact export. Maximum-capacity serializer regression must
  // fit the hard budget; fixed PSRAM reservation never grows into internal RAM.
  static constexpr size_t kMetadataJsonHardBudgetBytes = 640U * 1024U;
  json.clear();
  if (!json.ok()) return false;
  json += "{";
  json += "\"format\":\"" + String(energy_control_autonomous_mode_ ? "rwlog_energy_control_autonomous" :
      (energy_control_v0_mode_ ? "rwlog_energy_control_v0" :
      (q_ident_mode_ ? "rwlog_q_ident_fixed_schedule" :
       (passive_capture_ ? "rwlog_passive_absolute_roll_free_decay" : "rwlog_dynamic_beta_vbat_hold_time_compare")))) + "\",";
  json += "\"firmware_revision\":\"" RUNTIME_VERSION "\",";
#if defined(FOOT_EARLY_TRIAL) && FOOT_EARLY_TRIAL
  json += "\"diagnostic_build_id\":\"" RUNTIME_DIAGNOSTIC_BUILD_ID "\",";
#if defined(SWAY_BALANCE_TRIAL) && SWAY_BALANCE_TRIAL
  json += "\"sway_balance_trial\":{\"policy\":\"target8;fresh_paired_positive_peak_ge6p5_negative_le4p5_gap_ge2;positive_ordinary_proposal_cap_reduced_up_to5ms;floor20ms;existing_guards;no_rescue_or_early_change\",\"physical_response_identified\":false},";
#endif
#if defined(APPROACH_RESCUE_TRIAL) && APPROACH_RESCUE_TRIAL
  json += "\"approach_stall_rescue_trial\":{\"policy\":\"both_returns;inward_speed_cross_25dps_sustained_6ms_three_samples;angle_2p5_to_4deg;other_existing_rescue_gates_and_15ms_cap;15dps_and_6dps_rescues_remain_available;one_attempt_per_peak\",\"event_kind\":\"approach_inward_stall_rescue\",\"physical_response_identified\":false,\"hardware_validated\":false},";
#endif
#if defined(FAST_CROSSING_TRIAL) && FAST_CROSSING_TRIAL
  json += "\"fast_crossing_trial\":{\"policy\":\"target8;both_recent_paired_peaks_ge4p5deg;ordinary_abs_crossing_rate_ge44dps;proposal_ge30ms;cap_reduced_5ms;no_double_trim;no_rescue_or_early_change;existing_guards\",\"physical_response_identified\":false,\"hardware_validated\":false},";
#endif
#if defined(FAST_CROSSING_SHADOW) && FAST_CROSSING_SHADOW
  json += "\"fast_crossing_shadow\":{\"policy\":\"candidate_only;not_applied_to_output;other_guards_remain_active\",\"physical_response_identified\":false,\"hardware_validated\":false},";
#endif
#if defined(ANGLE_EARLY_TRIAL) && ANGLE_EARLY_TRIAL
  json += "\"angle_early_trial\":{\"policy\":\"inward_abs_angle_gt_0p3_le_1p5_deg;rate_gt_5_dps;constant_rate_eta_gt_20_le_30_ms;max_pulse_15ms;one_per_peak;no_send_zero_reevaluation;ambiguous_write_estop\",\"camera_role\":\"diagnostic_and_existing_balance_only\",\"geometry_worker_enabled\":false,\"applies_during_startup_ramp\":true,\"configured_target_deg\":8,\"pulse_width_changes_alongside_timing\":true,\"physical_timing_validated\":false},";
#else
  const auto early = g_early_diag.snapshot();
  json += "\"foot_early_trial\":{\"exposure\":\"unverified_frame_timestamp_plus_minus_20ms\",\"pitch_origin\":\"initial_pose_plus_minus_2deg_unverified\",\"geometry\":\"discrete_hypotheses_not_contact_ground_truth\",";
  json += "\"run\":" + String(early.run) + ",\"frames\":" + String(early.frames) + ",\"invalid_frames\":" + String(early.invalid_frames);
  json += ",\"submitted\":" + String(early.submitted) + ",\"submit_failed\":" + String(early.submit_failed) + ",\"pose_dropped\":" + String(early.pose_dropped);
  json += ",\"replies\":" + String(early.replies) + ",\"valid\":" + String(early.valid) + ",\"armed\":" + String(early.armed) + ",\"stale_replies\":" + String(early.stale_replies);
  json += ",\"worker_ready\":" + String(early.worker_ready) + ",\"max_compute_us\":" + String(early.max_compute_us) + ",\"max_step_us\":" + String(early.max_step_us) + ",\"min_stack_raw\":" + String(early.min_stack_raw);
  json += ",\"reason_codes\":[";
  for (unsigned i=0;i<early_diag::kReasonCount;++i) { if(i)json+=","; json+=String(early.reason[i]); }
  json += "],\"reason_names\":[\"none\",\"unknown_exposure\",\"provisional_disabled\",\"bad_exposure_interval\",\"pose_unavailable\",\"bad_identity\",\"bad_time\",\"bad_input\",\"stale_generation\",\"expired\",\"no_root\",\"disagree\"],\"async_reason_codes\":[";
  for (unsigned i=0;i<early_diag::kAsyncReasonCount;++i) { if(i)json+=","; json+=String(early.async_reason[i]); }
  json += "],\"async_reason_names\":[\"none\",\"pose_timeout\",\"replaced\",\"generation_changed\",\"calculation_rejected\",\"cpu_budget\"]},";
#endif // ANGLE_EARLY_TRIAL
#endif
  json += "\"wheel_cycle_trial\":{\"policy\":\"branch15_midpoint_guard;four_fresh_matched_alternating_peaks_at_least_4p5deg_with_pair_gap_at_most_2deg;no_extra_shortening_on_weaker_or_declining_side;above_100rpm_at_most_5ms_extra;no_reverse_or_added_torque;15ms_startup_recovery\",\"hardware_validated\":false},";
  json += "\"foot_camera_required_for_autonomous_start\":false,";
  json += "\"steering\":{\"revision\":\"amplitude_only_04730\",\"enabled\":false,\"profile\":\"disabled\",\"feedback_enabled\":false,\"heading_enabled\":false,\"response_check_enabled\":false,\"targets\":\"selected_target_for_both_sides;command_target_latched_until_response_peak\",\"previous_peak_target_gate\":\"disabled_in_04731;retired_8deg_residual\",\"v53_fields\":\"gyro_heading_diagnostic_only;difference_unavailable;delta_and_cycles_zero\",\"gyro_heading_diagnostic\":\"relative_from_start_sync;startup_bias;3_axis;no_magnetometer;drifts;rows_lag_one_sample;invalid_on_gap_over_10ms;never_control\",\"reason_codes\":{\"8\":\"disabled\"}},";
  const auto placement = RuntimeDiag::executionSnapshot();
  json += "\"execution_placement\":{\"revision\":\"core_isolation_04724\",\"camera_init_call_core\":" + String(placement.camera_init_core);
  json += ",\"camera_deinit_call_core\":" + String(placement.camera_deinit_core);
  json += ",\"wifi_event_observed_core\":" + String(placement.wifi_event_core);
  json += ",\"wifi_event_configured_core\":0,\"observation_scope\":\"boot_to_export;minus_one_means_not_observed;camera_call_core_not_ISR_trace\"},";
  json += "\"attitude_validation_revision\":\"" + String(Config::ATTITUDE_VALIDATION_REVISION) + "\",";
  json += "\"amplitude_control_observation_revision\":\"" + String(Config::AMPLITUDE_CONTROL_OBSERVATION_REVISION) + "\",";
  json += "\"v46ak_pre_input_observation_semantics\":\"independent_current_and_speed_snapshots_before_command;speed_register_0x60_x100_rpm;fresh_speed_selects_adaptation_cell;current_diagnostic_only\",";
  json += "\"measurement_mode\":\"" + String(energy_control_autonomous_mode_ ? Config::ENERGY_CONTROL_AUTONOMOUS_MEASUREMENT_MODE :
      (energy_control_v0_mode_ ? Config::ENERGY_CONTROL_V0_MEASUREMENT_MODE :
      (q_ident_mode_ ? Config::Q_IDENT_MEASUREMENT_MODE :
       (passive_capture_ ? "manual_passive_free_decay" : "legacy")))) + "\",";
  json += "\"energy_control_v0_policy\":\"sole_actual_output_path;frozen_q1_passive_plus_side_gain_and_p1_step_energy;Q_IDENT_and_all_legacy_paths_fail_closed\",";
  json += "\"energy_control_v0_output_gate\":\"independent_from_Q1_detector;WAIT_INITIAL_EXCURSION->ARMED_FOR_ZERO_CROSS->WAIT_OPPOSITE_EXCURSION\",";
  json += "\"energy_control_v0_rearm_angle_coordinate\":\"q1_detector_relative_angle_deg;continuous_detector_coordinate_only;physical_side_is_sign_of_plus_gy\",";
  json += "\"energy_control_v0_potential\":\"P1_STEP_gap;R=0.150m;inner_edge=0.005m;outer_edge=0.045m;cg_height=0.120m;mass=0.1997kg;g=9.80665m/s2\",";
  json += "\"energy_control_v0_potential_role\":\"geometry_potential_only;not_a_J_eff_or_dissipation_fit\",";
  json += "\"passive_motor_command_policy\":\"passive_capture_only:all_recorded_samples_command_0mA_and_output_off\",";
  json += "\"energy_control_v0_output_policy\":\"sole_actual_output_path;accepted_zero_cross_only;no_braking;all_other_paths_fail_closed\",";
  const bool inverse_mode = energy_bridge_enabled_ && !negative_probe_mode_ &&
      autonomous_brake_max_ms_ == 25;
  const bool work_mode = energy_bridge_enabled_ && !negative_probe_mode_ &&
      autonomous_brake_max_ms_ == 40;
  json += "\"energy_model\":\"" + String(work_mode ? "wheel_work_one_peak_control" : inverse_mode ? "one_peak_energy_inverse_negative" :
      energy_bridge_enabled_ ? "angle_energy_bridge" : "quadratic_energy_inverse") + "\",";
  if (work_mode) {
    json += "\"work_target_deg\":" + String(control_target_cdeg_/100.f,2) + ",";
    json += "\"work_target_experimental\":" + String(control_target_cdeg_==800 ? "false" : "true") + ",";
    json += "\"work_target_transfer_policy\":\"8deg_coefficients_retained;10_12deg_bounded_experimental_envelopes;target_potential_recomputed;no_current_or_pulse_limit_increase;no_claim_of_new_target_validation\",";
    json += "\"work_target_rate_ceiling_plus_dps\":" + String(work_target_policy::rateCeiling(control_target_cdeg_/100.f,1),4) + ",";
    json += "\"work_target_rate_ceiling_minus_dps\":" + String(work_target_policy::rateCeiling(control_target_cdeg_/100.f,-1),4) + ",";
    json += "\"wheel_work_model_revision\":\"" + String(control_target_cdeg_==1200 ? "target12_state_work_20261027" : wheel_work_energy_control::kRevision) + "\",";
    json += "\"wheel_work_model_scope\":\"fixed_leg;target12_state_work_3runs155events_with_smooth_legacy_transition;targets8_10_and_fallback_keep_historical_models;see_target12_state_model_for_coverage\",";
    json += "\"wheel_work_broad_negative_support\":\"rate_57.2522_to_76.2778dps;wheel_4.28_to_575.79rpm;battery_6210_to_8120mV;previous_peak_5.0288_to_8.56178deg;marginal_ranges_not_full_joint_coverage;opposite_peak_below_7deg_skips_braking\",";
    json += "\"wheel_work_negative_feedback\":\"targets8_10_legacy;target12_common_coast_brake_assist_context_residual;continuous_q_zero;CG_reexpression;context_cap_histories\",";
    json += "\"wheel_work_boundary_policy\":\"positive_energy_blend;negative_context_guard;cap_scales_with_support;freshness_and_voltage_gates;branch7_excluded_from_learning;cross_condition\",";
    json += "\"wheel_work_full_charge_calibration\":\"free_energy_minus_0.05972251687mJ_times_clamp((battery_mV-8000)/100,0,1);22_coasts_one_run;high_voltage_brake_unmeasured\",";
    json += "\"wheel_work_positive_current_candidates_mA\":[300,350,400],";
    json += "\"wheel_work_positive_feedback\":\"CG_reexpressed_side_energy_bias;targets8_10_legacy_side_residuals;target12_calibrated_context_weighted_rpm_voltage_residuals;CG_reexpression;latest_local_positive_underprediction_guard;reset_each_run\",";
    json += "\"wheel_work_negative_assist_policy\":\"target10_legacy_transferred_gain;target12_measured_signed_work_slopes_rpm_voltage;seed_caps25_to40ms;context_evidence_extensions5ms_max100ms;300mA;Q25mAs;voltage7000_to8120mV;assist_excluded_from_CG_and_brake_cap_learning\",";
    json += "\"wheel_work_target12_positive_guard\":\"context_weighted_energy_residuals;shrink_sparse_updates;large_underprediction_uses_context_weighted_latest_residual;over15deg_or_outside_scaled_residual_limit_coasts_until_accepted_peak;16deg_stop_unchanged\",";
    json += "\"wheel_work_target12_negative_policy\":\"state_work_model;continuous_coast_at_zero_charge;signed_dose_slopes;rpm_voltage_context_residuals;context_cap_evidence;overshoot_reduces_nearby_cap;seed25_to40ms_then5ms_to100ms;Q25mAs;16deg_stop\",";
    json += "\"gradual_start\":{\"revision\":\"paired_peaks_side_target_v3\",\"scope\":\"legacy_target8_fallback\",\"policy\":\"initial_4.5deg;two_fresh_opposite_peak_pairs_with_both_abs_rpm_at_most_500;increase_0.5deg_to8;normal_model_cap_80_to100ms;hold_on_invalid_or_high_speed;strong_start_kick_unchanged;tilt_priority_has_separate_ramp\",\"physical_left_sign_verified\":false,\"hardware_validated\":false},";
    json += "\"positive_left_boost\":{\"revision\":\"foot_feedback_v1\",\"branch14_automatic_trial_enabled\":false,\"ordinary_walking_continues\":true,\"limit_scope\":\"target8_wheel_work_mode\",\"current_limit_mA\":1200,\"pulse_limit_ms\":150,\"physical_left_assumption\":\"positive_mekf_pitch_using_mounting_axis_hypothesis\",\"sign_independently_verified\":false,\"foot_feedback\":\"relative_foot_motion_not_ground_displacement;response_and_physical_side_mapping_unverified\",\"historic_trial_branch\":14,\"hardware_validated\":false},";
    json += "\"tilt_priority\":{\"revision\":\"gradual_sway_unmodeled_v2\",\"scope\":\"target8_each_side\",\"branch\":15,\"command_mA\":1200,\"initial_target_deg\":4.5,\"final_target_deg\":8,\"target_step_deg\":0.5,\"initial_width_ms\":60,\"max_width_ms\":150,\"speed_guard\":{\"signal\":\"zero_cross_abs_rate_dps\",\"full_width_rate_per_target_deg\":4,\"taper_full_rate_per_target_deg\":12,\"taper_endpoint_ms\":20,\"policy\":\"linear_width_reduction_then20ms;hardware_timing_caps_may_be_below20ms;invalid_rate_fallback\",\"interpretation\":\"trial_thresholds_not_validated_foot_speed_or_energy_limits\"},\"policy\":\"two_valid_paired_cycles_raise_target_by_0p5deg;two_under_target_at_width_limit_raise_side_width_10ms;overshoot_reduces_width_10ms_or_coasts;peak_timing_caps_width;bounded_startup_recovery\",\"event_semantics\":\"coast_has_zero_current_and_width;drive_records_command_not_measurement;startup_positive_cap_ms_is_target_side_limit\",\"priority\":\"before_normal_energy_selector;camera_relative_trial_adjusts_positive_width;inward_stall_rescue_caps_arrival\",\"prediction\":\"unmodeled_pulse_no_Q_or_energy_prediction;not_used_for_model_learning\",\"physical_side_mapping\":\"plus_minus_to_left_right_not_verified\",\"current_semantics\":\"command_not_a_measured_current\",\"hardware_validated\":false},";
#if defined(SYMMETRIC_RESCUE_TRIAL) && SYMMETRIC_RESCUE_TRIAL
#if defined(EARLY_RESCUE_TRIAL) && EARLY_RESCUE_TRIAL
    json += "\"early_stall_rescue_trial\":{\"policy\":\"both_returns;inward_speed_cross_15dps_sustained_6ms_three_samples;angle_0p8_to_2p5deg;other_existing_rescue_gates_and_15ms_cap;6dps_rescue_remains_available;one_attempt_per_peak\",\"event_kind\":\"early_inward_stall_rescue\",\"physical_response_identified\":false,\"hardware_validated\":false},";
#endif
    json += "\"foot_timing\":{\"revision\":\"zero_centre_symmetric_inward_stall_rescue_v2_trial\",\"scope\":\"target8_zero_percent_both_returns\",\"policy\":\"normal_zero_centre;inward_speed_cross_6dps_sustained_6ms_three_samples;absolute_angle_0p35_to_3p25deg;peak_minus_angle_at_least2deg;200_to_800ms_after_peak;fresh_samples_at_most10ms_apart;one_per_peak;healthy_existing_gates;max15ms_at_1200mA;no_fallback_to_longer_pulse\",\"event_kind\":\"inward_stall_rescue\",\"physical_response_identified\":false,\"hardware_validated\":false},";
#else
    json += "\"foot_timing\":{\"revision\":\"zero_centre_inward_stall_rescue_v1\",\"scope\":\"target8_zero_percent_positive_return_only\",\"policy\":\"normal_zero_centre;inward_speed_cross_minus6dps_sustained_6ms_three_samples;angle_0p35_to_3p25deg;peak_minus_angle_at_least2deg;200_to_800ms_after_peak;fresh_samples_at_most10ms_apart;one_per_peak;healthy_existing_gates;max15ms_at_1200mA;no_fallback_to_longer_pulse\",\"event_kind\":\"inward_stall_rescue\",\"physical_response_identified\":false,\"hardware_validated\":false},";
#endif
    json += "\"foot_span_high_current\":{\"revision\":\"three_adjacent_pairs_trial_loss_v5\",\"scope\":\"target8_branch15_positive_drive\",\"command_mA\":1200,\"minimum_stage_target_deg\":6,\"valid_cycles\":6,\"valid_pairs\":3,\"max_age_ms\":12000,\"proposal_age_ms\":1500,\"step_ms\":5,\"max_offset_ms\":20,\"trial_cap_ms\":100,\"policy\":\"three_pairs_of_adjacent_valid_cycles;sequence_is_next_left_span_minus_prior_right_span_if_prior_right_weak;effective_accepted_output_for_both_trial_cycles;keep_each_cycle_safety_monitor;compare_weak_foot_ratio_and_sequence;accepted_cycles_field_counts_pairs;hold_three_pairs_after_success;restore_unfavorable;two_blocked_commands_reobserve_three_pairs_then_try_opposite;two_adjacent_effective_pairs_severe_weak_loss_and_ratio_worsening_restore;bounded_guarded_width\",\"diagnostic_fields\":\"long_foot_counterfactual_ms_is_trial_reference_after_guard;long_foot_actual_ms_is_actual_candidate_after_guard;long_foot_effective_is_accepted_effective_trial_direction\",\"interpretation\":\"camera_relative_not_ground_stride;observational_not_causal\",\"hardware_validated\":false},";
    json += "\"foot_observation_timeout\":{\"revision\":\"two_invalid_cycles_v1\",\"policy\":\"two_consecutive_post_change_invalid_foot_cycles_restore_trial_reference;alternate_next_exploration_after_accepted_restore_and_fresh_evidence;invalid_not_zero_step;no_causal_claim\",\"hardware_validated\":false},";
    json += "\"paired_sway_rollback\":{\"revision\":\"paired_gap_guard_v1\",\"policy\":\"two_paired_peak_gaps_at_least1p5deg_restore_only_taper_recovery_cap_increases;hold_until_three_paired_gaps_at_most0p8deg;ordinary_foot_and_wheel_controls_retained\",\"hardware_validated\":false},";
    json += "\"tapered_amplitude_recovery\":{\"revision\":\"paired_taper_v1\",\"scope\":\"target8_final_stage_both_sides\",\"policy\":\"two_paired_unmodified_tapered_commands_below_target_by_over1deg_raise_side_cap5ms_to80ms;no_learning_during_foot_or_probe_trial;existing_overshoot_wheel_timing_guards_retained\",\"physical_response_identified\":false,\"hardware_validated\":false},";
    json += "\"positive_sway_probe\":{\"revision\":\"one_bounded_probe_v1\",\"policy\":\"four_consecutive_valid_neutral_cycles;weak_positive_peak;two_effective_response_cycles_or_3s_max;always_restore;one_per_run\",\"fields\":\"foot_span_flags_bit14_marks_probe_path;shared_long_foot_counterfactual_ms_actual_ms_effective_are_guarded_baseline_candidate_and_executed;converter_separates_positive_sway_probe_columns\",\"physical_response_identified\":false,\"hardware_validated\":false},";
    json += "\"negative_cap_recovery\":{\"revision\":\"paired_baseline_v1\",\"scope\":\"target8_negative_branch15\",\"policy\":\"disabled_in_firmware_1p0p92;no_negative_cap_trial_or_continuation;legacy_fields_retained_for_older_logs\",\"fields\":\"negative_rows_reuse_long_foot_counterfactual_ms_actual_ms_effective_for_recovery;guarded_widths_and_accepted_effective_pulse;state_in_foot_span_flags_bits12_13_0_collect_1_trial_3_done;converter_splits_columns\",\"physical_response_identified\":false,\"hardware_validated\":false},";
    json += "\"peak_mpc\":{\"revision\":\"two_peak_observational_20261101\",\"active\":false,\"policy\":\"disabled_for_phase_cap_revision\",\"hardware_validated\":false},";
    json += "\"wheel_speed_balance\":{\"revision\":\"walking_rpm_bias_guard_v3\",\"scope\":\"target8_only\",\"policy\":\"paired_fresh_rpm;two_positive_high_speed_pairs_reduce_positive_cap_5ms_to60;other_bias_range_80_100;no_reverse_pulse\",\"positive_cap_field\":\"work_brake_cap_ms\",\"hardware_validated\":false},";
  json += "\"walking_energy_loss\":{\"revision\":\"walking_coast_eight_runs_v6\",\"source_run\":50881356,\"fit_coast_events\":32,\"low_rpm_coast_observations\":26,\"low_rpm_branch\":11,\"low_rpm_pulse_gain_validated\":false,\"low_rpm_policy\":\"branch11_2_matched_low_coasts_within4s;normal_bridge_check_and_negative_peak_gap_max1p5s;start10ms_then_5ms_steps_max25ms;fresh_positive_arrival_speed_and_prediction_guard;unvalidated_powered_gain\",\"independent_validation\":false,\"branch\":10,\"scope\":\"target8_negative_only;eight_run_rate_rpm_envelope;voltage_transfer_7000_8120mV;zero_CG_only\",\"loss_semantics\":\"signed_previous_peak_U_minus_predicted_coast_U;includes_model_and_attitude_bias;not_pure_impact_loss\",\"formula_mj\":\"coast=0.09046158+0.00018973*rate_dps^2;arrival=Uprev-loss+transferred_increment+branch_residual\",\"input_policy\":\"two_matched_coast_arrivals_then300mA_10to25ms;two_under_target_capped_arrivals_raise_5ms_to60ms;three_capped_below_target_arrivals_with_fresh_rpm_at_least30_extend_to75ms;invalid_or_low_rpm_restore60;powered_gain_provisional;no_old_side_bias_or_CG_learning\"},";
    json += "\"low_peak_energy_feedback\":{\"revision\":\"same_branch_startup_residual_v1\",\"active_for_output\":false,\"scope\":\"legacy_diagnostics_only\"},";
  json += "\"foot_gap_trend\":{\"revision\":\"bounded_observation_gaps_v1\",\"active_for_actuation\":false,\"interpretation\":\"legacy_observation_only\"},";
  json += "\"foot_span_control\":{\"revision\":\"three_adjacent_pairs_trial_loss_v5\",\"units\":\"camera_relative_calibrated_degrees_not_ground_step\",\"state_codes\":\"0_collect_1_trial_2_hold_3_restore\",\"flag_bits\":\"0_valid_1_selected_2_startup_3_left_low_4_right_low_5_left_both_6_right_both_7_continuous_8_left_outlier_9_right_outlier_10_left_edge_unverified_11_right_edge_unverified\",\"physical_response_identified\":false,\"hardware_validated\":false},";
  json += "\"weak_side_assist\":{\"revision\":\"negative_duration_trial_v2\",\"scope\":\"target8_negative_unmodeled_or_normal_coast\",\"branch\":13,\"current_mA\":300,\"policy\":\"target8_deficit;initial_cap50ms;two_comparable_under_target_at_cap_raise_cap10ms;two_unfavorable_trials_rollback;normal_powered_not_overridden;prior_peak_time_caps_output\",\"initial_limit_ms\":50,\"min_limit_ms\":20,\"max_limit_ms\":150,\"cap_field\":\"work_brake_cap_ms\",\"actual_command_from\":\"pulse_width_ms_command_current_mA_output_executed\",\"feedback\":\"time_matched_peaks_heuristic_not_identified_causal_effect;missing_or_incomparable_observation_discards_evidence_and_normal_control_continues\",\"prediction\":\"charge_model_unavailable_above_100ms;experimental_result_not_used_for_fitted_model_learning\",\"speed_readback\":\"may_lag_actual_wheel_speed;read_freshness_does_not_guarantee_measurement_latency\",\"hardware_validated\":false},";
    json += "\"foot_phase_cap\":{\"revision\":\"phase_pair_diagnostics_only_v6\",\"scope\":\"legacy_fields_unrecorded_no_motor_output\",\"observation\":\"signed_camera_relative_fixed_calibration_marker_changes_unbounded_endpoint_extrapolation_not_ground_step\",\"max_trial_reduction_ms\":5,\"baseline\":\"ordinary_energy_selector_without_legacy_low_peak_foot_energy_or_mpc\",\"state_codes\":\"0_waiting_1_trial_2_hold_3_release_4_locked\",\"reason_codes\":\"0_none_1_stale_2_invalid_3_input_failed_4_negative_loss_5_weak_peak_6_worsened_7_inconclusive_8_reversed_9_disallowed_10_target_changed_11_mode_changed\",\"powered_response_identified\":false,\"yaw_feedback_enabled\":false,\"hardware_validated\":false},";
    json += "\"work_control_branch_codes\":{\"0\":\"not_selected\",\"1\":\"positive_work\",\"2\":\"legacy_negative_extended\",\"3\":\"legacy_negative_broad\",\"4\":\"negative_brake_target12_state_or_legacy_pooled\",\"5\":\"positive_expanded_bridge\",\"6\":\"positive_work_target12_state_or_legacy8run\",\"7\":\"positive_energy_blend_guard_band\",\"8\":\"negative_bounded_guard_band\",\"9\":\"negative_assist_target12_state_or_legacy_transferred\",\"10\":\"walking_negative_loss_compensation\",\"11\":\"walking_low_rpm_coast_guarded_probe\",\"12\":\"walking_unmodeled_natural_coast\",\"13\":\"weak_side_experimental_unmodeled_assist\",\"14\":\"positive_asymmetry_unmodeled_trial\",\"15\":\"gradual_sway_unmodeled_trial_or_coast\",\"16\":\"foot_support_transfer_unmodeled_v11\",\"17\":\"foot_support_local_unmodeled_v12\"},";
    json += "\"wheel_work_target12_state_model\":\"fixed_leg_3runs_155_events;rate_squared_rpm_voltage_previous_peak_signed_Q;work_on_plus_J2.8e-5;retained_work_empirical;negative_assist_gain_fitted;bounded_legacy_transition;cross_condition;not_closed_loop_validated\",";
    json += "\"wheel_work_model_interpretation\":\"effective_arrival_energy_regression;work_coefficient_not_physical_efficiency;includes_post_pulse_work;target12_measured_state_work_model_and_smooth_legacy_transition;unmeasured_commands_cross_condition;negative_brake70ms;negative_assist300mA_max100ms_Q25mAs;no_closed_loop_validation_claim\",";
  }
  if (inverse_mode) {
    json += "\"negative_one_peak_model_revision\":\"" + String(one_peak_energy_inverse::kRevision) + "\",";
    json += "\"negative_one_peak_model_scope\":\"fixed_leg;target_8deg;command_300mA;fit_reference_abs_1.932539deg;runtime_uses_saved_reference_relative_angles;fit_0_10_15_20_25ms;voltage_7.22_to_8.00V\",";
    json += "\"negative_one_peak_input_policy\":\"choose_width_by_target_minus_predicted_peak_energy;within_run_three_peak_raw_fit_residual_mean;25_to_30ms_one_ms_completed_peak_feedback;stop_extension_when_previous_positive_peak_below_7deg;skip_outside_support_or_stale_speed\",";
  }
  json += "\"energy_control_autonomous_policy\":\"" + String(work_mode
      ? "MEKF_zero_cross;positive_choose_measured_300_350_400mA_and_Q;negative_pooled_energy_inverse_with_completed_peak_energy_residual_and_staged_cap;outside_support_coast"
      : inverse_mode
      ? "MEKF_peak_and_zero_cross;positive_side_historical_bridge;negative_side_one_peak_work_response;bounded_0_to_30ms_brake"
      : energy_bridge_enabled_
      ? "MEKF_peak_and_zero_cross;historical_positive_Q_angle_map_transformed_to_potential;excess_coasts;no_reverse_command;excite_max_100ms"
      : "MEKF_peak_and_zero_cross;quadratic_signed_energy_inverse;excite_max_100ms;selected_reverse_brake_limit") + "\",";
  json += "\"lateral_energy_revision\":\"" + String(energy_bridge_enabled_ ? lateral_energy::kRevision : lateral_energy::kQuadraticRevision) + "\",";
  json += "\"lateral_energy_units\":\"" + String(energy_bridge_enabled_
      ? "rate_deg_per_s;arrival_energy_J;Q_fit_mA_s;time_ms"
      : "omega_rad_per_s;effective_energy_J;Q_log_mA_s;Q_fit_A_s;time_ms") + "\",";
  json += "\"lateral_energy_formula\":\"" + String(inverse_mode
      ? "positive_side_historical_bridge;negative_side_E_peak=E_free(state)+0.4229016404*(W_on(T)+W_after(T)-W_after(0))+within_run_residual;choose_T_min_abs(E_peak-U_8deg)"
      : energy_bridge_enabled_
      ? "A_free=max(0,a_side+b_side*(abs_rate_dps-65));E_free=U(A_free);E_target=U(target_deg);E_arrival(Q)=U(A_free+g_side*Q_mAs);Q_positive=inverse_U(E_target+integral-bias);width=inverse_current_rise(Q);negative_request=coast"
      : "E_free=max(0,c_omega2*omega^2+c_offset);E_target=U(target_deg);DeltaE=E_target-E_free+I_side-bias;DeltaE=alpha_side*abs(omega)*Q+beta_side*Q^2;signed_Q=inverse(DeltaE)") + "\",";
  json += "\"lateral_energy_coefficient_status\":\"" + String(inverse_mode
      ? "negative_fixed_leg_fourteen_run_fit;measured_widths_0_10_15_20_25ms;25_to_30ms_extrapolated_closed_loop_validation_pending"
      : energy_bridge_enabled_
      ? "historical_04731_positive_Q_peak_angle_fit;energy_is_potential_mapping_not_measured_shaft_work;current_upright_reference_closed_loop_validation_pending"
      : "provisional_58_corrected_polarity_free_foot_events;reverse_response_not_separately_identified") + "\",";
  json += "\"energy_control_autonomous_revision\":\"" + String(energy_bridge_enabled_ ? lateral_energy::kRevision : lateral_energy::kQuadraticRevision) + "\",";
  json += "\"peak_error_adaptation_enabled\":" + String(peak_adaptation_enabled_ ? "true" : "false") + ",";
  json += "\"energy_request_audit_revision\":\"unrestricted_inverse_v1_20261014\",";
  json += "\"energy_request_audit_semantics\":\"" + String(energy_bridge_enabled_
      ? "unclamped_energy_request=target-free+integral-bias;Q=potential_inverse_then_historical_positive_inverse_or_null;max_input_work=null;max_arrival_increment=U(max_angle)-U(free);unapplied=request-selected_arrival_increment;loss_and_work_coefficients=null"
      : "unclamped_energy_request=target-free+integral-bias;Q=quadratic_inverse_or_null;max_input_work=quadratic_model;unapplied=request-selected_response") + "\",";
  json += "\"peak_telemetry_semantics\":\"snapshot_at_peak_confirmation_time_not_at_geometric_peak;ages_in_us;valid_does_not_imply_fresh;no_additional_hardware_read\",";
  json += "\"peak_error_adaptation_policy\":\"within_run_from_5s;physical_side_mode_signed_wheel_speed_bin;four_same_sign_residuals;max_bias_0.000070J;stale_invalid_or_coast_zero_bias;half_request_guard;mode_change_unadapted\",";
  json += "\"peak_error_adaptation_speed_bins_rpm\":\"less_than_-100;-100_to_less_than_100;100_to_less_than_300;300_or_more\",";
  json += "\"peak_error_adaptation_residual_definition\":\"measured_peak_energy_minus_uncorrected_model_energy_for_matched_zero_cross\",";
  json += "\"energy_control_autonomous_base_revision\":\"V7_hardware_and_MEKF_event_path\",";
  json += "\"legacy_rate_and_Q1_models_active\":false,";
  json += "\"historical_04731_angle_predictor_active\":" + String(energy_bridge_enabled_ ? "true" : "false") + ",";
  json += "\"normal_excitation_direction_policy\":\"" + String(energy_bridge_enabled_
      ? "positive_arrival_energy_request_same_as_zero_cross_motion;excess_coasts"
      : "energy_deficit_positive_same_as_zero_cross_roll_rate;negative_opposite_as_active_braking") + "\",";
  json += "\"normal_direction_sign_fix\":true,";
  json += "\"start_kick_direction_changed\":false,";
  json += "\"side_response_correction_enabled\":false,";
  json += "\"side_response_correction_source\":\"" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_SOURCE) + "\",";
  json += "\"side_response_correction_model\":\"A_pred=A_free+g_side_base*Q;legacy_residual_disabled;historical_fit_coordinate=firmware_scaled_gyro_peak\",";
  json += "\"side_response_correction_blend_lambda\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_BLEND_LAMBDA, 3) + ",";
  json += "\"side_response_correction_max_abs_deg\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG, 3) + ",";
  json += "\"side_response_correction_fit_c_plus_deg\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_PLUS_DEG, 6) + ",";
  json += "\"side_response_correction_fit_g_plus_deg_per_mA_s\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_PLUS_DEG_PER_MAS, 6) + ",";
  json += "\"side_response_correction_fit_c_minus_deg\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_MINUS_DEG, 6) + ",";
  json += "\"side_response_correction_fit_g_minus_deg_per_mA_s\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_MINUS_DEG_PER_MAS, 6) + ",";
  json += "\"side_response_correction_base_P1_unchanged\":false,";
  json += "\"side_response_correction_base_Q1_gains_unchanged\":true,";
  json += "\"side_response_correction_Ki_unchanged\":" + String(energy_bridge_enabled_ ? "false" : "true") + ",";
  json += "\"side_response_correction_direction_policy_unchanged\":false,";
  json += "\"energy_control_autonomous_state_machine\":\"IDLE->STRONG_START_KICK->WAIT_FIRST_PEAK->ENERGY_CONTROL->HOLD;physical_halfcycle=WAIT_PEAK->WAIT_ZERO_CROSS->PULSE_ACTIVE->WAIT_PEAK;STOP_only_abnormal\",";
  json += "\"physical_event_policy\":\"peak_zero_pulse_halfcycle_v5\",";
  json += "\"min_half_cycle_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS) + ",";
  json += "\"min_zero_to_peak_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_MIN_ZERO_TO_PEAK_MS) + ",";
  json += "\"events_accepted_during_pulse\":false,";
  const bool negative_probe_extended_mode = negative_probe_mode_ &&
      autonomous_brake_limit::extendedBatch(autonomous_brake_max_ms_);
  const bool negative_probe_batch_mode = negative_probe_mode_ &&
      (autonomous_brake_limit::batch(autonomous_brake_max_ms_) || negative_probe_extended_mode);
  const bool positive_current_batch_mode = negative_probe_mode_ &&
      autonomous_brake_limit::positiveCurrentBatch(autonomous_brake_max_ms_);
  json += "\"autonomous_duration_ms\":" + String((negative_probe_batch_mode || positive_current_batch_mode) ?
      negative_probe_batch::kDurationMs : work_mode ? Config::WHEEL_WORK_CONTROL_DURATION_MS : Config::ENERGY_CONTROL_AUTONOMOUS_DURATION_MS) + ",";
  json += "\"energy_control_autonomous_free_model\":\"" + String(energy_bridge_enabled_
      ? "U(max(0,a_side+b_side*(rate_dps-65)));historical_angle_predictor"
      : "max(0,c_omega2*omega^2+c_offset);empirical_effective_energy") + "\",";
  json += "\"energy_control_autonomous_peak_policy\":\"posterior_MEKF_extremum_plus_3_returning_rate_samples;no_gyro_sign_only_peak;no_events_during_pulse;expected_side_only;no_peak_rate_or_amplitude_minimum\",";
  json += "\"energy_control_autonomous_target_peak_deg\":" + String(control_target_cdeg_ / 100.0f, 3) + ",";
  json += "\"energy_control_autonomous_target_choices_deg\":\"8.0,10.0,12.0\",";
  json += "\"energy_control_autonomous_start_kick\":\"startup_only;configured_current_mA;100ms;command_direction=-1\",";
  json += "\"motor_driver_polarity\":" + String(Config::MOTOR_DRIVER_POLARITY) + ",";
  json += "\"normal_autonomous_motor_output_sign\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_NORMAL_OUTPUT_SIGN) + ",";
  json += "\"motor_direction_semantics\":\"pulse_direction_and_q_command_direction_are_body_motion_logical;normal_autonomous_output_uses_mount_polarity_times_trial_sign;start_kick_unchanged;motor_cmd_mA_is_signed_current_sent_to_Roller485;roller_actual_current_mA_is_driver_readback\",";
  json += "\"energy_control_autonomous_startup_pump\":false,";
  json += "\"energy_control_autonomous_peak_coordinate\":\"A=abs(pitch_mekf_measurement_relative_deg_at_posterior_extremum);work_mode_zero_is_MEKF_absolute_0;other_autonomous_saved_upright;no_delay_projection;no_output_scaling\",";
  json += "\"energy_control_autonomous_rate_coordinate\":\"(gy_dps-mekf_bias_y_dps)*mekf_gyro_y_scale;live_MEKF_bias;historical_Q1_rate_support_rescaled\",";
  json += "\"energy_control_autonomous_model_coordinate_status\":\"" + String(work_mode
      ? "historical_relative_energy_regressions_as_initial_models_at_MEKF_absolute_zero;hardware_validation_pending"
      : inverse_mode
      ? "negative_side_fit_relative_to_saved_upright;fit_reference_abs_1.932539deg_not_runtime_predictor;positive_side_historical_bridge_coordinate_validation_pending"
      : energy_bridge_enabled_
      ? "04731_measurement_start_relative_fit_reused_with_saved_upright_reference;coordinate_shift_and_hardware_validation_pending"
      : "fixed_foot_3run_MEKF_refit;new_closed_loop_hardware_validation_pending") + "\",";
  json += "\"rate_baseline_revision\":\"fixed_foot_mekf_rate_refit_04731_20260929\",";
  json += "\"rate_baseline_policy\":\"" + String(energy_bridge_enabled_
      ? "active_historical_04731_fit_in_arrival_energy_coordinates;rate_only_no_state_gate;positive_Q_only"
      : "historical_diagnostic_only") + "\",";
  json += "\"rate_baseline_blend\":1.0,\"rate_baseline_delta_cap_enabled\":false,\"rate_baseline_delta_cap_deg\":null,";
  json += "\"rate_baseline_formula\":\"A_rate_plus=6.730502640+0.187441879*(abs_rate-65);A_rate_minus=7.691200794+0.194970900*(abs_rate-65);A_baseline=max(0,A_rate);A_next=A_baseline+g_side*Q\",";
  json += "\"rate_baseline_state_gate\":\"none;previous_peak_target_delay_elapsed_time_not_predictor_inputs\",";
  json += "\"rate_baseline_reason_codes\":\"0=rate_applied,4=invalid_rate_or_side,5=rate_applied_with_zero_floor,255=not_evaluated;legacy1to3_unused\",";
  json += "\"rate_baseline_previous_peak_used\":false,";
  json += "\"previous_peak_residual_enabled\":false,\"previous_peak_residual_disabled_reason\":\"old_residual_belongs_to_retired_baseline\",";
  json += "\"rate_baseline_p1_fallback_enabled\":false,";
  json += "\"rate_baseline_zero_floor_deg\":0.0,";
  json += "\"rate_baseline_legacy_columns\":\"p1_free_peak_before_rate_deg=null;rate_baseline_correction_deg=null;rate_baseline_peak_deg=null;free_next_peak_amplitude_deg=selected_model_free_prediction\",";
  json += "\"rate_baseline_fit_source\":\"20260929_102026_917_7279;20260929_103329_014_99a5;20260929_103650_324_4ebe;204_matched_normal_commands;MEKF_posterior_teacher;fixed_Q_gains;leave_one_run_out_RMSE_0.497deg;not_closed_loop_validation\",";
  json += "\"energy_control_autonomous_zero_cross_detector\":\"posterior_measurement_relative_last_peak_percentage_inward_threshold;no_timing_projection;one_consumed_cross_per_accepted_peak\",";
  json += "\"energy_control_autonomous_side_policy\":\"next_side_from_interpolated_rate;peak_side_mismatch_logged_diagnostic_only\",";
  json += "\"energy_control_autonomous_integral_enable_rule\":\"matched_command_peak_only;per_physical_peak_side;energy_J;antiwindup\",";
  json += "\"energy_control_autonomous_integral_Ki_J_per_J\":";
  json += energy_bridge_enabled_ ? "null," : "0.1,";
  json += "\"energy_control_autonomous_integral_Ki_plus_J_per_J\":" + String(work_mode ? 0.f : energy_bridge_enabled_ ? .1f*lateral_energy::bridgeCoefficients(1).gain_deg_per_mAs : lateral_energy::kIntegralGain, 6) + ",";
  json += "\"energy_control_autonomous_integral_Ki_minus_J_per_J\":" + String(work_mode ? 0.f : energy_bridge_enabled_ ? .1f*lateral_energy::bridgeCoefficients(-1).gain_deg_per_mAs : lateral_energy::kIntegralGain, 6) + ",";
  json += "\"energy_control_autonomous_normal_current_mA\":" + String(run_current_mA_) + ",";
  json += "\"energy_control_autonomous_start_kick_current_mA\":" + String(run_current_mA_) + ",";
  json += "\"energy_control_autonomous_normal_width_min_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS) + ",";
  json += "\"energy_control_autonomous_normal_width_max_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS) + ",";
  json += "\"energy_control_autonomous_brake_width_max_ms\":" + String(work_mode ? wheel_work_energy_control::kNegativeMaxMs : energy_bridge_enabled_ ? 0 : lateral_energy::kBrakeMaxMs) + ",";
  json += "\"energy_control_autonomous_negative_side_brake_width_max_ms\":" + String(work_mode ? wheel_work_energy_control::kNegativeMaxMs : inverse_mode ? 30 : energy_bridge_enabled_ ? 0 : autonomous_brake_max_ms_) + ",";
  json += "\"fixed_negative_work_probe\":" + String(negative_probe_mode_ ? "true" : "false") + ",";
  json += "\"fixed_negative_work_probe_width_ms\":" + String(positive_current_batch_mode ?
      positive_current_batch::kNegativeWidthMs : negative_probe_batch_mode ? 0 :
      (negative_probe_mode_ ? autonomous_brake_max_ms_ : 0)) + ",";
  json += "\"negative_probe_batch\":" + String(negative_probe_batch_mode ? "true" : "false") + ",";
  json += "\"negative_probe_extended\":" + String(negative_probe_extended_mode ? "true" : "false") + ",";
  json += "\"positive_current_batch\":" + String(positive_current_batch_mode ? "true" : "false") + ",";
  if (positive_current_batch_mode) {
    json += "\"positive_current_batch_seed\":" + String(negative_probe_batch_seed_) + ",";
    json += "\"positive_current_batch_algorithm\":\"seeded_fisher_yates_three_event_blocks_v1\",";
    json += "\"positive_current_batch_schedule_mA\":[";
    for (uint16_t i = 0; i < positive_current_batch::kSchedulePreviewLength; ++i) {
      if (i) json += ",";
      json += String(positive_current_batch::current(negative_probe_batch_seed_, i));
    }
    json += "],";
  }
  if (negative_probe_batch_mode) {
    json += "\"negative_probe_batch_seed\":" + String(negative_probe_batch_seed_) + ",";
    json += "\"negative_probe_batch_algorithm\":\"" + String(negative_probe_extended_mode ?
        "seeded_fisher_yates_three_event_blocks_v1" : "seeded_fisher_yates_five_event_blocks_v1") + "\",";
    json += "\"negative_probe_batch_skip_policy\":\"" + String(negative_probe_extended_mode ?
        "rate_outside_bridge_support_or_previous_positive_peak_below_7deg_or_battery_outside_7000_to_8500mV" :
        "rate_outside_bridge_support") + "\",";
    json += "\"negative_probe_batch_schedule_ms\":[";
    for (uint16_t i = 0; i < negative_probe_batch::kSchedulePreviewLength; ++i) {
      if (i) json += ",";
      json += String(negative_probe_extended_mode ?
          negative_probe_extended_batch::width(negative_probe_batch_seed_, i) :
          negative_probe_batch::width(negative_probe_batch_seed_, i));
    }
    json += "],";
  }
  json += "\"autonomous_uses_q_ident_limits\":false,";
  json += "\"energy_control_autonomous_q_policy\":\"" + String(positive_current_batch_mode
      ? "positive_side_bridge_randomized_300_350_400mA;negative_side_fixed_40ms_300mA;startup_300mA;no_negative_peak_learning"
      : negative_probe_extended_mode
      ? "positive_side_bridge;negative_side_randomized_30_35_40ms_fixed_raw_positive_batch;previous_positive_peak_and_battery_gate;no_negative_peak_learning"
      : negative_probe_batch_mode
      ? "positive_side_bridge;negative_side_balanced_fixed_raw_positive_batch;no_negative_peak_learning"
      : negative_probe_mode_
      ? "positive_side_bridge;negative_side_fixed_raw_positive_probe;no_negative_peak_learning"
      : work_mode
      ? "positive_and_negative_predicted_wheel_work_to_one_peak_arrival_energy;bounded_measured_commands"
      : inverse_mode
      ? "positive_side_bridge;negative_side_one_peak_energy_width_inverse;bounded_25_to_30ms_feedback"
      : energy_bridge_enabled_
      ? "positive_Q_from_inverse_potential_and_historical_angle_gain;current_rise_binary_search;negative_request_coasts;reverse_Q_not_identified"
      : "signed_energy_error_to_signed_Q_quadratic_inverse;current_rise_binary_search;reverse_brake_limit_selected;negative_Q_extrapolated") + "\",";
  json += "\"energy_control_autonomous_event_overflow\":" + String(energy_control_autonomous_event_overflow_ ? "true" : "false") + ",";
  json += "\"energy_control_autonomous_event_overflow_policy\":\"diagnostic_only;no_control_stop\",";
  json += "\"q1_model_name\":\"" + String(Config::Q1_SHADOW_MODEL_NAME) + "\",";
  json += "\"q_model_axis_type\":\"" + String(Config::Q1_SHADOW_Q_MODEL_AXIS_TYPE) + "\",";
  json += "\"q_model_axis_field\":\"" + String(Config::Q1_SHADOW_Q_MODEL_AXIS_FIELD) + "\",";
  json += "\"q1_formula\":\"A_next_abs=0.01304+0.08812*abs(physical_roll_rate_dps)+0.11858*physical_next_peak_side+g_side*q_target_mA_s\",";
  json += "\"q1_intercept\":" + String(Config::Q1_SHADOW_INTERCEPT_DEG, 5) + ",";
  json += "\"q1_rate_gain\":" + String(Config::Q1_SHADOW_RATE_GAIN_DEG_PER_DPS, 5) + ",";
  json += "\"q1_side_term\":" + String(Config::Q1_SHADOW_SIDE_TERM_DEG, 5) + ",";
  json += "\"q1_gain_physical_plus\":" + String(Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS, 5) + ",";
  json += "\"q1_gain_physical_minus\":" + String(Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS, 5) + ",";
  json += "\"q1_q_support_mA_s\":\"0.454-1.197\",";
  json += "\"q1_rate_support_dps\":\"1.68-27.80\",";
  json += "\"q1_zero_cross_angle_detector\":\"MEKF_control_angle;autonomous_saved_upright_reference;legacy_measurement_start;detector_only\",";
  json += "\"q1_zero_cross_rate_axis\":\"physical_roll_rate_dps=+gy_dps-startup_y_bias\",";
  json += "\"q1_physical_next_peak_side_rule\":\"sign(physical_roll_rate_dps) after accepted zero-cross;positive_rate_means_positive_physical_next_peak\",";
  json += "\"q1_policy\":\"baseline_is_Q_equals_0;inverse_Q_augmentation_only;nonpositive_delta_is_braking_invalid;never_clip_support\",";
  json += "\"q1_motor_connected\":false,";
  json += "\"q1_target_next_peak_abs_deg\":";
  if (isfinite(q1_shadow_target_peak_abs_deg_)) json += String(q1_shadow_target_peak_abs_deg_, 4);
  else json += "null";
  json += ",";
  json += "\"e2_shadow_role\":\"offline_diagnostic_only;not_called_by_Q1_or_its_validity_logic\",";
  json += "\"control_angle_origin_abs_deg\":" + jsonFloatOrNull(work_mode ? 0.f : run_upright_reference_.pitch_deg,6) + ",";
  json += "\"work_absolute_reference_enabled\":" + String(work_mode ? "true" : "false") + ",";
  json += "\"work_reference_validation\":\"historical_relative_energy_regressions_reused_as_initial_models;absolute_zero_cross_state_shift_requires_hardware_validation;neutral_adaptive_seed\",";
  json += "\"angle_reference_policy\":\"work_mode_MEKF_absolute_zero;other_autonomous_saved_upright;legacy_measurement_start;no_work_rezero\",";
  json += "\"comparison_zero_source\":\"work=MEKF_absolute_0;other_autonomous=NVS_pose;legacy=posterior_snapshot;reference_subtraction_does_not_reset_MEKF_state\",";
  const bool cg_active=energy_bridge_enabled_ && !negative_probe_mode_ && autonomous_brake_max_ms_==40 &&
      lateral_cg::matchesAbsolute(run_lateral_cg_);
  const float cg_mm=run_adaptive_cg_.enabled ? run_adaptive_cg_.shift_mm : cg_active ? run_lateral_cg_.shift_mm : 0.f;
  json += "\"adaptive_lateral_cg\":{\"enabled\":" + String(run_adaptive_cg_.enabled ? "true" : "false") +
      ",\"initial_mm\":" + jsonFloatOrNull(run_adaptive_cg_.initial_mm,6) +
      ",\"final_mm\":" + jsonFloatOrNull(run_adaptive_cg_.shift_mm,6) +
      ",\"common_bias_mj\":" + jsonFloatOrNull(run_adaptive_cg_.common_bias_mj,6) +
      ",\"pairs\":" + String(run_adaptive_cg_.pairs) + ",\"updates\":" + String(run_adaptive_cg_.updates) +
      ",\"rejected\":" + String(run_adaptive_cg_.rejected) +
      ",\"interpretation\":\"model_relative_equivalent_CG_not_unique_physical_CG\"},";
  const lateral_energy::Geometry cg_geometry;
  json += "\"lateral_cg\":{\"valid\":" + String(cg_active || run_adaptive_cg_.enabled ? "true" : "false") +
      ",\"automatic\":" + String(run_adaptive_cg_.enabled ? "true" : "false") +
      ",\"physical_calibration_valid\":" + String(cg_active ? "true" : "false") +
      ",\"shift_mm\":" + jsonFloatOrNull(cg_mm,6) +
      ",\"positive_target_j\":" + jsonFloatOrNull(lateral_cg::potential(cg_geometry,control_target_cdeg_/100.f,cg_mm),9) +
      ",\"negative_target_j\":" + jsonFloatOrNull(lateral_cg::potential(cg_geometry,-control_target_cdeg_/100.f,cg_mm),9) +
      ",\"source\":" + String(run_lateral_cg_.source) +
      ",\"revision\":" + String(run_lateral_cg_.revision) +
      ",\"upright_revision\":" + String(run_lateral_cg_.upright_revision) +
      ",\"equilibrium_abs_deg\":" + jsonFloatOrNull(run_lateral_cg_.equilibrium_abs_deg,6) +
      ",\"potential\":\"U0(theta)+m*g*dx*sin(theta);geometric_upright_zero\"},";
  json += "\"upright_reference_revision\":" + String(run_upright_reference_.revision) + ",";
  json += "\"upright_reference_valid\":" + String(upright_reference::valid(run_upright_reference_) ? "true" : "false") + ",";
  json += "\"upright_reference_abs_pitch_deg\":" + jsonFloatOrNull(upright_reference::valid(run_upright_reference_) ? run_upright_reference_.pitch_deg : NAN, 6) + ",";
  json += "\"upright_reference_normalized_ax\":" + jsonFloatOrNull(upright_reference::valid(run_upright_reference_) ? run_upright_reference_.normalized_ax : NAN, 6) + ",";
  json += "\"upright_reference_samples\":" + String(run_upright_reference_.samples) + ",";
  json += "\"upright_reference_spread_deg\":" + jsonFloatOrNull(upright_reference::valid(run_upright_reference_) ? run_upright_reference_.spread_deg : NAN, 6) + ",";
  json += "\"measurement_start_abs_pitch_deg\":" + jsonFloatOrNull(measurement_start_abs_deg_, 6) + ",";
  json += "\"measurement_start_upright_relative_deg\":" + jsonFloatOrNull(upright_reference::relative(measurement_start_abs_deg_, run_upright_reference_), 6) + ",";
  json += "\"comparison_zero_sample_time_semantics\":\"autonomous_persistent_reference_has_no_current_boot_capture_timestamp;legacy_last_consumed_gyro_timestamp\",";
  json += "\"attitude_filter_adopted\":\"MEKF_6state_error_state\",";
  json += "\"attitude_filter_compare\":\"online_dynamic_beta_madgwick_hold073_only_for_post_run_comparison\",";
  json += "\"mekf_coordinate_mapping\":\"Ry180_body_frame:accel=(-ax,+ay,-az);gyro=(-gx,+gy_scaled,-gz);reported_pitch=physical_video_sign\",";
  json += "\"mekf_gyro_y_scale\":" + String(Config::MEKF_GYRO_Y_SCALE, 6) + ",";
  json += "\"mekf_gyro_y_scale_role\":\"pre_prediction_sensor_calibration_not_output_angle_scaling\",";
  json += "\"mekf_control_angle_reference\":\"work:MEKF_posterior_absolute_zero;other_autonomous:abs_minus_saved_upright;measurement_and_detector_share_reference;no_timing_projection;legacy_modes_unchanged\",";
  json += "\"mekf_abs_angle_reference\":\"continuous_gravity_frame_no_per_run_zero_subtraction;preferred_for_absolute_estimator_diagnostics\",";
  json += "\"mekf_detector_relative_reference\":\"posterior_measurement_relative_angle_without_timing_projection\",";
  json += "\"mekf_detector_zero_role\":\"work_MEKF_absolute_0_shared_by_amplitude_energy_timing_angle_stop;other_autonomous_saved_pose;no_per_run_rezero\",";
  json += "\"autonomous_control_prediction_enabled\":" + String(energy_control_autonomous_mode_ && autonomous_timing_compensation_us_ > 0 ? "true" : "false") + ",";
  json += "\"autonomous_control_prediction_type\":\"none_angle_threshold_only\",";
  json += "\"autonomous_timing_compensation_selectable\":false,";
  json += "\"autonomous_timing_compensation_us\":" + String(autonomous_timing_compensation_us_) + ",";
  json += "\"autonomous_input_peak_percent\":" + String(autonomous_input_peak_percent_, 3) + ",";
  json += "\"autonomous_input_percent_reference\":\"saved_upright_MEKF_posterior;inward_only;no_time_projection;threshold_is_fraction_of_last_confirmed_peak;already_inside_on_confirmation_fires_immediately\",";
  json += "\"autonomous_input_advance_us\":" + String(0UL) + ",";
  json += "\"autonomous_total_timing_projection_us\":" + String(0UL) + ",";
  json += "\"autonomous_input_event_coordinate\":\"zero_cross_named_events_are_selected_angle_threshold_events_not_actual_zero_angle\",";
  json += "\"autonomous_timing_prediction_formula\":\"theta_control=theta_posterior_measurement_relative;threshold=previous_peak_side*previous_peak_amplitude_deg*autonomous_input_peak_percent/100;target8_zero_percent_uses_zero_centre_with_explicit_inward_stall_rescue;inward_only\",";
  json += "\"mekf_prediction_role\":\"legacy_quaternion_forward_prediction_disabled_during_autonomous;retained_only_for_non_autonomous_legacy_modes\",";
  json += "\"madgwick_dynamic_abs_reference\":\"continuous_bias_corrected_dynamic_hold073_filter;online_comparison_only\",";
  json += "\"mekf_accel_mag_full_g\":" + String(Config::MEKF_ACCEL_MAG_FULL_G, 4) + ",";
  json += "\"mekf_accel_mag_reject_g\":" + String(Config::MEKF_ACCEL_MAG_REJECT_G, 4) + ",";
  json += "\"mekf_accel_angle_full_deg\":" + String(Config::MEKF_ACCEL_ANGLE_FULL_DEG, 3) + ",";
  json += "\"mekf_accel_angle_reject_deg\":" + String(Config::MEKF_ACCEL_ANGLE_REJECT_DEG, 3) + ",";
  json += "\"mekf_accel_min_confidence\":" + String(Config::MEKF_ACCEL_MIN_CONFIDENCE, 4) + ",";
  json += "\"physical_roll_axis\":\"IMU_y_axis; positive physical rate is gy_dps after startup y-bias subtraction\",";
  json += "\"physical_roll_candidate_formula\":\"atan2(ax_g,hypot(ay_g,az_g))*180/pi\",";
  json += "\"physical_roll_abs_formula\":\"0.9278941864271074*candidate_deg-0.49830848087090135\",";
  json += "\"physical_roll_confidence_policy\":\"accelerometer-derived angle is static-only; STATIC=NO suppresses static confidence during motion\",";
  json += "\"physical_roll_rate_formula\":\"gy_dps-startup_gyro_bias_y_dps\",";
  json += "\"static_rate_threshold_dps\":" + String(Config::STATIC_RATE_THRESHOLD_DPS, 3) + ",";
  json += "\"static_hold_time_ms\":" + String(Config::STATIC_HOLD_TIME_MS) + ",";
  json += "\"target_tolerance_deg\":" + String(Config::TARGET_TOLERANCE_DEG, 3) + ",";
  json += "\"current_roll_zero_policy\":\"display_only;physical_roll_abs_minus_display_zero_offset;no_sensor_or_model_change\",";
  json += "\"target_ready_policy\":\"display_only;STATIC_and_abs(current_roll-target_roll)<=target_tolerance;no_motor_or_control_change\",";
  json += "\"current_roll_ui_revision\":\"" + String(Config::CURRENT_ROLL_UI_REVISION) + "\",";
  json += "\"passive_static_window_t_test_ms\":\"0-" + String(Config::PASSIVE_STATIC_WINDOW_MS) + "\",";
  json += "\"v62_base_commit\":\"" + String(Config::V62_BASE_COMMIT) + "\",";
  json += "\"resolved_m5unified_version\":\"" + String(Config::RESOLVED_M5UNIFIED_VERSION) + "\",";
  json += "\"resolved_m5gfx_version\":\"" + String(Config::RESOLVED_M5GFX_VERSION) + "\",";
  json += "\"resolved_adafruit_ahrs_version\":\"" + String(Config::RESOLVED_ADAFRUIT_AHRS_VERSION) + "\",";
  json += "\"wheel_work_revision\":\"reaction_couple_position_work_v2_20261016\",";
  json += "\"wheel_work_inertia_kg_m2\":" + jsonFloatOrNull(Config::WHEEL_ROTATING_INERTIA_KG_M2, 9) + ",";
  json += "\"wheel_work_inertia_provisional\":true,";
  json += "\"wheel_work_inertia_basis\":\"motor_pulse_identification_conditional_on_nominal_200KV_torque_constant;wheel_weights_and_motor_rotor\",";
  json += "\"wheel_speed_to_body_sign\":" + String(Config::WHEEL_SPEED_TO_BODY_SIGN) + ",\"wheel_speed_to_body_sign_provisional\":true,";
  json += "\"wheel_work_body_rate\":\"physical_roll_rate_dps=raw_gy_minus_startup_y_bias;planar_lateral_motion\",";
  json += "\"wheel_work_formula\":\"omega_relative=s*d(position_deg)/dt*pi/180;omega_abs=omega_relative+Omega;W=-J*sum(mean(Omega)*delta(omega_abs))\",";
  json += "\"wheel_work_energy_definition\":\"reaction_couple_work_on_body;excludes_rotor_spin;not_net_next_peak_energy\",";
  json += "\"wheel_work_analysis\":\"post_run_browser;no_feedback;no_unmeasured_loss_or_readback_delay_compensation\",";
  json += "\"wheel_work_velocity_source\":\"position_local_quadratic_5_points\",";
  json += "\"wheel_position_read_period_us\":" + String(Config::WHEEL_POSITION_READ_PERIOD_US) + ",";
  json += "\"wheel_position_read_timeout_ms\":" + String(Config::WHEEL_POSITION_READ_TIMEOUT_MS) + ",";
  json += "\"wheel_position_register\":\"0x90_signed_int32_accumulated_centidegrees\",";
  json += "\"wheel_position_to_speed_sign\":1,";
  json += "\"wheel_position_sample_time_semantics\":\"micros_modulo_2pow32_host_read_midpoint;start_end_sequence_validity_saved;not_encoder_time\",";
  json += "\"wheel_position_analysis\":\"offline_centered_5_point_quadratic;20ms_span_at_5ms_reads;no_RPM_fallback_extrapolation_or_control;INT32_MIN_valid\",";
  json += "\"wheel_speed_read_period_us\":" + String(Config::WHEEL_SPEED_READ_PERIOD_US) + ",";
  json += "\"wheel_speed_read_timeout_ms\":" + String(Config::WHEEL_SPEED_READ_TIMEOUT_MS) + ",";
  json += "\"wheel_speed_sample_time_semantics\":\"micros_modulo_2pow32_host_I2C_read_midpoint;start_and_end_saved;not_internal_encoder_sample_time\",";
  json += "\"wheel_speed_sequence_semantics\":\"increments_each_attempt;failed_reads_invalid;repeated_sequence_is_not_new_measurement\",";
  json += "\"wheel_work_command_time_semantics\":\"successful_current_and_output_write_completion;not_measured_torque_onset\",";
  json += "\"actual_current_audit_revision\":\"v45_fresh_current_observation_only\",";
  json += "\"actual_current_audit_policy\":\"CURRENT_READBACK_and_observed_Q_are_diagnostic_only;never_change_motor_output\",";
  json += "\"roller_current_full_status_period_ms\":" + String(Config::ROLLER_READ_PERIOD_MS) + ",";
  json += "\"roller_current_fast_read_period_us\":" + String(Config::CURRENT_AUDIT_FAST_READ_PERIOD_US) + ",";
  json += "\"roller_current_fast_read_scope\":\"nonzero_command_only;full_six_register_status_reads_remain_enabled\",";
  json += "\"roller_current_sample_time_us_semantics\":\"micros_modulo_2pow32_at_successful_CURRENT_READBACK_completion;zero_means_no_fresh_sample_since_pulse_start\",";
  json += "\"roller_current_sequence_semantics\":\"CURRENT_READBACK_attempt_counter;increments_on_success_and_failure\",";
  json += "\"roller_current_valid_semantics\":\"true_only_if_the_most_recent_CURRENT_READBACK_attempt_succeeded\",";
  json += "\"roller_current_age_us_semantics\":\"time_since_last_successful_read;4294967295_means_unknown\",";
  json += "\"roller_q_meas_observed_semantics\":\"absolute_current_trapezoid_over_fresh_active_pulse_samples;edges_not_estimated;diagnostic_not_total_Q\",";
  json += "\"roller_q_meas_observed_valid_semantics\":\"at_least_two_fresh_pulse_samples;no_fast_read_failure;not_control\",";
  json += "\"mekf_roll_pitch_semantics\":\"v54_posterior_ZYX_roll_and_pitch_deg_same_sample;physical_roll_abs_deg_is_accel_diagnostic;no_yaw_heading_estimate\",";
  json += "\"pulse_q_target_pred_semantics\":\"normal_V7_selected_q_command_and_q_effective_pred_only;START_KICK_is_null\",";  json += "\"format_version\":" + String(RWLOG_FORMAT_VERSION) + ",";
  json += "\"shadow_forward_model_version\":\"" + String(Config::ZERO_CROSS_V57_FORWARD_MODEL_VERSION) + "\",";
  json += "\"shadow_model_version\":\"" + String(Config::ZERO_CROSS_V57_INVERSE_SHADOW_VERSION) + "\",";
  json += "\"shadow_reachability_audit_version\":\"" + String(Config::ZERO_CROSS_V54_REACHABILITY_AUDIT_VERSION) + "\",";
  json += "\"shadow_model_policy\":\"log_only_never_used_for_q_selection_pulse_width_or_motor_command\",";
  json += "\"shadow_v57_policy\":\"support_aware_inverse_shadow_log_only\",";
  json += "\"shadow_v58_repeatability_gate_version\":\"" + String(Config::ZERO_CROSS_V58_REPEATABILITY_GATE_VERSION) + "\",";
  json += "\"shadow_v58_repeatability_gate_policy\":\"log_only_marks_analysis_subset_never_changes_q_pulse_or_motor\",";
  json += "\"v59_state_wait_fixed_q_policy\":\"state_gate_admits_only_preplanned_fixed_q;inverse_q_and_hfree_are_diagnostic_only\",";
  json += "\"v60_rebuild_phase_policy\":\"same_fixed_q_gate_and_order;minimum_six_free_transitions_per_side_then_wait_for_measured_h_gate_support;unrestricted_rebuild_commands_only_the_opposite_arrival_side;the_observed_controlled_peak_must_predict_post_cooldown_H_and_C_inside_the_unchanged_gate\",";
  json += "\"v62_state_feasibility_version\":\"" + String(Config::ZERO_CROSS_V62_STATE_FEASIBILITY_VERSION) + "\",";
  json += "\"v62_state_feasibility_policy\":\"free_decay_rate=aH+bC+c_selects_only_supported_H_C_rate_intersection;fixed_Q_gate_plan_and_motor_command_policy_unchanged\",";
  json += "\"v62_calibration_algorithm_revision\":\"v62_joint_h_c_rate_feasibility_fixed_q_v51_video_hfree\",";
  json += "\"shadow_reachability_policy\":\"log_only_no_pulse_below_or_invalid,inverse_q_within,saturate_qmax_above\",";
  json += "\"shadow_delta_h_model\":\"DeltaH_video=-1.1393659-0.4714266*Hprev+0.0823895*abs(rate)-0.0961605*next_peak_side+0.1919561*Q_effective_pred\",";
  json += "\"shadow_model_state_timing\":\"Hprev_Cprev_rate_and_next_peak_side_are_captured_before_pulse; post_pulse_center_is_not_a_model_input\",";
  json += "\"shadow_q_effective_support_mA_s\":\"0.50-1.60\",";
  json += "\"shadow_current_control_candidate_q_range_mA_s\":\"0.50-1.50\",";
  json += "\"shadow_state_hprev_support_deg\":\"5.98-8.67\",";
  json += "\"shadow_state_abs_rate_support_dps\":\"51.97-73.15\",";
  json += "\"shadow_h_free_video_model\":\"Hfree_video=-0.1960+0.94286*Hfree_imu\",";
  json += "\"shadow_h_post_coordinate\":\"video_half_range\",";
  json += "\"shadow_h_ref_configured\":" + String(Config::ZERO_CROSS_V53_H_REF_CONFIGURED ? "true" : "false") + ",";
  json += "\"shadow_h_ref_video_deg\":" + String(Config::ZERO_CROSS_V53_H_REF_VIDEO_DEG, 3) + ",";
  json += "\"shadow_q_candidate_mA_s\":\"0,0.5,1.0,1.5\",";
  json += "\"shadow_q0_is_extrapolated\":true,";
  json += "\"shadow_q_req_region_codes\":\"0=unavailable,1=below_zero,2=within_zero_to_qmax1p5,3=above_qmax1p5\",";
  json += "\"shadow_q_req_reason_codes\":\"0=within_range,1=reference_unconfigured,2=state_invalid,3=below_zero,4=above_qmax,5=outside_model_support,6=state_out_of_support,7=h_free_invalid\",";
  json += "\"shadow_reachability_reason_codes\":\"0=unavailable,1=reference_unconfigured,2=state_invalid,3=state_out_of_support,4=h_free_invalid,5=candidates_invalid,6=target_below_candidate_range,7=target_within_candidate_range,8=target_above_candidate_range\",";
  json += "\"shadow_recommended_action_codes\":\"0=unavailable,1=no_pulse,2=inverse_q,3=saturate_qmax\",";
  json += "\"shadow_q_support_reason_codes\":\"0=unavailable,1=negative,2=below_forward_model_support,3=within_current_candidates,4=above_candidates_within_forward_model_support,5=above_forward_model_support\",";
  json += "\"shadow_v57_observed_hprev_envelope_deg\":\"5.69-9.01\",";
  json += "\"shadow_v57_observed_abs_rate_envelope_dps\":\"55.01-74.38\",";
  json += "\"shadow_v57_q_req_reason_codes\":\"0=unavailable,1=valid,2=reference_unconfigured,3=state_invalid,4=state_out_of_support,5=h_free_invalid,6=q_below_candidate_range,7=q_above_candidate_range,8=reference_unreachable\",";
  json += "\"shadow_v58_repeatability_gate\":\"Hprev=6.40-6.80deg,abs_rate=57.50-60.50dps,next_peak_side=+1\",";
  json += "\"shadow_v58_repeatability_gate_reason_codes\":\"0=unavailable,1=passed,2=disabled,3=state_invalid,4=hprev_below,5=hprev_above,6=rate_below,7=rate_above,8=direction_mismatch\",";
  json += "\"v60_gate_skipped_by_decay_definition\":\"same_desired_arrival_hprev_above_then_hprev_below_without_accepted_gate;diagnostic_not_proof_of_C_or_rate_match\",";
  json += "\"shadow_v57_imu_proxy_residual_definition\":\"shadow_delta_h_video_pred_deg-delta_half_range_dynamic_hold073_deg; video validation is offline only\",";
  json += "\"sample_size\":" + String(sizeof(LogSample)) + ",";
  json += "\"log_period_ms\":" + String(Config::LOG_PERIOD_MS) + ",";
  json += "\"imu_period_ms\":" + String(Config::IMU_PERIOD_MS) + ",";
  json += "\"measurement_goal\":\"" + String(passive_capture_ ?
      "absolute_angle_motor_inhibited_manual_free_decay" :
      "compare_fixed_beta_0p100_and_fixed_beta_0p000_with_dynamic_beta_max025_hold073_hold120_hold170_zero_cross_input") + "\",";
  json += "\"startup_gyro_calib_ms\":" + String(Config::STARTUP_GYRO_CALIB_MS) + ",";
  json += "\"madgwick_settling_ms\":" + String(Config::MADGWICK_SETTLING_MS) + ",";
  json += "\"sync_led_pin\":" + String(Config::SYNC_LED_PIN) + ",";
  json += "\"led_sync_pattern_id\":\"" + String(Config::LED_SYNC_PATTERN_ID) + "\",";
  json += "\"led_sync_start_pattern\":\"OFF_1000_ON_600_OFF_300_ON_600_OFF_300_ON_1200_OFF_1000\",";
  json += "\"led_sync_end_pattern\":\"OFF_1000_ON_1200_OFF_300_ON_600_OFF_300_ON_600_OFF_1000\",";
  json += "\"led_sync_short_on_ms\":" + String(Config::LED_SYNC_SHORT_ON_MS) + ",";
  json += "\"led_sync_long_on_ms\":" + String(Config::LED_SYNC_LONG_ON_MS) + ",";
  json += "\"led_sync_half_off_ms\":" + String(Config::LED_SYNC_HALF_OFF_MS) + ",";
  json += "\"led_sync_boundary_off_ms\":" + String(Config::LED_SYNC_BOUNDARY_OFF_MS) + ",";
  json += "\"led_sync_start_total_ms\":" + String(Config::START_SYNC_PATTERN_TOTAL_MS) + ",";
  json += "\"led_sync_end_total_ms\":" + String(Config::END_SYNC_PATTERN_TOTAL_MS) + ",";
  json += "\"led_sync_mid_pattern\":\"ON_300_OFF_300_ON_600\",";
  json += "\"led_sync_mid_first_ms\":" + String(Config::MID_SYNC_FIRST_MS) + ",";
  json += "\"led_sync_mid_interval_ms\":" + String(Config::MID_SYNC_INTERVAL_MS) + ",";
  json += "\"led_sync_mid_total_ms\":" + String(Config::MID_SYNC_TOTAL_MS) + ",";
  json += "\"video_sync_method\":\"fit_video_time_to_logged_start_mid_end_led_transitions\",";
  json += "\"experiment_start_definition\":\"t_test_zero_is_after_start_signature_final_off_all_led_states_logged\",";
  json += "\"trial_duration_ms\":" + String(Config::BETA_SWEEP_TRIAL_DURATION_MS) + ",";
  json += "\"inter_trial_rest_ms\":" + String(Config::BETA_SWEEP_INTER_TRIAL_REST_MS) + ",";
  json += "\"batch_total_duration_ms\":" + String(Config::BETA_SWEEP_TOTAL_DURATION_MS) + ",";
  json += "\"current_mA\":" + String(run_current_mA_) + ",";
  json += "\"pulse_width_ms\":" + String(run_pulse_width_ms_) + ",";
  json += "\"input_interval_ms\":" + String(run_input_interval_ms_) + ",";
  json += "\"input_pattern\":\"" + String(passive_capture_ ?
      "manual_release_no_motor_command" :
      "zero_cross_gated_alternating_fixed_current_after_bootstrap") + "\",";
  json += "\"beta_recovery_rule\":\"series_specific_input_hold_softstart_time_matched_linear__zero_cross_event_resets_each_timer\",";
  json += "\"beta_hold_after_input_ms_reference\":" +
          String(Config::DYNAMIC_BETA_HOLD_AFTER_INPUT_MS[Config::FILTER_ADOPTED_INDEX]) + ",";
  json += "\"beta_hold_series_ms\":\"73,120,170\",";
  json += "\"turn_fast_series\":\"low_beta_until_confirmed_turn_then_25ms_smooth_recovery\",";
  json += "\"turn_fast_confirm_samples\":" + String(Config::BETA_TURN_FAST_CONFIRM_SAMPLES) + ",";
  json += "\"turn_fast_recovery_ms\":" + String(Config::BETA_TURN_FAST_RECOVERY_MS) + ",";
  json += "\"turn_fast_fallback_after_input_ms\":" + String(Config::BETA_TURN_FAST_FALLBACK_MS) + ",";
  json += "\"fixed_beta_series\":\"0.100,0.000\",";
  json += "\"beta_soft_start_ms\":" + String(Config::BETA_SOFT_START_MS) + ",";
  json += "\"beta_time_match_reference_recovery_per_ms\":" + String(Config::BETA_TIME_MATCH_REFERENCE_RECOVERY_PER_MS, 7) + ",";
  json += "\"beta_dynamic_ceiling\":0.025,";
  const uint32_t zero_cross_duration_ms = q_run_mode_ == 2 && Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED
      ? Config::ZERO_CROSS_V59_TEST_DURATION_MS : Config::ZERO_CROSS_TEST_DURATION_MS;
  json += "\"zero_cross_test_duration_ms\":" + String(zero_cross_duration_ms) + ",";
  json += "\"zero_cross_amplitude_control\":\"disabled_fixed_current\",";
  json += "\"zero_cross_allowed_current_range_mA\":\"300 (fixed)\",";
  json += "\"zero_cross_allowed_pulse_width_range_ms\":\"5-25 (integer)\",";
  json += "\"zero_cross_fixed_current_mA\":" + String(run_current_mA_) + ",";
  json += "\"zero_cross_fixed_pulse_ms\":" + String(run_pulse_width_ms_) + ",";
  json += "\"zero_cross_start_kick_current_mA\":" + String(run_current_mA_) + ",";
  json += "\"zero_cross_start_kick_ms\":" + String(run_pulse_width_ms_) + ",";
  json += "\"zero_cross_start_refractory_ms\":" + String(Config::ZERO_CROSS_START_REFRACTORY_MS) + ",";
  json += "\"zero_cross_rearm_angle_deg\":" + String(Config::ZERO_CROSS_REARM_ANGLE_DEG, 3) + ",";
  json += "\"zero_cross_min_rate_dps\":" + String(Config::ZERO_CROSS_MIN_RATE_DPS, 3) + ",";
  json += "\"zero_cross_min_pulse_interval_ms\":" + String(Config::ZERO_CROSS_MIN_PULSE_INTERVAL_MS) + ",";
  json += "\"zero_cross_bootstrap_direction\":" + String(Config::ZERO_CROSS_BOOTSTRAP_DIRECTION) + ",";
  json += "\"zero_cross_direction_rule\":\"alternate_each_accepted_crossing_after_bootstrap\",";
  json += "\"beta_phase_return_test_enabled\":" + String(Config::BETA_PHASE_RETURN_TEST_ENABLED ? "true" : "false") + ",";
  json += "\"beta_phase_exponential_k\":" + String(Config::BETA_PHASE_EXPONENTIAL_K, 3) + ",";
  json += "\"beta_phase_outbound_rate_min_dps\":" + String(Config::BETA_PHASE_OUTBOUND_RATE_MIN_DPS, 3) + ",";
  json += "\"beta_phase_turn_confirm_rate_dps\":" + String(Config::BETA_PHASE_TURN_CONFIRM_RATE_DPS, 3) + ",";
  json += "\"beta_phase_min_peak_angle_deg\":" + String(Config::BETA_PHASE_MIN_PEAK_ANGLE_DEG, 3) + ",";
  json += "\"beta_phase_peak_source\":\"bias_corrected_gyro_integral_relative_to_pulse_start\",";
  json += "\"beta_recovery_tau_s\":" + String(Config::BETA_RECOVERY_TAU_S, 5) + ",";
  json += "\"beta_normal\":" + String(Config::MADGWICK_BETA_NORMAL, 5) + ",";
  json += "\"beta_min_at_zero_current\":" + String(Config::BETA_MIN_AT_ZERO_CURRENT, 5) + ",";
  json += "\"beta_min_at_reference_current\":" + String(Config::BETA_MIN_AT_REFERENCE_CURRENT, 5) + ",";
  json += "\"beta_min_reference_current_mA\":" + String(Config::BETA_MIN_REFERENCE_CURRENT_MA, 3) + ",";
  json += "\"model_vbat_reference_v\":" + String(Config::MODEL_VBAT_REFERENCE_V, 3) + ",";
  json += "\"model_vbat_min_v\":" + String(Config::MODEL_VBAT_MIN_V, 3) + ",";
  json += "\"model_vbat_max_v\":" + String(Config::MODEL_VBAT_MAX_V, 3) + ",";
  json += "\"model_i_sat_at_reference_mA\":" + String(Config::MODEL_I_SAT_AT_REFERENCE_MA, 6) + ",";
  json += "\"model_i_sat_slope_mA_per_v\":" + String(Config::MODEL_I_SAT_SLOPE_MA_PER_V, 6) + ",";
  json += "\"model_i_sat_exponent\":" + String(Config::MODEL_I_SAT_EXPONENT, 3) + ",";
  json += "\"model_tau_rise_min_ms\":" + String(Config::MODEL_TAU_RISE_MIN_MS, 3) + ",";
  json += "\"model_tau_rise_max_ms\":" + String(Config::MODEL_TAU_RISE_MAX_MS, 3) + ",";
  json += "\"model_tau_rise_u_mA\":" + String(Config::MODEL_TAU_RISE_U_MA, 3) + ",";
  json += "\"model_tau_rise_exponent\":" + String(Config::MODEL_TAU_RISE_EXPONENT, 3) + ",";
  json += "\"filter_strategies\":[";
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    if (i) json += ",";
    json += "\"" + String(Config::DYNAMIC_BETA_LABELS[i]) + "\"";
  }
  json += "],";
  json += "\"trials\":[{";
  json += "\"trial_index\":1,";
  json += "\"name\":\"" + String(passive_capture_ ? "manual_passive_release" : "manual_zero_cross_selected_condition") + "\",";
  json += "\"current_mA\":" + String(run_current_mA_) + ",";
  json += "\"pulse_width_ms\":" + String(run_pulse_width_ms_) + ",";
  json += "\"input_interval_ms\":" + String(run_input_interval_ms_) + ",";
  json += "\"duration_ms\":" + String(passive_capture_ ? Config::PASSIVE_CAPTURE_DURATION_MS : zero_cross_duration_ms);
  json += "}],";
  const char* q_mode = q_run_mode_ == 2 ? "control" : (q_run_mode_ == 1 ? "validation" : "none");
  const uint8_t q_schedule = q_probe_schedule_id_ <
      Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT ? q_probe_schedule_id_ :
      Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  const char* q_schedule_name = q_schedule == Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_B ? "B" :
      (q_schedule == Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_C ? "C" : "A");
  json += "\"q_run_mode\":\"" + String(q_mode) + "\",";
  json += "\"q_probe_schedule_id\":" + String(q_schedule) + ",";
  json += "\"q_probe_schedule_name\":\"" + String(q_schedule_name) + "\",";
  json += "\"q_control_target_peak_deg\":" + String(control_target_cdeg_ / 100.0f, 3) + ",";
  json += "\"q_response_model\":\"A_next=0.022622+0.088146*abs(rate0)-0.073766*abs(theta0)+0.269799*Q_command-0.124417*d-0.022127*Q_command*d\",";
  json += "\"q_control_selection\":\"continuous_inverse_target_peak_Q\",";
  json += "\"q_control_q_max_mA_s\":" + String(Config::ZERO_CROSS_CONTROL_Q_MAX_MAS, 4) + ",";
  json += "\"q_control_solver_min_pulse_ms\":" + String(Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS) + ",";
  json += "\"q_control_solver_max_pulse_ms\":" + String(Config::ZERO_CROSS_CONTROL_MAX_PULSE_MS) + ",";
  json += "\"shadow_offset_deg\":" + String(Config::ZERO_CROSS_Q_MODEL_SHADOW_OFFSET_DEG, 3) + ",";
  json += "\"identification_event_amplitude_fields\":\"a_min_pred_deg/a_max_pred_deg are the four-Q predicted range; a_pred_shadow_deg and a_pred_calibrated_deg are log-only; calibrated_prediction_valid gates the latter; bootstrap=true marks the forced start kick; a_next_imu_abs_deg is the on-device observed peak\",";
  json += "\"v61_state_target_valid\":" + String(calibration_result_.v61_state_target_valid ? "true" : "false") + ",";
  json += "\"v61_predicted_gate_h_deg\":" + String(calibration_result_.v61_predicted_gate_h_deg, 4) + ",";
  json += "\"v61_predicted_gate_c_deg\":" + String(calibration_result_.v61_predicted_gate_c_deg, 4) + ",";
  json += "\"v61_feedback_command_peak_deg\":" + String(calibration_result_.v61_feedback_command_peak_deg, 4) + ",";
  json += "\"v61_feedback_correction_count\":" + String(calibration_result_.v61_feedback_correction_count) + ",";
  json += "\"v62_rate_model_valid\":" + String(calibration_result_.v62_rate_model_valid ? "true" : "false") + ",";
  json += "\"v62_hc_target_feasible\":" + String(calibration_result_.v62_hc_target_feasible ? "true" : "false") + ",";
  json += "\"v62_state_target_valid\":" + String(calibration_result_.v62_state_target_valid ? "true" : "false") + ",";
  json += "\"v62_rate_sample_count\":" + String(calibration_result_.v62_rate_sample_count) + ",";
  json += "\"v62_rate_a_per_s\":" + String(calibration_result_.v62_rate_a_per_s, 5) + ",";
  json += "\"v62_rate_b_per_s\":" + String(calibration_result_.v62_rate_b_per_s, 5) + ",";
  json += "\"v62_rate_offset_dps\":" + String(calibration_result_.v62_rate_offset_dps, 5) + ",";
  json += "\"v62_rate_r2\":" + String(calibration_result_.v62_rate_r2, 5) + ",";
  json += "\"v62_target_h_deg\":" + String(calibration_result_.v62_target_h_deg, 4) + ",";
  json += "\"v62_target_c_deg\":" + String(calibration_result_.v62_target_c_deg, 4) + ",";
  json += "\"v62_target_rate_dps\":" + String(calibration_result_.v62_target_rate_dps, 4) + ",";
  json += "\"v62_target_controlled_peak_deg\":" + String(calibration_result_.v62_target_controlled_peak_deg, 4) + ",";
  if (calibration_result_.v60_rebuild_target_valid) {
    json += "\"v60_free_decay_after_one_cycle_h_deg\":" + String(calibration_result_.v60_free_decay_after_one_cycle_h_deg, 4) + ",";
    json += "\"v60_decay_per_cycle_h_deg\":" + String(calibration_result_.v60_decay_per_cycle_h_deg, 4) + ",";
    json += "\"v60_rebuild_target_h_deg\":" + String(calibration_result_.v60_rebuild_target_h_deg, 4) + ",";
    json += "\"v60_rebuild_target_peak_deg\":" + String(calibration_result_.v60_rebuild_target_peak_deg, 4) + ",";
  } else {
    json += "\"v60_free_decay_after_one_cycle_h_deg\":null,\"v60_decay_per_cycle_h_deg\":null,";
    json += "\"v60_rebuild_target_h_deg\":null,\"v60_rebuild_target_peak_deg\":null,";
  }
  json += "\"cal_q_probe_plan_count\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT) + ",";
  json += "\"cal_q_probe_schedule_legend\":\"V59: all arrivals are +; each schedule has four 3-level blocks and four observations per Q level\",";
  json += "\"cal_q_rebuild_entry_rule\":\"enter next Q probe when either the legacy A_prev common input domain or the valid dynamic-H predecessor input domain contains the confirmed predecessor; no A or H model is extrapolated\",";
  json += "\"cal_q_rebuild_target_audit\":\"per-probe rebuild_target_reached records whether the confirmed predecessor magnitude was at or above cal_q_rebuild_target_peak_deg; the target remains the rebuild-pulse setpoint, not an H-support requirement\",";
  json += "\"cal_free_selection_rule\":\"select lower-LOOCV-RMSE positive monotone half-cycle maps over measured A_prev domains; require each composed same-side full cycle 0<F(A)<A over its composition domain; no extrapolation\",";
  json += "\"cal_q_probe_input_rule\":\"use A_prev input-domain intersection across both arrival-side models, reduced by cal_q_probe_support_margin_deg; issue only the next deterministic desired arrival side; if the immediate arrival is the other side, send no Q pulse for one half-cycle then re-check support; out-of-support waits return through the existing bounded rebuild; BASE direction=-desired_arrival_side and is never auto-flipped\",";
  json += "\"cal_half_range_shadow_coordinate\":\"H=abs(theta_k-theta_k_minus_1)/2; C=(theta_k+theta_k_minus_1)/2; peak timing is dynamic_hold073 and fixed_b100 is sampled at that same time\",";
  json += "\"calibration_build_up_event_fields\":\"phase 0=initial build-up,1=post-fit Q-probe rebuild (one event per bounded attempt); target_peak_deg is the requested peak; q_required=unclamped inverse-model Q; q_command=after guard/Q cap; q_effective=current-model Q at the selected integer width; predicted_peak uses q_effective; q_cap_limited/guard_limited/width_limited mark active constraints\",";
  json += "\"calibration_build_up_events\":[";
  for (uint8_t i = 0; i < calibration_build_up_event_count_; ++i) {
    const CalibrationBuildUpEvent& e = events_->calibration_build_up_events_[i];
    if (i) json += ",";
    json += "{\"pulse_index\":" + String(e.pulse_index);
    json += ",\"start_ms\":" + String(e.start_ms);
    json += ",\"theta0_deg\":" + String(e.theta0_cdeg / 100.0f, 3);
    json += ",\"rate0_dps\":" + String(e.rate0_cdps / 100.0f, 3);
    json += ",\"direction\":" + String(e.direction);
    json += ",\"phase\":" + String(e.phase);
    json += ",\"target_peak_deg\":" + String(e.target_peak_cdeg / 100.0f, 3);
    json += ",\"vbat_mV\":" + String(e.vbat_mV);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    json += ",\"q_required_mA_s\":" + String(e.q_required_mAms / 1000.0f, 4);
    json += ",\"q_command_mA_s\":" + String(e.q_command_mAms / 1000.0f, 4);
    json += ",\"q_effective_mA_s\":" + String(e.q_effective_mAms / 1000.0f, 4);
    json += ",\"predicted_peak_deg\":" + String(e.predicted_peak_cdeg / 100.0f, 3);
    json += ",\"q_cap_limited\":" + String(e.q_cap_limited ? "true" : "false");
    json += ",\"guard_limited\":" + String(e.guard_limited ? "true" : "false");
    json += ",\"width_limited\":" + String(e.width_limited ? "true" : "false") + "}";
  }
  json += "],";
  json += "\"calibration_peak_event_fields\":\"candidate_peak_ms is the physical extremum time found by dynamic_hold073; confirmed_ms is the later reversal-confirmation time; signed, center, and half_range preserve dynamic_hold073 and fixed_b100 at the same candidate time\",";
  json += "\"calibration_peak_events\":[";
  for (uint8_t i = 0; i < calibration_peak_event_count_; ++i) {
    const CalibrationPeakEvent& p = events_->calibration_peak_events_[i];
    if (i) json += ",";
    json += "{\"peak_index\":" + String(p.peak_index);
    json += ",\"candidate_peak_ms\":" + String(p.candidate_peak_ms);
    json += ",\"confirmed_ms\":" + String(p.confirmed_ms);
    json += ",\"peak_side\":" + String(p.peak_side);
    json += ",\"phase\":" + String(p.phase);
    json += ",\"peak_abs_deg\":" + String(p.peak_abs_cdeg / 100.0f, 3);
    if (p.prev_peak_abs_cdeg == LOG_NAN_I16) json += ",\"prev_peak_abs_deg\":null";
    else json += ",\"prev_peak_abs_deg\":" + String(p.prev_peak_abs_cdeg / 100.0f, 3);
    if (p.peak_signed_dynamic_cdeg == LOG_NAN_I16) json += ",\"peak_signed_dynamic_hold073_deg\":null";
    else json += ",\"peak_signed_dynamic_hold073_deg\":" + String(p.peak_signed_dynamic_cdeg / 100.0f, 3);
    if (p.prev_peak_signed_dynamic_cdeg == LOG_NAN_I16) json += ",\"prev_peak_signed_dynamic_hold073_deg\":null";
    else json += ",\"prev_peak_signed_dynamic_hold073_deg\":" + String(p.prev_peak_signed_dynamic_cdeg / 100.0f, 3);
    if (p.center_dynamic_cdeg == LOG_NAN_I16) json += ",\"center_dynamic_hold073_deg\":null";
    else json += ",\"center_dynamic_hold073_deg\":" + String(p.center_dynamic_cdeg / 100.0f, 3);
    if (p.half_range_dynamic_cdeg == LOG_NAN_I16) json += ",\"half_range_dynamic_hold073_deg\":null";
    else json += ",\"half_range_dynamic_hold073_deg\":" + String(p.half_range_dynamic_cdeg / 100.0f, 3);
    if (p.peak_signed_fixed_cdeg == LOG_NAN_I16) json += ",\"peak_signed_fixed_b100_deg\":null";
    else json += ",\"peak_signed_fixed_b100_deg\":" + String(p.peak_signed_fixed_cdeg / 100.0f, 3);
    if (p.prev_peak_signed_fixed_cdeg == LOG_NAN_I16) json += ",\"prev_peak_signed_fixed_b100_deg\":null";
    else json += ",\"prev_peak_signed_fixed_b100_deg\":" + String(p.prev_peak_signed_fixed_cdeg / 100.0f, 3);
    if (p.center_fixed_cdeg == LOG_NAN_I16) json += ",\"center_fixed_b100_deg\":null";
    else json += ",\"center_fixed_b100_deg\":" + String(p.center_fixed_cdeg / 100.0f, 3);
    if (p.half_range_fixed_cdeg == LOG_NAN_I16) json += ",\"half_range_fixed_b100_deg\":null";
    else json += ",\"half_range_fixed_b100_deg\":" + String(p.half_range_fixed_cdeg / 100.0f, 3);
    json += ",\"q_effective_pred_mA_s\":" + String(p.q_effective_pred_mAms / 1000.0f, 4) + "}";
  }
  json += "],";
  json += "\"calibration_probe_event_fields\":\"v51 q_probe_schedule_id, planned_probe_index, and q_level_index identify the balanced signed-Q order; command_dynamic_angle and command_rate fields are sampled immediately before the Q pulse (raw and gyro-bias-corrected pitch rate); q_effective_pred is the current-model prediction from residual I0, not a sensor-integrated charge; half_range_previous, center, free_pred, observed, and delta fields are the dynamic-H H_prev, post-pulse C, H_free_pred, H_post, and Delta_H audit tuple. v52 shadow_* fields use only pre-pulse Hprev/C_prev/abs(rate)/expected next-peak side/Q_effective_pred; the post-pulse center is explicitly excluded. v57 independently records H/rate support, raw inverse-Q support and candidate validity, and an IMU dynamic-H proxy residual; video validation is joined offline only. All shadow fields are audit-only and cannot alter normal control, calibration, pulse width, or motor command\",";
  json += "\"calibration_probe_events\":[";
  for (uint8_t i = 0; i < calibration_probe_event_count_; ++i) {
    const CalibrationProbeEvent& e = events_->calibration_probe_events_[i];
    if (i) json += ",";
    json += "{\"probe_index\":" + String(e.probe_index);
    json += ",\"planned_probe_index\":" + String(e.planned_probe_index);
    json += ",\"q_level_index\":" + String(e.q_level_index);
    json += ",\"q_probe_schedule_id\":" + String(e.q_probe_schedule_id);
    json += ",\"polarity_mode\":" + String(e.polarity_mode);
    json += ",\"rebuild_count\":" + String(e.rebuild_count);
    json += ",\"rebuild_episode_id\":" + String(e.rebuild_episode_id);
    json += ",\"rebuild_attempt_in_episode\":" + String(e.rebuild_attempt_in_episode);
    json += ",\"rebuild_entry_source\":" + String(e.rebuild_entry_source);
    json += ",\"wait_halfcycle_count\":" + String(e.wait_halfcycle_count);
    json += ",\"result_code\":" + String(e.result_code);
    json += ",\"pulse_start_ms\":" + String(e.pulse_start_ms);
    json += ",\"candidate_peak_ms\":" + String(e.candidate_peak_ms);
    json += ",\"confirmed_ms\":" + String(e.confirmed_ms);
    json += ",\"previous_peak_candidate_ms\":" + String(e.previous_peak_candidate_ms);
    json += ",\"requested_side\":" + String(e.requested_side);
    json += ",\"predecessor_side\":" + String(e.predecessor_side);
    json += ",\"observed_side\":" + String(e.observed_side);
    json += ",\"pulse_direction\":" + String(e.pulse_direction);
    json += ",\"pulse_id\":" + String(e.pulse_id);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    if (e.command_dynamic_angle_cdeg == LOG_NAN_I16) json += ",\"command_dynamic_angle_deg\":null";
    else json += ",\"command_dynamic_angle_deg\":" + String(e.command_dynamic_angle_cdeg / 100.0f, 3);
    if (e.command_rate_raw_cdps == LOG_NAN_I16) json += ",\"command_rate_raw_dps\":null";
    else json += ",\"command_rate_raw_dps\":" + String(e.command_rate_raw_cdps / 100.0f, 3);
    if (e.command_rate_bias_corrected_cdps == LOG_NAN_I16) json += ",\"command_rate_bias_corrected_dps\":null";
    else json += ",\"command_rate_bias_corrected_dps\":" + String(e.command_rate_bias_corrected_cdps / 100.0f, 3);
    if (e.previous_peak_abs_cdeg == LOG_NAN_I16) json += ",\"previous_peak_abs_deg\":null";
    else json += ",\"previous_peak_abs_deg\":" + String(e.previous_peak_abs_cdeg / 100.0f, 3);
    if (e.observed_peak_abs_cdeg == LOG_NAN_I16) json += ",\"observed_peak_abs_deg\":null";
    else json += ",\"observed_peak_abs_deg\":" + String(e.observed_peak_abs_cdeg / 100.0f, 3);
    if (e.previous_peak_signed_dynamic_cdeg == LOG_NAN_I16) json += ",\"previous_peak_signed_dynamic_hold073_deg\":null";
    else json += ",\"previous_peak_signed_dynamic_hold073_deg\":" + String(e.previous_peak_signed_dynamic_cdeg / 100.0f, 3);
    if (e.observed_peak_signed_dynamic_cdeg == LOG_NAN_I16) json += ",\"observed_peak_signed_dynamic_hold073_deg\":null";
    else json += ",\"observed_peak_signed_dynamic_hold073_deg\":" + String(e.observed_peak_signed_dynamic_cdeg / 100.0f, 3);
    if (e.center_dynamic_cdeg == LOG_NAN_I16) json += ",\"center_dynamic_hold073_deg\":null";
    else json += ",\"center_dynamic_hold073_deg\":" + String(e.center_dynamic_cdeg / 100.0f, 3);
    if (e.half_range_previous_dynamic_cdeg == LOG_NAN_I16) json += ",\"half_range_previous_dynamic_hold073_deg\":null";
    else json += ",\"half_range_previous_dynamic_hold073_deg\":" + String(e.half_range_previous_dynamic_cdeg / 100.0f, 3);
    if (e.half_range_observed_dynamic_cdeg == LOG_NAN_I16) json += ",\"half_range_observed_dynamic_hold073_deg\":null";
    else json += ",\"half_range_observed_dynamic_hold073_deg\":" + String(e.half_range_observed_dynamic_cdeg / 100.0f, 3);
    if (e.half_range_free_pred_dynamic_cdeg == LOG_NAN_I16) json += ",\"half_range_free_pred_dynamic_hold073_deg\":null";
    else json += ",\"half_range_free_pred_dynamic_hold073_deg\":" + String(e.half_range_free_pred_dynamic_cdeg / 100.0f, 3);
    if (e.delta_half_range_dynamic_cdeg == LOG_NAN_I16) json += ",\"delta_half_range_dynamic_hold073_deg\":null";
    else json += ",\"delta_half_range_dynamic_hold073_deg\":" + String(e.delta_half_range_dynamic_cdeg / 100.0f, 3);
    if (e.gain_half_range_dynamic_cdeg_per_mAs == LOG_NAN_I16) json += ",\"gain_half_range_dynamic_hold073_deg_per_mA_s\":null";
    else json += ",\"gain_half_range_dynamic_hold073_deg_per_mA_s\":" + String(e.gain_half_range_dynamic_cdeg_per_mAs / 100.0f, 3);
    if (e.previous_peak_signed_fixed_cdeg == LOG_NAN_I16) json += ",\"previous_peak_signed_fixed_b100_deg\":null";
    else json += ",\"previous_peak_signed_fixed_b100_deg\":" + String(e.previous_peak_signed_fixed_cdeg / 100.0f, 3);
    if (e.observed_peak_signed_fixed_cdeg == LOG_NAN_I16) json += ",\"observed_peak_signed_fixed_b100_deg\":null";
    else json += ",\"observed_peak_signed_fixed_b100_deg\":" + String(e.observed_peak_signed_fixed_cdeg / 100.0f, 3);
    if (e.center_fixed_cdeg == LOG_NAN_I16) json += ",\"center_fixed_b100_deg\":null";
    else json += ",\"center_fixed_b100_deg\":" + String(e.center_fixed_cdeg / 100.0f, 3);
    if (e.half_range_previous_fixed_cdeg == LOG_NAN_I16) json += ",\"half_range_previous_fixed_b100_deg\":null";
    else json += ",\"half_range_previous_fixed_b100_deg\":" + String(e.half_range_previous_fixed_cdeg / 100.0f, 3);
    if (e.half_range_observed_fixed_cdeg == LOG_NAN_I16) json += ",\"half_range_observed_fixed_b100_deg\":null";
    else json += ",\"half_range_observed_fixed_b100_deg\":" + String(e.half_range_observed_fixed_cdeg / 100.0f, 3);
    if (e.half_range_free_pred_fixed_cdeg == LOG_NAN_I16) json += ",\"half_range_free_pred_fixed_b100_deg\":null";
    else json += ",\"half_range_free_pred_fixed_b100_deg\":" + String(e.half_range_free_pred_fixed_cdeg / 100.0f, 3);
    if (e.delta_half_range_fixed_cdeg == LOG_NAN_I16) json += ",\"delta_half_range_fixed_b100_deg\":null";
    else json += ",\"delta_half_range_fixed_b100_deg\":" + String(e.delta_half_range_fixed_cdeg / 100.0f, 3);
    if (e.gain_half_range_fixed_cdeg_per_mAs == LOG_NAN_I16) json += ",\"gain_half_range_fixed_b100_deg_per_mA_s\":null";
    else json += ",\"gain_half_range_fixed_b100_deg_per_mA_s\":" + String(e.gain_half_range_fixed_cdeg_per_mAs / 100.0f, 3);
    json += ",\"half_range_dynamic_in_support\":" + String(e.half_range_dynamic_in_support ? "true" : "false");
    json += ",\"half_range_fixed_in_support\":" + String(e.half_range_fixed_in_support ? "true" : "false");
    json += ",\"rebuild_target_reached\":" + String(e.rebuild_target_reached ? "true" : "false");
    json += ",\"dynamic_h_in_support_at_command\":" + String(e.dynamic_h_in_support_at_command ? "true" : "false");
    json += ",\"q_target_mA_s\":" + String(e.q_target_mAms / 1000.0f, 4);
    json += ",\"q_effective_pred_mA_s\":" + String(e.q_effective_pred_mAms / 1000.0f, 4);
    if (e.shadow_hprev_imu_cdeg == LOG_NAN_I16) json += ",\"shadow_Hprev_imu_deg\":null";
    else json += ",\"shadow_Hprev_imu_deg\":" + String(e.shadow_hprev_imu_cdeg / 100.0f, 3);
    if (e.shadow_c_prev_imu_cdeg == LOG_NAN_I16) json += ",\"shadow_C_prev_imu_deg\":null";
    else json += ",\"shadow_C_prev_imu_deg\":" + String(e.shadow_c_prev_imu_cdeg / 100.0f, 3);
    if (e.shadow_rate_abs_cdps == LOG_NAN_I16) json += ",\"shadow_rate_abs_dps\":null";
    else json += ",\"shadow_rate_abs_dps\":" + String(e.shadow_rate_abs_cdps / 100.0f, 3);
    json += ",\"shadow_next_peak_side\":" + String(e.shadow_next_peak_side);
    json += ",\"shadow_direction\":" + String(e.shadow_next_peak_side);
    if (!isfinite(e.shadow_q_effective_pred_mA_s)) json += ",\"shadow_q_effective_pred_mA_s\":null";
    else json += ",\"shadow_q_effective_pred_mA_s\":" + jsonFloatOrNull(e.shadow_q_effective_pred_mA_s, 4);
    if (!isfinite(e.shadow_delta_h_video_pred_deg)) json += ",\"shadow_delta_h_video_pred_deg\":null";
    else json += ",\"shadow_delta_h_video_pred_deg\":" + jsonFloatOrNull(e.shadow_delta_h_video_pred_deg, 4);
    if (!isfinite(e.shadow_delta_h_imu_actual_deg)) json += ",\"shadow_delta_h_imu_actual_deg\":null";
    else json += ",\"shadow_delta_h_imu_actual_deg\":" + jsonFloatOrNull(e.shadow_delta_h_imu_actual_deg, 4);
    if (!isfinite(e.shadow_delta_h_pred_minus_imu_dynamic_deg)) json += ",\"shadow_delta_h_pred_minus_imu_dynamic_deg\":null";
    else json += ",\"shadow_delta_h_pred_minus_imu_dynamic_deg\":" + jsonFloatOrNull(e.shadow_delta_h_pred_minus_imu_dynamic_deg, 4);
    if (!isfinite(e.shadow_h_free_imu_pred_deg)) json += ",\"shadow_H_free_imu_pred_deg\":null";
    else json += ",\"shadow_H_free_imu_pred_deg\":" + jsonFloatOrNull(e.shadow_h_free_imu_pred_deg, 4);
    if (!isfinite(e.shadow_h_free_video_pred_deg)) json += ",\"shadow_H_free_video_pred_deg\":null";
    else json += ",\"shadow_H_free_video_pred_deg\":" + jsonFloatOrNull(e.shadow_h_free_video_pred_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_deg)) json += ",\"shadow_H_post_pred_deg\":null";
    else json += ",\"shadow_H_post_pred_deg\":" + jsonFloatOrNull(e.shadow_h_post_pred_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_q0_deg)) json += ",\"shadow_H_post_pred_q0_deg\":null";
    else json += ",\"shadow_H_post_pred_q0_deg\":" + jsonFloatOrNull(e.shadow_h_post_pred_q0_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_q05_deg)) json += ",\"shadow_H_post_pred_q05_deg\":null";
    else json += ",\"shadow_H_post_pred_q05_deg\":" + jsonFloatOrNull(e.shadow_h_post_pred_q05_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_q10_deg)) json += ",\"shadow_H_post_pred_q10_deg\":null";
    else json += ",\"shadow_H_post_pred_q10_deg\":" + jsonFloatOrNull(e.shadow_h_post_pred_q10_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_q15_deg)) json += ",\"shadow_H_post_pred_q15_deg\":null";
    else json += ",\"shadow_H_post_pred_q15_deg\":" + jsonFloatOrNull(e.shadow_h_post_pred_q15_deg, 4);
    if (!isfinite(e.shadow_h_supported_min_pred_deg)) json += ",\"shadow_H_supported_min_pred_deg\":null";
    else json += ",\"shadow_H_supported_min_pred_deg\":" + jsonFloatOrNull(e.shadow_h_supported_min_pred_deg, 4);
    if (!isfinite(e.shadow_h_supported_max_pred_deg)) json += ",\"shadow_H_supported_max_pred_deg\":null";
    else json += ",\"shadow_H_supported_max_pred_deg\":" + jsonFloatOrNull(e.shadow_h_supported_max_pred_deg, 4);
    if (!isfinite(e.shadow_h_ref_deg)) json += ",\"shadow_H_ref_deg\":null";
    else json += ",\"shadow_H_ref_deg\":" + jsonFloatOrNull(e.shadow_h_ref_deg, 4);
    if (!isfinite(e.shadow_q_req_raw_mA_s)) json += ",\"shadow_q_req_raw_mA_s\":null";
    else json += ",\"shadow_q_req_raw_mA_s\":" + jsonFloatOrNull(e.shadow_q_req_raw_mA_s, 4);
    if (!isfinite(e.shadow_q_req_clamped_mA_s)) json += ",\"shadow_q_req_clamped_mA_s\":null";
    else json += ",\"shadow_q_req_clamped_mA_s\":" + jsonFloatOrNull(e.shadow_q_req_clamped_mA_s, 4);
    if (!isfinite(e.shadow_recommended_q_mA_s)) json += ",\"shadow_recommended_q_mA_s\":null";
    else json += ",\"shadow_recommended_q_mA_s\":" + jsonFloatOrNull(e.shadow_recommended_q_mA_s, 4);
    json += ",\"shadow_q_req_region\":" + String(e.shadow_q_req_region);
    json += ",\"shadow_q_req_reason\":" + String(e.shadow_q_req_reason);
    json += ",\"shadow_v57_q_req_reason\":" + String(e.shadow_v57_q_req_reason);
    json += ",\"shadow_v58_repeatability_gate_reason\":" + String(e.shadow_v58_repeatability_gate_reason);
    json += ",\"v59_block_index\":" + String(e.v59_block_index);
    json += ",\"v59_block_order\":" + String(e.v59_block_order);
    json += ",\"v59_state_gate_reason\":" + String(e.v59_state_gate_reason);
    json += ",\"shadow_reachability_reason\":" + String(e.shadow_reachability_reason);
    json += ",\"shadow_recommended_action\":" + String(e.shadow_recommended_action);
    json += ",\"shadow_q_support_reason\":" + String(e.shadow_q_support_reason);
    json += ",\"shadow_state_valid\":" + String(e.shadow_state_valid ? "true" : "false");
    json += ",\"shadow_h_in_support\":" + String(e.shadow_h_in_support ? "true" : "false");
    json += ",\"shadow_rate_in_support\":" + String(e.shadow_rate_in_support ? "true" : "false");
    json += ",\"shadow_state_in_support\":" + String(e.shadow_state_in_support ? "true" : "false");
    json += ",\"shadow_q_in_model_support\":" + String(e.shadow_q_in_model_support ? "true" : "false");
    json += ",\"shadow_q_req_in_model_support\":" + String(e.shadow_q_req_in_model_support ? "true" : "false");
    json += ",\"shadow_q_req_in_control_candidate_range\":" + String(e.shadow_q_req_in_control_candidate_range ? "true" : "false");
    json += ",\"shadow_q_req_candidate_valid\":" + String(e.shadow_q_req_candidate_valid ? "true" : "false");
    json += ",\"shadow_v58_repeatability_gate_passed\":" + String(e.shadow_v58_repeatability_gate_passed ? "true" : "false");
    json += ",\"v59_state_gate_passed\":" + String(e.v59_state_gate_passed ? "true" : "false");
    json += ",\"shadow_imu_proxy_residual_valid\":" + String(e.shadow_imu_proxy_residual_valid ? "true" : "false");
    json += ",\"shadow_delta_h_valid\":" + String(e.shadow_delta_h_valid ? "true" : "false");
    json += ",\"shadow_H_free_valid\":" + String(e.shadow_h_free_valid ? "true" : "false");
    json += ",\"shadow_H_free_video_valid\":" + String(e.shadow_h_free_video_valid ? "true" : "false");
    json += ",\"shadow_H_post_valid\":" + String(e.shadow_h_post_valid ? "true" : "false");
    json += ",\"shadow_H_post_candidates_valid\":" + String(e.shadow_h_post_candidates_valid ? "true" : "false");
    json += ",\"shadow_q0_extrapolated\":" + String(e.shadow_q0_extrapolated ? "true" : "false");
    json += ",\"shadow_q_req_valid\":" + String(e.shadow_q_req_valid ? "true" : "false");
    json += ",\"shadow_href_in_current_control_candidate_reachable_range\":" + String(e.shadow_href_in_current_control_candidate_reachable_range ? "true" : "false");
    json += ",\"shadow_reachability_valid\":" + String(e.shadow_reachability_valid ? "true" : "false");
    json += ",\"shadow_future_control_eligible\":" + String(e.shadow_future_control_eligible ? "true" : "false");
    json += ",\"shadow_control_candidate\":" + String(e.shadow_control_candidate ? "true" : "false");
    if (e.gain_cdeg_per_mAs == LOG_NAN_I16) json += ",\"gain_deg_per_mA_s\":null";
    else json += ",\"gain_deg_per_mA_s\":" + String(e.gain_cdeg_per_mAs / 100.0f, 3);
    json += ",\"post_pulse\":" + String(e.post_pulse ? "true" : "false");
    json += ",\"in_support\":" + String(e.in_support ? "true" : "false");
    json += ",\"side_matched\":" + String(e.side_matched ? "true" : "false");
    json += ",\"direction_matches_base\":" + String(e.direction_matches_base ? "true" : "false");
    json += ",\"positive_gain\":" + String(e.positive_gain ? "true" : "false");
    json += ",\"gain_stored\":" + String(e.gain_stored ? "true" : "false") + "}";
  }
  json += "],";
  json += "\"calibration_state_gate_events\":[";
  for (uint8_t i = 0; i < calibration_state_gate_event_count_; ++i) {
    const CalibrationStateGateEvent& e = events_->calibration_state_gate_events_[i];
    if (i) json += ",";
    json += "{\"event_index\":" + String(e.event_index);
    json += ",\"planned_probe_index\":" + String(e.planned_probe_index);
    json += ",\"q_level_index\":" + String(e.q_level_index);
    json += ",\"reason\":" + String(e.reason);
    json += ",\"action\":" + String(e.action);
    json += ",\"wait_halfcycle_count\":" + String(e.wait_halfcycle_count);
    json += ",\"rebuild_total_count\":" + String(e.rebuild_total_count);
    json += ",\"crossing_ms\":" + String(e.crossing_ms);
    if (e.hprev_cdeg == LOG_NAN_I16) json += ",\"Hprev_imu_deg\":null";
    else json += ",\"Hprev_imu_deg\":" + String(e.hprev_cdeg / 100.0f, 3);
    if (e.cprev_cdeg == LOG_NAN_I16) json += ",\"Cprev_imu_deg\":null";
    else json += ",\"Cprev_imu_deg\":" + String(e.cprev_cdeg / 100.0f, 3);
    if (e.rate_abs_cdps == LOG_NAN_I16) json += ",\"rate_abs_dps\":null";
    else json += ",\"rate_abs_dps\":" + String(e.rate_abs_cdps / 100.0f, 3);
    json += ",\"next_peak_side\":" + String(e.next_peak_side);
    json += ",\"state_valid\":" + String(e.state_valid ? "true" : "false");
    json += ",\"dynamic_h_in_support\":" + String(e.dynamic_h_in_support ? "true" : "false");
    json += ",\"v60_cooldown_free_decay\":" + String(e.v60_cooldown_free_decay ? "true" : "false");
    json += ",\"v60_gate_skipped_by_decay\":" + String(e.v60_gate_skipped_by_decay ? "true" : "false") + "}";
  }
  json += "],";  json += "\"identification_events\":[";
  for (uint16_t i = 0; i < identification_event_count_; ++i) {
    const IdentificationEvent& e = events_->identification_events_[i];
    if (i) json += ",";
    json += "{\"event_id\":" + String(e.event_id);
    json += ",\"start_ms\":" + String(e.start_ms) + ",\"peak_ms\":" + String(e.peak_ms);
    json += ",\"theta0_deg\":" + String(e.theta0_cdeg / 100.0f, 3);
    json += ",\"rate0_dps\":" + String(e.rate0_cdps / 100.0f, 3);
    json += ",\"theta_peak_deg\":" + String(e.theta_peak_cdeg / 100.0f, 3);
    json += ",\"direction\":" + String(e.direction) + ",\"vbat_mV\":" + String(e.vbat_mV);
    json += ",\"i0_estimated_mA\":" + String(e.i0_mA) + ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    json += ",\"q_target_mA_s\":" + String(e.q_target_mAms / 1000.0f, 4);
    json += ",\"q_requested_mA_s\":" + String(e.q_requested_mAms / 1000.0f, 4);
    json += ",\"q_command_mA_s\":" + String(e.q_command_mAms / 1000.0f, 4);
    json += ",\"q_estimated_signed_mA_s\":" + String(e.q_estimated_mAms / 1000.0f, 4);
    json += ",\"q_predicted_signed_mA_s\":" + String(e.q_estimated_mAms / 1000.0f, 4);
    json += ",\"a_target_deg\":" + String(e.a_target_cdeg / 100.0f, 3);
    json += ",\"a_pred_deg\":" + String(e.a_pred_cdeg / 100.0f, 3);
    json += ",\"a_predicted_next_deg\":" + String(e.a_pred_cdeg / 100.0f, 3);
    json += ",\"a_pred_shadow_deg\":" + String(e.a_pred_shadow_cdeg / 100.0f, 3);
    if (e.a_pred_calibrated_cdeg == LOG_NAN_I16) json += ",\"a_pred_calibrated_deg\":null";
    else json += ",\"a_pred_calibrated_deg\":" + String(e.a_pred_calibrated_cdeg / 100.0f, 3);
    if (e.prev_peak_abs_cdeg == LOG_NAN_I16) json += ",\"prev_peak_abs_deg\":null";
    else json += ",\"prev_peak_abs_deg\":" + String(e.prev_peak_abs_cdeg / 100.0f, 3);
    json += ",\"next_peak_side\":" + String(e.next_peak_side);
    json += ",\"q_effective_pred_mA_s\":" + String(e.q_effective_pred_mAms / 1000.0f, 4);
    json += ",\"calibrated_prediction_valid\":" + String(e.calibrated_prediction_valid ? "true" : "false");
    json += ",\"a_min_pred_deg\":" + String(e.a_min_cdeg / 100.0f, 3);
    json += ",\"a_max_pred_deg\":" + String(e.a_max_cdeg / 100.0f, 3);
    json += ",\"target_reachable\":" + String(e.target_reachable ? "true" : "false");
    json += ",\"bootstrap\":" + String(e.bootstrap ? "true" : "false");
    json += ",\"imu_peak_abs_deg\":" + String(fabsf(e.theta_peak_cdeg / 100.0f), 3);
    json += ",\"a_next_imu_abs_deg\":" + String(fabsf(e.theta_peak_cdeg / 100.0f), 3);
    json += ",\"pulse_suppressed\":" + String(e.pulse_suppressed ? "true" : "false");
    json += ",\"peak_detected\":" + String(e.peak_detected ? "true" : "false") + "}";
  }
  json += "],";
  json += "\"e2_shadow_peak_events\":[";
  for (uint16_t i = 0; i < e2_shadow_peak_event_count_; ++i) {
    const E2ShadowPeakEvent& e = events_->e2_shadow_peak_events_[i];
    if (i) json += ",";
    json += "{\"event_index\":" + String(e.event_index);
    json += ",\"e2_armed\":" + String(e.e2_armed ? "true" : "false");
    json += ",\"release_detected_ms\":" + String(e.release_detected_ms);
    json += ",\"turn_index\":" + String(e.turn_index);    json += ",\"candidate_peak_ms\":" + String(e.candidate_peak_ms);
    json += ",\"confirmed_ms\":" + String(e.confirmed_ms);
    json += ",\"turn_side_from_rate\":" + String(e.turn_side_from_rate);
    json += ",\"alternating_side_ok\":" + String(e.alternating_side_ok ? "true" : "false");
    auto appendNullable = [&json](const char* name, float value, uint8_t decimals) {
      json += ",\"";
      json += name;
      json += "\":";
      if (isfinite(value)) json += String(value, static_cast<unsigned int>(decimals));
      else json += "null";
    };
    appendNullable("previous_peak", e.previous_peak_deg, 4);
    appendNullable("current_peak", e.current_peak_deg, 4);
    appendNullable("halfcycle_gyro_integral_deg", e.halfcycle_gyro_integral_deg, 4);
    appendNullable("H_prev_gyro_deg", e.h_prev_gyro_deg, 4);
    appendNullable("H_prev", e.h_prev_deg, 4);
    appendNullable("H_next_E2_deg", e.h_next_e2_explicit_deg, 4);
    appendNullable("H_next_E2", e.h_next_e2_deg, 4);
    appendNullable("static_anchor_abs_deg", e.static_anchor_abs_deg, 4);
    appendNullable("peak_gyro_integrated_abs_deg", e.peak_gyro_integrated_abs_deg, 4);
    appendNullable("peak_accel_abs_diag_deg", e.peak_accel_abs_diag_deg, 4);
    appendNullable("passive_next_peak", e.passive_next_peak_deg, 4);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);

    appendNullable("target_peak", e.target_peak_deg, 4);
    appendNullable("delta_A_required", e.delta_a_required_deg, 4);
    appendNullable("shadow_gain", e.shadow_gain_deg_per_mA_s, 10);
    appendNullable("Q_req_shadow", e.q_req_shadow_mA_s, 5);
    json += ",\"shadow_valid\":" + String(e.shadow_valid ? "true" : "false");
    json += ",\"shadow_invalid_reason\":\"" +
        String(e2ShadowInvalidReasonName(e.shadow_invalid_reason)) + "\"";
    json += ",\"shadow_invalid_reason_code\":" + String(e.shadow_invalid_reason);
    json += "}";
  }
  json += "],";
  json += "\"q1_shadow_events\":[";
  for (uint16_t i = 0; i < q1_shadow_event_count_; ++i) {
    const Q1ShadowEvent& e = events_->q1_shadow_events_[i];
    if (i) json += ",";
    json += "{\"q1_shadow_event_index\":" + String(e.q1_shadow_event_index);
    json += ",\"zero_cross_time_ms\":" + String(e.zero_cross_time_ms);
    auto appendQ1Nullable = [&json](const char* name, float value, uint8_t decimals) {
      json += ",\"";
      json += name;
      json += "\":";
      if (isfinite(value)) json += String(value, static_cast<unsigned int>(decimals));
      else json += "null";
    };
    appendQ1Nullable("zero_cross_rate_dps", e.zero_cross_rate_dps, 4);
    appendQ1Nullable("zero_cross_abs_rate_dps", e.zero_cross_abs_rate_dps, 4);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);

    json += ",\"detector_crossing_direction\":" + String(e.detector_crossing_direction);
    appendQ1Nullable("detector_angle_before_deg", e.detector_angle_before_deg, 5);
    appendQ1Nullable("detector_angle_after_deg", e.detector_angle_after_deg, 5);
    appendQ1Nullable("crossing_interpolation_alpha", e.crossing_interpolation_alpha, 6);
    json += ",\"interpolated_zero_cross_time_ms\":" +
        String(e.interpolated_zero_cross_time_ms);
    appendQ1Nullable("physical_roll_rate_before_dps", e.physical_roll_rate_before_dps, 4);
    appendQ1Nullable("physical_roll_rate_after_dps", e.physical_roll_rate_after_dps, 4);
    appendQ1Nullable("interpolated_physical_roll_rate_dps",
                     e.interpolated_physical_roll_rate_dps, 4);
    json += ",\"sign_gate_passed\":" + String(e.sign_gate_passed ? "true" : "false");
    appendQ1Nullable("q1_intercept_deg", e.q1_intercept_deg, 5);
    appendQ1Nullable("q1_rate_term_deg", e.q1_rate_term_deg, 5);
    appendQ1Nullable("q1_side_term_deg", e.q1_side_term_deg, 5);
    appendQ1Nullable("q1_baseline_next_peak_abs_deg", e.q1_baseline_next_peak_abs_deg, 5);
    appendQ1Nullable("target_next_peak_abs_deg", e.target_next_peak_abs_deg, 5);
    appendQ1Nullable("delta_peak_required_deg", e.delta_peak_required_deg, 5);
    appendQ1Nullable("q1_gain_deg_per_mAs", e.q1_gain_deg_per_mA_s, 5);
    appendQ1Nullable("q_model_axis_mA_s", e.q_model_axis_mA_s, 5);
    appendQ1Nullable("q_req_shadow_mA_s", e.q_req_shadow_mA_s, 5);
    json += ",\"q1_shadow_valid\":" + String(e.q1_shadow_valid ? "true" : "false");
    json += ",\"q1_shadow_invalid_reason\":\"" +
        String(q1ShadowInvalidReasonName(e.q1_shadow_invalid_reason)) + "\"";
    json += ",\"q1_shadow_invalid_reason_code\":" + String(e.q1_shadow_invalid_reason);
    json += "}";
  }
  json += "],";
  json += "\"q_ident_events\":[";
  for (uint16_t i = 0; i < q_ident_event_count_; ++i) {
    const QIdentEvent& e = events_->q_ident_events_[i];
    if (i) json += ",";
    json += "{\"q_ident_event_index\":" + String(e.q_ident_event_index);
    json += ",\"q_ident_run_schedule_id\":" + String(e.q_ident_run_schedule_id);
    json += ",\"zero_cross_time_ms\":" + String(e.zero_cross_time_ms);
    auto appendQIdentNullable = [&json](const char* name, float value, uint8_t decimals) {
      json += ",\"";
      json += name;
      json += "\":";
      if (isfinite(value)) json += String(value, static_cast<unsigned int>(decimals));
      else json += "null";
    };
    appendQIdentNullable("zero_cross_rate_dps", e.zero_cross_rate_dps, 4);
    appendQIdentNullable("zero_cross_abs_rate_dps", e.zero_cross_abs_rate_dps, 4);
    json += ",\"interpolated_zero_cross_time_ms\":" + String(e.interpolated_zero_cross_time_ms);
    appendQIdentNullable("interpolated_physical_roll_rate_dps", e.interpolated_physical_roll_rate_dps, 4);
    json += ",\"detector_crossing_direction\":" + String(e.detector_crossing_direction);
    appendQIdentNullable("detector_angle_before_deg", e.detector_angle_before_deg, 5);
    appendQIdentNullable("detector_angle_after_deg", e.detector_angle_after_deg, 5);
    json += ",\"sign_gate_passed\":" + String(e.sign_gate_passed ? "true" : "false");
    appendQIdentNullable("physical_roll_abs_diag_deg", e.physical_roll_abs_diag_deg, 4);
    appendQIdentNullable("current_roll_deg", e.current_roll_deg, 4);
    appendQIdentNullable("physical_roll_rate_dps", e.physical_roll_rate_dps, 4);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);

    json += ",\"side_occurrence_index\":" + String(e.side_occurrence_index);
    json += ",\"q_ident_armed\":" + String(e.q_ident_armed ? "true" : "false");
    json += ",\"arm_consecutive_count\":" + String(e.arm_consecutive_count);
    appendQIdentNullable("q_schedule_target_mA_s", e.q_schedule_target_mA_s, 4);
    json += ",\"q_command_direction\":" + String(e.q_command_direction);
    json += ",\"command_current_mA\":" + String(e.command_current_mA);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    appendQIdentNullable("q_target_mA_s", e.q_target_mA_s, 4);
    appendQIdentNullable("q_effective_pred_mA_s", e.q_effective_pred_mA_s, 4);
    json += ",\"vbat_mV\":" + String(e.vbat_mV);
    appendQIdentNullable("i0_estimated_mA", e.i0_estimated_mA, 4);
    appendQIdentNullable("solver_required_width_ms", e.solver_required_width_ms, 4);
    json += ",\"solver_selected_integer_width_ms\":" + String(e.solver_selected_integer_width_ms);
    json += ",\"pulse_width_guard_max_ms\":" + String(e.pulse_width_guard_max_ms);
    json += ",\"pulse_start_ms\":" + String(e.pulse_start_ms);
    json += ",\"pulse_end_ms\":" + String(e.pulse_end_ms);
    json += ",\"q_ident_valid\":" + String(e.q_ident_valid ? "true" : "false");
    json += ",\"q_ident_invalid_reason\":\"" + String(qIdentInvalidReasonName(e.q_ident_invalid_reason)) + "\"";
    json += ",\"q_ident_invalid_reason_code\":" + String(e.q_ident_invalid_reason);
    json += "}";
  }
  json += "],";
  json += "\"energy_control_v0_events\":[";
  for (uint16_t i = 0; i < energy_control_v0_event_count_; ++i) {
    const EnergyControlV0Event& e = events_->energy_control_v0_events_[i];
    if (i) json += ",";
    json += "{\"event_index\":" + String(e.event_index);
    json += ",\"zero_cross_time_ms\":" + String(e.zero_cross_time_ms);
    auto appendEnergyNullable = [&json](const char* name, float value, uint8_t decimals) {
      json += ",\""; json += name; json += "\":";
      if (isfinite(value)) json += String(value, static_cast<unsigned int>(decimals));
      else json += "null";
    };
    appendEnergyNullable("zero_cross_rate_dps", e.zero_cross_rate_dps, 4);
    appendEnergyNullable("zero_cross_abs_rate_dps", e.zero_cross_abs_rate_dps, 4);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);

    json += ",\"output_gate_state\":" + String(e.output_gate_state);
    json += ",\"previous_accepted_physical_next_peak_side\":" +
        String(e.previous_accepted_physical_next_peak_side);
    json += ",\"candidate_physical_next_peak_side\":" +
        String(e.candidate_physical_next_peak_side);
    json += ",\"rearm_excursion_seen\":" + String(e.rearm_excursion_seen ? "true" : "false");
    appendEnergyNullable("maximum_excursion_since_previous_cross_deg",
                         e.maximum_excursion_since_previous_cross_deg, 5);
    appendEnergyNullable("rearm_threshold_deg", e.rearm_threshold_deg, 5);
    json += ",\"side_alternation_passed\":" + String(e.side_alternation_passed ? "true" : "false");
    json += ",\"output_authorized\":" + String(e.output_authorized ? "true" : "false");
    json += ",\"output_blocked_reason\":\"" +
        String(energyControlV0ReasonName(e.output_blocked_reason)) + "\"";
    json += ",\"output_blocked_reason_code\":" + String(e.output_blocked_reason);
    json += ",\"q_command_direction\":" + String(e.q_command_direction);
    appendEnergyNullable("passive_next_peak_abs_deg", e.passive_next_peak_abs_deg, 5);
    appendEnergyNullable("passive_energy_j", e.passive_energy_j, 8);
    appendEnergyNullable("target_peak_abs_deg", e.target_peak_abs_deg, 5);
    appendEnergyNullable("target_energy_j", e.target_energy_j, 8);
    appendEnergyNullable("delta_energy_required_j", e.delta_energy_required_j, 8);
    appendEnergyNullable("q1_gain_deg_per_mA_s", e.q1_gain_deg_per_mA_s, 5);
    appendEnergyNullable("q_selected_mA_s", e.q_selected_mA_s, 5);
    appendEnergyNullable("q_effective_pred_mA_s", e.q_effective_pred_mA_s, 5);
    appendEnergyNullable("predicted_next_peak_abs_deg", e.predicted_next_peak_abs_deg, 5);
    appendEnergyNullable("predicted_next_energy_j", e.predicted_next_energy_j, 8);
    json += ",\"q_saturated_at_max\":" + String(e.q_saturated_at_max ? "true" : "false");
    json += ",\"rate_support_status\":" + String(e.rate_support_status);
    json += ",\"q_support_status\":" + String(e.q_support_status);
    json += ",\"vbat_mV\":" + String(e.vbat_mV);
    appendEnergyNullable("i0_estimated_mA", e.i0_estimated_mA, 4);
    appendEnergyNullable("solver_required_width_ms", e.solver_required_width_ms, 4);
    json += ",\"solver_selected_integer_width_ms\":" + String(e.solver_selected_integer_width_ms);
    json += ",\"command_current_mA\":" + String(e.command_current_mA);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    json += ",\"pulse_start_ms\":" + String(e.pulse_start_ms);
    json += ",\"pulse_end_ms\":" + String(e.pulse_end_ms);
    json += ",\"output_executed\":" + String(e.output_executed ? "true" : "false");
    json += ",\"valid\":" + String(e.valid ? "true" : "false");
    json += ",\"reason\":\"" + String(energyControlV0ReasonName(e.reason)) + "\"";
    json += ",\"reason_code\":" + String(e.reason);
    json += "}";
  }
  json += "],";
  json += "\"v46n_imu_acquisition\":";
  imu.appendAcquisitionDiagnostics(json);
  json += ",";
  json += "\"v46p_control_worker\":" + run_control.diagnosticsJson() + ",";
  json += "\"control_work_profile\":" + control_work::profile.json() + ",";
  json += "\"control_latency\":";
  control_latency::profile.appendJson(json);
  json += ",";
  {
    // Export runs only after completion; do not infer terminal output from the
    // last regular sample, which may still precede an emergency stop.
    const auto s = run_control.snapshot();
    json += "\"terminal_state\":{\"state_id\":" + String(s.state_id);
    json += ",\"state\":\"" + String(s.state_name) + "\"";
    json += ",\"last_error\":\"" + String(s.last_error) + "\"";
    json += ",\"motor_cmd_mA\":" + String(s.motor_cmd_mA);
    json += ",\"actual_current_mA\":" + String(s.actual_current_mA);
    json += ",\"heartbeat_us\":" + String(s.heartbeat_us) + "},";
  }
  {
    const auto t = roller.telemetrySnapshot();
    json += "\"v46u_current_timing\":{\"scope\":\"since_boot_pulse_audit_only_not_per_run\",\"budget_us\":2000";
    json += ",\"read_count\":" + String(t.pulse_current_read_work.count);
    json += ",\"read_over_budget\":" + String(t.pulse_current_read_work.over);
    json += ",\"read_max_us\":" + String(t.pulse_current_read_work.maximum);
    json += ",\"interval_count\":" + String(t.pulse_current_intervals.count);
    json += ",\"interval_over_budget\":" + String(t.pulse_current_intervals.over);
    json += ",\"interval_max_us\":" + String(t.pulse_current_intervals.maximum) + "},";
  }
  json += "\"v46t_current_observation_policy\":\"single_snapshot_before_row_reference_clock;age_from_same_snapshot;zero_sample_time_is_missing;no_clamp\",";
  json += "\"v46s_solver_audit\":";
  if (solver_audit_) { if (!solver_audit_->appendJson(json)) json.fail(); }
  else json += "{\"schema_version\":2,\"available\":false,\"reason\":\"audit_psram_allocation_failed\"}";
  json += ",";
  json += "\"v46l_solver_shadow_revision\":\"v46l_discrete_ternary_shadow_20260914\",";
  json += "\"v46l_solver_shadow_policy\":\"disabled_since_v46r;legacy_comparison_offline_only;use_v46s_solver_audit\",";
  json += "\"v46l_solver_shadow_event_overflow\":" + String(solver_shadow_event_overflow_ ? "true" : "false") + ",";
  json += "\"v46l_solver_shadow_events\":[";
  for (uint16_t i = 0; i < solver_shadow_event_count_; ++i) {
    const SolverShadowEvent& e = events_->solver_shadow_events_[i];
    if (i) json += ",";
    json += "{\"event_index\":" + String(e.event_index);
    json += ",\"pulse_id\":" + String(e.pulse_id);
    json += ",\"t_test_ms\":" + String(e.t_test_ms);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);
    json += ",\"q_command_direction\":" + String(e.q_command_direction);
    json += ",\"command_current_mA\":" + String(e.command_current_mA);
    json += ",\"model_vbat_mV\":" + String(e.model_vbat_mV);
    json += ",\"i0_estimated_mA\":" + jsonFloatOrNull(e.i0_estimated_mA, 5);
    json += ",\"free_next_peak_deg\":" + jsonFloatOrNull(e.free_next_peak_deg, 6);
    json += ",\"target_peak_deg\":" + jsonFloatOrNull(e.target_peak_deg, 6);
    json += ",\"target_energy_j\":" + jsonFloatOrNull(e.target_energy_j, 9);
    json += ",\"q_available_mA_s\":" + jsonFloatOrNull(e.q_available_mA_s, 6);
    json += ",\"integral_side_mA_s\":" + jsonFloatOrNull(e.integral_side_mA_s, 6);
    json += ",\"legacy_ff_width_ms\":" + String(e.legacy_ff_width_ms);
    json += ",\"legacy_ff_q_mA_s\":" + jsonFloatOrNull(e.legacy_ff_q_mA_s, 6);
    json += ",\"legacy_selected_width_ms\":" + String(e.legacy_selected_width_ms);
    json += ",\"legacy_selected_q_mA_s\":" + jsonFloatOrNull(e.legacy_selected_q_mA_s, 6);
    json += ",\"fast_ff_width_ms\":" + String(e.fast_ff_width_ms);
    json += ",\"fast_ff_q_mA_s\":" + jsonFloatOrNull(e.fast_ff_q_mA_s, 6);
    json += ",\"fast_selected_width_ms\":" + String(e.fast_selected_width_ms);
    json += ",\"fast_selected_q_mA_s\":" + jsonFloatOrNull(e.fast_selected_q_mA_s, 6);
    json += ",\"free_model_us\":" + String(e.free_model_us);
    json += ",\"legacy_ff_scan_us\":" + String(e.legacy_ff_scan_us);
    json += ",\"legacy_selected_scan_us\":" + String(e.legacy_selected_scan_us);
    json += ",\"legacy_decision_us\":" + String(e.legacy_decision_us);
    json += ",\"fast_shadow_us\":" + String(e.fast_shadow_us);
    json += ",\"fast_eval_count\":" + String(e.fast_eval_count);
    json += ",\"fast_valid\":" + String(e.fast_valid ? "true" : "false");
    json += ",\"ff_width_match\":" + String(e.ff_width_match ? "true" : "false");
    json += ",\"selected_width_match\":" + String(e.selected_width_match ? "true" : "false") + "}";
  }
  json += "],";
  json += "\"v46k_timing_probe_revision\":\"v46k_pulse_start_core1_profile_20260914\",";
  json += "\"v46k_timing_probe_scope\":\"measurement_only;no_controller_or_motor_decision_reads_timing_values\",";
  json += "\"v46k_timing_probe_event_overflow\":" + String(timing_probe_event_overflow_ ? "true" : "false") + ",";
  json += "\"v46k_timing_probe_events\":";
  CompactJsonTable<PsramString> timing_table(json);
  for (uint16_t i = 0; i < timing_probe_event_count_; ++i) {
    const TimingProbeEvent& e = events_->timing_probe_events_[i];
    String row;
    row += "{\"event_index\":" + String(e.event_index);
    row += ",\"pulse_id\":" + String(e.pulse_id);
    row += ",\"pulse_kind\":\"" + String(e.pulse_kind == 1 ? "strong_start_kick" : "normal_zero_cross") + "\"";
    row += ",\"t_test_ms\":" + String(e.t_test_ms);
    row += ",\"command_mA\":" + String(e.command_mA);
    row += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    row += ",\"pulse_start_us\":" + String(e.pulse_start_us);
    row += ",\"gyro_sequence_at_start\":" + String(e.gyro_sequence_at_start);
    row += ",\"set_current_us\":" + String(e.set_current_us);
    row += ",\"state_update_us\":" + String(e.state_update_us);
    row += ",\"current_model_us\":" + String(e.current_model_us);
    row += ",\"update_pulse_model_us\":" + String(e.update_pulse_model_us);
    row += ",\"pulse_begin_total_us\":" + String(e.pulse_begin_total_us);
    row += ",\"first_audit_log_offset_us\":" + String(e.first_audit_log_offset_us);
    row += ",\"first_audit_log_us\":" + String(e.first_audit_log_us);
    row += ",\"imu_update_call_us\":" + String(e.imu_update_call_us);
    row += ",\"runner_update_call_us\":" + String(e.runner_update_call_us);
    row += ",\"core1_path_us\":" + String(e.core1_path_us);
    row += ",\"first_imu_dt_after_start_us\":" + String(e.first_imu_dt_after_start_us);
    row += ",\"first_imu_sample_offset_us\":" + String(e.first_imu_sample_offset_us);
    row += ",\"capture_mask\":" + String(e.capture_mask);
    row += ",\"complete\":" + String(e.complete ? "true" : "false") + "}";
    if (!timing_table.append(row)) json.fail();
  }
  if (!timing_table.finish()) json.fail();
  json += ",";
  json += "\"energy_control_autonomous_peak_events\":";
  CompactJsonTable<PsramString> peak_table(json);
  bool metadata_event_detail_truncated = false;
  uint16_t autonomous_peak_event_rendered_count = 0;
  uint16_t autonomous_zero_cross_event_rendered_count = 0;
  for (uint16_t i = 0; i < energy_control_autonomous_peak_event_count_; ++i) {
    const EnergyControlAutonomousPeakEvent& e = events_->energy_control_autonomous_peak_events_[i];
    String detail = "{\"peak_index\":" + String(e.peak_index);
    detail += ",\"peak_time_ms\":" + String(e.peak_time_ms);
    detail += ",\"physical_peak_side\":" + String(e.physical_peak_side);
    detail += ",\"peak_amplitude_deg\":" + jsonFloatOrNull(e.peak_amplitude_deg, 5);
    if (isfinite(e.detector_peak_angle_deg)) detail += ",\"detector_peak_angle_deg\":" + jsonFloatOrNull(e.detector_peak_angle_deg, 5);
    else detail += ",\"detector_peak_angle_deg\":null";
    detail += ",\"target_peak_deg\":" + jsonFloatOrNull(e.target_peak_deg, 5);
    detail += ",\"peak_error_deg\":" + jsonFloatOrNull(e.peak_error_deg, 5);
    detail += ",\"cg_shift_mm\":" + jsonFloatOrNull(e.cg_shift_mm, 6);
    detail += ",\"target_energy_j\":" + jsonFloatOrNull(e.target_energy_j, 8);
    detail += ",\"measured_energy_j\":" + jsonFloatOrNull(e.measured_energy_j, 8);
    detail += ",\"energy_error_j\":" + jsonFloatOrNull(e.energy_error_j, 8);
    detail += ",\"integral_plus_j\":" + jsonFloatOrNull(e.integral_plus_j, 8);
    detail += ",\"integral_minus_j\":" + jsonFloatOrNull(e.integral_minus_j, 8);
    detail += ",\"phase\":" + String(e.phase);
    detail += ",\"integral_plus_mA_s\":" + jsonFloatOrNull(e.integral_plus_mA_s, 5);
    detail += ",\"integral_minus_mA_s\":" + jsonFloatOrNull(e.integral_minus_mA_s, 5);
    detail += ",\"first_peak\":" + String(e.first_peak ? "true" : "false");
    detail += ",\"pending_command_matched\":" + String(e.pending_command_matched ? "true" : "false");
    if (isfinite(e.pending_q_command_mA_s)) detail += ",\"pending_q_command_mA_s\":" + jsonFloatOrNull(e.pending_q_command_mA_s, 5);
    else detail += ",\"pending_q_command_mA_s\":null";
    detail += ",\"model_residual_j\":" + jsonFloatOrNull(e.model_residual_j, 8);
    detail += ",\"peak_telemetry_capture_time_ms\":" + String(e.peak_telemetry_capture_time_ms);
    detail += ",\"peak_wheel_speed_rpm\":" + jsonFloatOrNull(e.peak_wheel_speed_rpm, 3);
    detail += ",\"peak_measured_current_mA\":" + jsonFloatOrNull(e.peak_measured_current_mA, 3);
    detail += ",\"peak_speed_age_us\":" + String(e.peak_speed_age_us);
    detail += ",\"peak_current_age_us\":" + String(e.peak_current_age_us);
    detail += ",\"peak_speed_valid\":" + String(e.peak_speed_valid ? "true" : "false");
    detail += ",\"peak_current_valid\":" + String(e.peak_current_valid ? "true" : "false");
    detail += ",\"model_residual_observed\":" + String(e.model_residual_observed ? "true" : "false");
    detail += ",\"antiwindup_upper_hold\":" + String(e.antiwindup_upper_hold ? "true" : "false");
    detail += ",\"antiwindup_lower_hold\":" + String(e.antiwindup_lower_hold ? "true" : "false") + "}";
    if (!peak_table.append(detail)) { json.fail(); break; }
    ++autonomous_peak_event_rendered_count;
  }
  if (!peak_table.finish()) json.fail();
  json += ",\"energy_control_autonomous_zero_cross_events\":";
  CompactJsonTable<PsramString> zero_table(json);
  for (uint16_t i = 0; i < energy_control_autonomous_zero_cross_event_count_; ++i) {
    const EnergyControlAutonomousZeroCrossEvent& e = events_->energy_control_autonomous_zero_cross_events_[i];
    String detail = "{\"event_index\":" + String(e.event_index);
#if defined(ANGLE_EARLY_TRIAL) && ANGLE_EARLY_TRIAL
    const char* early_kind="angle_rate_early_trial";
#else
    const char* early_kind="foot_geometry_early_trial";
#endif
    detail += ",\"event_kind\":\"" + String(e.event_kind == 1 ? "strong_start_kick" : e.event_kind == 2 ? "inward_stall_rescue" : e.event_kind == 3 ? early_kind : e.event_kind == 4 ? "early_inward_stall_rescue" : e.event_kind == 5 ? "approach_inward_stall_rescue" : "energy_control_zero_cross") + "\"";
    detail += ",\"zero_cross_time_ms\":" + String(e.zero_cross_time_ms);
#if defined(SWAY_BALANCE_TRIAL) && SWAY_BALANCE_TRIAL
    detail += ",\"sway_trim_before_ms\":" + String(e.sway_trim_before_ms);
    detail += ",\"sway_trim_cap_ms\":" + String(e.sway_trim_cap_ms);
#endif
#if (defined(FAST_CROSSING_TRIAL) && FAST_CROSSING_TRIAL) || (defined(FAST_CROSSING_SHADOW) && FAST_CROSSING_SHADOW)
    detail += ",\"fast_trim_before_ms\":" + String(e.fast_trim_before_ms);
    detail += ",\"fast_trim_cap_ms\":" + String(e.fast_trim_cap_ms);
#endif
    auto appendAutonomousNullable = [&detail](const char* name, float value, uint8_t decimals) {
      detail += ",\""; detail += name; detail += "\":";
      if (isfinite(value)) detail += String(value, static_cast<unsigned int>(decimals)); else detail += "null";
    };
    appendAutonomousNullable("zero_cross_rate_dps", e.zero_cross_rate_dps, 4);
    appendAutonomousNullable("zero_cross_abs_rate_dps", e.zero_cross_abs_rate_dps, 4);
    appendAutonomousNullable("detector_angle_before_deg", e.detector_angle_before_deg, 5);
    appendAutonomousNullable("detector_angle_after_deg", e.detector_angle_after_deg, 5);
    appendAutonomousNullable("detector_crossing_alpha", e.detector_crossing_alpha, 5);
    appendAutonomousNullable("interpolated_crossing_time_ms", e.interpolated_crossing_time_ms, 4);
    detail += ",\"rate_support_diagnostic\":" + String(e.rate_support_diagnostic ? "true" : "false");
    detail += ",\"previous_peak_time_ms\":" + String(e.previous_peak_time_ms);
    detail += ",\"previous_peak_side\":" + String(e.previous_peak_side);
    appendAutonomousNullable("previous_peak_amplitude_deg", e.previous_peak_amplitude_deg, 5);
    detail += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);
    detail += ",\"side_mismatch_diagnostic\":" + String(e.side_mismatch_diagnostic ? "true" : "false");
    detail += ",\"phase\":" + String(e.phase);
    appendAutonomousNullable("free_next_peak_amplitude_deg", e.free_next_peak_amplitude_deg, 5);
    appendAutonomousNullable("p1_free_peak_before_rate_deg", e.p1_free_peak_before_rate_deg, 5);
    appendAutonomousNullable("rate_baseline_peak_deg", e.rate_baseline_peak_deg, 5);
    appendAutonomousNullable("rate_baseline_correction_deg", e.rate_baseline_correction_deg, 5);
    detail += ",\"rate_baseline_reason\":" + String(e.rate_baseline_reason);
    const bool work_event = e.work_control_branch != 0;
    const bool inverse_event = energy_bridge_enabled_ && !negative_probe_mode_ &&
        autonomous_brake_max_ms_ == 25 && e.physical_next_peak_side < 0;
    detail += ",\"free_model_revision\":\"" + String(e.work_control_branch==17 ? "foot_support_local_unmodeled_v12" : e.work_control_branch==16 ? "foot_support_transfer_unmodeled_v11" : e.work_control_branch==15 ? "gradual_sway_unmodeled_trial_or_coast" : e.work_control_branch==14 ? "positive_asymmetry_unmodeled_trial" : e.work_control_branch==13 ? "weak_side_assist_v1_unmodeled" : (e.work_control_branch==10 || e.work_control_branch==11) ? walking_energy_loss::kRevision : (e.work_control_branch>=4) ? wheel_work_energy_control::kRevision :
        inverse_event ? one_peak_energy_inverse::kRevision :
        energy_bridge_enabled_ ? lateral_energy::kRevision : lateral_energy::kQuadraticRevision) + "\"";
    if (work_event && e.work_control_branch!=13 && e.work_control_branch!=14 && e.work_control_branch!=15 && e.work_control_branch!=16 && e.work_control_branch!=17)
      detail += ",\"arrival_energy_model_revision\":\"" + String((e.work_control_branch==10 || e.work_control_branch==11) ? walking_energy_loss::kRevision : control_target_cdeg_==1200 ? "target12_state_work_20261027" : wheel_work_energy_control::kRevision) + "\"";
    else
      // The compact table stores one shared field list for every row. Startup
      // and fallback events must keep the work-model column, with a null value.
      detail += ",\"arrival_energy_model_revision\":null";
    detail += ",\"work_control_branch\":" + String(e.work_control_branch);
    detail += ",\"work_prediction_cross_condition\":" + String(e.work_prediction_cross_condition ? "true" : "false");
    detail += ",\"work_brake_cap_ms\":" + String(e.work_brake_cap_ms);
    detail += ",\"work_bias_samples\":" + String(e.work_bias_samples);
    appendAutonomousNullable("work_model_weight",e.work_model_weight,6);
    detail += ",\"mpc_valid\":" + String(e.mpc_valid ? "true" : "false");
    detail += ",\"mpc_alternate\":" + String(e.mpc_alternate ? "true" : "false");
    appendAutonomousNullable("mpc_second_peak_deg",e.mpc_second_peak_deg,5);
    appendAutonomousNullable("mpc_speed_proxy_rpm",e.mpc_speed_proxy_rpm,3);
    appendAutonomousNullable("predicted_wheel_work_j", e.predicted_wheel_work_j, 8);
    appendAutonomousNullable("predicted_input_work_j", e.predicted_input_work_j, 8);
    appendAutonomousNullable("predicted_post_input_work_j", e.predicted_post_input_work_j, 8);
    appendAutonomousNullable("predicted_work_change_j", e.predicted_work_change_j, 8);
    appendAutonomousNullable("passive_energy_j", e.passive_energy_j, 8);
    appendAutonomousNullable("target_peak_deg", e.target_peak_deg, 5);
    appendAutonomousNullable("foot_balance_target_deg", e.foot_balance_target_deg, 5);
    appendAutonomousNullable("foot_balance_left_deg", e.foot_balance_left_deg, 5);
    appendAutonomousNullable("foot_balance_right_deg", e.foot_balance_right_deg, 5);
    appendAutonomousNullable("foot_balance_left_mean", e.foot_balance_left_mean, 5);
    appendAutonomousNullable("foot_balance_right_mean", e.foot_balance_right_mean, 5);
    appendAutonomousNullable("startup_target_deg", e.startup_target_deg, 5);
    detail += ",\"startup_positive_cap_ms\":" + String(e.startup_positive_cap_ms);
    detail += ",\"wheel_guard_legacy_ms\":" + String(e.wheel_guard_legacy_ms);
    detail += ",\"wheel_guard_extra_ms\":" + String(e.wheel_guard_extra_ms);
    detail += ",\"wheel_guard_trial_eligible\":" + String(e.wheel_guard_trial_eligible ? "true" : "false");
    // Shared storage avoids increasing fixed PSRAM and metadata row sizes.
    // Metadata specifies that negative rows describe cap-recovery output;
    // the CSV converter separates both uses into clearly named columns.
    detail += ",\"long_foot_counterfactual_ms\":" + String(static_cast<unsigned>(e.weak_probe_baseline_ms));
    detail += ",\"long_foot_actual_ms\":" + String(static_cast<unsigned>(e.weak_probe_requested_ms));
    detail += ",\"long_foot_effective\":" + String(e.weak_probe_executed ? "true" : "false");
    appendAutonomousNullable("foot_energy_bias",e.foot_energy_bias,5);
    appendAutonomousNullable("foot_energy_left_px",e.foot_energy_left_px,4);
    appendAutonomousNullable("foot_energy_right_px",e.foot_energy_right_px,4);
    detail += ",\"foot_energy_valid\":" + String(e.foot_energy_valid ? "true" : "false");
    detail += ",\"foot_energy_phase\":" + String(e.foot_energy_phase);
    detail += ",\"foot_energy_reason\":" + String(e.foot_energy_reason);
    appendAutonomousNullable("low_peak_bias_mj",e.low_peak_bias_mj,5);
    detail += ",\"low_peak_active\":" + String(e.low_peak_active ? "true" : "false");
    detail += ",\"low_peak_selected\":" + String(e.low_peak_selected ? "true" : "false");
    detail += ",\"low_peak_base_width_ms\":" + String(e.low_peak_base_width_ms);
    appendAutonomousNullable("low_peak_base_prediction_mj",e.low_peak_base_prediction_mj,5);
    appendAutonomousNullable("low_peak_applied_bias_mj",e.low_peak_applied_bias_mj,5);
    // Legacy phase fields are absent in new records; raw foot frames and
    // body peak records remain available for offline reconstruction.
    appendAutonomousNullable("foot_span_left_deg",e.foot_span_left_deg,5);
    appendAutonomousNullable("foot_span_right_deg",e.foot_span_right_deg,5);
    appendAutonomousNullable("foot_span_trimmed_left_deg",e.foot_span_trimmed_left_deg,5);
    appendAutonomousNullable("foot_span_trimmed_right_deg",e.foot_span_trimmed_right_deg,5);
    detail += ",\"foot_span_start_ms\":" + String(e.foot_span_start_ms);
    detail += ",\"foot_span_end_ms\":" + String(e.foot_span_end_ms);
    detail += ",\"foot_span_baseline_ms\":" + String(e.foot_span_baseline_ms);
    detail += ",\"foot_span_requested_ms\":" + String(e.foot_span_requested_ms);
    detail += ",\"foot_span_state\":" + String(e.foot_span_state);
    detail += ",\"foot_span_accepted_cycles\":" + String(e.foot_span_accepted_cycles);
    detail += ",\"foot_span_flags\":" + String(e.foot_span_flags);
    appendAutonomousNullable("cg_shift_mm", e.cg_shift_mm, 6);
    appendAutonomousNullable("target_energy_j", e.target_energy_j, 8);
    appendAutonomousNullable("delta_energy_required_j", e.delta_energy_required_j, 8);
    appendAutonomousNullable("kinetic_energy_j", e.kinetic_energy_j, 8);
    appendAutonomousNullable("predicted_loss_j", e.predicted_loss_j, 8);
    appendAutonomousNullable("integral_energy_j", e.integral_energy_j, 8);
    appendAutonomousNullable("commanded_energy_deficit_j", e.commanded_energy_deficit_j, 8);
    appendAutonomousNullable("unclamped_energy_request_j", e.unclamped_energy_request_j, 8);
    appendAutonomousNullable("maximum_input_work_j", energy_bridge_enabled_ ? NAN : e.maximum_input_work_j, 8);
    appendAutonomousNullable("maximum_effective_arrival_energy_increment_j", energy_bridge_enabled_ ? e.maximum_input_work_j : NAN, 8);
    appendAutonomousNullable("unapplied_energy_request_j", e.unapplied_energy_request_j, 8);
    appendAutonomousNullable("predicted_current_goal_mA", e.predicted_current_goal_mA, 4);
    appendAutonomousNullable("predicted_current_rise_tau_s", e.predicted_current_rise_tau_s, 6);
    appendAutonomousNullable("aligned_initial_current_mA", e.aligned_initial_current_mA, 4);
    detail += ",\"unclamped_inverse_valid\":" + String(e.unclamped_inverse_valid ? "true" : "false");
    appendAutonomousNullable("corrected_target_energy_j", e.corrected_target_energy_j, 8);
    appendAutonomousNullable("work_omega_q_coefficient", e.work_omega_q_coefficient, 8);
    appendAutonomousNullable("work_q2_coefficient", e.work_q2_coefficient, 8);
    detail += ",\"energy_model_extrapolated\":" + String(e.energy_model_extrapolated ? "true" : "false");
    detail += ",\"fixed_negative_probe\":" + String(e.fixed_negative_probe ? "true" : "false");
    const bool batch_condition = e.negative_probe_batch_index != UINT16_MAX;
    if (batch_condition) {
      detail += ",\"negative_probe_batch_index\":" + String(e.negative_probe_batch_index);
      detail += ",\"negative_probe_batch_planned_width_ms\":" + String(e.negative_probe_batch_planned_width_ms);
      detail += ",\"negative_probe_batch_rate_skipped\":" + String(e.negative_probe_batch_rate_skipped ? "true" : "false");
    } else {
      // CompactJsonTable requires identical fields for every ZERO-cross row.
      detail += ",\"negative_probe_batch_index\":null";
      detail += ",\"negative_probe_batch_planned_width_ms\":null";
      detail += ",\"negative_probe_batch_rate_skipped\":null";
    }
    const bool positive_current_condition = e.positive_current_batch_index != UINT16_MAX;
    if (positive_current_condition) {
      detail += ",\"positive_current_batch_index\":" + String(e.positive_current_batch_index);
      detail += ",\"positive_current_batch_planned_mA\":" + String(e.positive_current_batch_planned_mA);
    } else {
      detail += ",\"positive_current_batch_index\":null";
      detail += ",\"positive_current_batch_planned_mA\":null";
    }
    appendAutonomousNullable("q1_gain_deg_per_mA_s", e.q1_gain_deg_per_mA_s, 5);
    appendAutonomousNullable("q_ff_energy_mA_s", e.q_ff_energy_mA_s, 5);
    appendAutonomousNullable("q_angle_diagnostic_mA_s", e.q_angle_diagnostic_mA_s, 5);
    appendAutonomousNullable("integral_side_mA_s", e.integral_side_mA_s, 5);
    appendAutonomousNullable("q_unclamped_mA_s", e.q_unclamped_mA_s, 5);
    appendAutonomousNullable("q_command_mA_s", e.q_command_mA_s, 5);
    appendAutonomousNullable("q_effective_pred_mA_s", e.q_effective_pred_mA_s, 5);
    appendAutonomousNullable("q_available_mA_s", e.q_available_mA_s, 5);
    detail += ",\"q_gain_extrapolated\":" + String(e.q_gain_extrapolated ? "true" : "false");
    appendAutonomousNullable("a_pred_base_deg", e.a_pred_base_deg, 5);
    appendAutonomousNullable("a_pred_corrected_deg", e.a_pred_corrected_deg, 5);
    appendAutonomousNullable("correction_deg", e.correction_deg, 5);
    appendAutonomousNullable("c_side_used_deg", e.c_side_used_deg, 5);
    appendAutonomousNullable("g_side_base_deg_per_mA_s", e.g_side_base_deg_per_mA_s, 5);
    appendAutonomousNullable("g_side_corrected_deg_per_mA_s", e.g_side_corrected_deg_per_mA_s, 5);
    appendAutonomousNullable("correction_blend_lambda", e.correction_blend_lambda, 5);
    appendAutonomousNullable("predicted_next_peak_amplitude_deg", e.predicted_next_peak_amplitude_deg, 5);
    appendAutonomousNullable("predicted_energy_j", e.predicted_energy_j, 8);
    appendAutonomousNullable("prediction_bias_j", e.prediction_bias_j, 8);
    appendAutonomousNullable("predicted_observed_energy_j", e.predicted_observed_energy_j, 8);
    appendAutonomousNullable("predicted_observed_next_peak_deg", e.predicted_observed_next_peak_deg, 5);
    detail += ",\"adaptation_mode\":" + String(e.adaptation_mode);
    detail += ",\"wheel_speed_bin\":" + String(e.wheel_speed_bin);
    detail += ",\"adaptation_consistent_samples\":" + String(e.adaptation_consistent_samples);
    detail += ",\"q_saturated_upper\":" + String(e.q_saturated_upper ? "true" : "false");
    detail += ",\"q_saturated_lower\":" + String(e.q_saturated_lower ? "true" : "false");
    detail += ",\"q_command_direction\":" + String(e.q_command_direction);
    detail += ",\"command_matches_zero_cross_motion\":" +
        String(e.command_matches_zero_cross_motion ? "true" : "false");
    detail += ",\"vbat_mV\":" + String(e.vbat_mV);
    appendAutonomousNullable("i0_estimated_mA", e.i0_estimated_mA, 4);
    detail += ",\"pre_input_capture_time_us\":" + String(e.pre_input_capture_time_us);
    appendAutonomousNullable("pre_input_measured_current_mA", e.pre_input_measured_current_mA, 3);
    detail += ",\"pre_input_current_sample_time_us\":" + String(e.pre_input_current_sample_time_us);
    detail += ",\"pre_input_current_age_us\":" + String(e.pre_input_current_age_us);
    detail += ",\"pre_input_current_valid\":" + String(e.pre_input_current_valid ? "true" : "false");
    appendAutonomousNullable("pre_input_wheel_speed_rpm", e.pre_input_wheel_speed_rpm, 3);
    detail += ",\"pre_input_wheel_speed_sample_time_us\":" + String(e.pre_input_wheel_speed_sample_time_us);
    detail += ",\"pre_input_wheel_speed_age_us\":" + String(e.pre_input_wheel_speed_age_us);
    detail += ",\"pre_input_wheel_speed_valid\":" + String(e.pre_input_wheel_speed_valid ? "true" : "false");
    appendAutonomousNullable("solver_required_width_ms", e.solver_required_width_ms, 4);
    detail += ",\"solver_selected_integer_width_ms\":" + String(e.solver_selected_integer_width_ms);
    detail += ",\"command_current_mA\":" + String(e.command_current_mA);
    detail += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    detail += ",\"pulse_start_ms\":" + String(e.pulse_start_ms);
    detail += ",\"pulse_end_ms\":" + String(e.pulse_end_ms);
    detail += ",\"output_executed\":" + String(e.output_executed ? "true" : "false");
    detail += ",\"valid\":" + String(e.valid ? "true" : "false");
    detail += ",\"reason\":\"" + String(energyControlAutonomousReasonName(e.reason)) + "\"";
    detail += ",\"reason_code\":" + String(e.reason) + "}";
    if (!zero_table.append(detail)) { json.fail(); break; }
    ++autonomous_zero_cross_event_rendered_count;
  }
  if (!zero_table.finish()) json.fail();
  json += ",";
  json += "\"metadata_event_detail_truncated\":" + String(metadata_event_detail_truncated ? "true" : "false") + ",";
  json += "\"energy_control_autonomous_peak_event_total_count\":" +
      String(energy_control_autonomous_peak_event_count_) + ",";
  json += "\"energy_control_autonomous_peak_event_rendered_count\":" +
      String(autonomous_peak_event_rendered_count) + ",";
  json += "\"energy_control_autonomous_zero_cross_event_total_count\":" +
      String(energy_control_autonomous_zero_cross_event_count_) + ",";
  json += "\"energy_control_autonomous_zero_cross_event_rendered_count\":" +
      String(autonomous_zero_cross_event_rendered_count) + ",";
  json += "\"metadata_json_budget_bytes\":" + String(kMetadataJsonHardBudgetBytes) + ",";
  json += "\"metadata_json_reserve_bytes\":" + String(kMetadataJsonReserveBytes) + ",";
    json += "\"states\":{";
  json += "\"0\":\"STARTUP_GYRO_CALIB\",\"1\":\"MADGWICK_SETTLING\",\"2\":\"READY_TO_MEASURE\",";
  json += "\"3\":\"RUNNING_BATCH_SWEEP\",\"4\":\"FINISHED\",\"5\":\"ESTOP\",";
  json += "\"6\":\"START_SYNC\",\"7\":\"END_SYNC\",\"8\":\"TRIAL_REST\"},";
  json += "\"beta_phase_states\":{\"0\":\"idle\",\"1\":\"outbound_wait_turn\",\"2\":\"return_to_zero\"},";
  json += "\"sync_event_ids\":{";
  json += "\"0\":\"none\",\"1\":\"start_sync_pattern_active\",";
  json += "\"2\":\"experiment_start_t0_after_start_sync_pattern\",";
  json += "\"3\":\"measurement_end_before_end_sync_pattern\",";
  json += "\"4\":\"end_sync_pattern_active\",\"5\":\"periodic_mid_sync_anchor_active\",";
  json += "\"6\":\"trial_start\",\"7\":\"trial_end\"},";
  json += "\"scales\":{";
  json += "\"pitch_cdeg\":\"degrees * 100\",";
  json += "\"gyro_cdps\":\"degrees_per_second * 100\",";
  json += "\"accel_mg\":\"g * 1000\",";
  json += "\"beta_x10000\":\"beta * 10000\",";
  json += "\"acc_norm_mg\":\"g * 1000\",\"roller_battery_mV\":\"mV\",";
  json += "\"beta_model_vbat_mV\":\"mV used after fallback or clamp\"},";
  json += "\"columns\":{\"timeseries\":[\"time_s\",\"log_time_s\",\"t_test_ms\",\"state_id\",\"pulse_id\",\"pulse_active\",\"pulse_direction\",\"motor_cmd_mA\",\"pulse_width_ms_setting\",\"gyro_bias_x_dps\",\"gyro_bias_y_dps\",\"gyro_bias_z_dps\",\"ax_g\",\"ay_g\",\"az_g\",\"gx_dps\",\"gy_dps\",\"gz_dps\",\"acc_norm_g\",\"roller_actual_current_mA\",\"roller_battery_mV\",\"led_state\",\"sync_event_id\",\"physical_roll_abs_deg\",\"roller_current_sample_time_us\",\"roller_current_sequence\",\"roller_q_meas_observed_mA_s\",\"pulse_q_target_mA_s\",\"pulse_q_pred_mA_s\",\"roller_current_valid\",\"roller_q_meas_observed_valid\",\"pitch_mekf_abs_deg\",\"roll_mekf_abs_deg\",\"pitch_mekf_measurement_relative_deg\",\"pitch_mekf_detector_relative_deg\",\"mekf_bias_x_dps\",\"mekf_bias_y_dps\",\"mekf_bias_z_dps\",\"mekf_accel_confidence\",\"mekf_accel_residual_deg\",\"mekf_accel_mag_error_g\",\"imu_update_dt_us\",\"imu_sample_age_us\",\"mekf_accel_used\",\"gyro_heading_deg\",\"steering_delta_deg\",\"steering_actual_difference_deg\",\"steering_desired_difference_deg\",\"steering_cycle_yaw_rate_dps\",\"steering_cycles\",\"gyro_heading_valid\",\"steering_reason\",\"roller_speed_rpm\",\"roller_speed_read_start_us\",\"roller_speed_sample_time_us\",\"roller_speed_read_end_us\",\"roller_speed_sequence\",\"roller_speed_valid\",\"physical_roll_rate_dps\",\"body_rate_sample_time_us\",\"roller_applied_current_mA\",\"roller_applied_command_sequence\",\"roller_applied_command_time_us\",\"roller_position_raw_cdeg\",\"roller_position_read_start_us\",\"roller_position_sample_time_us\",\"roller_position_read_end_us\",\"roller_position_sequence\",\"roller_position_valid\",\"roller_position_deg\"]}";
  json += ",";
  feet.appendMetadata(json);
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  runner.appendFootSupportMetadata(json);
#endif
  const String final_size_key = ",\"metadata_json_final_bytes\":";
  size_t final_size = json.length() + final_size_key.length() + 2U;
  for (uint8_t i = 0; i < 4; ++i) {
    const size_t candidate_size = json.length() + final_size_key.length() +
        String(final_size).length() + 1U;
    if (candidate_size == final_size) break;
    final_size = candidate_size;
  }
  json += final_size_key + String(final_size) + "}";
  if (json.length() > kMetadataJsonHardBudgetBytes) json.fail();
  return json.ok();
}
RwLogFileHeader PsramLogger::buildHeader(uint32_t metadata_size) const {
  RwLogFileHeader header{};
  memcpy(header.magic, "RWLOG01", 8);
  header.format_version = RWLOG_FORMAT_VERSION;
  header.header_size = sizeof(RwLogFileHeader);
  header.run_id = current_run_id_;
  header.run_start_us = run_start_us_;
  header.metadata_json_size = metadata_size;
  header.sample_count = sample_count_;
  header.summary_count = 0;
  header.event_count = 0;
  header.log_sample_size = sizeof(LogSample);
  header.summary_row_size = 0;
  header.event_row_size = 0;
  header.log_period_ms = Config::LOG_PERIOD_MS;
  header.imu_period_ms = Config::IMU_PERIOD_MS;
  header.reserved[0] = Config::IMU_POLL_PERIOD_US;
  header.reserved[1] = Config::BMI270_GYRO_ODR_HZ;
  header.reserved[2] = Config::BMI270_ACCEL_ODR_HZ;
  header.reserved[3] = Config::MEKF_CONTROL_PREDICTION_FIXED_US;
  header.roller_read_period_ms = Config::ROLLER_READ_PERIOD_MS;
  header.web_update_period_ms = Config::WEB_UPDATE_PERIOD_MS;
  header.total_trials = Config::BETA_SWEEP_TRIAL_COUNT;
  header.preset_id = 32;
  header.flags = RWLOG_FLAG_CRC32;
  header.samples_offset = sizeof(RwLogFileHeader) + metadata_size;
  header.summaries_offset = header.samples_offset + sample_count_ * sizeof(LogSample);
  header.events_offset = header.summaries_offset;
  header.crc_offset = header.events_offset;
  return header;
}
