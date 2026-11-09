#pragma once

#include <Adafruit_AHRS.h>
#include <Arduino.h>

#include "beta_phase_controller.h"
#include "beta_turn_fast_controller.h"
#include "config.h"
#include "autonomous_input_percent.h"
#include "autonomous_input_current.h"
#include "autonomous_brake_limit.h"
#include "lateral_energy_control.h"
#include "wheel_work_energy_control.h"
#include "walking_energy_loss.h"
#include "low_peak_energy.h"
#include "wheel_speed_balance.h"
#include "gradual_start.h"
#include "foot_balance_adaptive.h"
#include "foot_energy_balance.h"
#include "foot_phase_observer.h"
#include "foot_phase_cap.h"
#include "foot_fullstride_observer.h"
#include "foot_span_control.h"
#include "weak_side_assist.h"
#include "positive_left_boost.h"
#include "tilt_priority.h"
#include "positive_sway_probe.h"
#include "wheel_cycle_guard.h"
#include "target12_state_work.h"
#include "peak_error_adaptation.h"
#include "upright_reference.h"
#include "lateral_cg_energy.h"
#include "adaptive_lateral_cg.h"
#include "side_energy_feedback.h"
#include "control_math_cache.h"
#include "imu_manager.h"
#include "log_types.h"
#include "mekf6.hpp"
#include "mekf_attitude_diagnostics.h"
#include "psram_logger.h"
#include "roller485_manager.h"
#include "realtime_code.h"
#include "tilt_stop.h"

#include "experiment_status.h"
#include "gyro_heading_diagnostic.h"
#include "stall_rescue.h"
#include "early_motion_gate.h"
#ifndef FOOT_EARLY_TRIAL
#define FOOT_EARLY_TRIAL 0
#endif
#ifndef ANGLE_EARLY_TRIAL
#define ANGLE_EARLY_TRIAL 0
#endif
#if ANGLE_EARLY_TRIAL && !FOOT_EARLY_TRIAL
#error ANGLE_EARLY_TRIAL requires FOOT_EARLY_TRIAL
#endif
#include "long_foot_balance.h"
#include "negative_cap_recovery.h"

class ExperimentRunner {
public:
  enum class QRunMode : uint8_t { NONE = 0, VALIDATION = 1, CONTROL = 2 };
  void begin(PsramLogger& logger, ImuManager& imu, Roller485Manager& roller);
  void serviceFast();
  void RW_HOT_CODE update();
  void updateImuDynamicBetaContext();
  void observeFootFrame(uint16_t run_id,uint32_t frame_ms,bool left_valid,bool right_valid,float left_deg,float right_deg,float left_px,float right_px,
                        bool phase_left_detected,bool phase_right_detected,float phase_left_x,float phase_right_x);
  void setLoopDt(uint32_t dt_us) { status_.loop_dt_us = dt_us; }
  void recordTimingProbeLoop(uint32_t imu_update_us, uint32_t runner_update_us, uint32_t core1_path_us);

  bool startBatchSweepTest();
  bool startSingleTrialTest(uint8_t trial_number);
  bool startZeroCrossTest(int16_t fixed_current_mA, uint16_t fixed_pulse_width_ms,
                          QRunMode q_run_mode = QRunMode::NONE,
                          float control_target_deg = 0.0f,
                          uint8_t q_probe_schedule_id = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A);
  bool startZeroCrossIdentificationTest();
  bool startZeroCrossControlTest(float target_peak_deg, uint8_t q_probe_schedule_id);
  bool startPassiveCapture();
  // Dedicated actual-output fixed-Q protocol. No Q1/E2/legacy control caller
  // can enter this path.
  bool startQIdentCapture(uint8_t q_ident_run_schedule_id);  // Frozen: fail closed in this revision.
  bool startEnergyControlV0Capture();
  bool startEnergyControlAutonomousCapture();
  bool setEnergyControlAutonomousTarget(float target_deg);
  bool setEnergyControlAutonomousInputPeakPercent(float peak_percent);
  bool setEnergyControlAutonomousInputCurrentMa(int16_t current_mA);
  float energyControlAutonomousInputPeakPercent() const { return energy_control_autonomous_input_peak_percent_; }
  int16_t energyControlAutonomousInputCurrentMa() const { return energy_control_autonomous_input_current_mA_; }
  bool setEnergyControlAutonomousBrakeMaxMs(uint16_t brake_max_ms);
  uint16_t energyControlAutonomousBrakeMaxMs() const { return energy_control_autonomous_brake_max_ms_; }
  bool setPeakAdaptationEnabled(bool enabled);
  bool peakAdaptationEnabled() const { return peak_adaptation_enabled_; }
  bool setEnergyBridgeEnabled(bool enabled);
  bool energyBridgeEnabled() const { return energy_bridge_enabled_; }
  bool setNegativeProbeMode(bool enabled);
  bool negativeProbeMode() const { return negative_probe_mode_; }
  bool energyInverseMode() const {
    return energy_bridge_enabled_ && !negative_probe_mode_ &&
        energy_control_autonomous_brake_max_ms_ == 25;
  }
  bool wheelWorkMode() const {
    return energy_bridge_enabled_ && !negative_probe_mode_ &&
        energy_control_autonomous_brake_max_ms_ == 40;
  }
  bool negativeProbeBatchMode() const {
    return negative_probe_mode_ && autonomous_brake_limit::batch(energy_control_autonomous_brake_max_ms_);
  }
  bool negativeProbeExtendedMode() const {
    return negative_probe_mode_ && autonomous_brake_limit::extendedBatch(energy_control_autonomous_brake_max_ms_);
  }
  bool positiveCurrentBatchMode() const {
    return negative_probe_mode_ && autonomous_brake_limit::positiveCurrentBatch(energy_control_autonomous_brake_max_ms_);
  }
  bool anyNegativeProbeBatchMode() const { return negativeProbeBatchMode() || negativeProbeExtendedMode(); }
  bool anyProbeBatchMode() const { return anyNegativeProbeBatchMode() || positiveCurrentBatchMode(); }
  bool beginUprightReferenceCapture(bool equilibrium = false);
  bool setLateralCgMm(float shift_mm);
  const lateral_cg::Record& lateralCg() const { return lateral_cg_; }
  lateral_cg::AdaptiveSnapshot adaptiveCg() const { return adaptive_cg_.snapshot(); }
  bool equilibriumCapturing() const { return equilibrium_capture_ && uprightReferenceCapturing(); }
  float controlPotential(float signed_deg) const;
  float controlAngle(float energy, int8_t side) const;
  bool pollUprightReferenceCapture(bool& ok, const char*& result);
  const upright_reference::Record& uprightReference() const { return upright_reference_; }
  bool uprightReferenceCapturing() const { return upright_reference_capture_.pending(); }
  uint32_t uprightReferenceCaptureSamples() const { return upright_reference_capture_.samples(); }
  float uprightPitchErrorDeg(const ImuReading& r) const;
  bool isUprightStableSample(const ImuReading& r) const;
  void zeroAngleNow();
  bool zeroCurrentRollDisplay();
  bool setCurrentRollTarget(float target_deg);
  // Absolute next-peak target for Q1 direct shadow. It is snapshotted at
  // passive-run start and never feeds a motor command.
  bool setQ1ShadowTargetPeakAbs(float target_deg);
  float q1ShadowTargetPeakAbsDeg() const { return q1_shadow_target_peak_abs_deg_; }
  float q1ShadowActiveTargetPeakAbsDeg() const { return q1_shadow_run_target_peak_abs_deg_; }
  void requestEmergencyStop(const char* reason);
  void clearFinishedOrEstop();
  void sealCompletedLog() {
    if (running() || !logger_) return;
    maybeFinalizeTimingProbe();
    logger_->seal();
  }
  void setInputSettings(int16_t current_mA, uint16_t pulse_width_ms, uint16_t input_interval_ms);

  const ExperimentStatus& status() const { return status_; }
  bool zeroCrossMode() const { return zero_cross_mode_; }
  bool identificationMode() const { return identification_mode_; }
  bool controlMode() const { return q_run_mode_ == QRunMode::CONTROL; }
  bool passiveCaptureMode() const { return passive_capture_mode_; }
  bool qIdentMode() const { return q_ident_mode_; }
  bool energyControlV0Mode() const { return energy_control_v0_mode_; }
  bool energyControlAutonomousMode() const { return energy_control_autonomous_mode_; }
  float energyControlAutonomousTargetPeakDeg() const { return energy_control_autonomous_target_peak_deg_; }
  const char* energyControlAutonomousPhaseName() const;
  bool qIdentArmed() const { return q_ident_armed_; }
  uint8_t qIdentRunScheduleId() const { return q_ident_run_schedule_id_ + 1; }
  uint8_t qIdentPlusOccurrenceCount() const { return q_ident_side_occurrence_count_[0]; }
  uint8_t qIdentMinusOccurrenceCount() const { return q_ident_side_occurrence_count_[1]; }
  const char* qRunModeName() const;
  uint8_t qProbeScheduleId() const { return q_probe_schedule_id_; }
  const char* qProbeScheduleName() const;
  float controlTargetPeakDeg() const { return control_target_peak_deg_; }
  int16_t zeroCrossFixedCurrentMa() const { return zero_cross_fixed_current_mA_; }
  bool running() const {
    return status_.state == ExperimentState::START_SYNC || status_.state == ExperimentState::RUNNING_BATCH_SWEEP ||
           status_.state == ExperimentState::TRIAL_REST || status_.state == ExperimentState::END_SYNC;
  }
  const char* stateName() const;
  const char* presetName() const { return "MEKF_ADOPTED_DYNAMIC_BETA_COMPARE_V46"; }

#if FOOT_EARLY_TRIAL
  void setFootEarlyEpoch(uint64_t epoch_us);
  void invalidateFootEarlyCandidate() { early_motion_gate_.clear(); }
  uint32_t footEarlyPeakIdentity() const;
  int8_t footEarlyPeakSide() const;
  bool offerFootEarlyCandidate(uint16_t run_id,uint32_t peak_id,uint32_t frame_ms,
                               uint32_t ready_ms,float threshold_deg);
#endif

private:
  void updateGyroHeadingDiagnostic(const ImuReading& r);
  gradual_start::Controller gradual_start_;
  foot_balance_adaptive::Controller foot_balance_;
  foot_energy_balance::Controller foot_energy_;
  foot_phase_observer::Observer foot_phase_observer_;
  foot_phase_cap::Controller foot_phase_cap_;
  weak_side_assist::Controller weak_side_assist_;
  positive_left_boost::Controller positive_left_boost_;
  tilt_priority::Controller tilt_priority_;
  positive_left_boost::Input positive_left_input_;
  struct PhaseNegativePeak { uint32_t ms=0; float amplitude=NAN; };
  struct PhaseNegativeDecision {
    uint32_t ms=0; foot_phase_cap::NegativeDecision value;
    float trigger_deg=NAN,rate_dps=NAN,rpm=NAN;
  };
  struct SpanPositiveDecision {
    uint32_t ms=0; uint16_t width=0; int16_t ma=0; int8_t sign=0;
    float target_mj=NAN; bool executed=false;
  };
  struct SpanPositiveContext { float rate_dps=NAN,rpm=NAN; };
  PhaseNegativePeak phase_negative_peaks_[8];
  PhaseNegativeDecision phase_negative_decisions_[8];
  SpanPositiveDecision span_positive_decisions_[8];
  unsigned phase_negative_peak_next_=0,phase_negative_decision_next_=0;
  uint32_t phase_observer_sequence_=0;
  unsigned span_positive_decision_next_=0;
  uint32_t span_observer_sequence_=0;
  void __attribute__((noinline)) processFootPhaseCycle(uint32_t now_ms);
  void __attribute__((noinline)) processFootSpanCycle();
  void __attribute__((noinline)) recordFootPhasePeak(int8_t physical_side,uint32_t peak_ms,float amplitude_deg);
  wheel_speed_balance::Controller wheel_speed_balance_;
  uint16_t foot_frame_run_id_=0;
  uint32_t foot_frame_ms_=0;
  float foot_frame_left_=NAN,foot_frame_right_=NAN;
  bool foot_frame_left_valid_=false,foot_frame_right_valid_=false;
  float pending_target_energy_j_=NAN;
  uint32_t last_logged_speed_sequence_ = 0;
  uint32_t last_logged_position_sequence_ = 0;
  uint32_t last_logged_command_sequence_ = 0;
  upright_reference::Record upright_reference_;
  lateral_cg::Record lateral_cg_;
  bool equilibrium_capture_ = false;
  float run_cg_shift_mm_ = 0;
  float pending_cg_shift_mm_ = 0;
  lateral_cg::Adaptive adaptive_cg_;
  upright_reference::Capture upright_reference_capture_;
  float energy_control_autonomous_pending_target_deg_ = NAN;
  void beginFilters();
  void RW_SPEED_CODE updateFilterSeries(const ImuReading& r);
  void RW_SPEED_CODE finishDeferredComparison();
  void RW_SPEED_CODE updateComparisonDisplayAngles();
  struct DeferredComparison {
    bool pending = false, pulse_at_entry = false;
    float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0, beta = 0;
  } deferred_comparison_;
  void updateStartupCalibration(const ImuReading& r);
  void beginStartSync(uint32_t now_ms);
  void updateStartSync(uint32_t now_ms);
  void updateEndSync(uint32_t now_ms);
  bool updateSyncPattern(uint32_t now_ms, const Config::LedSyncStep* pattern, uint8_t step_count);
  void beginMeasurementRun();
  void beginTrial(uint8_t trial_index);
  void finishTrial(uint32_t now_ms);
  void updateTrialRest(uint32_t now_ms);
  void updateMidSyncLed(uint32_t now_ms);
  void beginEndSync(uint32_t now_ms);
  void updateInputPulse(uint32_t now_ms);
  void beginPulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction);
  void RW_SPEED_CODE stopActivePulse(uint32_t now_ms);
  void RW_SPEED_CODE updatePulseModelPrediction();
  float RW_SPEED_CODE predictedChargeMaS(float signed_i0_mA, int8_t direction, float width_ms,
                            int16_t commanded_current_mA) const;
  float identificationChargeMaS(float signed_i0_mA, int8_t direction, float width_ms) const;
  uint16_t widthForChargeTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction,
                                uint16_t min_width_ms, uint16_t max_width_ms) const;
  uint16_t identificationWidthForTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction) const;
  uint16_t calibrationProbeWidthForTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction) const;
  uint16_t calibrationWidthForTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction) const;
  uint16_t controlWidthForTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction) const;
  float predictNextPeakAbsDeg(float angle_deg, float rate_dps, float q_command_mA_s, int8_t direction) const;
  float requiredQForTargetPeakMaS(float target_peak_deg, float angle_deg, float rate_dps, int8_t direction) const;
  float requiredControlQMaS(float angle_deg, float rate_dps, int8_t direction) const;
  struct CalibrationBuildUpCommand {
    float q_required_mA_s = 0.0f; float q_command_mA_s = 0.0f; float q_effective_mA_s = 0.0f;
    float predicted_peak_deg = 0.0f; uint16_t width_ms = 0;
    bool q_cap_limited = false; bool guard_limited = false; bool width_limited = false;
  };
  CalibrationBuildUpCommand calibrationBuildUpCommand(float angle_deg, float rate_dps,
                                                        float signed_i0_mA, int8_t direction,
                                                        float target_peak_deg,
                                                        bool unrestricted = false) const;
  void recordCalibrationBuildUpPulse(uint32_t t_test_ms, float angle_deg, float rate_dps,
                                     int8_t direction, uint8_t phase, float target_peak_deg,
                                     const CalibrationBuildUpCommand& command);
  uint8_t calibrationProbeQLevelIndex(uint8_t plan_index) const;
  uint8_t classifyV59StateGate(float hprev_deg, float cprev_deg, float abs_rate_dps,
                               int8_t next_peak_side, bool dynamic_h_in_support) const;
  void recordV59StateGateEvent(uint32_t crossing_ms, float hprev_deg, float cprev_deg,
                               float abs_rate_dps, int8_t next_peak_side,
                               bool state_valid, bool dynamic_h_in_support,
                               uint8_t reason, uint8_t action,
                               bool v60_cooldown_free_decay = false,
                               bool v60_gate_skipped_by_decay = false);
  struct ControlQEvaluation {
    float q_command_mA_s = 0.0f;
    float predicted_peak_deg = 0.0f;
    float min_predicted_peak_deg = 0.0f;
    float max_predicted_peak_deg = 0.0f;
    bool target_reachable = false;
  };
  ControlQEvaluation evaluateControlQ(float angle_deg, float rate_dps, int8_t direction) const;
  enum class CalibrationPhase : uint8_t { INACTIVE, INITIAL_EXCITE, FREE_DECAY, Q_REBUILD,
                                           WAIT_Q_REBUILD, Q_CAL_POS, WAIT_Q_CAL_POS,
                                           Q_CAL_NEG, WAIT_Q_CAL_NEG, MAIN_CONTROL };
  struct CalibrationFit {
    uint8_t count = 0;
    float sum_x = 0.0f; float sum_y = 0.0f; float sum_x2 = 0.0f; float sum_y2 = 0.0f; float sum_xy = 0.0f;
    // Keep the individual pairs for leave-one-out cross-validation. The count
    // is bounded by the V60 free-decay support-wait limit per arrival side.
    float x_deg[Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE] = {};
    float y_deg[Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE] = {};
    // The selected model is valid only over this measured union of predecessor
    // and arrival amplitudes; it is never extrapolated beyond it.
    float support_min_deg = 0.0f; float support_max_deg = 0.0f;
    // Input domain is predecessor amplitude A_prev, separate from the
    // predecessor/arrival union used by the physical fit.
    float input_min_deg = 0.0f; float input_max_deg = 0.0f;
  };
  struct HalfRangeFit {
    static constexpr uint8_t kMaxPairs = 2 * Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE;
    uint8_t count = 0;
    float x_deg[kMaxPairs] = {}; float y_deg[kMaxPairs] = {};
    float sum_x = 0.0f; float sum_y = 0.0f; float sum_x2 = 0.0f; float sum_xy = 0.0f;
    float input_min_deg = 0.0f; float input_max_deg = 0.0f;
  };
  struct RateStateFit {
    uint8_t count = 0;
    float sum_h = 0.0f; float sum_c = 0.0f; float sum_rate = 0.0f;
    float sum_h2 = 0.0f; float sum_hc = 0.0f; float sum_c2 = 0.0f;
    float sum_hrate = 0.0f; float sum_crate = 0.0f; float sum_rate2 = 0.0f;
    float h_min_deg = 0.0f; float h_max_deg = 0.0f;
    float c_min_deg = 0.0f; float c_max_deg = 0.0f;
    float a_per_s = 0.0f; float b_per_s = 0.0f; float offset_dps = 0.0f;
    float r2 = 0.0f;
  };
  void resetCalibrationShadow();
  void updateCalibrationPeak(uint32_t now_ms, uint32_t t_test_ms, float angle_deg, float rate_dps);
  void recordCalibrationPeak(uint32_t candidate_peak_ms, uint32_t confirmed_ms,
                             int8_t side, float peak_signed_dynamic_deg, float peak_signed_fixed_deg, uint8_t phase,
                             float previous_peak_abs_deg, float q_effective_pred_mA_s);
  bool fitCalibrationFreeDecay();
  void addHalfRangeFreePair(HalfRangeFit& fit, float previous_half_range_deg, float half_range_deg);
  bool fitHalfRangeShadow(HalfRangeFit& fit, bool dynamic);
  bool configureV60RebuildTarget();
  void addV62RateStateSample(float hprev_deg, float cprev_deg, float abs_rate_dps);
  bool fitV62RateStateModel();
  bool v62HasFeasibleHCTarget() const;
  bool configureV62RebuildTarget();
  bool predictV61GateState(float controlled_peak_abs_deg, float* predicted_h_deg,
                           float* predicted_c_deg) const;
  bool v60GateCenterInDynamicHInputSupport() const;
  float halfRangeShadowPrediction(float previous_half_range_deg, bool dynamic, bool* in_support) const;
  bool dynamicHalfRangePredecessorInSupport() const;
  void finishCalibrationShadow(uint8_t failure_reason);
  float calibratedPredictionDeg(float q_effective_pred_mA_s, int8_t next_peak_side) const;
  bool calibrationIsMainControl() const { return calibration_phase_ == CalibrationPhase::MAIN_CONTROL || calibration_phase_ == CalibrationPhase::INACTIVE; }
  void beginIdentificationEvent(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                                float q_target_mA_s, float q_requested_mA_s,
                                float q_command_mA_s, float target_peak_deg,
                                float predicted_peak_deg, float min_predicted_peak_deg,
                                float max_predicted_peak_deg, bool target_reachable, bool bootstrap,
                                uint16_t width_ms, bool suppressed);
  void updateIdentificationPeak(uint32_t now_ms, float angle_deg, float rate_dps);
  float RW_SPEED_CODE predictCurrentGoalMa(float command_mA, float model_vbat_v) const;
  float RW_SPEED_CODE predictRiseTauS(float command_mA) const;
  float RW_SPEED_CODE predictBetaMin(float peak_current_mA) const;
  float betaFloorForStrategy(uint8_t index) const;
  float betaCeilingForStrategy(uint8_t index) const;
  uint16_t betaHoldAfterInputMsForStrategy(uint8_t index) const;
  void captureAngleOffsets();
  void captureMekfComparisonZero(float& zero_abs_deg, uint32_t& zero_sample_us);
  void updateMekfComparisonRelativeAngles();
  void RW_SPEED_CODE updateDisplayedAngles(const ImuReading& r);
  void RW_SPEED_CODE updateCurrentRollState(const ImuReading& r, uint32_t now_ms);
  // Legacy E2 is compiled only as an offline diagnostic reference and is not
  // called by the passive Q1 runtime.
  void resetE2ShadowPeakTracker();
  void updateE2ShadowPeakTracker(uint32_t now_ms);
  void recordE2ShadowPeak(uint32_t candidate_peak_ms, uint32_t confirmed_ms,
                          int8_t turn_side_from_rate,
                          float candidate_gyro_relative_deg,
                          float candidate_accel_abs_diag_deg);
  // Diagnostic values derived while accepting a Q1 zero-cross. They are not
  // inputs to the Q1 equation; the official Q1 inputs remain the current
  // sample's time and +gy rate.
  struct Q1ZeroCrossDiagnostics {
    int8_t detector_crossing_direction = 0;  // +1: detector - to +; -1: + to -.
    float detector_angle_before_deg = NAN;
    float detector_angle_after_deg = NAN;
    float crossing_interpolation_alpha = NAN;
    uint32_t interpolated_zero_cross_time_ms = 0;
    float physical_roll_rate_before_dps = NAN;
    float physical_roll_rate_after_dps = NAN;
    float interpolated_physical_roll_rate_dps = NAN;
    bool sign_gate_passed = false;
  };
  void resetQ1ShadowZeroCrossTracker();
  void updateQ1ShadowAtZeroCross(uint32_t now_ms);
  void recordQ1ShadowZeroCross(uint32_t t_test_ms, float rate_dps,
                               const Q1ZeroCrossDiagnostics& diagnostics);
  void resetQIdentTracker();
  void updateQIdentAtZeroCross(uint32_t t_test_ms, float rate_dps,
                                const Q1ZeroCrossDiagnostics& diagnostics);
  void updateQIdentPulse(uint32_t now_ms);
  void updateEnergyControlV0AtZeroCross(uint32_t t_test_ms, float rate_dps,
                                         const Q1ZeroCrossDiagnostics& diagnostics);
  enum class EnergyControlV0OutputGateState : uint8_t {
    WAIT_INITIAL_EXCURSION = 0,
    ARMED_FOR_ZERO_CROSS = 1,
    WAIT_OPPOSITE_EXCURSION = 2,
  };
  void resetEnergyControlV0OutputGate();
  void updateEnergyControlV0OutputGate(float detector_relative_angle_deg);
  void disarmEnergyControlV0AfterAcceptedCross(int8_t physical_next_peak_side);
  void updateEnergyControlV0Pulse(uint32_t now_ms);
  bool beginEnergyControlV0Pulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                                 uint16_t pulse_width_ms);
  enum class EnergyControlAutonomousPhase : uint8_t {
    IDLE = 0, STRONG_START_KICK = 1, WAIT_FIRST_PEAK = 2, ENERGY_CONTROL = 3, HOLD = 4, STOP = 5,
  };
  // Separate from the high-level target phase: one accepted physical peak, one
  // following accepted central passage, then at most one normal pulse.
  enum class EnergyControlAutonomousHalfCycleState : uint8_t {
    WAIT_PEAK = 0, WAIT_ZERO_CROSS = 1, PULSE_ACTIVE = 2,
  };
  void resetEnergyControlAutonomous();
  void RW_SPEED_CODE resetEnergyControlAutonomousPeakTracker(bool enable);
  void beginEnergyControlAutonomousStartKick(uint32_t now_ms);
  void RW_SPEED_CODE updateEnergyControlAutonomousMotion(uint32_t now_ms);
  bool __attribute__((noinline)) observeInwardStallRescue(uint32_t t_test_ms,
      bool physical_event_suppressed,bool peak_just_confirmed,float angle_deg,float rate_dps);
  void __attribute__((noinline)) acceptMotionCrossing(bool rescue,uint32_t t_test_ms,
      float interpolated_rate_dps,float before_deg,float after_deg,float alpha,float interpolated_time_ms);
  // Future experimentally enabled pre-zero source only. There is no caller in
  // the current firmware; normal percentage and zero triggers are unchanged.
  void __attribute__((noinline)) acceptOptionalEarlyMotion(uint32_t t_test_ms,float rate_dps,float angle_deg);
  bool __attribute__((noinline)) tryTiltPriorityOptionalEarly(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms);
  void __attribute__((noinline)) restoreUnsentStallRescue(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      bool previous_accepted_valid,uint32_t previous_accepted_ms);
  bool __attribute__((noinline)) dispatchTiltPriority(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms,
      bool previous_accepted_valid,uint32_t previous_accepted_ms);
  void __attribute__((noinline)) finishSolverAudit(solver_audit::Record& audit,
      const PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t end_us);
  void RW_SPEED_CODE updateEnergyControlAutonomousPeakTracker(uint32_t now_ms,
                                                float peak_relative_angle_deg,
                                                float rate_dps);
  bool RW_SPEED_CODE recordEnergyControlAutonomousPeak(uint32_t peak_ms, int8_t physical_side,
                                         float amplitude_deg, float detector_peak_angle_deg);
  float __attribute__((noinline)) energyControlPeakTarget(int8_t physical_side) const;
  void __attribute__((noinline)) notifyPositiveLeftPeak(
      int8_t physical_side,uint32_t peak_ms,float amplitude_deg,
      const PsramLogger::EnergyControlAutonomousPeakEvent& event);
  void RW_SPEED_CODE updateEnergyControlAutonomousAtZeroCross(uint32_t t_test_ms, float rate_dps,
                                                 float detector_before_deg,
                                                 float detector_after_deg,
                                                 float crossing_alpha,
                                                 float interpolated_time_ms);
  void RW_SPEED_CODE updateEnergyControlAutonomousPulse(uint32_t now_ms);
  // Candidate search runs only at an accepted ZERO crossing. Keep its fitted
  // regression and 183-candidate scan out of the periodic IRAM motion path.
  wheel_work_energy_control::Choice __attribute__((noinline)) chooseWheelWork(
      const PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      float fast_v, bool* state_supported, int16_t* selected_current_ma,
      const lateral_energy::CurrentTable** selected_current_table,uint16_t positive_or_walking_cap_ms=100) const;
  wheel_work_energy_control::Choice __attribute__((noinline)) chooseFootPhaseWidth(
      const PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      const wheel_work_energy_control::Choice& baseline,uint16_t width,float fast_v) const;
  bool __attribute__((noinline)) selectFootPhaseOutput(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms,float fast_v,
      wheel_work_energy_control::Choice& choice,int16_t& current,
      const lateral_energy::CurrentTable*& table);
  bool __attribute__((noinline)) prepareUnmodeledWalkCoast(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event) const;
  // Returns true only when an experimental assist was actually attempted.
  // The caller records the event and rearms only after a failed command.
  bool __attribute__((noinline)) tryWeakSideAssist(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      const wheel_work_energy_control::Choice& normal,bool unmodeled_coast,
      uint32_t t_test_ms,float fast_v);
  bool __attribute__((noinline)) tryPositiveLeftBoost(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      const wheel_work_energy_control::Choice& normal,uint32_t t_test_ms);
  bool __attribute__((noinline)) tryTiltPriority(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms);
  bool __attribute__((noinline)) tryTiltPriorityImpl(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms,bool* healthy_out);
  bool __attribute__((noinline)) tryTiltPriorityRescue(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms);
  void __attribute__((noinline)) choosePeakMpc(PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      float fast_v,wheel_work_energy_control::Choice& choice,int16_t& current,
      const lateral_energy::CurrentTable*& table) const;
  void __attribute__((noinline)) applyFootEnergyTarget(PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_ms);
  void __attribute__((noinline)) finishFootEnergyDecision(PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_ms);
  void __attribute__((noinline)) capturePendingWheelWork(const PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      const wheel_work_energy_control::Choice& choice,uint16_t selected_width_ms,float prediction_bias_j);
  void __attribute__((noinline)) selectPositiveCurrentBatch(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      int16_t& selected_current_mA,
      const lateral_energy::CurrentTable*& selected_current_table);
  uint16_t __attribute__((noinline)) selectFixedNegativeProbeWidth(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event) const;
  void runEnergyControlAutonomousSolverShadow();
  bool RW_SPEED_CODE beginEnergyControlAutonomousPulse(uint32_t now_ms, uint32_t t_test_ms,
      int8_t direction, uint16_t pulse_width_ms, int16_t command_current_mA);
  bool RW_SPEED_CODE beginEnergyControlAutonomousStartKickPulse(uint32_t now_ms, int8_t direction);
  float RW_SPEED_CODE energyControlPotentialJ(float amplitude_deg) const;
  // V46ai: no previous-amplitude free-peak method.
  float RW_SPEED_CODE energyControlAutonomousGainForSide(int8_t physical_side) const;
  void RW_SPEED_CODE energyControlAutonomousCorrectionParameters(int8_t physical_side,
                                                    float* c_side_used_deg,
                                                    float* g_side_corrected_deg_per_mA_s) const;
  float RW_SPEED_CODE energyControlAutonomousCorrectedPrediction(float free_next_peak_deg, int8_t physical_side,
                                                    float q_mA_s, float* correction_deg) const;
  bool qIdentRateInSupport(float abs_rate_dps) const;
  float qIdentRequiredWidthMs(float q_target_mA_s, float signed_i0_mA, int8_t direction) const;
  bool qIdentSolvePulse(float q_target_mA_s, float signed_i0_mA, int8_t direction,
                         float* required_width_ms, uint16_t* selected_integer_width_ms,
                         uint16_t* width_ms, float* q_effective_pred_mA_s) const;
  bool beginQIdentPulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                        uint16_t pulse_width_ms);
  void RW_SPEED_CODE startTimingProbe(uint8_t pulse_kind, uint32_t t_test_ms, int16_t command_mA,
                        uint16_t pulse_width_ms, uint32_t pulse_start_us,
                        uint32_t set_current_us, uint32_t state_update_us,
                        uint32_t current_model_us, uint32_t update_pulse_model_us,
                        uint32_t pulse_begin_total_us);
  void RW_SPEED_CODE maybeFinalizeTimingProbe();
  void RW_HOT_CODE logSampleIfDue();
  void RW_HOT_CODE logSampleNow();
  void finishRun();
  void RW_SPEED_CODE stopMotor();
  void setSyncLed(bool on);
  uint32_t measurementTotalDurationMs() const;
  float accelPitchDeg(const ImuReading& r) const;
  float physicalRollCandidateDeg(const ImuReading& r) const;
  float pitchBiasFromGyroBias() const;
  int16_t centi(float value) const;
  int16_t milli(float value) const;
  int16_t betaScaled(float value) const;

  PsramLogger* logger_ = nullptr;
  ImuManager* imu_ = nullptr;
  Roller485Manager* roller_ = nullptr;
  ExperimentStatus status_;
  mutable control_math::CurrentModelCache current_model_cache_;

  uint32_t boot_start_ms_ = 0;
  uint32_t calib_start_ms_ = 0;
  uint32_t settling_start_ms_ = 0;
  uint32_t run_start_ms_ = 0;
  uint32_t trial_start_ms_ = 0;
  uint32_t rest_start_ms_ = 0;
  uint64_t run_start_us_ = 0;
  uint32_t last_imu_update_us_ = 0;  // V46g: last consumed gyro sequence
  uint32_t last_mekf_accel_sequence_ = 0;
  uint32_t last_log_us_ = 0;
  uint32_t static_rate_since_ms_ = 0;
  float display_zero_offset_deg_ = 0.0f;
  float target_roll_deg_ = 0.0f;
  // Separate from Current Roll / READY. A zero target is intentionally a
  // braking request and is logged INVALID_BRAKING_NOT_IDENTIFIED.
  float q1_shadow_target_peak_abs_deg_ = 0.0f;
  float q1_shadow_run_target_peak_abs_deg_ = 0.0f;
  bool e2_shadow_armed_ = false;
  uint32_t e2_shadow_release_detected_ms_ = 0;
  bool e2_shadow_gyro_integrator_ready_ = false;
  uint32_t e2_shadow_last_gyro_sample_us_ = 0;
  float e2_shadow_last_gyro_rate_dps_ = 0.0f;
  float e2_shadow_gyro_relative_deg_ = 0.0f;
  float e2_shadow_pending_static_anchor_abs_deg_ = NAN;
  float e2_shadow_static_anchor_abs_deg_ = NAN;
  int8_t e2_shadow_candidate_motion_sign_ = 0;
  uint8_t e2_shadow_reverse_samples_ = 0;
  float e2_shadow_candidate_gyro_relative_deg_ = 0.0f;
  float e2_shadow_candidate_accel_abs_diag_deg_ = NAN;
  uint32_t e2_shadow_candidate_peak_ms_ = 0;
  bool e2_shadow_last_peak_valid_ = false;
  int8_t e2_shadow_last_turn_side_ = 0;
  float e2_shadow_last_peak_gyro_relative_deg_ = NAN;
  uint16_t e2_shadow_turn_index_ = 0;
  // Q1 detects a zero-cross from the adopted-filter relative angle. The model
  // state itself is official +gy physical rate; the angle only detects crossing.
  bool q1_shadow_has_previous_angle_ = false;
  bool q1_shadow_zero_cross_armed_ = false;
  float q1_shadow_angle_zero_deg_ = 0.0f;
  float q1_shadow_previous_relative_angle_deg_ = 0.0f;
  uint32_t q1_shadow_previous_time_ms_ = 0;
  float q1_shadow_previous_rate_dps_ = 0.0f;
  uint32_t q1_shadow_last_zero_cross_ms_ = 0;
  // Q_IDENT state has no shared scheduler, target, or controller state.
  bool q_ident_mode_ = false;
  bool q_ident_armed_ = false;
  bool q_ident_pulse_authorized_ = false;
  uint8_t q_ident_run_schedule_id_ = 0;
  uint8_t q_ident_arm_consecutive_count_ = 0;
  int8_t q_ident_last_arm_side_ = 0;
  uint8_t q_ident_side_occurrence_count_[Config::Q_IDENT_SIDE_COUNT] = {};
  // Separate from Q_IDENT: only this V0 mode can authorize an actual pulse.
  bool energy_control_v0_mode_ = false;
  bool energy_control_v0_pulse_authorized_ = false;
  EnergyControlV0OutputGateState energy_control_v0_output_gate_state_ =
      EnergyControlV0OutputGateState::WAIT_INITIAL_EXCURSION;
  int8_t energy_control_v0_previous_accepted_next_peak_side_ = 0;
  bool energy_control_v0_rearm_excursion_seen_ = false;
  float energy_control_v0_max_abs_detector_excursion_deg_ = 0.0f;
  // Dedicated autonomous path.  None of these fields are shared with the
  // Q_IDENT schedule or the manual-release V0 output gate.
  bool energy_control_autonomous_mode_ = false;
  bool energy_control_autonomous_pulse_authorized_ = false;
  EnergyControlAutonomousPhase energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::IDLE;
  EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_ =
      EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
  float energy_control_autonomous_input_peak_percent_ = autonomous_input_percent::kDefaultPercent;
  int16_t energy_control_autonomous_input_current_mA_ = autonomous_input_current::kDefaultMa;
  uint16_t energy_control_autonomous_brake_max_ms_ = autonomous_brake_limit::kDefaultMs;
  bool negative_probe_mode_ = false;
  uint16_t negative_probe_batch_crossing_index_ = 0;
  uint16_t positive_current_batch_crossing_index_ = 0;
  uint32_t negative_probe_batch_seed_ = 0;
  float inverse_negative_residual_j_[3] = {};
  uint8_t inverse_negative_residual_count_ = 0;
  uint8_t inverse_negative_residual_next_ = 0;
  uint8_t inverse_negative_extra_ms_ = 0;
  uint16_t inverse_negative_pending_width_ms_ = 0;
  float inverse_negative_pending_bias_j_ = 0.f;
  wheel_work_energy_control::NegativeFeedback work_negative_feedback_;
  walking_energy_loss::Feedback walking_loss_feedback_;
  walking_energy_loss::LowSpeedGuard walking_low_speed_guard_;
  low_peak_energy::Feedback low_peak_feedback_;
  float work_positive_low_applied_bias_mj_=0.f;
  bool work_walking_pending_=false,work_walking_low_pending_=false;
  wheel_work_energy_control::NegativeAssistFeedback work_negative_assist_feedback_;
  target12_state_work::Controller work_target12_controller_;
  float work_pending_battery_mv_=NAN;
  float work_pending_calibration_weight_=0.f;
  uint16_t work_pending_cap_ms_=0;
  wheel_work_energy_control::SideEnergyFeedback work_positive_feedback_[4]; // bridge, work, negative brake/coast, negative assist
  bool work_positive_pending_ = false;
  uint8_t work_positive_pending_branch_ = 0;
  float work_positive_pending_raw_j_ = NAN;
  bool work_negative_pending_ = false;
  bool work_negative_pending_assist_ = false;
  bool work_negative_pending_supported_ = false;
  uint16_t work_negative_pending_width_ms_ = 0;
  float work_negative_pending_bias_j_ = 0.f;
  void __attribute__((noinline)) captureWheelWorkDecision(
      PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
      const wheel_work_energy_control::Choice& choice) const;
  void __attribute__((noinline)) observeWheelWorkPeak(
      const PsramLogger::EnergyControlAutonomousPeakEvent& event);
  float energy_control_autonomous_target_peak_deg_ = Config::ENERGY_CONTROL_AUTONOMOUS_DEFAULT_TARGET_PEAK_DEG;
  lateral_energy::Geometry lateral_energy_geometry_{
      Config::ENERGY_CONTROL_V0_MASS_KG,Config::ENERGY_CONTROL_V0_GRAVITY_M_S2,
      Config::ENERGY_CONTROL_V0_CG_HEIGHT_M,Config::ENERGY_CONTROL_V0_RADIUS_M,
      Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M,Config::ENERGY_CONTROL_V0_OUTER_EDGE_X_M};
  lateral_energy::CurrentTable lateral_current_table_;
  lateral_energy::CurrentTable positive_current_probe_tables_[2];
  peak_error_adaptation::Model peak_error_model_;
  bool peak_adaptation_enabled_ = false;
  bool energy_bridge_enabled_ = true;
  float lateral_target_energy_j_ = NAN;
  float energy_control_autonomous_integral_plus_j_ = 0.0f;
  float energy_control_autonomous_integral_minus_j_ = 0.0f;
  // V46ae: one MEKF state supplies peak amplitude and zero-cross timing.
  // Repeated delivery of a gyro sample cannot count as another return sample.
  uint32_t energy_control_autonomous_last_motion_sample_us_ = 0;
  bool energy_control_autonomous_detector_has_previous_angle_ = false;
  float energy_control_autonomous_detector_zero_angle_deg_ = 0.0f;  // legacy retained for layout/source compatibility; not used by detector.
  float energy_control_autonomous_previous_detector_relative_angle_deg_ = 0.0f;
  float energy_control_autonomous_previous_detector_rate_dps_ = 0.0f;
  uint32_t energy_control_autonomous_previous_detector_test_ms_ = 0;
  bool energy_control_autonomous_zero_cross_consumed_for_peak_ = false;
  bool energy_control_autonomous_last_accepted_zero_cross_valid_ = false;
  uint32_t energy_control_autonomous_last_accepted_zero_cross_ms_ = 0;
  bool energy_control_autonomous_peak_tracker_enabled_ = false;
  bool energy_control_autonomous_peak_tracker_started_ = false;
  int8_t energy_control_autonomous_candidate_detector_side_ = 0;
  uint8_t energy_control_autonomous_return_samples_ = 0;
  float energy_control_autonomous_candidate_detector_peak_abs_deg_ = 0.0f;
  float energy_control_autonomous_candidate_peak_amplitude_deg_ = 0.0f;
  uint32_t energy_control_autonomous_candidate_peak_ms_ = 0;
  bool energy_control_autonomous_last_peak_valid_ = false;
  float energy_control_autonomous_last_peak_amplitude_deg_ = NAN;
  int8_t energy_control_autonomous_last_peak_side_ = 0;
  uint32_t energy_control_autonomous_last_peak_ms_ = 0;
  bool energy_control_autonomous_pending_peak_ = false;
  bool energy_control_autonomous_pending_probe_ = false;
  int8_t energy_control_autonomous_pending_next_side_ = 0;
  float energy_control_autonomous_pending_q_command_mA_s_ = 0.0f;
  float energy_control_autonomous_pending_model_energy_j_ = NAN;
  float energy_control_autonomous_pending_wheel_rpm_ = NAN;
  uint32_t energy_control_autonomous_pending_wheel_age_us_ = UINT32_MAX;
  uint32_t energy_control_autonomous_pending_zero_ms_ = 0;
  peak_error_adaptation::Mode energy_control_autonomous_pending_mode_ = peak_error_adaptation::Coast;
  bool energy_control_autonomous_pending_wheel_valid_ = false;
  bool energy_control_autonomous_pending_saturated_upper_ = false;
  bool energy_control_autonomous_pending_saturated_lower_ = false;
  uint16_t energy_control_autonomous_pending_zero_event_index_ = 0;
  PsramLogger::SolverShadowEvent solver_shadow_event_{};
  bool solver_shadow_pending_ = false;
  PsramLogger::TimingProbeEvent timing_probe_event_{};
  bool timing_probe_pending_ = false;
  bool timing_probe_loop_captured_ = false;
  bool timing_probe_log_captured_ = false;
  bool timing_probe_imu_captured_ = false;
  uint32_t next_pulse_start_test_ms_ = 0;  uint32_t active_pulse_start_ms_ = 0;
  uint32_t active_pulse_start_test_ms_ = 0;
  bool passive_capture_mode_ = false;
  uint32_t last_pulse_end_ms_ = 0;
  int8_t next_pulse_direction_ = 1;
  bool zero_cross_mode_ = false;
  bool zero_cross_bootstrap_pending_ = false;
  bool zero_cross_armed_ = false;
  bool zero_cross_has_previous_angle_ = false;
  float zero_cross_previous_angle_deg_ = 0.0f;
  uint32_t last_zero_cross_pulse_start_ms_ = 0;
  uint32_t zero_cross_start_refractory_until_ms_ = 0;
  float zero_cross_half_cycle_peak_abs_deg_ = 0.0f;
  int8_t zero_cross_next_direction_ = -Config::ZERO_CROSS_BOOTSTRAP_DIRECTION;
  int16_t zero_cross_fixed_current_mA_ = Config::ZERO_CROSS_DEFAULT_FIXED_CURRENT_MA;
  uint16_t zero_cross_fixed_pulse_ms_ = Config::ZERO_CROSS_DEFAULT_FIXED_PULSE_MS;
  bool identification_mode_ = false;
  QRunMode q_run_mode_ = QRunMode::NONE;
  uint8_t q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  float control_target_peak_deg_ = 0.0f;
  CalibrationPhase calibration_phase_ = CalibrationPhase::INACTIVE;
  bool calibration_enabled_ = false;
  bool calibration_peak_tracker_ready_ = false;
  // After a confirmed turn, candidate tracking stays disarmed until motion has
  // crossed into the opposite half-cycle and is moving outward again.
  int8_t calibration_expected_candidate_side_ = 0;
  int8_t calibration_outbound_rate_sign_ = 0;
  uint8_t calibration_reverse_samples_ = 0;
  float calibration_peak_candidate_deg_ = 0.0f;
  float calibration_peak_candidate_fixed_deg_ = 0.0f;
  uint32_t calibration_peak_candidate_ms_ = 0;
  bool calibration_last_peak_valid_ = false;
  int8_t calibration_last_peak_side_ = 0;
  float calibration_last_peak_abs_deg_ = 0.0f;
  float calibration_last_peak_signed_dynamic_deg_ = 0.0f;
  float calibration_last_peak_signed_fixed_deg_ = 0.0f;
  float calibration_last_half_range_dynamic_deg_ = 0.0f;
  float calibration_last_half_range_fixed_deg_ = 0.0f;
  bool calibration_last_half_range_valid_ = false;
  float calibration_last_center_dynamic_deg_ = 0.0f;
  bool calibration_last_center_dynamic_valid_ = false;
  uint32_t calibration_last_peak_candidate_ms_ = 0;
  CalibrationFit calibration_fit_pos_;
  CalibrationFit calibration_fit_neg_;
  HalfRangeFit calibration_half_range_dynamic_fit_;
  HalfRangeFit calibration_half_range_fixed_fit_;
  float calibration_free_last_half_range_dynamic_deg_ = 0.0f;
  float calibration_free_last_half_range_fixed_deg_ = 0.0f;
  bool calibration_free_last_half_range_valid_ = false;
  PsramLogger::CalibrationResult calibration_result_;
  float calibration_probe_previous_peak_abs_deg_ = 0.0f;
  float calibration_probe_previous_peak_signed_dynamic_deg_ = 0.0f;
  float calibration_probe_previous_peak_signed_fixed_deg_ = 0.0f;
  float calibration_probe_previous_half_range_dynamic_deg_ = 0.0f;
  float calibration_probe_previous_half_range_fixed_deg_ = 0.0f;
  bool calibration_probe_previous_half_range_valid_ = false;
  float calibration_probe_previous_center_dynamic_deg_ = 0.0f;
  bool calibration_probe_previous_center_dynamic_valid_ = false;
  uint32_t calibration_probe_previous_peak_candidate_ms_ = 0;
  float calibration_probe_q_target_mA_s_ = 0.0f;
  float calibration_probe_q_effective_pred_mA_s_ = 0.0f;
  float calibration_probe_command_dynamic_angle_deg_ = 0.0f;
  float calibration_probe_command_rate_raw_dps_ = 0.0f;
  float calibration_probe_command_rate_bias_corrected_dps_ = 0.0f;
  uint32_t calibration_probe_pulse_start_test_ms_ = 0;
  uint16_t calibration_probe_pulse_id_ = 0;
  int8_t calibration_probe_requested_side_ = 0;
  int8_t calibration_probe_predecessor_side_ = 0;
  int8_t calibration_probe_direction_ = 0;
  uint8_t calibration_probe_plan_index_ = 0;
  uint8_t calibration_probe_active_plan_index_ = 0;
  uint8_t calibration_probe_q_level_index_ = 0;
  uint8_t calibration_probe_rebuild_total_count_ = 0;
  uint8_t calibration_probe_rebuild_episode_id_ = 0;
  uint8_t calibration_probe_rebuild_attempt_in_episode_ = 0;
  // Audit how the predecessor was allowed to enter a Q probe: 0=legacy A
  // input-domain support, 1=dynamic-H support. V60 logs its per-run
  // free-decay-derived rebuild target separately in CalibrationResult.
  uint8_t calibration_probe_rebuild_entry_source_ = 0;
  bool calibration_probe_rebuild_target_reached_ = false;
  bool calibration_probe_dynamic_h_in_support_at_command_ = false;
  uint8_t calibration_probe_wait_halfcycle_count_ = 0;
  uint8_t calibration_probe_wait_halfcycle_total_ = 0;
  uint8_t calibration_probe_attempt_count_ = 0;
  uint8_t calibration_v59_gate_event_count_ = 0;
  uint8_t calibration_v59_gate_pass_count_ = 0;
  uint8_t calibration_v59_gate_skip_count_ = 0;
  uint8_t calibration_v59_rebuild_from_low_state_count_ = 0;
  uint8_t calibration_v60_cooldown_halfcycles_remaining_ = 0;
  bool calibration_v60_high_desired_wait_pending_ = false;
  // V61 always commands the side opposite to the first fixed-Q arrival, then
  // evaluates the two subsequently unforced peaks as an H/C state.
  bool calibration_v61_rebuild_plan_active_ = false;
  float calibration_v61_nominal_controlled_peak_target_deg_ = 0.0f;
  float calibration_v61_command_peak_target_deg_ = 0.0f;
  RateStateFit calibration_v62_rate_fit_;
  float calibration_gain_sum_pos_ = 0.0f;
  float calibration_gain_sum_neg_ = 0.0f;
  float calibration_gain_sum_sq_pos_ = 0.0f;
  float calibration_gain_sum_sq_neg_ = 0.0f;
  uint8_t calibration_initial_kick_count_ = 0;
  float calibration_initial_kick_q_effective_pred_mA_s_ = 0.0f;
  uint16_t calibration_rebuild_pulse_id_ = 0;
  uint32_t calibration_rebuild_pulse_start_test_ms_ = 0;
  uint8_t calibration_rebuild_total_count_ = 0;
  uint8_t calibration_rebuild_episode_id_ = 0;
  uint8_t calibration_rebuild_attempt_in_episode_ = 0;
  uint8_t identification_target_index_ = 0;
  uint16_t identification_event_id_ = 0;
  bool identification_peak_waiting_ = false;
  int8_t identification_outbound_rate_sign_ = 0;
  uint8_t identification_reverse_samples_ = 0;
  float identification_peak_angle_deg_ = 0.0f;
  float predicted_signed_current_end_mA_ = 0.0f;
  uint32_t predicted_current_end_ms_ = 0;
  uint8_t sync_step_ = 0;
  uint32_t sync_step_start_ms_ = 0;
  uint32_t sync_led_until_ms_ = 0;
  bool single_trial_mode_ = false;
  uint8_t selected_trial_index_ = 0;

  double bias_sum_x_ = 0.0;
  double bias_sum_y_ = 0.0;
  double bias_sum_z_ = 0.0;
  uint32_t bias_sample_count_ = 0;
  bool bias_ready_ = false;

  float beta_smooth_[Config::DYNAMIC_BETA_COUNT] = {};
  BetaPhaseController beta_phase_;
  BetaTurnFastController beta_turn_fast_;
  float raw_beta1_raw_pitch_deg_ = 0.0f;
  float raw_dynamic_raw_pitch_deg_[Config::DYNAMIC_BETA_COUNT] = {};
  float raw_beta1_bias_pitch_deg_ = 0.0f;
  float raw_dynamic_bias_pitch_deg_[Config::DYNAMIC_BETA_COUNT] = {};
  float raw_accel_pitch_deg_ = 0.0f;
  float offset_beta1_raw_deg_ = 0.0f;
  float offset_dynamic_raw_deg_[Config::DYNAMIC_BETA_COUNT] = {};
  float offset_beta1_bias_deg_ = 0.0f;
  float offset_dynamic_bias_deg_[Config::DYNAMIC_BETA_COUNT] = {};
  float offset_accel_deg_ = 0.0f;
  float raw_mekf_pitch_abs_deg_ = 0.0f;
  float raw_mekf_predicted_abs_deg_ = 0.0f;
  float offset_mekf_pitch_deg_ = 0.0f;
  bool mekf_initialized_ = false;
  mekf6::Mekf6 mekf_;
  tilt_stop::ForeAftGuard tilt_guard_;
  float gyro_raw_deg_ = 0.0f;
  float gyro_bias_corrected_deg_ = 0.0f;

  Adafruit_Madgwick filter_beta1_raw_;
  Adafruit_Madgwick filter_dynamic_raw_[Config::DYNAMIC_BETA_COUNT];
  Adafruit_Madgwick filter_beta1_bias_;
  Adafruit_Madgwick filter_dynamic_bias_[Config::DYNAMIC_BETA_COUNT];
  // Place this additional camera-feedback array after the motion state so
  // these per-event fields do not shift hot members that follow the ring.
  SpanPositiveContext span_positive_context_[8];
  // Keep the experimental wheel state after the hot motion fields.
  wheel_cycle_guard::Controller wheel_cycle_guard_;
  // Experimental foot feedback follows hot motion fields to preserve IRAM size.
  foot_span_control::Controller foot_span_control_;
  // Camera history is cold state; place it after the motion control fields.
  foot_fullstride_observer::Observer foot_span_observer_;
  // Diagnostics are cold and must not change hot member offsets or control.
  GyroHeadingDiagnostic gyro_heading_diagnostic_;
  stall_rescue::Detector stall_rescue_detector_;
#if defined(EARLY_RESCUE_TRIAL) && EARLY_RESCUE_TRIAL
  stall_rescue::Detector early_stall_rescue_detector_;
  bool early_stall_rescue_event_=false;
#endif
  uint32_t stall_rescue_attempted_peak_ms_=0;
  bool stall_rescue_event_=false;
  bool optional_early_event_=false;
  bool optional_early_write_uncertain_=false;
#if FOOT_EARLY_TRIAL
#if ANGLE_EARLY_TRIAL
  uint32_t angle_late_blocked_peak_ms_=0,angle_last_check_ms_=0;
  void RW_HOT_CODE __attribute__((optimize("Os", "no-fast-math"),noinline)) tryAngleEarlyCandidate(
      uint32_t t_test_ms,uint32_t sample_us,float angle_deg,float rate_dps);
  void __attribute__((noinline)) decideAngleEarlyCandidate(
      uint32_t t_test_ms,uint32_t sample_us,float angle_deg,float rate_dps);
#endif
  early_motion_gate::Gate early_motion_gate_;
  uint64_t early_measurement_epoch_us_=0;
  void RW_HOT_CODE __attribute__((optimize("Os", "no-fast-math"),noinline)) tryEarlyFootCandidate(uint32_t t_test_ms,uint32_t sample_us,
      float angle_deg,float rate_dps);
  void __attribute__((noinline)) decideEarlyFootCandidate(uint32_t t_test_ms,uint32_t sample_us,
      float angle_deg,float rate_dps);
#endif
  uint32_t optional_early_attempted_peak_ms_=0;
  bool gyro_heading_start_pending_ = false;
  long_foot_balance::Controller long_foot_balance_;
  negative_cap_recovery::Controller negative_cap_recovery_;
  uint32_t negative_recovery_last_applied_ms_=0;
  // Pair a guarded intervention with the exact positive command in a later
  // camera cycle. Kept separate from the hot decision record.
  uint32_t foot_trial_pending_ms_=0,foot_trial_pending_generation_=0;
  bool foot_trial_pending_effective_=false;
  uint32_t foot_trial_generation_[8]={};
  bool foot_trial_effective_[8]={};
  float negative_preceding_positive_peaks_[8] = {NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN};
  uint32_t negative_preceding_positive_peak_ms_[8] = {};
  uint8_t span_positive_branch_[8] = {};
  positive_sway_probe::Controller positive_sway_probe_;
};
