from pathlib import Path

OLD_REV = "v46j_mekf_dual_core_roller_ready_20260913"
NEW_REV = "v46k_pulse_start_timing_probe_20260914"


def read(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def write(path: str, text: str) -> None:
    Path(path).write_text(text, encoding="utf-8")


def replace_once(path: str, old: str, new: str) -> None:
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected exactly one match, got {count}: {old[:120]!r}")
    write(path, text.replace(old, new, 1))


def replace_between(path: str, start: str, end: str, replacement: str) -> None:
    text = read(path)
    i = text.find(start)
    if i < 0:
        raise RuntimeError(f"{path}: start marker not found: {start!r}")
    j = text.find(end, i + len(start))
    if j < 0:
        raise RuntimeError(f"{path}: end marker not found: {end!r}")
    write(path, text[:i] + replacement + text[j:])


# ---------------------------------------------------------------------------
# Identity only. V46k intentionally changes no controller constants, current
# audit rate, MEKF tuning, motor current, pulse-width selection, or RWLOG v46
# sample layout. It adds measurement probes around the existing V46j path.
# ---------------------------------------------------------------------------
replace_once(
    "src/config.h",
    f'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "{OLD_REV}";',
    f'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "{NEW_REV}";',
)

main = read("src/main.cpp")
main = main.replace("V46j", "V46k")
write("src/main.cpp", main)

site = read("site/index.html")
site = site.replace("V46j", "V46k").replace(OLD_REV, NEW_REV)
write("site/index.html", site)
replace_once("site/manifest.json", '"version": "0.46.9"', '"version": "0.46.10"')
manifest = read("site/manifest.json").replace("V46j", "V46k").replace(OLD_REV, NEW_REV)
write("site/manifest.json", manifest)

# Existing guards are retained, with only their revision/UI identity advanced.
for guard_path in (
    "tools/test_v46g_highrate_source_guards.py",
    "tools/test_v46i_task_split_source_guards.py",
    "tools/test_v46_motor_validation_source_guards.py",
):
    s = read(guard_path)
    s = s.replace(OLD_REV, NEW_REV).replace("V46j", "V46k").replace("0.46.9", "0.46.10")
    write(guard_path, s)

# ---------------------------------------------------------------------------
# PsramLogger: one compact timing event per physical motor pulse. These events
# live only in metadata JSON, so sizeof(LogSample), RWLOG format v46 and every
# existing timeseries column remain unchanged.
# ---------------------------------------------------------------------------
logger_h = read("src/psram_logger.h")
needle = "  void addEnergyControlAutonomousZeroCrossEvent(const EnergyControlAutonomousZeroCrossEvent& event);\n"
if logger_h.count(needle) != 1:
    raise RuntimeError("psram_logger.h: autonomous zero-cross method marker changed")
timing_struct = r'''  struct TimingProbeEvent {
    uint16_t event_index = 0;
    uint32_t pulse_id = 0;
    uint8_t pulse_kind = 0;  // 1=strong_start_kick, 2=normal_zero_cross.
    uint32_t t_test_ms = 0;
    int16_t command_mA = 0;
    uint16_t pulse_width_ms = 0;
    uint32_t pulse_start_us = 0;
    uint32_t gyro_sequence_at_start = 0;
    uint32_t set_current_us = 0;
    uint32_t state_update_us = 0;
    uint32_t current_model_us = 0;
    uint32_t update_pulse_model_us = 0;
    uint32_t pulse_begin_total_us = 0;
    uint32_t first_audit_log_offset_us = 0;
    uint32_t first_audit_log_us = 0;
    uint32_t imu_update_call_us = 0;
    uint32_t runner_update_call_us = 0;
    uint32_t core1_path_us = 0;
    uint32_t first_imu_dt_after_start_us = 0;
    uint32_t first_imu_sample_offset_us = 0;
  };
'''
logger_h = logger_h.replace(
    needle,
    timing_struct + needle + "  void addTimingProbeEvent(const TimingProbeEvent& event);\n",
    1,
)
logger_h = logger_h.replace(
    "  static constexpr uint16_t kMaxEnergyControlAutonomousEvents = Config::ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS;\n",
    "  static constexpr uint16_t kMaxEnergyControlAutonomousEvents = Config::ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS;\n"
    "  static constexpr uint16_t kMaxTimingProbeEvents = 96;\n",
    1,
)
private_marker = (
    "  EnergyControlAutonomousZeroCrossEvent energy_control_autonomous_zero_cross_events_[kMaxEnergyControlAutonomousEvents] = {};\n"
    "  uint16_t energy_control_autonomous_zero_cross_event_count_ = 0;\n"
    "  bool energy_control_autonomous_event_overflow_ = false;\n"
)
if logger_h.count(private_marker) != 1:
    raise RuntimeError("psram_logger.h: autonomous private storage marker changed")
logger_h = logger_h.replace(
    private_marker,
    private_marker
    + "  TimingProbeEvent timing_probe_events_[kMaxTimingProbeEvents] = {};\n"
    + "  uint16_t timing_probe_event_count_ = 0;\n"
    + "  bool timing_probe_event_overflow_ = false;\n",
    1,
)
write("src/psram_logger.h", logger_h)

logger_cpp = read("src/psram_logger.cpp")
reset_line = "  energy_control_autonomous_event_overflow_ = false;\n"
if logger_cpp.count(reset_line) != 2:
    raise RuntimeError(f"psram_logger.cpp: expected two autonomous reset sites, got {logger_cpp.count(reset_line)}")
logger_cpp = logger_cpp.replace(
    reset_line,
    reset_line + "  timing_probe_event_count_ = 0;\n  timing_probe_event_overflow_ = false;\n",
)
method_marker = "void PsramLogger::setCalibrationResult(const CalibrationResult& result) {"
if logger_cpp.count(method_marker) != 1:
    raise RuntimeError("psram_logger.cpp: setCalibrationResult marker changed")
add_method = r'''void PsramLogger::addTimingProbeEvent(const TimingProbeEvent& event) {
  if (timing_probe_event_count_ >= kMaxTimingProbeEvents) {
    timing_probe_event_overflow_ = true;
    return;
  }
  TimingProbeEvent stored = event;
  stored.event_index = timing_probe_event_count_ + 1;
  timing_probe_events_[timing_probe_event_count_++] = stored;
}
'''
logger_cpp = logger_cpp.replace(method_marker, add_method + method_marker, 1)

metadata_marker = '  json += "\\\"energy_control_autonomous_peak_events\\\":[";\n'
if logger_cpp.count(metadata_marker) != 1:
    raise RuntimeError("psram_logger.cpp: autonomous metadata marker changed")
timing_metadata = r'''  json += "\"v46k_timing_probe_revision\":\"v46k_pulse_start_core1_profile_20260914\",";
  json += "\"v46k_timing_probe_scope\":\"measurement_only;no_controller_or_motor_decision_reads_timing_values\",";
  json += "\"v46k_timing_probe_event_overflow\":" + String(timing_probe_event_overflow_ ? "true" : "false") + ",";
  json += "\"v46k_timing_probe_events\":[";
  for (uint16_t i = 0; i < timing_probe_event_count_; ++i) {
    const TimingProbeEvent& e = timing_probe_events_[i];
    if (i) json += ",";
    json += "{\"event_index\":" + String(e.event_index);
    json += ",\"pulse_id\":" + String(e.pulse_id);
    json += ",\"pulse_kind\":\"" + String(e.pulse_kind == 1 ? "strong_start_kick" : "normal_zero_cross") + "\"";
    json += ",\"t_test_ms\":" + String(e.t_test_ms);
    json += ",\"command_mA\":" + String(e.command_mA);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    json += ",\"pulse_start_us\":" + String(e.pulse_start_us);
    json += ",\"gyro_sequence_at_start\":" + String(e.gyro_sequence_at_start);
    json += ",\"set_current_us\":" + String(e.set_current_us);
    json += ",\"state_update_us\":" + String(e.state_update_us);
    json += ",\"current_model_us\":" + String(e.current_model_us);
    json += ",\"update_pulse_model_us\":" + String(e.update_pulse_model_us);
    json += ",\"pulse_begin_total_us\":" + String(e.pulse_begin_total_us);
    json += ",\"first_audit_log_offset_us\":" + String(e.first_audit_log_offset_us);
    json += ",\"first_audit_log_us\":" + String(e.first_audit_log_us);
    json += ",\"imu_update_call_us\":" + String(e.imu_update_call_us);
    json += ",\"runner_update_call_us\":" + String(e.runner_update_call_us);
    json += ",\"core1_path_us\":" + String(e.core1_path_us);
    json += ",\"first_imu_dt_after_start_us\":" + String(e.first_imu_dt_after_start_us);
    json += ",\"first_imu_sample_offset_us\":" + String(e.first_imu_sample_offset_us) + "}";
  }
  json += "],";
'''
logger_cpp = logger_cpp.replace(metadata_marker, timing_metadata + metadata_marker, 1)
write("src/psram_logger.cpp", logger_cpp)

# ---------------------------------------------------------------------------
# ExperimentRunner state and API. The event remains pending for only a few ms:
# pulse start -> same-loop core timing/log timing -> first subsequent gyro sample.
# It is then copied to PsramLogger metadata storage.
# ---------------------------------------------------------------------------
runner_h = read("src/experiment_runner.h")
runner_h = runner_h.replace(
    "  void setLoopDt(uint32_t dt_us) { status_.loop_dt_us = dt_us; }\n",
    "  void setLoopDt(uint32_t dt_us) { status_.loop_dt_us = dt_us; }\n"
    "  void recordTimingProbeLoop(uint32_t imu_update_us, uint32_t runner_update_us, uint32_t core1_path_us);\n",
    1,
)
runner_h = runner_h.replace(
    "  void logSampleIfDue();\n",
    "  void startTimingProbe(uint8_t pulse_kind, uint32_t t_test_ms, int16_t command_mA,\n"
    "                        uint16_t pulse_width_ms, uint32_t pulse_start_us,\n"
    "                        uint32_t set_current_us, uint32_t state_update_us,\n"
    "                        uint32_t current_model_us, uint32_t update_pulse_model_us,\n"
    "                        uint32_t pulse_begin_total_us);\n"
    "  void maybeFinalizeTimingProbe();\n"
    "  void logSampleIfDue();\n",
    1,
)
pending_marker = (
    "  uint16_t energy_control_autonomous_pending_zero_event_index_ = 0;  uint32_t next_pulse_start_test_ms_ = 0;"
)
if runner_h.count(pending_marker) != 1:
    raise RuntimeError("experiment_runner.h: pending event marker changed")
runner_h = runner_h.replace(
    pending_marker,
    "  uint16_t energy_control_autonomous_pending_zero_event_index_ = 0;\n"
    "  PsramLogger::TimingProbeEvent timing_probe_event_{};\n"
    "  bool timing_probe_pending_ = false;\n"
    "  bool timing_probe_loop_captured_ = false;\n"
    "  bool timing_probe_log_captured_ = false;\n"
    "  bool timing_probe_imu_captured_ = false;\n"
    "  uint32_t next_pulse_start_test_ms_ = 0;",
    1,
)
write("src/experiment_runner.h", runner_h)

runner_cpp = read("src/experiment_runner.cpp")
imu_marker = "  if (r.gyro_sequence != 0 && r.gyro_sequence != last_imu_update_us_) {\n    updateFilterSeries(r);\n"
if runner_cpp.count(imu_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: fresh gyro marker changed")
runner_cpp = runner_cpp.replace(
    imu_marker,
    "  if (r.gyro_sequence != 0 && r.gyro_sequence != last_imu_update_us_) {\n"
    "    if (timing_probe_pending_ && !timing_probe_imu_captured_ &&\n"
    "        r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start) {\n"
    "      timing_probe_event_.first_imu_dt_after_start_us = r.gyro_update_dt_us;\n"
    "      timing_probe_event_.first_imu_sample_offset_us = r.last_gyro_update_us == 0 ? 0 :\n"
    "          static_cast<uint32_t>(r.last_gyro_update_us - timing_probe_event_.pulse_start_us);\n"
    "      timing_probe_imu_captured_ = true;\n"
    "      maybeFinalizeTimingProbe();\n"
    "    }\n"
    "    updateFilterSeries(r);\n",
    1,
)

reset_marker = "void ExperimentRunner::resetEnergyControlAutonomous() {\n"
if runner_cpp.count(reset_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: reset autonomous marker changed")
runner_cpp = runner_cpp.replace(
    reset_marker,
    reset_marker
    + "  timing_probe_event_ = PsramLogger::TimingProbeEvent{};\n"
    + "  timing_probe_pending_ = false;\n"
    + "  timing_probe_loop_captured_ = false;\n"
    + "  timing_probe_log_captured_ = false;\n"
    + "  timing_probe_imu_captured_ = false;\n",
    1,
)

normal_pulse = r'''bool ExperimentRunner::beginEnergyControlAutonomousPulse(uint32_t now_ms, uint32_t t_test_ms,
                                                            int8_t direction, uint16_t pulse_width_ms) {
  if (!energy_control_autonomous_mode_ || status_.emergency_stop ||
      status_.state != ExperimentState::RUNNING_BATCH_SWEEP || status_.pulse_active ||
      !roller_ || !roller_->ok() || (direction != -1 && direction != 1) ||
      pulse_width_ms < 1 || pulse_width_ms > Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS ||
      (energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::ENERGY_CONTROL &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::HOLD) ||
      energy_control_autonomous_half_cycle_state_ !=
          EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS) {
    return false;
  }
  const uint32_t begin_t0_us = micros();
  energy_control_autonomous_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(
      direction * Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA);
  const uint32_t set_current_t0_us = micros();
  const bool set_current_ok = roller_->setCurrentMa(command_current_mA);
  const uint32_t set_current_us = static_cast<uint32_t>(micros() - set_current_t0_us);
  if (!set_current_ok) {
    energy_control_autonomous_pulse_authorized_ = false;
    stopMotor();
    return false;
  }

  const uint32_t state_t0_us = micros();
  status_.current_mA_setting = Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = t_test_ms;
  const uint32_t pulse_start_us = micros();
  const uint32_t state_update_us = static_cast<uint32_t>(pulse_start_us - state_t0_us);

  const uint32_t model_t0_us = micros();
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(
      Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA);
  const float width_s = static_cast<float>(pulse_width_ms) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + pulse_width_ms;
  const uint32_t current_model_us = static_cast<uint32_t>(micros() - model_t0_us);

  const uint32_t pulse_model_t0_us = micros();
  updatePulseModelPrediction();
  const uint32_t update_pulse_model_us = static_cast<uint32_t>(micros() - pulse_model_t0_us);
  const uint32_t total_us = static_cast<uint32_t>(micros() - begin_t0_us);
  startTimingProbe(2, t_test_ms, command_current_mA, pulse_width_ms, pulse_start_us,
                   set_current_us, state_update_us, current_model_us,
                   update_pulse_model_us, total_us);
  return true;
}
'''
replace_between(
    "src/experiment_runner.cpp",
    "bool ExperimentRunner::beginEnergyControlAutonomousPulse(uint32_t now_ms, uint32_t t_test_ms,",
    "bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse(uint32_t now_ms, int8_t direction)",
    normal_pulse,
)

# Reload after replacement before replacing the next adjacent function.
runner_cpp = read("src/experiment_runner.cpp")
start_kick = r'''bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse(uint32_t now_ms, int8_t direction) {
  if (!energy_control_autonomous_mode_ ||
      energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::STRONG_START_KICK ||
      status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP ||
      status_.pulse_active || !roller_ || !roller_->ok() ||
      direction != Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION) return false;
  const uint32_t begin_t0_us = micros();
  energy_control_autonomous_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(
      direction * Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA);
  const uint32_t set_current_t0_us = micros();
  const bool set_current_ok = roller_->setCurrentMa(command_current_mA);
  const uint32_t set_current_us = static_cast<uint32_t>(micros() - set_current_t0_us);
  if (!set_current_ok) {
    energy_control_autonomous_pulse_authorized_ = false;
    stopMotor();
    return false;
  }

  const uint32_t state_t0_us = micros();
  status_.current_mA_setting = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA;
  status_.pulse_width_ms_setting = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = 0;
  const uint32_t pulse_start_us = micros();
  const uint32_t state_update_us = static_cast<uint32_t>(pulse_start_us - state_t0_us);

  const uint32_t model_t0_us = micros();
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(
      Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA);
  const float width_s = static_cast<float>(Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  const uint32_t current_model_us = static_cast<uint32_t>(micros() - model_t0_us);

  const uint32_t pulse_model_t0_us = micros();
  updatePulseModelPrediction();
  const uint32_t update_pulse_model_us = static_cast<uint32_t>(micros() - pulse_model_t0_us);
  const uint32_t total_us = static_cast<uint32_t>(micros() - begin_t0_us);
  startTimingProbe(1, 0, command_current_mA,
                   Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS, pulse_start_us,
                   set_current_us, state_update_us, current_model_us,
                   update_pulse_model_us, total_us);
  return true;
}
'''
replace_between(
    "src/experiment_runner.cpp",
    "bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse(uint32_t now_ms, int8_t direction)",
    "void ExperimentRunner::beginEnergyControlAutonomousStartKick(uint32_t now_ms)",
    start_kick,
)

runner_cpp = read("src/experiment_runner.cpp")
helper_marker = "void ExperimentRunner::captureAngleOffsets() {"
if runner_cpp.count(helper_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: captureAngleOffsets marker changed")
helpers = r'''void ExperimentRunner::startTimingProbe(uint8_t pulse_kind, uint32_t t_test_ms,
                                              int16_t command_mA, uint16_t pulse_width_ms,
                                              uint32_t pulse_start_us, uint32_t set_current_us,
                                              uint32_t state_update_us, uint32_t current_model_us,
                                              uint32_t update_pulse_model_us,
                                              uint32_t pulse_begin_total_us) {
  if (!logger_ || !energy_control_autonomous_mode_ || timing_probe_pending_) return;
  timing_probe_event_ = PsramLogger::TimingProbeEvent{};
  timing_probe_event_.pulse_id = status_.pulse_id;
  timing_probe_event_.pulse_kind = pulse_kind;
  timing_probe_event_.t_test_ms = t_test_ms;
  timing_probe_event_.command_mA = command_mA;
  timing_probe_event_.pulse_width_ms = pulse_width_ms;
  timing_probe_event_.pulse_start_us = pulse_start_us;
  timing_probe_event_.gyro_sequence_at_start = imu_ ? imu_->reading().gyro_sequence : 0;
  timing_probe_event_.set_current_us = set_current_us;
  timing_probe_event_.state_update_us = state_update_us;
  timing_probe_event_.current_model_us = current_model_us;
  timing_probe_event_.update_pulse_model_us = update_pulse_model_us;
  timing_probe_event_.pulse_begin_total_us = pulse_begin_total_us;
  timing_probe_pending_ = true;
  timing_probe_loop_captured_ = false;
  timing_probe_log_captured_ = false;
  timing_probe_imu_captured_ = false;
}

void ExperimentRunner::recordTimingProbeLoop(uint32_t imu_update_us, uint32_t runner_update_us,
                                               uint32_t core1_path_us) {
  if (!timing_probe_pending_ || timing_probe_loop_captured_) return;
  timing_probe_event_.imu_update_call_us = imu_update_us;
  timing_probe_event_.runner_update_call_us = runner_update_us;
  timing_probe_event_.core1_path_us = core1_path_us;
  timing_probe_loop_captured_ = true;
  maybeFinalizeTimingProbe();
}

void ExperimentRunner::maybeFinalizeTimingProbe() {
  if (!timing_probe_pending_ || !timing_probe_loop_captured_ ||
      !timing_probe_log_captured_ || !timing_probe_imu_captured_ || !logger_) return;
  logger_->addTimingProbeEvent(timing_probe_event_);
  timing_probe_pending_ = false;
}

'''
runner_cpp = runner_cpp.replace(helper_marker, helpers + helper_marker, 1)
write("src/experiment_runner.cpp", runner_cpp)

# Replace logSampleIfDue as one unit so only the first high-rate row after a
# pulse start is timed. Normal 20 ms and 2 ms scheduling remain identical.
log_sample_if_due = r'''void ExperimentRunner::logSampleIfDue() {
  const uint32_t now_us = micros();
  // Preserve the normal 20 ms time series. While an already-authorized pulse
  // is live, add rows at the current-audit period so fresh-read gaps can be
  // evaluated offline. Logging rate cannot alter the motor command.
  const uint32_t period_us = status_.pulse_active
      ? Config::CURRENT_AUDIT_LOG_PERIOD_US : Config::LOG_PERIOD_MS * 1000UL;
  if (last_log_us_ != 0 && static_cast<uint32_t>(now_us - last_log_us_) < period_us) return;
  const bool probe_log = timing_probe_pending_ && !timing_probe_log_captured_ &&
      status_.pulse_active && status_.pulse_id == timing_probe_event_.pulse_id;
  const uint32_t log_start_us = probe_log ? micros() : 0;
  logSampleNow();
  if (probe_log) {
    timing_probe_event_.first_audit_log_offset_us =
        static_cast<uint32_t>(log_start_us - timing_probe_event_.pulse_start_us);
    timing_probe_event_.first_audit_log_us = static_cast<uint32_t>(micros() - log_start_us);
    timing_probe_log_captured_ = true;
    maybeFinalizeTimingProbe();
  }
}

'''
replace_between(
    "src/experiment_runner.cpp",
    "void ExperimentRunner::logSampleIfDue() {",
    "void ExperimentRunner::logSampleNow() {",
    log_sample_if_due,
)

# ---------------------------------------------------------------------------
# Core-1 call timing. Only autonomous measurement states pay this diagnostic
# overhead. No Serial output is made in the measurement path.
# ---------------------------------------------------------------------------
main = read("src/main.cpp")
old_core = r'''  runner.serviceFast();
  runner.updateImuDynamicBetaContext();
  imu.update();
  runner.update();
'''
new_core = r'''  runner.serviceFast();
  runner.updateImuDynamicBetaContext();
  const bool v46k_timing_probe_active = runner.energyControlAutonomousMode() && runner.running();
  if (v46k_timing_probe_active) {
    const uint32_t imu_t0_us = micros();
    imu.update();
    const uint32_t imu_update_us = static_cast<uint32_t>(micros() - imu_t0_us);
    const uint32_t runner_t0_us = micros();
    runner.update();
    const uint32_t runner_update_us = static_cast<uint32_t>(micros() - runner_t0_us);
    runner.recordTimingProbeLoop(imu_update_us, runner_update_us,
                                 static_cast<uint32_t>(micros() - loop_start_us));
  } else {
    imu.update();
    runner.update();
  }
'''
if main.count(old_core) != 1:
    raise RuntimeError("main.cpp: Core1 loop marker changed")
main = main.replace(old_core, new_core, 1)
write("src/main.cpp", main)

# ---------------------------------------------------------------------------
# Dedicated V46k guard. It explicitly freezes the physical settings that the
# timing diagnostic is not allowed to change.
# ---------------------------------------------------------------------------
guard = r'''from pathlib import Path

config = Path("src/config.h").read_text(encoding="utf-8")
main = Path("src/main.cpp").read_text(encoding="utf-8")
runner_h = Path("src/experiment_runner.h").read_text(encoding="utf-8")
runner = Path("src/experiment_runner.cpp").read_text(encoding="utf-8")
logger_h = Path("src/psram_logger.h").read_text(encoding="utf-8")
logger = Path("src/psram_logger.cpp").read_text(encoding="utf-8")
log_types = Path("src/log_types.h").read_text(encoding="utf-8")
manifest = Path("site/manifest.json").read_text(encoding="utf-8")

assert "v46k_pulse_start_timing_probe_20260914" in config
assert "V46k" in main
assert '"version": "0.46.10"' in manifest

# Physical/control behavior is deliberately frozen from V46j.
assert "CURRENT_AUDIT_FAST_READ_PERIOD_US = 2000UL" in config
assert "CURRENT_AUDIT_LOG_PERIOD_US = 2000UL" in config
assert "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 300" in config
assert "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 300" in config
assert "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100" in config
assert "IMU_POLL_PERIOD_US = 1000UL" in config
assert "BMI270_GYRO_ODR_HZ = 400" in config
assert "BMI270_ACCEL_ODR_HZ = 200" in config
assert "sizeof(LogSample) == 226" in log_types
assert "RWLOG_FORMAT_VERSION = 46" in logger

# Timing probes are metadata-only and cannot be consulted by control selection.
for token in (
    "TimingProbeEvent", "addTimingProbeEvent", "v46k_timing_probe_events",
    "set_current_us", "state_update_us", "current_model_us",
    "update_pulse_model_us", "pulse_begin_total_us",
    "first_audit_log_us", "imu_update_call_us", "runner_update_call_us",
    "first_imu_dt_after_start_us", "first_imu_sample_offset_us",
):
    assert token in logger_h or token in logger, token

for token in (
    "recordTimingProbeLoop", "startTimingProbe", "maybeFinalizeTimingProbe",
    "set_current_t0_us", "pulse_model_t0_us", "probe_log",
    "r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start",
):
    assert token in runner_h or token in runner, token

assert "v46k_timing_probe_active" in main
assert "runner.recordTimingProbeLoop" in main

# Never print timing inside a Run: that would perturb the quantity being measured.
probe_region = runner[runner.index("void ExperimentRunner::startTimingProbe"):runner.index("void ExperimentRunner::captureAngleOffsets")]
assert "Serial." not in probe_region
assert "printf" not in probe_region

# The logger remains observational: controller code before logSampleNow may not
# read timing values for pulse selection or stopping.
controller_region = runner[:runner.index("void ExperimentRunner::logSampleNow()")]
for forbidden in (
    "timing_probe_event_.set_current_us >",
    "timing_probe_event_.first_imu_dt_after_start_us >",
    "timing_probe_event_.runner_update_call_us >",
):
    assert forbidden not in controller_region

print("V46k timing-probe guards passed")
'''
write("tools/test_v46k_timing_probe_source_guards.py", guard)

print("Applied V46k pulse-start timing probe patch")
