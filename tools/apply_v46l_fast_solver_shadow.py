from pathlib import Path

OLD_REV = "v46k_pulse_start_timing_probe_20260914"
NEW_REV = "v46l_fast_solver_shadow_20260914"


def read(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def write(path: str, text: str) -> None:
    Path(path).write_text(text, encoding="utf-8")


def replace_once(path: str, old: str, new: str) -> None:
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected exactly one match, got {count}: {old[:160]!r}")
    write(path, text.replace(old, new, 1))


# Identity only; physical control remains the V46k legacy exhaustive solver.
replace_once(
    "src/config.h",
    f'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "{OLD_REV}";',
    f'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "{NEW_REV}";',
)
for path in ("src/main.cpp", "site/index.html"):
    s = read(path).replace("V46k", "V46l").replace(OLD_REV, NEW_REV)
    write(path, s)
replace_once("site/manifest.json", '"version": "0.46.10"', '"version": "0.46.11"')
manifest = read("site/manifest.json").replace("V46k", "V46l").replace(OLD_REV, NEW_REV)
write("site/manifest.json", manifest)
for guard_path in (
    "tools/test_v46g_highrate_source_guards.py",
    "tools/test_v46i_task_split_source_guards.py",
    "tools/test_v46_motor_validation_source_guards.py",
    "tools/test_v46k_timing_probe_source_guards.py",
):
    s = read(guard_path)
    s = s.replace(OLD_REV, NEW_REV).replace("V46k", "V46l").replace("0.46.10", "0.46.11")
    write(guard_path, s)

# PsramLogger metadata-only solver shadow.
logger_h = read("src/psram_logger.h")
timing_marker = "  struct TimingProbeEvent {\n"
if logger_h.count(timing_marker) != 1:
    raise RuntimeError("psram_logger.h: TimingProbeEvent marker changed")
solver_struct = r'''  struct SolverShadowEvent {
    uint16_t event_index = 0;
    uint32_t pulse_id = 0;
    uint32_t t_test_ms = 0;
    int8_t physical_next_peak_side = 0;
    int8_t q_command_direction = 0;
    int16_t command_current_mA = 0;
    uint16_t model_vbat_mV = 0;
    float i0_estimated_mA = NAN;
    float free_next_peak_deg = NAN;
    float target_peak_deg = NAN;
    float target_energy_j = NAN;
    float q_available_mA_s = NAN;
    float integral_side_mA_s = NAN;
    uint16_t legacy_ff_width_ms = 0;
    float legacy_ff_q_mA_s = NAN;
    uint16_t legacy_selected_width_ms = 0;
    float legacy_selected_q_mA_s = NAN;
    uint16_t fast_ff_width_ms = 0;
    float fast_ff_q_mA_s = NAN;
    uint16_t fast_selected_width_ms = 0;
    float fast_selected_q_mA_s = NAN;
    uint32_t free_model_us = 0;
    uint32_t legacy_ff_scan_us = 0;
    uint32_t legacy_selected_scan_us = 0;
    uint32_t legacy_decision_us = 0;
    uint32_t fast_shadow_us = 0;
    uint16_t fast_eval_count = 0;
    bool fast_valid = false;
    bool ff_width_match = false;
    bool selected_width_match = false;
  };
'''
logger_h = logger_h.replace(timing_marker, solver_struct + timing_marker, 1)
method_marker = "  void addTimingProbeEvent(const TimingProbeEvent& event);\n"
if logger_h.count(method_marker) != 1:
    raise RuntimeError("psram_logger.h: addTimingProbeEvent marker changed")
logger_h = logger_h.replace(
    method_marker,
    "  void addSolverShadowEvent(const SolverShadowEvent& event);\n" + method_marker,
    1,
)
capacity_marker = (
    "  static constexpr uint16_t kMaxTimingProbeEvents = "
    "Config::ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS;\n"
)
if logger_h.count(capacity_marker) != 1:
    raise RuntimeError("psram_logger.h: timing capacity marker changed")
logger_h = logger_h.replace(
    capacity_marker,
    capacity_marker
    + "  static constexpr uint16_t kMaxSolverShadowEvents = "
      "Config::ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS;\n",
    1,
)
private_marker = (
    "  TimingProbeEvent timing_probe_events_[kMaxTimingProbeEvents] = {};\n"
    "  uint16_t timing_probe_event_count_ = 0;\n"
    "  bool timing_probe_event_overflow_ = false;\n"
)
if logger_h.count(private_marker) != 1:
    raise RuntimeError("psram_logger.h: timing private storage marker changed")
logger_h = logger_h.replace(
    private_marker,
    "  SolverShadowEvent solver_shadow_events_[kMaxSolverShadowEvents] = {};\n"
    "  uint16_t solver_shadow_event_count_ = 0;\n"
    "  bool solver_shadow_event_overflow_ = false;\n"
    + private_marker,
    1,
)
write("src/psram_logger.h", logger_h)

logger_cpp = read("src/psram_logger.cpp")
reset_marker = (
    "  timing_probe_event_count_ = 0;\n"
    "  timing_probe_event_overflow_ = false;\n"
)
if logger_cpp.count(reset_marker) != 2:
    raise RuntimeError(
        f"psram_logger.cpp: expected two timing reset sites, got {logger_cpp.count(reset_marker)}"
    )
logger_cpp = logger_cpp.replace(
    reset_marker,
    "  solver_shadow_event_count_ = 0;\n"
    "  solver_shadow_event_overflow_ = false;\n"
    + reset_marker,
)
add_timing_marker = "void PsramLogger::addTimingProbeEvent(const TimingProbeEvent& event) {\n"
if logger_cpp.count(add_timing_marker) != 1:
    raise RuntimeError("psram_logger.cpp: addTimingProbeEvent marker changed")
add_solver_method = r'''void PsramLogger::addSolverShadowEvent(const SolverShadowEvent& event) {
  if (solver_shadow_event_count_ >= kMaxSolverShadowEvents) {
    solver_shadow_event_overflow_ = true;
    return;
  }
  SolverShadowEvent stored = event;
  stored.event_index = solver_shadow_event_count_ + 1;
  solver_shadow_events_[solver_shadow_event_count_++] = stored;
}
'''
logger_cpp = logger_cpp.replace(add_timing_marker, add_solver_method + add_timing_marker, 1)
metadata_marker = (
    '  json += "\\\"v46k_timing_probe_revision\\\":'
    '\\\"v46k_pulse_start_core1_profile_20260914\\\",";\n'
)
if logger_cpp.count(metadata_marker) != 1:
    raise RuntimeError("psram_logger.cpp: V46k timing metadata marker changed")
solver_metadata = r'''  json += "\"v46l_solver_shadow_revision\":\"v46l_discrete_ternary_shadow_20260914\",";
  json += "\"v46l_solver_shadow_policy\":\"legacy_exhaustive_solver_controls_motor;fast_solver_runs_only_after_normal_pulse_end;metadata_only\",";
  json += "\"v46l_solver_shadow_event_overflow\":" + String(solver_shadow_event_overflow_ ? "true" : "false") + ",";
  json += "\"v46l_solver_shadow_events\":[";
  for (uint16_t i = 0; i < solver_shadow_event_count_; ++i) {
    const SolverShadowEvent& e = solver_shadow_events_[i];
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
'''
logger_cpp = logger_cpp.replace(metadata_marker, solver_metadata + metadata_marker, 1)
write("src/psram_logger.cpp", logger_cpp)

# ExperimentRunner declarations/state.
runner_h = read("src/experiment_runner.h")
pulse_decl = "  void updateEnergyControlAutonomousPulse(uint32_t now_ms);\n"
if runner_h.count(pulse_decl) != 1:
    raise RuntimeError("experiment_runner.h: autonomous pulse declaration marker changed")
runner_h = runner_h.replace(
    pulse_decl,
    pulse_decl + "  void runEnergyControlAutonomousSolverShadow();\n",
    1,
)
timing_state = "  PsramLogger::TimingProbeEvent timing_probe_event_{};\n"
if runner_h.count(timing_state) != 1:
    raise RuntimeError("experiment_runner.h: timing state marker changed")
runner_h = runner_h.replace(
    timing_state,
    "  PsramLogger::SolverShadowEvent solver_shadow_event_{};\n"
    "  bool solver_shadow_pending_ = false;\n"
    + timing_state,
    1,
)
write("src/experiment_runner.h", runner_h)

runner = read("src/experiment_runner.cpp")
reset_marker = (
    "void ExperimentRunner::resetEnergyControlAutonomous() {\n"
    "  timing_probe_event_ = PsramLogger::TimingProbeEvent{};\n"
)
if runner.count(reset_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: autonomous reset marker changed")
runner = runner.replace(
    reset_marker,
    "void ExperimentRunner::resetEnergyControlAutonomous() {\n"
    "  solver_shadow_event_ = PsramLogger::SolverShadowEvent{};\n"
    "  solver_shadow_pending_ = false;\n"
    "  timing_probe_event_ = PsramLogger::TimingProbeEvent{};\n",
    1,
)
event_marker = (
    "  PsramLogger::EnergyControlAutonomousZeroCrossEvent event;\n"
    "  event.zero_cross_time_ms = accepted_cross_ms;\n"
)
if runner.count(event_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: autonomous event creation marker changed")
runner = runner.replace(
    event_marker,
    "  const uint32_t v46l_decision_t0_us = micros();\n" + event_marker,
    1,
)
free_marker = (
    "  event.free_next_peak_amplitude_deg = energyControlAutonomousFreeNextPeakAmplitude(\n"
    "      energy_control_autonomous_last_peak_amplitude_deg_);\n"
)
if runner.count(free_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: autonomous free-model marker changed")
runner = runner.replace(
    free_marker,
    "  const uint32_t v46l_free_model_t0_us = micros();\n"
    + free_marker
    + "  const uint32_t v46l_free_model_us = "
      "static_cast<uint32_t>(micros() - v46l_free_model_t0_us);\n",
    1,
)
ff_marker = (
    "  uint16_t ff_width_ms = 0;\n"
    "  float ff_q_mA_s = 0.0f;\n"
)
if runner.count(ff_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: FF scan start marker changed")
runner = runner.replace(
    ff_marker,
    "  const uint32_t v46l_ff_scan_t0_us = micros();\n" + ff_marker,
    1,
)
ff_end = "  event.q_ff_energy_mA_s = ff_q_mA_s;\n"
if runner.count(ff_end) != 1:
    raise RuntimeError("experiment_runner.cpp: FF scan end marker changed")
runner = runner.replace(
    ff_end,
    "  const uint32_t v46l_ff_scan_us = "
    "static_cast<uint32_t>(micros() - v46l_ff_scan_t0_us);\n"
    + ff_end,
    1,
)
sel_marker = (
    "  uint16_t selected_width_ms = 0;\n"
    "  float selected_q_mA_s = 0.0f;\n"
)
if runner.count(sel_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: selected scan start marker changed")
runner = runner.replace(
    sel_marker,
    "  const uint32_t v46l_selected_scan_t0_us = micros();\n" + sel_marker,
    1,
)
sel_end = "  event.q_command_mA_s = selected_q_mA_s;\n"
if runner.count(sel_end) != 1:
    raise RuntimeError("experiment_runner.cpp: selected scan end marker changed")
shadow_prepare = r'''  const uint32_t v46l_selected_scan_us =
      static_cast<uint32_t>(micros() - v46l_selected_scan_t0_us);
  PsramLogger::SolverShadowEvent v46l_shadow;
  v46l_shadow.t_test_ms = t_test_ms;
  v46l_shadow.physical_next_peak_side = event.physical_next_peak_side;
  v46l_shadow.q_command_direction = event.q_command_direction;
  v46l_shadow.command_current_mA = Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA;
  v46l_shadow.model_vbat_mV = status_.beta_model_vbat_mV;
  v46l_shadow.i0_estimated_mA = event.i0_estimated_mA;
  v46l_shadow.free_next_peak_deg = event.free_next_peak_amplitude_deg;
  v46l_shadow.target_peak_deg = event.target_peak_deg;
  v46l_shadow.target_energy_j = event.target_energy_j;
  v46l_shadow.q_available_mA_s = event.q_available_mA_s;
  v46l_shadow.integral_side_mA_s = event.integral_side_mA_s;
  v46l_shadow.legacy_ff_width_ms = ff_width_ms;
  v46l_shadow.legacy_ff_q_mA_s = ff_q_mA_s;
  v46l_shadow.legacy_selected_width_ms = selected_width_ms;
  v46l_shadow.legacy_selected_q_mA_s = selected_q_mA_s;
  v46l_shadow.free_model_us = v46l_free_model_us;
  v46l_shadow.legacy_ff_scan_us = v46l_ff_scan_us;
  v46l_shadow.legacy_selected_scan_us = v46l_selected_scan_us;
  v46l_shadow.legacy_decision_us =
      static_cast<uint32_t>(micros() - v46l_decision_t0_us);
'''
runner = runner.replace(sel_end, shadow_prepare + sel_end, 1)
begin_success_marker = (
    "  event.output_executed = true;\n"
    "  event.valid = true;\n"
    "  event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE;\n"
)
if runner.count(begin_success_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: normal pulse success marker changed")
runner = runner.replace(
    begin_success_marker,
    "  v46l_shadow.pulse_id = status_.pulse_id;\n"
    "  solver_shadow_event_ = v46l_shadow;\n"
    "  solver_shadow_pending_ = true;\n"
    + begin_success_marker,
    1,
)
pulse_end_marker = (
    "  } else if (completed_normal_pulse) {\n"
    "    // Do not carry a detector extremum formed by the 100 ms pulse transient\n"
    "    // into the next physical half-cycle.\n"
    "    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;\n"
    "    resetEnergyControlAutonomousPeakTracker(true);\n"
    "  }\n"
)
if runner.count(pulse_end_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: normal pulse completion marker changed")
runner = runner.replace(
    pulse_end_marker,
    "  } else if (completed_normal_pulse) {\n"
    "    // Do not carry a detector extremum formed by the pulse transient\n"
    "    // into the next physical half-cycle.\n"
    "    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;\n"
    "    resetEnergyControlAutonomousPeakTracker(true);\n"
    "    // V46l shadow runs only after stopActivePulse(); it cannot delay or\n"
    "    // alter the legacy physical command selected at the zero crossing.\n"
    "    runEnergyControlAutonomousSolverShadow();\n"
    "  }\n",
    1,
)

shadow_method_marker = "void ExperimentRunner::startTimingProbe("
if runner.count(shadow_method_marker) != 1:
    raise RuntimeError("experiment_runner.cpp: startTimingProbe marker changed")
shadow_method = r'''void ExperimentRunner::runEnergyControlAutonomousSolverShadow() {
  if (!solver_shadow_pending_ || !logger_) return;

  PsramLogger::SolverShadowEvent result = solver_shadow_event_;
  const uint32_t shadow_t0_us = micros();
  uint16_t eval_count = 0;

  const float v = result.model_vbat_mV > 0
      ? static_cast<float>(result.model_vbat_mV) / 1000.0f
      : Config::MODEL_VBAT_REFERENCE_V;
  const float signed_target_current_mA =
      static_cast<float>(result.q_command_direction) *
      predictCurrentGoalMa(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA);

  struct FastCandidate {
    bool valid = false;
    uint16_t width_ms = 0;
    float q_mA_s = NAN;
    float energy_j = NAN;
    float error_j = NAN;
  };

  auto evaluate_width = [&](uint16_t width_ms, float target_energy_j) -> FastCandidate {
    FastCandidate c;
    c.width_ms = width_ms;
    ++eval_count;
    float q_mA_s = 0.0f;
    if (width_ms > 0) {
      const float t_s = static_cast<float>(width_ms) / 1000.0f;
      const float signed_q_mA_s =
          signed_target_current_mA * t_s +
          (result.i0_estimated_mA - signed_target_current_mA) * tau_s *
              (1.0f - expf(-t_s / tau_s));
      q_mA_s = fabsf(signed_q_mA_s);
    }
    const float predicted_peak_deg = energyControlAutonomousCorrectedPrediction(
        result.free_next_peak_deg, result.physical_next_peak_side, q_mA_s, nullptr);
    const float energy_j = energyControlPotentialJ(predicted_peak_deg);
    if (!isfinite(q_mA_s) || !isfinite(energy_j) || !isfinite(target_energy_j)) return c;
    c.valid = true;
    c.q_mA_s = q_mA_s;
    c.energy_j = energy_j;
    c.error_j = fabsf(target_energy_j - energy_j);
    return c;
  };

  auto fast_pick_width = [&](float target_energy_j) -> FastCandidate {
    uint16_t lo = Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS;
    uint16_t hi = Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS;
    while (hi > lo && static_cast<uint16_t>(hi - lo) > 8U) {
      const uint16_t third = static_cast<uint16_t>((hi - lo) / 3U);
      if (third == 0) break;
      const uint16_t m1 = static_cast<uint16_t>(lo + third);
      const uint16_t m2 = static_cast<uint16_t>(hi - third);
      const FastCandidate c1 = evaluate_width(m1, target_energy_j);
      const FastCandidate c2 = evaluate_width(m2, target_energy_j);
      if (!c1.valid || !c2.valid) return FastCandidate{};
      if (c1.error_j <= c2.error_j) hi = m2;
      else lo = m1;
    }
    FastCandidate best;
    for (uint16_t width_ms = lo; width_ms <= hi; ++width_ms) {
      const FastCandidate c = evaluate_width(width_ms, target_energy_j);
      if (!c.valid) return FastCandidate{};
      if (!best.valid || c.error_j < best.error_j) best = c;
    }
    return best;
  };

  const FastCandidate ff = fast_pick_width(result.target_energy_j);
  if (ff.valid) {
    result.fast_ff_width_ms = ff.width_ms;
    result.fast_ff_q_mA_s = ff.q_mA_s;
    const float q_unclamped_mA_s = ff.q_mA_s + result.integral_side_mA_s;
    const float corrected_q_target_mA_s =
        fmaxf(0.0f, fminf(result.q_available_mA_s, q_unclamped_mA_s));
    const float corrected_target_prediction_deg = energyControlAutonomousCorrectedPrediction(
        result.free_next_peak_deg, result.physical_next_peak_side,
        corrected_q_target_mA_s, nullptr);
    const float corrected_target_energy_j =
        energyControlPotentialJ(corrected_target_prediction_deg);
    const FastCandidate selected = fast_pick_width(corrected_target_energy_j);
    if (selected.valid) {
      result.fast_selected_width_ms = selected.width_ms;
      result.fast_selected_q_mA_s = selected.q_mA_s;
      result.fast_valid = true;
      result.ff_width_match = result.fast_ff_width_ms == result.legacy_ff_width_ms;
      result.selected_width_match =
          result.fast_selected_width_ms == result.legacy_selected_width_ms;
    }
  }

  result.fast_eval_count = eval_count;
  result.fast_shadow_us = static_cast<uint32_t>(micros() - shadow_t0_us);
  logger_->addSolverShadowEvent(result);
  solver_shadow_pending_ = false;
  solver_shadow_event_ = PsramLogger::SolverShadowEvent{};
}

'''
runner = runner.replace(shadow_method_marker, shadow_method + shadow_method_marker, 1)
write("src/experiment_runner.cpp", runner)

print("Applied V46l fast-solver shadow patch")
