#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def write(path: str, text: str) -> None:
    (ROOT / path).write_text(text, encoding="utf-8")


def replace_once(path: str, old: str, new: str) -> None:
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected exactly one match, got {count}: {old[:120]!r}")
    write(path, text.replace(old, new, 1))


# V46r changes only the autonomous zero-cross pulse-width selector and release identity.
# Physical limits and runtime safety/ESTOP thresholds remain unchanged.
replace_once(
    "src/config.h",
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46l_fast_solver_shadow_20260914";',
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46r_fast_solver_control_20260915";',
)

runner = read("src/experiment_runner.cpp")

old_ff = r'''  const uint32_t v46l_ff_scan_t0_us = micros();
  uint16_t ff_width_ms = 0;
  float ff_q_mA_s = 0.0f;
  float ff_energy_j = energyControlPotentialJ(zero_q_corrected_prediction_deg);
  float ff_error_j = fabsf(event.target_energy_j - ff_energy_j);
  for (uint16_t width_ms = Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS;
       width_ms <= Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS; ++width_ms) {
    const float q_mA_s = width_ms == 0 ? 0.0f : fabsf(predictedChargeMaS(event.i0_estimated_mA,
        event.q_command_direction, static_cast<float>(width_ms), Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA));
    const float predicted_peak_deg = energyControlAutonomousCorrectedPrediction(
        event.free_next_peak_amplitude_deg, event.physical_next_peak_side, q_mA_s, nullptr);
    const float energy_j = energyControlPotentialJ(predicted_peak_deg);
    if (!isfinite(q_mA_s) || !isfinite(energy_j)) break;
    const float error_j = fabsf(event.target_energy_j - energy_j);
    if (error_j < ff_error_j) {
      ff_width_ms = width_ms;
      ff_q_mA_s = q_mA_s;
      ff_energy_j = energy_j;
      ff_error_j = error_j;
    }
  }
  const uint32_t v46l_ff_scan_us = static_cast<uint32_t>(micros() - v46l_ff_scan_t0_us);
'''

new_ff = r'''  // V46r: the V46l shadow selector matched the legacy 0..100 ms exhaustive
  // selector on every completed hardware comparison. Promote that bounded fast
  // selector to the physical path so a control decision no longer occupies
  // Core1 for ~7 ms. Safety limits are unchanged; invalid solver states coast.
  struct FastCandidate {
    bool valid = false;
    uint16_t width_ms = 0;
    float q_mA_s = NAN;
    float energy_j = NAN;
    float error_j = NAN;
  };
  uint16_t fast_eval_count = 0;
  const float fast_v = status_.beta_model_vbat_mV > 0
      ? static_cast<float>(status_.beta_model_vbat_mV) / 1000.0f
      : Config::MODEL_VBAT_REFERENCE_V;
  const float fast_signed_target_current_mA =
      static_cast<float>(event.q_command_direction) *
      predictCurrentGoalMa(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA, fast_v);
  const float fast_tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA);

  auto evaluate_width = [&](uint16_t width_ms, float target_energy_j) -> FastCandidate {
    FastCandidate c;
    c.width_ms = width_ms;
    ++fast_eval_count;
    float q_mA_s = 0.0f;
    if (width_ms > 0) {
      const float t_s = static_cast<float>(width_ms) / 1000.0f;
      const float signed_q_mA_s =
          fast_signed_target_current_mA * t_s +
          (event.i0_estimated_mA - fast_signed_target_current_mA) * fast_tau_s *
              (1.0f - expf(-t_s / fast_tau_s));
      q_mA_s = fabsf(signed_q_mA_s);
    }
    const float predicted_peak_deg = energyControlAutonomousCorrectedPrediction(
        event.free_next_peak_amplitude_deg, event.physical_next_peak_side, q_mA_s, nullptr);
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

  const uint32_t v46r_fast_solver_t0_us = micros();
  const FastCandidate ff = fast_pick_width(event.target_energy_j);
  if (!ff.valid) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  const uint16_t ff_width_ms = ff.width_ms;
  const float ff_q_mA_s = ff.q_mA_s;
  const float ff_energy_j = ff.energy_j;
'''

if runner.count(old_ff) != 1:
    raise RuntimeError("experiment_runner.cpp: legacy feed-forward exhaustive scan marker changed")
runner = runner.replace(old_ff, new_ff, 1)

old_selected = r'''  const uint32_t v46l_selected_scan_t0_us = micros();
  uint16_t selected_width_ms = 0;
  float selected_q_mA_s = 0.0f;
  float selected_energy_j = event.passive_energy_j;
  float selected_error_j = fabsf(corrected_target_energy_j - selected_energy_j);
  for (uint16_t width_ms = Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS;
       width_ms <= Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS; ++width_ms) {
    const float q_mA_s = width_ms == 0 ? 0.0f : fabsf(predictedChargeMaS(event.i0_estimated_mA,
        event.q_command_direction, static_cast<float>(width_ms), Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA));
    const float predicted_peak_deg = energyControlAutonomousCorrectedPrediction(
        event.free_next_peak_amplitude_deg, event.physical_next_peak_side, q_mA_s, nullptr);
    const float energy_j = energyControlPotentialJ(predicted_peak_deg);
    if (!isfinite(q_mA_s) || !isfinite(energy_j)) break;
    const float error_j = fabsf(corrected_target_energy_j - energy_j);
    if (error_j < selected_error_j) {
      selected_width_ms = width_ms;
      selected_q_mA_s = q_mA_s;
      selected_energy_j = energy_j;
      selected_error_j = error_j;
    }
  }
  const uint32_t v46l_selected_scan_us =
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

new_selected = r'''  const FastCandidate selected = fast_pick_width(corrected_target_energy_j);
  if (!selected.valid) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  const uint16_t selected_width_ms = selected.width_ms;
  const float selected_q_mA_s = selected.q_mA_s;
  const float selected_energy_j = selected.energy_j;
  const uint32_t v46r_fast_solver_us =
      static_cast<uint32_t>(micros() - v46r_fast_solver_t0_us);
  (void)v46r_fast_solver_us;
  (void)fast_eval_count;
'''

if runner.count(old_selected) != 1:
    raise RuntimeError("experiment_runner.cpp: legacy selected-width exhaustive scan marker changed")
runner = runner.replace(old_selected, new_selected, 1)

old_success = r'''  v46l_shadow.pulse_id = status_.pulse_id;
  solver_shadow_event_ = v46l_shadow;
  solver_shadow_pending_ = true;
  event.output_executed = true;
'''
new_success = r'''  // V46r performs no deferred solver shadow after the pulse. The old deferred
  // computation consumed Core1 time without affecting the already-issued command.
  event.output_executed = true;
'''
if runner.count(old_success) != 1:
    raise RuntimeError("experiment_runner.cpp: V46l shadow scheduling marker changed")
runner = runner.replace(old_success, new_success, 1)

old_deferred = r'''    // V46l shadow runs only after stopActivePulse(); it cannot delay or
    // alter the legacy physical command selected at the zero crossing.
    runEnergyControlAutonomousSolverShadow();
'''
new_deferred = r'''    // V46r has no post-pulse solver workload on Core1. All controller decisions
    // were completed by the bounded fast selector at the accepted zero crossing.
'''
if runner.count(old_deferred) != 1:
    raise RuntimeError("experiment_runner.cpp: deferred shadow call marker changed")
runner = runner.replace(old_deferred, new_deferred, 1)

# Keep the historical V46l shadow function compiled for offline/source comparison,
# but it is no longer scheduled by the V46r physical controller.
write("src/experiment_runner.cpp", runner)

manifest = read("site/manifest.json")
manifest = manifest.replace('"name": "AtomS3R V46q MEKF Motor Validation"',
                            '"name": "AtomS3R V46r Fast Solver Motor Validation"')
manifest = manifest.replace('"version": "0.46.16"', '"version": "0.46.17"')
write("site/manifest.json", manifest)

site = read("site/index.html")
site = site.replace("V46q / 0.46.16：継承する起動・モータ駆動機能",
                    "V46r / 0.46.17：高速ソルバ実制御 + V46q取得系")
needle = "Run中の10 ms超の受け渡し遅延・キューあふれによる停止は維持しています。"
if needle not in site:
    raise RuntimeError("site/index.html: V46q safety paragraph marker changed")
site = site.replace(
    needle,
    needle + "\n      ゼロクロス直後の0～100 ms全探索×2は、V46L実機比較で一致確認した高速探索へ置換しました。"
             "300 mA・最大100 ms・非常停止条件は変更していません。",
    1,
)
write("site/index.html", site)

print("Applied V46r fast-solver control patch; safety limits unchanged")
