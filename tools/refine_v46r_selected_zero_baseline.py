#!/usr/bin/env python3
from pathlib import Path

P = Path(__file__).resolve().parents[1] / "src" / "experiment_runner.cpp"
s = P.read_text(encoding="utf-8")
old = r'''  const FastCandidate selected = fast_pick_width(corrected_target_energy_j);
  if (!selected.valid) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  const uint16_t selected_width_ms = selected.width_ms;
  const float selected_q_mA_s = selected.q_mA_s;
  const float selected_energy_j = selected.energy_j;
'''
new = r'''  const FastCandidate selected_fast = fast_pick_width(corrected_target_energy_j);
  if (!selected_fast.valid) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  // Preserve the legacy selector's special zero-output baseline exactly. The
  // exhaustive implementation started from passive_energy_j and only replaced
  // it on a strict error improvement. This matters at the no-output boundary.
  uint16_t selected_width_ms = 0;
  float selected_q_mA_s = 0.0f;
  float selected_energy_j = event.passive_energy_j;
  const float zero_output_error_j = fabsf(corrected_target_energy_j - selected_energy_j);
  if (selected_fast.error_j < zero_output_error_j) {
    selected_width_ms = selected_fast.width_ms;
    selected_q_mA_s = selected_fast.q_mA_s;
    selected_energy_j = selected_fast.energy_j;
  }
'''
if s.count(old) != 1:
    raise RuntimeError(f"expected one V46r selected block, found {s.count(old)}")
P.write_text(s.replace(old, new, 1), encoding="utf-8")
print("Preserved V46 legacy zero-output baseline semantics")
