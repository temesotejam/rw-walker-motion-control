#!/usr/bin/env python3
from pathlib import Path
import json, subprocess, tempfile

R = Path(__file__).resolve().parents[1]
config = (R / "src/config.h").read_text(encoding="utf-8")
runner = (R / "src/experiment_runner.cpp").read_text(encoding="utf-8")
logger = (R / "src/psram_logger.cpp").read_text(encoding="utf-8")
logger_h = (R / "src/psram_logger.h").read_text(encoding="utf-8")
converter = (R / "tools/convert_rwlog_to_csv.py").read_text(encoding="utf-8")
manifest = json.loads((R / "site/manifest.json").read_text(encoding="utf-8"))
site = (R / "site/index.html").read_text(encoding="utf-8")

assert 'ATTITUDE_VALIDATION_REVISION[] = "v46aj_fixed_3ms_compensation_20260920"' in config
assert 'AMPLITUDE_CONTROL_OBSERVATION_REVISION[] = "v46ak_pre_input_state_observation_20260920"' in config
assert 'AMPLITUDE_CONTROL_REVISION[] = "v46alr2_previous_peak_active_control_stable_rwlog_20260921"' in config
assert 'atoms3r-amplitude-control-v46ak-stable@bb9c5ed07c5ca8b3c6c6b5813b6c2f1b1f57a6ec' in config
assert manifest["version"] == "0.46.42"
assert "V46al-R2" in manifest["name"]
assert "V46al-R2 / 0.46.42" in site

for token in (
    "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 1000",
    "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100",
    "ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US = 3000UL",
    "ENERGY_CONTROL_AUTONOMOUS_INTEGRAL_KI_MAS_PER_DEG = 0.10f",
):
    assert token in config, token

start = runner.index("// V46al-R2 previous-peak active control begin")
end = runner.index("// V46al-R2 previous-peak active control end", start)
block = runner[start:end]
assert "previous_peak_control::evaluate(" in block
assert "baseline.adjusted_deg" in block
assert "event.previous_peak_amplitude_deg" in block
assert "event.physical_next_peak_side" in block
assert "event.target_peak_deg" in block
assert "t_test_ms" in block
assert "event.free_next_peak_amplitude_deg = previous_peak_result.corrected_free_peak_deg;" in block
assert "pre_input_" not in block
assert runner.index("const auto baseline = rate_baseline::evaluate(") < start
assert end < runner.index("event.passive_energy_j = energyControlPotentialJ", end)

# R2 deliberately adds no logger/event fields. The correction is reconstructible
# offline from stable fields already logged at each zero-cross.
for token in (
    "free_next_peak_before_previous_peak_correction_deg",
    "previous_peak_control_raw_correction_deg",
    "previous_peak_control_correction_deg",
    "previous_peak_control_reason",
    "previous_peak_control_applied",
    "previous_peak_control_clamped",
    "previous_peak_control_model_revision",
):
    assert token not in logger_h
    assert token not in logger
    assert token not in converter

for token in (
    "zero_cross_time_ms",
    "previous_peak_amplitude_deg",
    "physical_next_peak_side",
    "target_peak_deg",
    "rate_baseline_peak_deg",
    "free_next_peak_amplitude_deg",
):
    assert token in logger and token in converter

assert "RWLOG_FORMAT_VERSION = 51" in logger
assert "sizeof(LogSample) == 258" in (R / "src/log_types.h").read_text(encoding="utf-8")

code = r'''
#include <cassert>
#include <cmath>
#include "previous_peak_control_correction.h"
int main() {
  using namespace previous_peak_control;
  auto early = evaluate(7.5f, 8.0f, 1, 8.0f, 9999);
  assert(!early.applied && early.reason == PREV_REASON_BEFORE_ENABLE_TIME && early.corrected_free_peak_deg == 7.5f);
  auto other = evaluate(7.5f, 8.0f, 1, 10.0f, 15000);
  assert(!other.applied && other.reason == PREV_REASON_TARGET_UNSUPPORTED);
  auto outside = evaluate(7.5f, 6.0f, 1, 8.0f, 15000);
  assert(!outside.applied && outside.reason == PREV_REASON_OUTSIDE_SUPPORT);
  auto plus = evaluate(7.5f, 8.0f, 1, 8.0f, 15000);
  assert(plus.applied && std::fabs(plus.applied_correction_deg - 0.591392151f) < 1e-5f);
  auto minus = evaluate(8.5f, 8.0f, -1, 8.0f, 15000);
  assert(minus.applied && std::fabs(minus.applied_correction_deg + 0.157912422f) < 1e-5f);
  auto cap = evaluate(7.0f, 7.19424f, 1, 8.0f, 15000);
  assert(cap.applied && cap.clamped && std::fabs(cap.applied_correction_deg - 0.70f) < 1e-6f);
}
'''
with tempfile.TemporaryDirectory() as d:
    p = Path(d)
    (p / "t.cpp").write_text(code)
    exe = p / "t"
    subprocess.run([
        "g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I" + str(R / "tools/host_v46o"), "-I" + str(R / "src"),
        str(p / "t.cpp"), "-o", str(exe)
    ], check=True)
    subprocess.run([str(exe)], check=True)

print("V46al-R2 active A_prev control with stable RWLOG guards PASS")
