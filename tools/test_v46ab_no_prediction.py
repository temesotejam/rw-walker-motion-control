#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf-8")
config = (ROOT / "src/config.h").read_text(encoding="utf-8")

assert "v46aj_fixed_3ms_compensation_20260920" in config

def method(name: str) -> str:
    start = runner.index(name)
    brace = runner.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (runner[end] == "{") - (runner[end] == "}")
        end += 1
    return runner[start:end]

motion = method("void ExperimentRunner::updateEnergyControlAutonomousMotion")
assert "const float detector_relative_angle_deg = status_.pitch_mekf_detector_relative_deg;" in motion
for forbidden in ("pitch_mekf_predicted_abs_deg", "raw_mekf_predicted_abs_deg_",
                  "mekf_prediction_horizon_us", "predictEulerDeg"):
    assert forbidden not in motion, forbidden

filter_series = method("void ExperimentRunner::updateFilterSeries")
assert "if (energy_control_autonomous_mode_)" in filter_series
assert "raw_mekf_predicted_abs_deg_ = NAN;" in filter_series
assert "mekf_.predictEulerDeg" in filter_series

print("V46aj PASS: heavy quaternion forward prediction is not used by Autonomous timing")
