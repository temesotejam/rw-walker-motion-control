"""Reverse V46ac autonomous light delay compensation for retained hashes."""
import re
from v46ad_timing_selection_contract import normalize_v46ad

OLD_PREDICTION = """    const uint32_t sample_age_us = r.last_gyro_update_us == 0 ? 0 : static_cast<uint32_t>(micros() - r.last_gyro_update_us);
    const uint32_t horizon_us = min<uint32_t>(Config::MEKF_CONTROL_PREDICTION_MAX_US,
        sample_age_us + Config::MEKF_CONTROL_PREDICTION_FIXED_US);
    status_.mekf_prediction_horizon_us = horizon_us;
    raw_mekf_predicted_abs_deg_ = mekf_.predictEulerDeg(mekf_gyro, static_cast<float>(horizon_us) * 1.0e-6f).pitch;
    status_.pitch_mekf_predicted_abs_deg = raw_mekf_predicted_abs_deg_;"""

OLD_CONTROL = """  // V46ab no-control-prediction begin
  // Autonomous control and video comparison use the exact same posterior,
  // measurement-start-relative coordinate. Predicted MEKF remains diagnostic only.
  if (energy_control_autonomous_mode_) {
    status_.pitch_mekf_deg = status_.pitch_mekf_measurement_relative_deg;
  }
  status_.pitch_mekf_detector_relative_deg =
      status_.pitch_mekf_measurement_relative_deg;
  // V46ab no-control-prediction end"""

def normalize_runner(text: str) -> str:
    text = normalize_v46ad(text, "src/experiment_runner.cpp")
    text = re.sub(
        r"    // V46ac autonomous diagnostic prediction begin\n.*?\n    // V46ac autonomous diagnostic prediction end",
        OLD_PREDICTION,
        text,
        flags=re.S,
    )
    text = re.sub(
        r"\n  // V46ac delay compensation begin\n.*?\n  // V46ac delay compensation end",
        "\n" + OLD_CONTROL,
        text,
        flags=re.S,
    )
    return text.replace(
        "void ExperimentRunner::updateDisplayedAngles(const ImuReading& r) {",
        "void ExperimentRunner::updateDisplayedAngles(const ImuReading&) {",
    )

def normalize_config(text: str) -> str:
    text = normalize_v46ad(text, "src/config.h")
    text = text.replace(
        "v46ac_light_delay_compensation_20260918",
        "v46ab_no_control_prediction_20260918",
    )
    text = re.sub(
        r"\n// V46ac autonomous timing compensation begin\n.*?\n// V46ac autonomous timing compensation end",
        "",
        text,
        flags=re.S,
    )
    return text
