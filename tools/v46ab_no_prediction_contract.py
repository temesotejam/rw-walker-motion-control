"""Reverse V46ab autonomous no-prediction changes for retained legacy hashes."""
import re

OLD_DETECTOR_UPDATE = """  // V46aa control-zero update begin
  status_.pitch_mekf_detector_relative_deg =
      isfinite(status_.mekf_detector_zero_predicted_abs_deg)
          ? raw_mekf_predicted_abs_deg_ - status_.mekf_detector_zero_predicted_abs_deg
          : NAN;
  // V46aa control-zero update end"""

def normalize_runner(text: str) -> str:
    return re.sub(
        r"\n\s*// V46ab no-control-prediction begin\n.*?\n\s*// V46ab no-control-prediction end",
        "\n" + OLD_DETECTOR_UPDATE,
        text,
        flags=re.S,
    )

def normalize_config(text: str) -> str:
    text = text.replace(
        "v46ab_no_control_prediction_20260918",
        "v46aa_control_upright_zero_20260918",
    )
    text = text.replace(
        "// V46ab: retained for diagnostic predicted-angle logging and legacy non-Autonomous\n"
        "// modes only. Autonomous peak/zero-cross timing uses posterior measurement-relative MEKF.\n",
        "",
    )
    return text
