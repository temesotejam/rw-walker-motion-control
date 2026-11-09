"""Reverse V46aa's algebraically-equivalent explicit control-zero refactor."""
import re

OLD_RESET = """  energy_control_autonomous_detector_zero_angle_deg_ =
      status_.pitch_mekf_deg;
  if (!isfinite(energy_control_autonomous_detector_zero_angle_deg_)) {
    energy_control_autonomous_detector_zero_angle_deg_ = 0.0f;
  }"""

OLD_DETECTOR = """  const float detector_relative_angle_deg = status_.pitch_mekf_deg -
      energy_control_autonomous_detector_zero_angle_deg_;"""

def normalize_runner(text: str) -> str:
    text = re.sub(
        r"\n  // V46aa control-zero capture begin\n.*?\n  // V46aa control-zero capture end",
        "",
        text,
        flags=re.S,
    )
    text = re.sub(
        r"\n  // V46aa control-zero update begin\n.*?\n  // V46aa control-zero update end",
        "",
        text,
        flags=re.S,
    )
    text = re.sub(
        r"  // V46aa control-zero reset begin\n.*?\n  // V46aa control-zero reset end",
        OLD_RESET,
        text,
        flags=re.S,
    )
    text = re.sub(
        r"  // V46aa control-zero detector begin\n.*?\n  // V46aa control-zero detector end",
        OLD_DETECTOR,
        text,
        flags=re.S,
    )
    text = re.sub(
        r"\n  // V46aa control-zero log begin\n.*?\n  // V46aa control-zero log end",
        "",
        text,
        flags=re.S,
    )
    return text

def normalize_log_types(text: str) -> str:
    text = re.sub(
        r"\n  // RWLOG v48: explicit predicted-MEKF control-zero coordinate\.\n.*?\n  // RWLOG v48 end",
        "",
        text,
        flags=re.S,
    )
    return text.replace(
        'static_assert(sizeof(LogSample) == 258, "LogSample binary size changed");',
        'static_assert(sizeof(LogSample) == 250, "LogSample binary size changed");',
    )
