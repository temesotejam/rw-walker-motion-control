"""Reverse V46ak observation-only additions before retained V46aj/V46u hashes."""
import re

def normalize_runner(text: str) -> str:
    return re.sub(
        r"\n  // V46ak observation-only pre-input snapshot\..*?\n  }\n\n(?=  event\.q_available_mA_s)",
        "",
        text,
        flags=re.S,
    )

def normalize_config(text: str) -> str:
    return re.sub(
        r'\n// V46ak changes observation only\. ATTITUDE_VALIDATION_REVISION intentionally remains V46aj\.\n'
        r'static constexpr char AMPLITUDE_CONTROL_OBSERVATION_REVISION\[\] = "v46ak_pre_input_state_observation_20260920";',
        "",
        text,
    )

def normalize_roller_h(text: str) -> str:
    text = re.sub(
        r"\n  // V46ak observation-only wheel-state telemetry\. Never read by control logic\.\n"
        r"  float speed_rpm = NAN;\n"
        r"  uint32_t speed_sample_time_us = 0;\n"
        r"  uint32_t speed_sequence = 0;\n"
        r"  uint32_t speed_read_failure_count = 0;\n"
        r"  bool speed_valid = false;\n",
        "",
        text,
    )
    text = text.replace("  bool readSpeedFresh();\n", "")
    text = text.replace("  void recordSpeedReadFailure();\n", "")
    return text

def normalize_roller_cpp(text: str) -> str:
    text = text.replace(
        "// M5Stack Unit Roller485 I2C protocol: Speed Readback X100 Int at 0x60.\n"
        "static constexpr uint8_t REG_SPEED_READBACK = 0x60;\n",
        "",
    )
    text = re.sub(
        r"\n  // V46ak: speed is observation-only\. Read it only while no current command is\n"
        r"  // active or pending, so this extra I2C transaction never joins the active\n"
        r"  // pulse current-audit path\. Failure does not change roller_ok or authorize/\n"
        r"  // block control; the sample is simply marked invalid\.\n"
        r"  if \(command_mA_ == 0 && requested_current_mA_ == 0\) \{\n"
        r"    readSpeedFresh\(\);\n"
        r"  \}\n",
        "",
        text,
    )
    text = text.replace(
        "  bool ok = current_already_fresh || readCurrentFresh(command_mA_ != 0);\n\n"
        "  ok &= readI32(REG_VIN, vin_raw);",
        "  bool ok = current_already_fresh || readCurrentFresh(command_mA_ != 0);\n"
        "  ok &= readI32(REG_VIN, vin_raw);",
    )
    text = re.sub(
        r"\nbool Roller485Manager::readSpeedFresh\(\) \{.*?\n"
        r"void Roller485Manager::recordSpeedReadFailure\(\) \{.*?\n\}\n",
        "",
        text,
        flags=re.S,
    )
    return text

def normalize_file(path: str, text: str) -> str:
    if path == "src/config.h":
        return normalize_config(text)
    if path == "src/roller485_manager.h":
        return normalize_roller_h(text)
    if path == "src/roller485_manager.cpp":
        return normalize_roller_cpp(text)
    return text
