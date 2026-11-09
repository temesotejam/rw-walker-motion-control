"""Reverse the declared V46al-R2 control delta before retained V46aj/V46ak hashes."""
import re

def normalize_runner(text: str) -> str:
    text = text.replace('#include "previous_peak_control_correction.h"\n', '')
    return re.sub(
        r"\n  // V46al-R2 previous-peak active control begin\n.*?\n  // V46al-R2 previous-peak active control end",
        "\n  event.free_next_peak_amplitude_deg = baseline.adjusted_deg;",
        text,
        flags=re.S,
    )

def normalize_config(text: str) -> str:
    return re.sub(
        r"\n// V46al-R2 previous-peak active control begin\n.*?\n// V46al-R2 previous-peak active control end",
        "",
        text,
        flags=re.S,
    )

def normalize_file(path: str, text: str) -> str:
    if path == "src/config.h":
        return normalize_config(text)
    return text
