"""Normalize V46z observational comparison-zero additions for retained controller hashes."""
import re

def normalize_runner(text: str) -> str:
    return re.sub(
        r"\n\s*// V46z comparison-zero begin\n.*?\n\s*// V46z comparison-zero end",
        "",
        text,
        flags=re.S,
    )

def normalize_log_types(text: str) -> str:
    text = re.sub(
        r"\n  // RWLOG v47: comparison-only event-relative MEKF angle references\.\n.*?\n  // RWLOG v47 end",
        "",
        text,
        flags=re.S,
    )
    return text.replace(
        'static_assert(sizeof(LogSample) == 250, "LogSample binary size changed");',
        'static_assert(sizeof(LogSample) == 226, "LogSample binary size changed");',
    )
