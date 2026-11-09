"""Invert exact V46ae coordinate hunks for historical controller hash gates.

Live V46ae behavior is exercised separately by test_v46ae_mekf_amplitude.py.
Only the remainder, after these documented changes, equals the old baseline.
"""
import json
from pathlib import Path
from v46af_peak_side_contract import normalize_v46af

def normalize_v46ae(text: str, path: str) -> str:
    text = normalize_v46af(text, path)
    deltas = json.loads(Path(__file__).with_name("v46ae_mekf_amplitude_delta.json").read_text())
    for delta in reversed(deltas):
        if delta["path"] != path:
            continue
        if text.count(delta["new"]) == 1:
            text = text.replace(delta["new"], delta["old"], 1)
        elif text.count(delta["old"]) != 1:
            raise ValueError("V46ae coordinate delta changed: " + path)
    return text
