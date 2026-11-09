"""Reverse only reviewed V46ad selection hunks for retained, unchanged hash gates."""
import json
from v46ae_mekf_amplitude_contract import normalize_v46ae
from pathlib import Path

def normalize_v46ad(text: str, path: str) -> str:
    text = normalize_v46ae(text, path)
    deltas = json.loads(Path(__file__).with_name("v46ad_timing_selection_delta.json").read_text())
    for delta in reversed(deltas):
        if delta["path"] != path:
            continue
        if delta["new"] in text:
            assert text.count(delta["new"]) == 1
            text = text.replace(delta["new"], delta["old"])
    return text
