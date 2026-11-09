"""Invert only the explicit V46ai delta when running historical hash guards.

The new production baseline and integration are tested directly, not through
this normalization. Existing frozen checksums remain unchanged.
"""
import json
from pathlib import Path
from v46aj_fixed_timing_contract import normalize_v46aj

def normalize_v46ai(text: str, path: str) -> str:
    text = normalize_v46aj(text, path)
    changes=json.loads(Path(__file__).with_name('v46ai_rate_only_delta.json').read_text())
    for d in reversed(changes):
        if d['path'] != path:
            continue
        if text.count(d['new']) == 1:
            text=text.replace(d['new'],d['old'],1)
        elif text.count(d['old']) != 1:
            raise ValueError('V46ai rate-only delta changed: '+path)
    return text
