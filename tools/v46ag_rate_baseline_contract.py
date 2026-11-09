"""Invert the exact, reviewed V46ag baseline delta for historical hash gates."""
import json
from pathlib import Path

def normalize_v46ag(text: str, path: str) -> str:
    from v46ah_full_rate_contract import normalize_v46ah
    text = normalize_v46ah(text, path)
    deltas=json.loads(Path(__file__).with_name('v46ag_rate_baseline_delta.json').read_text())
    for d in reversed(deltas):
        if d['path'] != path:continue
        if text.count(d['new'])==1:text=text.replace(d['new'],d['old'],1)
        elif text.count(d['old'])!=1:raise ValueError('V46ag baseline delta changed: '+path)
    return text
