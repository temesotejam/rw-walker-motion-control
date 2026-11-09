"""Invert the explicit V46aj delta for retained historical hash guards.

The production projection, HTTP start and UI are tested directly by the V46aj
host test. These reviewed replacements preserve earlier frozen checksums.
"""
import json
from pathlib import Path


def normalize_v46aj(text: str, path: str) -> str:
    changes = json.loads(Path(__file__).with_name('v46aj_fixed_timing_delta.json').read_text())
    for delta in reversed(changes):
        if delta['path'] != path:
            continue
        if text.count(delta['new']) == 1:
            text = text.replace(delta['new'], delta['old'], 1)
        elif text.count(delta['old']) != 1:
            raise ValueError('V46aj fixed-timing delta changed: ' + path)
    return text
