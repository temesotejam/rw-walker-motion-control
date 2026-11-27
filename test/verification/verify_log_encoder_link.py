#!/usr/bin/env python3
"""Guard the linked encoder's code footprint after the 0.47.13 regression.

This checks emitted code, not hardware execution time or deadline compliance.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess


def audit(nm_output):
    symbols = {}
    for line in nm_output.splitlines():
        match = re.match(r"^[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+[tTwW]\s+(.+)$", line)
        if not match:
            continue
        size, name = int(match[1], 16), match[2]
        if (name.startswith("encodeLogSample(") or
                name.startswith("(anonymous namespace)::quantize(") or
                name.startswith("log_quantization::scaledI16(")):
            symbols[name] = size
    encoders = [s for s in symbols if s.startswith("encodeLogSample(")]
    shared = [s for s in symbols if s.startswith("log_quantization::scaledI16(")]
    if len(encoders) != 1 or len(shared) != 1 or len(symbols) != 2:
        raise ValueError("Expected one encoder and one canonical out-of-line quantizer")
    if any(size <= 0 for size in symbols.values()) or sum(symbols.values()) > 4096:
        raise ValueError("Log encoder code footprint exceeds 4096 bytes: " + str(symbols))
    return {"revision": "compact_log_encoder_04716", "symbols_bytes": symbols,
            "total_code_bytes": sum(symbols.values()), "limit_bytes": 4096,
            "hardware_timing_verified": False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--nm", required=True)
    args = parser.parse_args()
    output = subprocess.check_output([args.nm, "-C", "-S", str(args.elf)], text=True)
    print(json.dumps(audit(output), indent=2))
