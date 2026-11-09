#!/usr/bin/env python3
"""Check that pitch extraction is tested against the exact installed library."""
from pathlib import Path
import argparse
import hashlib
import json
from patch_madgwick_hot import annotated_header, HEADER

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / 'tools/fixtures/adafruit_ahrs_2_4_0'

def verify(library=None):
    manifest = json.loads((FIXTURES / 'provenance.json').read_text())
    for name, expected in manifest['sha256'].items():
        raw = (FIXTURES / name).read_bytes()
        if hashlib.sha256(raw).hexdigest() != expected:
            raise RuntimeError('Madgwick upstream fixture hash mismatch: ' + name)
        if library is not None:
            actual = (library / 'src' / name).read_bytes()
            allowed = (raw, annotated_header(raw)) if name == HEADER else (raw,)
            if actual not in allowed:
                raise RuntimeError('Installed Madgwick differs from tested 2.4.0 source: ' + name)
            if name == HEADER and actual != raw:
                if (library / 'src/RW_RuntimeHot.hpp').read_bytes() != (ROOT / 'src/realtime_code.h').read_bytes():
                    raise RuntimeError('Installed Madgwick hot-code policy differs')
    print('PASS: Madgwick 2.4.0 pitch reference provenance' +
          (' and installed dependency / allowed declaration annotations match' if library is not None else ''))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--library', type=Path)
    verify(parser.parse_args().library)
