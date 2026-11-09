#!/usr/bin/env python3
"""Annotate exact AHRS 2.4.0 declarations; leave all upstream arithmetic intact.

Only both updateIMU overloads and their invSqrt helper get O2/no-fast-math/IRAM.
An unexpected source edit is an error, not permission to overwrite a library.
"""
from pathlib import Path
import hashlib
import json

try:
    Import('env')
except NameError:
    ROOT = Path(__file__).resolve().parents[1]
else:
    ROOT = Path(env.subst('$PROJECT_DIR'))
FIXTURES = ROOT / 'tools/fixtures/adafruit_ahrs_2_4_0'
HEADER = 'Adafruit_AHRS_Madgwick.h'


def annotated_header(original):
    text = original.decode()
    if text.count('  void updateIMU(') != 2 or text.count('  static float invSqrt(') != 1:
        raise RuntimeError('Unexpected Madgwick declarations')
    text = text.replace('#include <math.h>', '#include <math.h>\n#include "RW_RuntimeHot.hpp"', 1)
    text = text.replace('  void updateIMU(', '  void RW_SPEED_CODE updateIMU(')
    text = text.replace('  static float invSqrt(', '  static float RW_SPEED_CODE invSqrt(')
    return text.encode()


def patch(root, library):
    manifest = json.loads((FIXTURES / 'provenance.json').read_text())
    expected_header = annotated_header((FIXTURES / HEADER).read_bytes())
    # Verify everything before writing, including repeated/partially cached builds.
    for name, expected in manifest['sha256'].items():
        original = (FIXTURES / name).read_bytes()
        if hashlib.sha256(original).hexdigest() != expected:
            raise RuntimeError('Madgwick reference hash mismatch: ' + name)
        raw = (library / 'src' / name).read_bytes()
        allowed = (original, expected_header) if name == HEADER else (original,)
        if raw not in allowed:
            raise RuntimeError('Unexpected installed Madgwick edit: ' + name)
    (library / 'src' / HEADER).write_bytes(expected_header)
    policy = (root / 'src/realtime_code.h').read_bytes()
    (library / 'src/RW_RuntimeHot.hpp').write_bytes(policy)
    provenance = {
        'revision': 'madgwick_hot_04720',
        'upstream_sha256': manifest['sha256'],
        'patched_header_sha256': hashlib.sha256(expected_header).hexdigest(),
        'policy_header_sha256': hashlib.sha256(policy).hexdigest(),
        'policy': 'updateIMU_both_overloads_and_invSqrt_O2_no_fast_math_IRAM;unchanged_upstream_function_bodies',
    }
    (root / 'madgwick-build-patch.json').write_text(json.dumps(provenance, indent=2) + '\n')
    print('Madgwick hot-code annotations verified:', provenance['patched_header_sha256'])
    return provenance


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--library', type=Path, required=True)
    args = parser.parse_args()
    patch(ROOT, args.library)
elif 'env' in globals():
    patch(ROOT, Path(env.subst('$PROJECT_LIBDEPS_DIR')) / env.subst('$PIOENV') / 'Adafruit AHRS')
