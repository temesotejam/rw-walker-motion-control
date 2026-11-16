"""Summarize camera-only boot probes (sensor registers and sampled luminance)."""
import argparse
import json
from pathlib import Path
import statistics


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    rows = []
    for line in args.source.read_text(encoding='utf8', errors='replace').splitlines():
        if not line.startswith('AUDIT,hb='):
            continue
        try:
            rows.append({k: int(v) for k, v in (item.split('=', 1) for item in line.split(',')[1:])})
        except ValueError:
            continue
    phases = []
    for hb in sorted({r['hb'] for r in rows}):
        subset = [r for r in rows if r['hb'] == hb]
        changes = []
        for a, b in zip(subset, subset[1:]):
            diff = {key: [a[key], b[key]] for key in ['exp', 'g50', 'g51', 'g52', 'd2', 'd4'] if a[key] != b[key]}
            if diff:
                changes.append({'ms': b['ms'], 'mean': b['mean'], 'changes': diff})
        phases.append({'hblank': hb, 'samples': len(subset), 'elapsed_ms': subset[-1]['ms'],
                       'median_frame_age_us': statistics.median(r['age_us'] for r in subset),
                       'mean_range': [min(r['mean'] for r in subset), max(r['mean'] for r in subset)],
                       'first_white_ms': next((r['ms'] for r in subset if r['min'] > 220), None),
                       'last': subset[-1], 'changes': changes})
    result = {'scope': 'camera-only boot; no motor/controller/Web startup during probe',
              'rows': rows, 'phases': phases}
    args.out.write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps(phases, indent=2))


if __name__ == '__main__':
    main()
