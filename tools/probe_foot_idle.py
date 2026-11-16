"""Read-only camera timing probe. Never sends a start, clear or motor command.

These are HTTP-sampled idle frames, not all frames or walking measurements.
"""
import argparse
import json
from pathlib import Path
import statistics
import time
import urllib.request


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default='http://192.168.4.1')
    p.add_argument('--seconds', type=float, default=20)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    rows, errors = [], []
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        try:
            with opener.open(args.url.rstrip('/') + '/status.json', timeout=3) as r:
                s = json.load(r)
            rows.append(s)
            if s['running'] or s['motor_mA'] != 0:
                raise RuntimeError('Idle probe requires stopped motor; no commands sent')
        except (OSError, ValueError) as exc:
            errors.append(str(exc))
        time.sleep(.1)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({'scope': 'HTTP-sampled idle frames', 'rows': rows,
                                   'errors': errors}, ensure_ascii=False, indent=2), encoding='utf8')
    if not rows:
        raise SystemExit('No status replies; see ' + str(args.out))
    frames = list({(s['boot_id'], s['foot']['sequence']): s['foot'] for s in rows}.values())
    def stats(key):
        values = sorted(f[key] for f in frames if isinstance(f.get(key), (float, int)) and f[key] >= 0)
        return dict(n=len(values), median=statistics.median(values),
                    p95=values[int((len(values)-1)*.95)], max=max(values)) if values else None
    summary = {'builds': sorted({s['diagnostic_build_id'] for s in rows}),
               'boots': sorted({s['boot_id'] for s in rows}), 'status_replies': len(rows),
               'unique_sampled_frames': len(frames), 'http_errors': len(errors),
               'camera': rows[-1]['camera'], 'state': rows[-1]['state'],
               'all_motor_commands_zero': all(s['motor_mA'] == 0 and not s['running'] for s in rows),
               'both_detected': sum(f['left_valid'] and f['right_valid'] for f in frames),
               'zero_ready': rows[-1]['foot']['zero_ready'],
               'metrics': {k: stats(k) for k in ['driver_delivery_ms', 'frame_to_ready_ms',
                   'recognition_us', 'row_copy_us', 'processing_us', 'fps']}}
    args.out.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2), encoding='utf8')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
