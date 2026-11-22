"""Check post-aid stall evidence against the preceding return at the same angle."""
import argparse
from bisect import bisect_right
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-22-walking-simulator'))
from receiving_carry import CarryReplay


def preceding_speed(plans, angle, before):
    """Interpolate between two earlier 25 ms control snapshots, without a long gap."""
    for i in range(1, len(plans)):
        t0, u0, p = plans[i-1]
        t1, u1, q = plans[i]
        if (max(u0, u1) >= before or not 0 < t1-t0 <= 40 or
                p['side'] != '-1' or q['side'] != '-1'):
            continue
        hi, lo = float(p['lateral_deg']), float(q['lateral_deg'])
        a, b = -float(p['rate_dps']), -float(q['rate_dps'])
        if hi >= angle > lo and min(a, b) > 0:
            return a + (b-a)*(hi-angle)/(hi-lo)
    return None


def trial(folder):
    replay = CarryReplay(folder)
    peaks = [k for _, k, side in replay.confirmations if side == 1]
    rows = []
    for event in replay.archive['records']:
        if not (event['side'] == -1 and event['return_event'] and event['executed']):
            continue
        peak, off = event['peak_ms'], event['output_end_ms']
        result = replay.follow(dict(peak_ms=peak, decision_ms=event['output_start_ms'], terminal_deg=0.))
        row = dict(run=folder.name, peak_ms=peak, outcome=result['status'], opportunity=False, warning=None)
        rows.append(row)
        if row['outcome'] not in ('crossed', 'reversed'):
            continue
        prior_i = bisect_right(peaks, peak-1)-1
        previous = replay.by_peak.get(peaks[prior_i], ()) if prior_i >= 0 else ()
        latest = None
        for state_ms, ready_ms, plan in sorted(replay.by_peak.get(peak, ()), key=lambda s:s[1]):
            # A warning becomes available only once the computation finishes,
            # following the original short aid. Stop scoring at the measured
            # crossing/turn. Never use that outcome to compute a warning.
            if state_ms < off or ready_ms >= result['end_ms']:
                continue
            if not (plan.get('phase') == '2' and plan.get('anchor') in ('1','2') and plan['side'] == '-1'):
                latest = None
                continue
            row['opportunity'] = True
            x, v, a = float(plan['lateral_deg']), -float(plan['rate_dps']), float(plan['accel_dps2'])
            if v <= 0 or x <= 0 or a <= 0 or v*v/(2*a) >= x:
                latest = None
                continue
            packet = int(plan['packet'])
            if (latest and packet == latest[0]+1 and 0 < state_ms-latest[1] <= 40 and
                    0 < ready_ms-latest[2] <= 50):
                prior_v = preceding_speed(previous, x, ready_ms)
                row['warning'] = dict(time_ms=round(ready_ms,2), before_outcome_ms=round(result['end_ms']-ready_ms,2),
                                      x_deg=x, speed_dps=v, prior_speed_dps=prior_v,
                                      ratio=v/prior_v if prior_v else None)
                break
            latest = packet, state_ms, ready_ms
    return rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', nargs='+', type=Path)
    args = parser.parse_args()
    folders = sorted({file.parent for root in args.roots for file in root.glob('*/header.json')})
    print(json.dumps([r for folder in folders for r in trial(folder)], indent=2, allow_nan=False))
