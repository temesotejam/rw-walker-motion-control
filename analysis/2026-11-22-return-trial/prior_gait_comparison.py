"""Compare the measured initial posture and the first leftward returns of walking trials."""
import argparse
import csv
import json
from pathlib import Path
from statistics import median
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'analysis/2026-11-20-smallstep-baseline'))
sys.path.insert(0, str(ROOT / 'tools'))
from summarize_start import summarize
from score_observed_transfers import gyro_samples, measurement_epoch


def compare(folder):
    start = summarize(folder)
    meta = json.loads((folder / 'metadata.json').read_bytes())
    epoch = measurement_epoch(meta)
    with (folder / 'timeseries.csv').open(encoding='utf-8-sig', newline='') as stream:
        times, rates, angles = gyro_samples(meta, csv.DictReader(stream), epoch)
    # Use the measured leftward crossing even when an early pulse consumed the
    # firmware ZERO event. Limit each right-to-left peak interval to one crossing.
    speeds = []
    peaks = meta['energy_control_autonomous_peak_events']
    for a, b in zip(peaks, peaks[1:]):
        if a['physical_peak_side'] != 1 or b['physical_peak_side'] != -1:
            continue
        lo, hi = a['peak_time_ms'], b['peak_time_ms']
        for k, t in enumerate(times[1:], 1):
            if t > min(4000, hi):
                break
            if times[k-1] < max(1000, lo) or t-times[k-1] > 10:
                continue
            before, after = angles[k-1:k+1]
            if before is not None and after is not None and before > 0 >= after:
                fraction = before / (before-after)
                speeds.append(-(rates[k-1] + fraction*(rates[k]-rates[k-1])))
                break
    events = meta['energy_control_autonomous_zero_cross_events']
    first_outputs = [e for e in events if e['output_executed'] and e['pulse_start_ms'] <= peaks[0]['peak_time_ms']]
    if len(first_outputs) != 1 or first_outputs[0]['event_kind'] != 'strong_start_kick':
        raise ValueError(f'{folder}: expected only the kick before the first peak')
    first = first_outputs[0]
    right_limits = [(r['time_ms'], r['right_arrival_base_ms'], r['guarded_ms'])
                    for r in meta['walking_energy']['records'] if r.get('right_arrival_eligible') and
                    r['executed'] and r['guarded_ms'] < r['right_arrival_base_ms']]
    return dict(run=folder.name, profile=meta.get('walking_profile',meta['diagnostic_build_id']),
                prestart_abs_deg=start['prestart_roll_median_deg'],
                first_right_peak_deg=start['first_peak_signed_deg'],
                first_zero_ms=start['first_center_crossing_interval_ms'],
                last_peak_ms=start['last_accepted_body_peak_ms'], kick_mA=first['command_current_mA'],
                kick_ms=first['pulse_width_ms'], left_crossings_1_to_4s_deg_s=speeds,
                actual_right_limit_time_base_final_ms=right_limits,
                median_left_crossing_deg_s=median(speeds) if speeds else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', nargs='+', type=Path, help='One converted RWLOG directory per trial')
    args = parser.parse_args()
    print(json.dumps([compare(f) for f in args.folders], indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
