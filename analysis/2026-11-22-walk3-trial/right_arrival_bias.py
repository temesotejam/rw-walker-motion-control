"""Compare bounded corrections of the 4.5-degree right arrival forecast across trials."""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-20-ab-trials'))
import calibration

CALIBRATE = {195727101, 135284989}  # First two Nov 22 runs with logged, completed eligible mains.
HELD = {68907120, 41749536, 37339215}


def charge(row, width):
    duration = width / 1000
    return row['goal']*duration + (row['initial']-row['goal'])*row['tau']*(-math.expm1(-duration/row['tau']))


def maximum_charge(rate):
    # Interpolation support used by src/right_arrival_energy.h
    knots = [(21., 12.4), (36., 11.), (40., 8.5), (47., 3.3), (51., 0.)]
    for (a, x), (b, y) in zip(knots, knots[1:]):
        if a <= rate <= b:
            return x + (y-x)*(rate-a)/(b-a)
    return -1.


def predicted_uJ(rate, q):
    omega = math.radians(rate)
    return 152.963884 + 465.018868*omega*omega + 93.4855564*omega*q


def arrivals(folder):
    meta = json.loads((folder/'metadata.json').read_bytes())
    run = int(json.loads((folder/'header.json').read_bytes())['run_start_us'])
    events = meta['energy_control_autonomous_zero_cross_events']
    for event in events:
        if (event['event_kind'] != 'energy_control_zero_cross' or event['physical_next_peak_side'] != 1
                or event['startup_target_deg'] != 4.5):
            continue
        t, previous = event['zero_cross_time_ms'], event['previous_peak_time_ms']
        w = next((r for r in meta['walking_energy']['records'] if r['side'] == 1 and r['peak_ms'] == previous
                  and not r['rescue'] and not r['optional'] and abs(t-r['time_ms']) <= 8), None)
        if not w or not w['valid'] or w['assisted_cycle']:
            continue
        if w['executed'] and not (w['planned_duration_completed'] and w['completion_valid']):
            continue
        peak = next((p for p in meta['energy_control_autonomous_peak_events'] if p['peak_time_ms'] > t), None)
        if not peak or peak['physical_peak_side'] != 1 or peak['peak_time_ms']-t > 750:
            continue
        if any(e is not event and e['output_executed'] and previous < e['zero_cross_time_ms'] < peak['peak_time_ms']
               for e in events):
            continue
        elapsed = w['actual_elapsed_ms'] if w['executed'] else 0
        row = dict(run=run, time_ms=t, elapsed=elapsed, planned=w['guarded_ms'] if w['executed'] else 0,
                   rate=w['start_rate_dps'], initial=w['start_i0_ma'], goal=w['start_goal_ma'], tau=w['start_tau_s'],
                   target=1e6*w['target_j'], actual_deg=peak['peak_amplitude_deg'])
        q = charge(row, elapsed)
        if not (2.13 <= event['previous_peak_amplitude_deg'] <= 6 and 0 <= q <= maximum_charge(row['rate'])):
            continue
        predicted = predicted_uJ(row['rate'], q)
        logged = w.get('right_arrival_elapsed_j')
        if logged is not None and abs(predicted-1e6*logged) > .3:
            raise ValueError(f'Prediction mismatch for {run}/{t}')
        yield dict(row, actual=1e6*calibration.potential(row['actual_deg']), predicted=predicted, logged=logged is not None)


def evaluate(rows):
    trained = [r for r in rows if r['run'] in CALIBRATE and r['logged'] and r['elapsed']]
    if {r['run'] for r in trained} != CALIBRATE:
        raise ValueError('The two chronological calibration runs are required')
    correction = max(0., statistics.median(r['actual']-r['predicted'] for r in trained))
    print('Calibration error median (uJ):', correction)
    for label, group in [('original model fit', calibration.TRAIN), ('original holdout', calibration.HOLD),
                         ('recent calibration', CALIBRATE), ('later runs', HELD)]:
        for powered in (False, True):
            selected = [r for r in rows if r['run'] in group and bool(r['elapsed']) == powered]
            if not selected:
                print(label, 'powered' if powered else 'coast', 0)
                continue
            print(label, 'powered' if powered else 'coast', len(selected))
            for mode in ('baseline', 'constant', 'power_limited'):
                energy_errors, angle_errors, widths = [], [], []
                for r in selected:
                    limit = min(.25*r['target'], correction)
                    coast = predicted_uJ(r['rate'], 0)

                    def corrected(raw):
                        if mode == 'constant':
                            return raw + limit
                        if mode == 'power_limited':
                            return raw + min(limit, max(0., raw-coast))
                        return raw

                    y = corrected(r['predicted'])
                    energy_errors.append(abs(y-r['actual']))
                    angle_errors.append(abs(calibration.angle(y/1e6)-r['actual_deg']))
                    options = [(n, corrected(predicted_uJ(r['rate'], charge(r, n)))) for n in range(r['planned']+1)
                               if 0 <= charge(r, n) <= maximum_charge(r['rate'])]
                    width, _ = min(options, key=lambda item: abs(item[1]-r['target']))
                    widths.append((r['run'], r['time_ms'], r['planned'], width))
                print(mode, 'MAE_uJ', round(statistics.mean(energy_errors), 2),
                      'MAE_deg', round(statistics.mean(angle_errors), 3),
                      'proposed widths' if group == HELD else '', widths if group == HELD else '')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', type=Path, nargs='+')
    args = parser.parse_args()
    folders = {p.parent for root in args.roots for p in root.glob('*/header.json')}
    rows = [r for folder in sorted(folders) for r in arrivals(folder)]
    print('Logged powered residuals (peak energy minus prediction, uJ):',
          [(r['run'], r['time_ms'], round(r['actual']-r['predicted'], 1)) for r in rows if r['elapsed'] and r['logged']])
    evaluate(rows)
