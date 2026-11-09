#!/usr/bin/env python3
"""Leave-one-run-out check of observational peak-to-peak transitions.

Pass converted log directories. These scores evaluate historical actions,
not alternative actions selected by an optimizer. Requires NumPy.
"""
import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np

def feature(v):
    return np.array([1.0, v[0], v[1]/100.0, v[2], v[3]/10.0, v[5]/1000.0])

def extract(folder):
    x = json.loads((folder / 'metadata.json').read_text(encoding='utf-8'))
    z, peaks = x['energy_control_autonomous_zero_cross_events'], x['energy_control_autonomous_peak_events']
    samples, chains = [], []
    for i in range(len(z)-1):
        a, b = z[i], z[i+1]
        try:
            if (a['event_kind'] != 'energy_control_zero_cross' or b['event_kind'] != 'energy_control_zero_cross'
                or not a['pre_input_wheel_speed_valid'] or not b['pre_input_wheel_speed_valid']
                or a['pre_input_wheel_speed_age_us'] > 50000 or b['pre_input_wheel_speed_age_us'] > 50000
                or not a['valid'] or not b['valid'] or not a['output_executed']
                or a['zero_cross_time_ms'] < 5000 or b['zero_cross_time_ms'] > 24000
                or b['physical_next_peak_side'] != -a['physical_next_peak_side']):
                continue
            matched = [p for p in peaks if a['zero_cross_time_ms'] < p['peak_time_ms'] < b['zero_cross_time_ms']
                       and p['physical_peak_side'] == a['physical_next_peak_side'] and p['pending_command_matched']]
            if len(matched) != 1:
                continue
            values = [a['zero_cross_abs_rate_dps'], a['pre_input_wheel_speed_rpm'], a['previous_peak_amplitude_deg'],
                      a['pulse_width_ms'], a['command_current_mA'], a['vbat_mV']]
            target = [matched[0]['peak_amplitude_deg'], b['zero_cross_abs_rate_dps'], b['pre_input_wheel_speed_rpm']]
            if any(v is None or not np.isfinite(v) for v in values + target):
                continue
            samples.append((a['physical_next_peak_side'], values, target))
            if i+2 >= len(z) or not b['output_executed']:
                continue
            c = z[i+2]
            future = [p for p in peaks if b['zero_cross_time_ms'] < p['peak_time_ms'] < c['zero_cross_time_ms']
                      and p['physical_peak_side'] == b['physical_next_peak_side'] and p['pending_command_matched']]
            if (len(future) != 1 or not c['valid'] or not c['pre_input_wheel_speed_valid']
                or c['pre_input_wheel_speed_age_us'] > 50000 or c['physical_next_peak_side'] != a['physical_next_peak_side']):
                continue
            future_target = [future[0]['peak_amplitude_deg'], c['zero_cross_abs_rate_dps'], c['pre_input_wheel_speed_rpm']]
            if all(v is not None and np.isfinite(v) for v in future_target):
                chains.append((a['physical_next_peak_side'], values, b['pulse_width_ms'], b['command_current_mA'], future_target))
        except (KeyError, TypeError):
            continue
    return samples, chains

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path, help='Converted log directories with metadata.json')
    args = parser.parse_args()
    runs = {str(p): extract(p) for p in args.directories}
    for name, (samples, chains) in runs.items():
        print(name, 'transitions', len(samples), 'widths',
              {side: dict(sorted(Counter(s[1][3] for s in samples if s[0] == side).items())) for side in (1, -1)})
        models = {}
        for side in (1, -1):
            training = [s for other, (ss, _) in runs.items() if other != name for s in ss if s[0] == side]
            if len(training) < 20:
                break
            models[side] = np.linalg.lstsq(np.array([feature(s[1]) for s in training]),
                                            np.array([s[2] for s in training]), rcond=None)[0]
        if len(models) != 2:
            continue
        errors = []
        for side, values, width, current, target in chains:
            predicted = feature(values) @ models[side]
            next_values = [predicted[1], predicted[2], predicted[0], width, current, values[5]]
            errors.append(feature(next_values) @ models[-side] - target)
        if errors:
            errors = np.array(errors)
            print('  2-step historical-action replay:', len(errors),
                  'MAE [deg, dps, rpm]', np.round(np.mean(np.abs(errors), axis=0), 3),
                  'max abs', np.round(np.max(np.abs(errors), axis=0), 3))
    print('These are observational replay errors; they do not validate alternative input effects.')

if __name__ == '__main__':
    main()
