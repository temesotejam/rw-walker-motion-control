"""Audit converted v12-v14 trials; a log alone cannot establish walking/contact."""
import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def rows(directory, name):
    path = directory / (name + '.csv')
    if not path.exists():
        return []
    with path.open(encoding='utf8', newline='') as stream:
        return list(csv.DictReader(stream))


def num(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float('nan')


def measurement_rows(samples):
    # Format56 state3 is the actual measurement; states6/7 are LED sync.
    # END sync retains t_test_ms=30000 for another ~5s. Folding those rows
    # onto t=30 invents extra crossings and corrupts terminal statistics.
    return [r for r in samples if r['state_id'] == '3' and 0 <= num(r['t_test_ms']) <= 30000]


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {'count': 0}
    return dict(zip(['min', 'median', 'p95', 'max'],
                    map(float, np.percentile(values, [0, 50, 95, 100]))), count=len(values))


def analyze(directory, original):
    metadata = json.loads((directory / 'metadata.json').read_text(encoding='utf8'))
    header = json.loads((directory / 'header.json').read_text(encoding='utf8'))
    samples = rows(directory, 'timeseries')
    feet = rows(directory, 'foot_angles')
    history = rows(directory, 'foot_pose_prediction')
    local = rows(directory, 'foot_local_prediction')
    assist = rows(directory, 'foot_support_assist')
    peaks = rows(directory, 'energy_control_autonomous_peak_events')
    events = rows(directory, 'energy_control_autonomous_zero_cross_events')
    measured = measurement_rows(samples)
    t = np.array([num(r['t_test_ms']) / 1000 for r in measured])
    angle = np.array([num(r['pitch_mekf_detector_relative_deg']) for r in measured])
    rate = np.array([num(r['physical_roll_rate_dps']) for r in measured])
    # Converted angle is quantized to .01deg, so zero-valued plateaus must be
    # bridged before looking for the last *sampled* sign crossing.
    nz = np.flatnonzero(np.isfinite(angle) & (angle != 0))
    transitions = [(int(a), int(b)) for a, b in zip(nz[:-1], nz[1:])
                   if angle[a] * angle[b] < 0]
    observed = [r for r in feet if r['control_observed_us']]
    executed = [r for r in events if r['output_executed'] == 'True']
    full_valid = [r for r in feet if all(r[k] == 'True' for k in
        ['timestamp_valid', 'frame_valid', 'zero_ready', 'left_valid',
         'right_valid', 'left_in_range', 'right_in_range'])]
    support = metadata.get('foot_support_assist', {})
    support_branch = str(support.get('work_control_branch', 17))
    # v14 shares branch19 among ordinary, generic early and foot inputs.
    # A branch number alone no longer identifies a foot-triggered command.
    support_times = [num(a['time_ms']) for a in assist if a.get('output_executed')]
    support_events = [r for r in executed if r['work_control_branch'] == support_branch
                      and any(abs(num(r['pulse_start_ms']) - ms) <= 10 for ms in support_times)]
    support_event_indices = {r['event_index'] for r in support_events}
    zero_locked = bool(feet) and all(r['zero_ready'] == 'True' for r in feet)
    reject = dict(zip(support.get('reason_names', []), support.get('candidate_check_counts', [])))
    summary = {
        'input_sha256': hashlib.sha256(original.read_bytes()).hexdigest(),
        'build_id': metadata.get('diagnostic_build_id'),
        'format_version': header['format_version'], 'samples': len(samples),
        'measurement_samples': len(measured),
        'terminal': metadata.get('terminal_state'),
        'feet': len(feet), 'fully_control_valid_feet': len(full_valid),
        'zero_ready': dict(Counter(r['zero_ready'] for r in feet)),
        'zero_reason_recorded': dict(Counter(r['zero_reason'] for r in feet)),
        'detection_reason': {side: dict(Counter(r[side + '_reason'] for r in feet))
                             for side in ['left', 'right']},
        'prediction_reasons': {name: dict(Counter(r['reason'] for r in data))
                               for name, data in [('history', history), ('local', local)]},
        'support_audits': len(assist), 'support_total': support.get('total'),
        'support_candidate_checks': reject,
        'branch17_executed': sum(r['work_control_branch'] == '17' for r in executed),
        'branch18_executed': sum(r['work_control_branch'] == '18' for r in executed),
        'branch19_executed': sum(r['work_control_branch'] == '19' for r in executed),
        'support_branch': support_branch,
        'support_executed': len(support_events),
        'executed': dict(Counter(r['event_kind'] for r in executed)),
        'peaks': len(peaks),
        'last_peaks': [{k: r[k] for k in ['peak_time_ms', 'physical_peak_side',
                       'peak_amplitude_deg', 'target_peak_deg']} for r in peaks[-6:]],
        'last_executed': {k: executed[-1][k] for k in ['event_kind', 'pulse_start_ms',
                         'pulse_width_ms', 'command_current_mA', 'physical_next_peak_side']},
        'last_sign_transitions': [{'start_s': float(t[a]), 'end_s': float(t[b]),
                                  'from_deg': float(angle[a]), 'to_deg': float(angle[b])}
                                 for a, b in transitions[-4:]],
        'last_26_to_30s_body_deg': stats(angle[(t >= 26) & (t <= 30)]),
        'driver_to_delivery_ms': stats([(num(r['delivered_us']) - num(r['frame_us'])) / 1000 for r in feet]),
        'processing_ms': stats([num(r['processing_us']) / 1000 for r in feet]),
        'driver_to_control_ms': stats([(num(r['control_observed_us']) - num(r['frame_us'])) / 1000 for r in observed]),
        'delivery_to_control_ms': stats([(num(r['control_observed_us']) - num(r['delivered_us'])) / 1000 for r in observed]),
        'control_observation_capture': metadata.get('foot_control_observations'),
        'scope': ('log audit only; camera-relative foot angles; no contact or forward-displacement inference'
                  if zero_locked else 'log audit only; provisional nominal-zero angles are not calibrated control observations; no contact or forward-displacement inference'),
    }
    # Avoid duplicating detail rows in the compact output.
    summary['control_observation_capture'] = {k: v for k, v in
        (summary['control_observation_capture'] or {}).items() if k != 'rows'}
    ft = np.array([num(r['measurement_time_us']) / 1e6 for r in feet])
    last_input_s = num(executed[-1]['pulse_start_ms']) / 1000 if executed else 25
    detail_start = max(0, last_input_s - 1)
    for name, limits in [('timeline', (0, 30)), ('stop_detail', (detail_start, min(30, last_input_s + 1.6)))]:
        fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
        axes[0].plot(t, angle, lw=.8, color='purple')
        axes[0].axhline(0, color='black', lw=.5)
        axes[0].set_ylabel('Body detector [deg]')
        for side, color in [('left', 'tab:blue'), ('right', 'tab:red')]:
            values = [num(r[side + '_deg']) if r[side + '_reason'] == 'detected'
                      and r[side + '_in_range'] == 'True'
                      and (not zero_locked or r[side + '_valid'] == 'True') else np.nan for r in feet]
            axes[1].plot(ft, values, '.--', color=color, ms=3, label=side)
        axes[1].legend()
        axes[1].set_ylabel('Camera foot [deg]' if zero_locked else 'Provisional foot [deg]\nZERO NOT LOCKED; invalid')
        axes[2].plot(t, rate, lw=.8, color='tab:orange')
        axes[2].set_ylabel('Body rate [deg/s]')
        for r in executed:
            tm = num(r['pulse_start_ms']) / 1000
            width = num(r['pulse_width_ms']) * num(r['physical_next_peak_side'])
            color = 'red' if r['event_index'] in support_event_indices else 'green'
            axes[3].vlines(tm, 0, width, color=color)
        axes[3].set_ylabel('Command width x side [ms]')
        axes[3].set_xlabel('Measurement time [s]')
        for ax in axes:
            ax.set_xlim(*limits)
            ax.grid(alpha=.3)
        fig.suptitle(f'{summary["build_id"]} trial {directory.name}: support outputs={summary["support_executed"]}; '
                     + ('foot zero locked' if zero_locked else 'foot zero unconfirmed'))
        fig.savefig(directory / (name + '.png'), dpi=140)
        plt.close(fig)
    (directory / 'trial_audit.json').write_text(json.dumps(summary, indent=2), encoding='utf8')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted', type=Path)
    parser.add_argument('original', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.converted, args.original), indent=2))
