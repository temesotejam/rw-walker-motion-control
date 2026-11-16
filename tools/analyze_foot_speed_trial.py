"""Audit recorded camera timing and aid/main spending without operating hardware.

Foot poses at decisions use only a recorded control receipt at/before that
decision, never a nearest future frame. Physical contact/forward progress and
energy after a hypothetical different input are not inferred.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import statistics
import zlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from analyze_foot_latency import analyze as analyze_latency


def read_csv(path):
    with path.open(encoding='utf8', newline='') as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        return
    with path.open('w', encoding='utf8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def review(trial, source, observation, baseline=None):
    raw = source.read_bytes()
    header = json.loads((trial / 'header.json').read_text(encoding='utf8'))
    meta = json.loads((trial / 'metadata.json').read_text(encoding='utf8'))
    assert len(raw) == header['crc_offset'] + 4
    assert zlib.crc32(raw[:-4]) == int.from_bytes(raw[-4:], 'little')
    samples = [r for r in read_csv(trial / 'timeseries.csv') if r['state_id'] == '3']
    frames = [r for r in meta['foot_frames'] if r['state_id'] == 3]
    assert frames and samples and not meta['foot_observation']['overflow']
    epochs = {r['frame_us'] - r['measurement_time_us'] for r in frames}
    assert len(epochs) == 1
    epoch = epochs.pop()
    events = meta['walking_energy']['records']
    assert len(events) == meta['walking_energy']['total'] and not meta['walking_energy']['truncated']
    peaks = read_csv(trial / 'energy_control_autonomous_peak_events.csv')
    zeros = [r for r in read_csv(trial / 'energy_control_autonomous_zero_cross_events.csv')
             if r['event_kind'] == 'energy_control_zero_cross']
    outputs = [r for r in events if r['executed']]
    aids = [r for r in outputs if r['return_event']]
    receipts = [r for r in frames if r['control_observed_us'] > 0]
    assert all(r['frame_us'] <= r['ready_us'] <= r['control_observed_us'] for r in receipts)
    decision_feet = []
    for event in events:
        now = epoch + event['time_ms'] * 1000
        eligible = [r for r in receipts if r['control_observed_us'] <= now]
        foot = max(eligible, key=lambda r: r['control_observed_us']) if eligible else None
        row = {'decision_ms': event['time_ms'], 'source_peak_ms': event['peak_ms'],
               'executed': event['executed'], 'return_aid': event['return_event'],
               'known_sequence': foot['sequence'] if foot else None,
               'known_frame_time_ms': foot['measurement_time_us'] / 1000 if foot else None,
               'known_frame_age_ms': (now - foot['frame_us']) / 1000 if foot else None}
        row.update({key: foot[key] if foot else None for key in
                    ['left_deg', 'right_deg', 'left_valid', 'right_valid']})
        decision_feet.append(row)
    cycles = []
    for peak in sorted({r['peak_ms'] for r in events}):
        group = [r for r in events if r['peak_ms'] == peak]
        spent = 0
        for r in group:
            assert r['envelope_before_ms'] == 80 - spent
            if r['executed']:
                assert r['output_end_ms'] - r['output_start_ms'] == r['actual_elapsed_ms']
                spent += r['actual_elapsed_ms']
            assert r['envelope_after_ms'] == 80 - spent and spent <= 80
        cycles.append({'source_peak_ms': peak, 'spent_ms': spent, 'remaining_ms': 80 - spent,
                       'aid_ms': sum(r['actual_elapsed_ms'] for r in group if r['executed'] and r['return_event']),
                       'main_ms': sum(r['actual_elapsed_ms'] for r in group if r['executed'] and not r['return_event'])})
    gaps = [{'time_s': b['measurement_time_us'] / 1e6, 'interval_ms': (b['frame_us'] - a['frame_us']) / 1000}
            for a, b in zip(frames, frames[1:]) if b['frame_us'] - a['frame_us'] > 60000]
    last_peak_ms = int(peaks[-1]['peak_time_ms'])
    after_peak = [r for r in frames if last_peak_ms * 1000 <= r['measurement_time_us'] <= (last_peak_ms + 800) * 1000
                  and r['left_valid'] and r['right_valid']]
    latency, _, _ = analyze_latency(trial)
    result = {
        'trial': trial.name, 'source': str(source), 'bytes': len(raw),
        'sha256': hashlib.sha256(raw).hexdigest(), 'crc_ok': True,
        'build': meta['diagnostic_build_id'], 'terminal': meta['terminal_state'],
        'sample_count': header['sample_count'], 'foot_frames': len(meta['foot_frames']),
        'active_foot_frames': len(frames), 'user_observation': observation,
        'active_both_valid': sum(r['left_valid'] and r['right_valid'] for r in frames),
        'foot_reasons': {side: dict(Counter(r[side + '_reason'] for r in frames)) for side in ['left', 'right']},
        'valid_foot_angle_ranges_deg': {side: [min(r[side + '_deg'] for r in frames if r[side + '_valid']),
                                             max(r[side + '_deg'] for r in frames if r[side + '_valid'])]
                                       for side in ['left', 'right']},
        'valid_foot_control_range_rejections': {side: sum(r[side + '_valid'] and not r[side + '_control_range_ok'] for r in frames)
                                              for side in ['left', 'right']},
        'latency': latency, 'baseline_latency': analyze_latency(baseline)[0] if baseline else None,
        'active_sustained_capture_hz': (len(frames) - 1) * 1e6 / (frames[-1]['frame_us'] - frames[0]['frame_us']),
        'gaps_over_60ms': gaps, 'energy_decisions': len(events), 'executed_energy_inputs': len(outputs),
        'executed_early_main_inputs': sum(r['optional'] and not r['return_event'] for r in outputs),
        'return_aids': aids, 'last_energy_input': outputs[-1], 'last_energy_decision': events[-1],
        'last_real_zero': zeros[-1], 'last_peak': peaks[-1],
        'aid_half_cycles': [r for r in cycles if r['aid_ms']],
        'last_second_lateral_median_deg': statistics.median(float(r['pitch_mekf_abs_deg']) for r in samples if float(r['time_s']) >= 29),
        'post_final_peak_800ms_max_observed_foot_difference_deg': max(abs(r['left_deg'] - r['right_deg']) for r in after_peak) if after_peak else None,
        'fore_aft_guard': meta['walking_energy']['fore_aft_motion_guard'],
        'dma_counters': {k: v for k, v in meta['foot_observation'].items() if k.startswith('dma_')},
        'checks': {'crc_length': True, 'epoch_unique': True, 'receipt_order': True, 'shared_80ms_accounting': True},
        'limits': ['Driver timestamps are not calibrated physical exposure times.',
                   'CRC/recognition validity does not independently validate angle accuracy or contact.',
                   'Known control receipt is a causal latest-recorded association, not a per-decision frame ID.',
                   'J quantities are provisional model predictions, not measured mechanical work.',
                   'A zero-width exhausted-envelope record is invalid; its requested_ms=1 is only a calculation sentinel.',
                   'No changed-trajectory replay, no inferred forward travel, no hardware operation.']}
    (trial / 'speed_trial_review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')
    write_csv(trial / 'latest_foot_at_energy.csv', decision_feet)
    write_csv(trial / 'energy_half_cycles.csv', cycles)
    write_csv(trial / 'foot_gaps.csv', gaps)
    plot(trial, samples, frames, events, result)
    return result


def plot(trial, samples, frames, events, result):
    t = [float(r['time_s']) for r in samples]
    ft = [r['measurement_time_us'] / 1e6 for r in frames]
    for suffix, bounds in [('overview', (0, 30)), ('final_cycle', (13.4, 16.8))]:
        fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
        fig.subplots_adjust(left=.11, right=.985, bottom=.08, top=.94, hspace=.12)
        axes[0].plot(t, [float(r['pitch_mekf_abs_deg']) for r in samples], lw=1, color='#285b9c')
        axes[0].axhline(0, color='gray', lw=.6)
        axes[0].set_ylabel('Body lateral (deg)')
        for side, color in [('left', '#c36024'), ('right', '#258463')]:
            axes[1].plot(ft, [r[side + '_deg'] if r[side + '_valid'] else float('nan') for r in frames], label=side, lw=.9, color=color)
        axes[1].set_ylabel('Observed foot (deg)'); axes[1].legend(loc='upper right')
        axes[2].plot(t, [float(r['motor_cmd_mA']) for r in samples], lw=.8, label='Command')
        axes[2].plot(t, [float(r['roller_actual_current_mA']) for r in samples], lw=.7, alpha=.7, label='Measured current')
        axes[2].set_ylabel('Current (mA)'); axes[2].legend(loc='upper right')
        for r in events:
            axes[3].scatter(r['time_ms']/1000, r['envelope_after_ms'], marker='^' if r['return_event'] else 'o',
                            color='#c36024' if r['return_event'] else '#285b9c', s=18)
        axes[3].set_ylabel('Remaining budget (ms)');axes[3].set_ylim(-5, 85)
        axes[3].set_xlabel('Measurement time (s)')
        for ax in axes:
            ax.set_xlim(*bounds);ax.grid(alpha=.2)
            ax.axvline(result['last_energy_input']['time_ms']/1000, color='#c36024', ls='--', lw=.8, label='Last input decision')
            ax.axvline(float(result['last_real_zero']['zero_cross_time_ms'])/1000, color='#824b94', ls=':', lw=.9, label='Last real ZERO')
        axes[0].legend(loc='upper right', fontsize=8)
        axes[0].set_title(trial.name + ': recorded body, feet, current and shared input budget')
        fig.savefig(trial / ('speed_trial_' + suffix + '.png'), dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--rwlog', type=Path, required=True)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--user-observation', required=True)
    args = parser.parse_args()
    r = review(args.trial, args.rwlog, args.user_observation, args.baseline)
    print(json.dumps({k: r[k] for k in ['build', 'crc_ok', 'terminal', 'active_foot_frames',
          'active_both_valid', 'active_sustained_capture_hz', 'aid_half_cycles', 'checks']}, indent=2))


if __name__ == '__main__':
    main()
