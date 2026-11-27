"""Apply the fixed 11/20 reference to two 11/13 v14 trials with sparse receipt times.

Compare directly observed foot angles only if the calibration, lane and sign match.
The older logs provide camera frame/delivery times, but only an explicitly archived
control receipt proves availability. A delivered frame with an unknown receipt
invalidates the last known foot state. Current-camera 25 ms packets did not exist.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
from statistics import mean

import add_early_watch
import return_model


LEGACY = (172594684, 294107213)


def metadata(root, run):
    folder = root / str(run)
    if json.loads((folder / 'header.json').read_text(encoding='utf8'))['run_start_us'] != run:
        raise ValueError(f'Incorrect run in {folder}')
    return json.loads((folder / 'metadata.json').read_text(encoding='utf8'))


def angle_signature(meta):
    obs = meta['foot_observation']
    return (obs['calibration']['source_sha256'], obs['calibration']['zero'], obs['mapping'], obs['positive_direction'])


def observed_packets(meta):
    frames = [f for f in meta['foot_frames'] if f['state_id'] == 3]
    epochs = {f['frame_us'] - f['measurement_time_us'] for f in frames if f['timestamp_valid']}
    if len(epochs) != 1 or meta['foot_observation']['overflow']:
        raise ValueError('Incomplete frame archive or ambiguous trial clock')
    epoch = epochs.pop()
    packets, latency = [], []
    for f in frames:
        received = f.get('control_observed_us', 0)
        available_us = (received or f['delivered_us']) - epoch
        if packets and available_us <= packets[-1]['publish_us']:
            raise ValueError('Out-of-order v14 frame availability markers')
        packet = {'publish_us': available_us}
        if received:
            age = received - f['frame_us']
            if f['frame_us'] > f['delivered_us'] or received < f['delivered_us']:
                raise ValueError('Inconsistent capture, delivery and control receipt')
            latency.append(age / 1000)
        for side in ('left', 'right'):
            valid = (received and f['timestamp_valid'] and f['frame_valid'] and f['zero_ready']
                     and f[side + '_valid'] and f[side + '_in_range'])
            packet.update({
                side + '_age_us': str(age) if valid else '',
                side + '_observed_deg': str(f[side + '_deg']) if valid else '',
                side + '_sequence': f['sequence'] if valid else 0})
        packets.append(packet)
    return packets, {'active_frames': len(frames), 'archived_receipts': len(latency),
                     'receipt_age_range_ms': [min(latency), max(latency)] if latency else None}


def executed_intervals(meta):
    # The early format lacks completion time in walking_energy. The executed event
    # archive carries it. Verify every non-start pulse against the independent work rows.
    events = [e for e in meta['energy_control_autonomous_zero_cross_events'] if e['output_executed']]
    work = meta['walking_energy']
    rows = [r for r in work['records'] if r['executed']]
    starts = [e for e in events if e['event_kind'] == 'strong_start_kick']
    other = [e for e in events if e['event_kind'] != 'strong_start_kick']
    if (len(starts) != 1 or len(other) != len(rows) or work['truncated']
            or any(e['pulse_end_ms'] <= e['pulse_start_ms'] for e in events)):
        raise ValueError('Cannot verify all v14 command intervals')
    if len({e['pulse_start_ms'] for e in events}) != len(events):
        raise ValueError('Duplicate v14 command starts')
    for row in rows:
        matches = [e for e in other if e['physical_next_peak_side'] == row['side']
                   and abs(row['time_ms'] - e['zero_cross_time_ms']) <= 8 and row['guarded_ms'] == e['pulse_width_ms']]
        if len(matches) != 1:
            raise ValueError('Missing or ambiguous executed v14 command')
        other.remove(matches[0])
    return [(e['pulse_start_ms'], e['pulse_end_ms']) for e in events]


def run(reference_root: Path, legacy_root: Path) -> dict:
    reference_root, legacy_root = Path(reference_root), Path(legacy_root)
    reference, signatures = [], set()
    for run_id in return_model.PRIOR:
        signatures.add(angle_signature(metadata(reference_root, run_id)))
        rows, _ = return_model.load_sections(reference_root / str(run_id), add_early_watch.SECTIONS)
        reference.extend(rows)
    comparison = {}
    for run_id in LEGACY:
        meta = metadata(legacy_root, run_id)
        if signatures != {angle_signature(meta)}:
            raise ValueError('Old and training angle calibration, zero, side or sign differ')
        packets, camera = observed_packets(meta)
        peaks = []
        rows, _ = return_model.load_sections(legacy_root / str(run_id), add_early_watch.SECTIONS, peaks,
                                             foot_packets=packets, executed_intervals=executed_intervals(meta))
        by_section = {}
        for section in add_early_watch.SECTIONS:
            sample = [r for r in rows if r.state.section == section]
            coast = return_model.coast_reference(sample)
            body = [(r, return_model.predict(reference, r.state, use_foot=False)) for r in coast]
            body = [(r, p) for r, p in body if p and p['distance'] <= 1]
            by_section[str(section)] = {
                'observations': len(sample), 'with_confirmed_fresh_feet': sum(math.isfinite(r.state.left) for r in sample),
                'no_intervening_command': len(coast),
                'no_intervening_command_with_feet': sum(math.isfinite(r.state.left) for r in coast),
                'body_only_supported_n': len(body),
                'body_only_speed_mae_dps': round(mean(abs(r.zero_speed-p['zero_speed']) for r,p in body), 3) if body else None}
        watches = {}
        for label, sections in [('base', return_model.SECTIONS), ('with_early_section', add_early_watch.SECTIONS)]:
            scored = return_model.evaluate_episodes(reference, [r for r in rows if r.state.section in sections], peaks)
            watches[label] = {'status': scored['counts'], 'metrics_on_supported_cases': scored['metrics']['warning']}
        comparison[str(run_id)] = {'camera': camera, 'positive_peak_outcomes': dict(Counter(p['outcome'] for p in peaks)),
                                   'sections': by_section, 'foot_models': watches}
    return {'training_runs': list(return_model.PRIOR), 'older_test_runs': list(LEGACY), 'per_run': comparison}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference_root', type=Path, help='Root containing the eight decoded 11/20 references')
    parser.add_argument('legacy_root', type=Path, help='Root containing decoded 172594684 and 294107213')
    args = parser.parse_args()
    print(json.dumps(run(args.reference_root, args.legacy_root), indent=2, allow_nan=False))
