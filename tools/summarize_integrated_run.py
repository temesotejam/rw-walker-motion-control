#!/usr/bin/env python3
"""CRC-check a supplied RWLOG, compare every foot CSV field, and save a summary.

Source files are read-only. Decoder products use a temporary directory. Timing
statistics come from firmware counters, not sparsely sampled ordinary rows.
"""
import argparse
from collections import Counter
import csv
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
from pathlib import Path
import tempfile
import convert_rwlog_to_csv as converter

BOOLEAN_FIELDS = {'timestamp_valid', 'frame_valid', 'zero_ready', 'right_valid',
                  'left_valid', 'right_in_range', 'left_in_range'}


def rows(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def normalized(key, value):
    if key in BOOLEAN_FIELDS:
        if value.lower() not in ('true', 'false', '0', '1'):
            raise ValueError('Unexpected boolean: ' + value)
        return value.lower() in ('true', '1')
    try:
        return Decimal(value)
    except InvalidOperation:
        return value


def summarize(source, foot_source):
    raw = source.read_bytes()
    header = converter.parse_header(raw)
    if not converter.verify_crc(raw, header):
        raise ValueError('Source CRC mismatch')
    begin = header['header_size']
    metadata = json.loads(raw[begin:begin + header['metadata_json_size']])
    with tempfile.TemporaryDirectory(prefix='integrated-run-') as temporary:
        decoded = Path(temporary)
        converter.convert(source, decoded)
        feet = rows(decoded / 'foot_angles.csv')
        samples = rows(decoded / 'timeseries.csv')
    uploaded = rows(foot_source)
    if len(feet) != len(uploaded):
        raise ValueError('Foot CSV row count mismatch')
    for index, (a, b) in enumerate(zip(feet, uploaded)):
        if a.keys() != b.keys():
            raise ValueError('Foot CSV columns mismatch')
        for key in a:
            if normalized(key, a[key]) != normalized(key, b[key]):
                raise ValueError(f'Foot CSV mismatch: row {index}, field {key}')
    acquisition = metadata['v46n_imu_acquisition']
    worker = metadata['v46p_control_worker']
    completion = worker['v46u_deadline']
    cohorts = worker['deadline_input_cohorts']
    for source_key, target_key in [('count', 'count'), ('over_budget', 'over_budget')]:
        if sum(x[source_key] for x in cohorts['groups']) != completion[target_key]:
            raise ValueError('Completion cohort totals disagree')
    acquisition_keys = ['duration_us', 'captured', 'delivered', 'queue_drops',
                        'delivery_sequence_gaps', 'queue_high_water', 'delivery_age_mean_us',
                        'delivery_age_max_us', 'dt_mean_us', 'dt_max_us', 'over_4ms',
                        'over_5ms', 'over_10ms', 'samples_per_second', 'fault', 'fault_reason']
    profile = metadata['control_work_profile']
    out = dict(source=source.name, foot_csv_source=foot_source.name,
               foot_csv_matches_all_decoded_fields=True, foot_csv_compared_rows=len(feet),
               foot_csv_compared_columns=len(feet[0]) if feet else 0,
               format_version=header['format_version'], crc_verified=True,
               firmware_revision=metadata['firmware_revision'], terminal_state=metadata['terminal_state'],
               acquisition={k: acquisition[k] for k in acquisition_keys}, completion=completion,
               deadline_input_cohorts=cohorts, profile=profile,
               poll_profile=acquisition['v46q_poll_profile'],
               start_pulse_timing=next(iter(metadata['v46k_timing_probe_events']), None))
    out['profile_weighted'] = {}
    for stage in profile['pulse_off']:
        a, b = profile['pulse_off'][stage], profile['pulse_on'][stage]
        count, total = a['count'] + b['count'], a['sum_us'] + b['sum_us']
        out['profile_weighted'][stage] = dict(count=count, sum_us=total,
            mean_us=total / count if count else 0, max_us=max(a['max_us'], b['max_us']))
    running = [r for r in samples if r['state_id'] == '3']
    angles = ['pitch_mekf_control_deg', 'pitch_mekf_abs_deg', 'pitch_madgwick_dynamic_abs_deg',
              'mekf_q_w', 'mekf_q_x', 'mekf_q_y', 'mekf_q_z']
    out.update(ordinary_rows=len(samples), rows_by_state=dict(Counter(r['state_id'] for r in samples)),
               running_rows=len(running), running_last_t_test_ms=max((int(r['t_test_ms']) for r in running), default=None),
               all_running_mekf_adopted=bool(running) and all(r['attitude_filter_adopted'] == '1' for r in running),
               all_running_attitude_fields_finite=bool(running) and all(math.isfinite(float(r[k])) for r in running for k in angles),
               running_row_sample_age_max_us=max((int(r['imu_sample_age_us']) for r in running), default=None),
               peak_event_count=len(metadata['energy_control_autonomous_peak_events']),
               zero_cross_event_count=len(metadata['energy_control_autonomous_zero_cross_events']))
    running_feet = [r for r in feet if r['state_id'] == '3']
    foot = dict(total_rows=len(feet), rows_by_state=dict(Counter(r['state_id'] for r in feet)),
                running_rows=len(running_feet), measurement_observation_validated=bool(running_feet),
                overflow=metadata['foot_observation']['overflow'],
                running_frames_per_second=len(running_feet) / (acquisition['duration_us'] / 1e6)
                if acquisition['duration_us'] else None)
    for side in ('right', 'left'):
        for key in ('zero_x', 'support_x'):
            foot[side + '_' + key] = metadata['foot_observation'][side + '_' + key]
        for prefix, group in [('', feet), ('running_', running_feet)]:
            valid = [r for r in group if normalized(side + '_valid', r[side + '_valid'])]
            base = side + '_' + prefix
            foot[base + 'valid'] = len(valid)
            foot[base + 'invalid_reasons'] = dict(Counter(r[side + '_reason'] for r in group
                if not normalized(side + '_valid', r[side + '_valid'])))
            outside = [r for r in valid if not normalized(side + '_in_range', r[side + '_in_range'])]
            foot[base + 'out_of_range'] = len(outside)
            if not prefix:
                foot[base + 'out_of_range_records'] = outside
            for key in ('x', 'deg'):
                values = [float(r[side + '_' + key]) for r in valid]
                foot[base + key + '_min'] = min(values, default=None)
                foot[base + key + '_max'] = max(values, default=None)
    foot['running_frame_interval_max_us'] = max((int(b['frame_us']) - int(a['frame_us'])
        for a, b in zip(running_feet, running_feet[1:])), default=None)
    out['foot'] = foot
    out['sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (source, foot_source)}
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('rwlog', type=Path)
    parser.add_argument('foot_csv', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    summary = summarize(args.rwlog, args.foot_csv)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print('Summary:', args.out)
    print(json.dumps({k: summary[k] for k in ('firmware_revision', 'terminal_state', 'completion', 'sha256')}, indent=2))
