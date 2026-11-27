"""Replay the bounded return-history option on recorded left aids at their original decision times."""
import argparse
from bisect import bisect_right
from collections import Counter
import csv
import json
import math
from pathlib import Path
import subprocess
import tempfile

import return_model


HERE = Path(__file__).resolve().parent
CERTIFIED_REASONS = {'applied', 'no_main_room', 'no_width_gain'}


def recorded_state(meta, event, row, arc):
    if not arc or not arc.get('arc_terminal_deg') or arc['arc_foot_source'] not in (1, 2):
        return None
    name = meta['walking_energy']['arc_aid_reason_names'][arc['arc_aid_reason']]
    if name not in CERTIFIED_REASONS or row['return_evidence'] != 1:
        return None
    now, age = row['time_ms'], row['return_foot_age_ms']
    # The integer frame age is saved with the original decision. Match that acquisition
    # exactly and independently require the frame to have reached the control core.
    matches = [f for f in meta['foot_frames'] if f['state_id'] == 3
               and f['timestamp_valid'] and f['measurement_time_us'] // 1000 == now - age
               and f.get('control_observed_us', 0) and
               (f['control_observed_us'] - f['frame_us'] + f['measurement_time_us']) <= now * 1000
               and all(f.get(k) for k in ('frame_valid', 'zero_ready', 'left_valid', 'right_valid',
                                           'left_control_range_ok', 'right_control_range_ok'))]
    if len(matches) != 1:
        return None
    frame = matches[0]
    return [row['peak_ms'], age, event['detector_angle_after_deg'], event['zero_cross_abs_rate_dps'],
            frame['left_deg'], frame['right_deg'], row['fore_aft_angle_deg'],
            row['return_deceleration_dps2'], arc['arc_terminal_deg'], row['target_j']]


def replay(folder, binary):
    meta = json.loads((folder / 'metadata.json').read_bytes())
    archive = meta['walking_energy']
    rows = archive['records']
    events = meta['energy_control_autonomous_zero_cross_events']
    if archive['truncated'] or len(rows) != archive['total'] or len(rows) + 1 != len(events):
        raise ValueError(f'{folder}: incomplete decision archive')
    arcs = {a['row_index']: a for a in archive.get('arc_aids', [])}
    with (folder / 'timeseries.csv').open(encoding='utf8', newline='') as f:
        imu = [(int(r['t_test_ms']), int(r['imu_sample_age_us'])) for r in csv.DictReader(f)
               if r['state_id'] == '3' and r['imu_sample_age_us']]
    imu_time = [t for t, _ in imu]
    stream, decisions = [], []
    counts = Counter()
    completed = []
    for index, (row, event) in enumerate(zip(rows, events[1:])):
        t, side = row['time_ms'], row['side']
        if event['previous_peak_time_ms'] != row['peak_ms'] or row['executed'] != event['output_executed']:
            raise ValueError(f'{folder}: event mismatch at {t}')
        if not row['return_event'] and row['executed']:
            stream.append((t, 2, 'W', t, side, event['pre_input_wheel_speed_rpm'], row['requested_ms']))
        if row['executed'] and row['output_end_ms']:
            completed.append((row, event))
        if not (row['return_event'] and side == -1):
            continue
        counts['left_return_decisions'] += 1
        counts['executed_left_aids'] += int(row['executed'])
        arc = arcs.get(index)
        st = recorded_state(meta, event, row, arc)
        if st is not None and not row['fore_aft_allowed']:
            st = None
        if st:
            left_peaks = [p for p in meta['energy_control_autonomous_peak_events'] if p['physical_peak_side']==-1
                          and p['peak_telemetry_capture_time_ms']<t and t-p['peak_time_ms']<=3000]
            last_left = left_peaks[-1]['peak_amplitude_deg'] if left_peaks else 0.
            counts['certified_with_matched_camera_frame'] += 1
            stream.append((t, 1, 'D', t, *st, event['startup_target_deg'], event['previous_peak_amplitude_deg'],
                           event['aligned_initial_current_mA'], event['predicted_current_goal_mA'],
                           event['predicted_current_rise_tau_s'], event['pre_input_wheel_speed_rpm'],
                           event['work_brake_cap_ms'], row['guarded_ms'], row['requested_ms'],
                           arc['arc_main_reserve_ms'], row['envelope_before_ms'], row['request_j'],last_left,
                           int(archive['arc_aid_reason_names'][arc['arc_aid_reason']]=='applied')))
            decisions.append({'decision_ms': t, 'recorded_width_ms': row['guarded_ms'],
                              'executed': row['executed'], 'recorded_request_j': row['request_j'],
                              'recorded_work_j': row['predicted_work_j'], 'arc_reason': arc['arc_aid_reason'],
                              'base_width_ms': arc['arc_base_guarded_ms'],
                              'recorded_inverse_ms': row['requested_ms'], 'main_reserve_ms': arc['arc_main_reserve_ms'],
                              'envelope_ms': row['envelope_before_ms'], 'trigger_deceleration_dps2': st[7]})
        if row['executed']:
            valid = bool(st and row['completion_valid'] and row['completion_reason'] == 1
                         and row['completion_work_valid'] and row['completion_width_supported']
                         and row['planned_duration_completed'])
            stream.append((row['output_end_ms'], 0, 'C', row['output_end_ms'], row['actual_elapsed_ms'], int(valid)))
    for peak in meta['energy_control_autonomous_peak_events']:
        when, t = peak['peak_telemetry_capture_time_ms'], peak['peak_time_ms']
        recent = [pair for pair in completed if pair[0]['output_end_ms'] < when and pair[0]['output_start_ms'] < t]
        row, event = recent[-1] if recent else ({}, {})
        i = bisect_right(imu_time, when) - 1
        fresh = i >= 0 and when - imu_time[i] <= 10 and imu[i][1] <= 10000
        # Acquisition health itself is not archived at the function call. Require a nearby
        # fresh IMU sample as well as the logged peak/command match and a completed main.
        powered = bool(peak['pending_command_matched'] and (peak['pending_q_command_mA_s'] or 0) > 0 and fresh)
        matched = bool(powered and row.get('assisted_cycle') and event.get('event_kind') == 'energy_control_zero_cross'
                       and row['completion_reason'] == 1 and row['completion_valid'] and row['completion_work_valid']
                       and row['side'] == peak['physical_peak_side'])
        full_pulse = all(row.get(k) for k in ('planned_duration_completed','completion_width_supported','charge_supported'))
        stream.append((when, 0, 'P', t, peak['physical_peak_side'], peak['peak_amplitude_deg'], int(powered),
                       row.get('peak_ms', 0), event.get('zero_cross_time_ms', 0), int(matched),
                       row.get('start_rate_dps') or 0, row.get('target_j') or 0, int(full_pulse)))
    payload = ''.join(' '.join(map(str, item[2:])) + '\n' for item in sorted(stream))
    outputs = subprocess.check_output([binary], input=payload, text=True).splitlines()
    if len(outputs) != len(decisions):
        raise ValueError(f'{folder}: selector response missing')
    for r, line in zip(decisions, outputs):
        (t, first, second, decel, request, max_ms, inv, old_guarded,
         new_inv, new_guarded, selected, final_work,
         source_zero, source_peak, source_speed, centre_j, centre_inverse, centre_width, centre_work) = line.split()
        if int(t) != r['decision_ms'] or int(inv) != r['recorded_inverse_ms']:
            raise ValueError(f'{folder}: baseline inverse did not reproduce at {t}, {inv} vs {r["recorded_inverse_ms"]}')
        # A certified extension (reason Applied) was limited by the remaining room
        # after reserving the ordinary main. Other rows used their base guard directly.
        expected = int(old_guarded)
        if r['arc_reason'] == 8:
            expected = max(r['base_width_ms'], min(expected, r['envelope_ms'] - r['main_reserve_ms']))
        if expected != r['recorded_width_ms']:
            raise ValueError(f'{folder}: inconsistent baseline wheel guard at {t}')
        r.update(reference_peaks_ms=[int(first), int(second)] if int(first) else [],
                 history_deceleration_dps2=float(decel), history_requested_j=float(request),
                 donor_duration_cap_ms=int(max_ms), baseline_wheel_ms=int(old_guarded),
                 candidate_inverse_ms=int(new_inv), candidate_wheel_ms=int(new_guarded), selected_ms=int(selected),
                 selected_predicted_work_j=float(final_work), crossing_zero_ms=int(source_zero),
                 crossing_peak_ms=int(source_peak), crossing_source_speed_dps=float(source_speed),
                 crossing_request_j=float(centre_j), crossing_candidate_ms=int(centre_inverse),
                 crossing_selected_ms=int(centre_width), crossing_predicted_work_j=float(centre_work))
    differences = [r for r in decisions if r['recorded_width_ms'] != r['selected_ms']]
    crossing_changes = [r for r in decisions if r['crossing_selected_ms'] != r['selected_ms']]
    return {'run': folder.name, 'has_arc_audit': 'arc_aids' in archive, 'counts': dict(counts),
            'decisions': decisions, 'first_width_difference_ms': differences[0]['decision_ms'] if differences else None,
            'crossing_width_change_count': len(crossing_changes)}


def _observed_feet(frames, start, until):
    result = {}
    for side in ('left', 'right'):
        samples = {}
        for f in frames:
            t = f.get('measurement_time_us')
            angle = f.get(side + '_deg')
            if (t is not None and start * 1000 <= t <= until * 1000 and f.get('state_id') == 3
                    and all(f.get(k) for k in ('timestamp_valid', 'frame_valid', 'zero_ready', side + '_valid'))
                    and f.get(side + '_control_range_ok', f.get(side + '_in_range')) and
                    isinstance(angle, (int, float)) and math.isfinite(angle)):
                samples[t] = angle
        values = [samples[t] for t in sorted(samples)]
        result[side] = {'images': len(values), 'first_deg': values[0] if values else None,
                        'last_deg': values[-1] if values else None,
                        'first_ms': min(samples)/1000 if values else None,
                        'last_ms': max(samples)/1000 if values else None,
                        'excursion_deg': max(values)-min(values) if len(values) >= 2 else None}
    return result


def _observed_motion(folder, meta, row, terminal, archive_complete):
    start, peak = row['output_start_ms'], row['peak_ms']
    end = row.get('output_end_ms') or None
    if not isinstance(terminal, (int, float)) or not math.isfinite(terminal) or terminal >= 0:
        terminal = None
    next_peak = min((p for p in meta.get('energy_control_autonomous_peak_events', [])
                     if p['peak_time_ms'] > peak), key=lambda p: p['peak_time_ms'], default=None)
    until = min(peak + 1000, next_peak['peak_time_ms']) if next_peak else peak + 1000
    left_peak = next_peak if next_peak and next_peak['physical_peak_side'] == -1 and start < next_peak['peak_time_ms'] <= until else None
    zeros = [e for e in meta.get('energy_control_autonomous_zero_cross_events', [])
             if e['event_kind'] == 'energy_control_zero_cross' and e['physical_next_peak_side'] == -1
             and e['previous_peak_time_ms'] == peak and start < e['zero_cross_time_ms'] <= until]
    later = [{'start_ms': r['output_start_ms'], 'side': r['side'], 'width_ms': r['guarded_ms']}
             for r in meta['walking_energy']['records'] if r.get('executed') is True
             and start < r['output_start_ms'] <= until]
    out = {'decision_ms': row['time_ms'], 'departure_peak_ms': peak,
           'guarded_ms': row['guarded_ms'], 'start_ms': start, 'end_ms': end,
           'elapsed_ms': row.get('actual_elapsed_ms') if end else None,
           'completed_as_planned': row.get('planned_duration_completed') if end else None,
           'completion_reason': row.get('completion_reason') if end else None, 'terminal_deg': terminal,
           'ordinary_left_zero_ms': min((e['zero_cross_time_ms'] for e in zeros), default=None),
           'left_peak_ms': left_peak['peak_time_ms'] if left_peak else None,
           'left_peak_deg': -left_peak['peak_amplitude_deg'] if left_peak else None,
           'subsequent_commands_observed': later, 'command_archive_complete': archive_complete,
           'mechanical_zero': None, 'receiving_end': None, 'follow_until_ms': until}
    frames = meta.get('foot_frames', [])
    if frames and (folder / 'timeseries.csv').is_file() and meta.get('mekf_gyro_y_scale') is not None:
        from receiving_carry import CarryReplay
        carry = CarryReplay(folder)
        if carry.clock is not None:
            for key, target in [('mechanical_zero', 0.), ('receiving_end', terminal)]:
                if target is None:
                    continue
                observed = carry.follow({'peak_ms': peak, 'decision_ms': start, 'terminal_deg': target})
                status = observed['status']
                out[key] = {'status': status, 'reached': True if status == 'crossed' else False if status == 'reversed' else None,
                            'event_ms': observed.get('end_ms') if status in ('crossed', 'reversed') else None,
                            'crossing_speed_dps': observed.get('inward_speed_dps') if status == 'crossed' else None}
    # Distinguish the later main from the pre-centre aid even when both occurred in the same half-cycle.
    outcome = out['receiving_end'] or out['mechanical_zero']
    foot_end = outcome['event_ms'] if outcome and outcome['status'] == 'reversed' else until
    out['foot_window_end_ms'] = foot_end
    out['observed_foot_angles'] = _observed_feet(frames, start, foot_end)
    return out


def _first_intervention(folder, meta, row, arc, archive_complete):
    base = arc['arc_history_base_ms']
    out = _observed_motion(folder, meta, row, arc.get('arc_terminal_deg'), archive_complete)
    return dict(out, first_in_stored_archive=True, base_ms=base, selected_ms=arc['arc_history_selected_ms'],
                reference_peaks_ms=[arc['arc_history_first_peak_ms'], arc['arc_history_second_peak_ms']],
                elapsed_exceeded_base=out['elapsed_ms'] > base if out['elapsed_ms'] is not None else None)


def observed_prediction(folder):
    """Report predictions, changes of guarded width and actual commands from a new run."""
    folder = Path(folder)
    meta = json.loads((folder / 'metadata.json').read_bytes())
    profile = meta.get('walking_profile')
    carry = profile in ('left_transfer_carry_v1', 'left_transfer_carry_v2', 'left_transfer_carry_v3', 'left_transfer_carry_v4', 'left_transfer_carry_v5')
    if profile != 'left_return_prediction_v1' and not carry:
        return None
    archive = meta.get('walking_energy') or {}
    rows = archive.get('records') or []
    details = {a['row_index']: a for a in archive.get('arc_aids', [])}
    left_aids = [(row, details.get(i, {})) for i, row in enumerate(rows)
                 if row.get('side') == -1 and row.get('return_event')]
    expected = ('arc_predicted_terminal_deg', 'arc_predicted_transfer_ms', 'arc_crossing_base_ms',
                'arc_crossing_selected_ms', 'arc_crossing_ref_zero_ms')
    if carry:
        expected += ('arc_approach_transfer_ms', 'arc_approach_terminal_deg', 'arc_approach_request_j',
                     'arc_crossing_request_j')
    if profile in ('left_transfer_carry_v2', 'left_transfer_carry_v3', 'left_transfer_carry_v4', 'left_transfer_carry_v5'):
        expected += ('arc_passage_main_ms', 'arc_passage_reserve_ms', 'arc_passage_main_j', 'arc_passage_zero_dps')
    if profile in ('left_transfer_carry_v3', 'left_transfer_carry_v4', 'left_transfer_carry_v5'):
        expected += ('arc_post_cap_ms', 'arc_post_increment_j', 'arc_post_elapsed_increment_j')
    if profile in ('left_transfer_carry_v4', 'left_transfer_carry_v5'):
        expected += ('arc_priority_cap_ms', 'arc_priority_base_ms', 'arc_priority_base_j', 'arc_priority_zero_j')
    complete = (archive.get('archive_available') is True and archive.get('truncated') is False and
                archive.get('stored') == archive.get('total') == len(rows) and
                isinstance(archive.get('arc_aids'), list) and all(k in arc for _, arc in left_aids for k in expected))
    events = []
    for row, arc in left_aids:
        base, selected = (arc.get(k) for k in ('arc_crossing_base_ms', 'arc_crossing_selected_ms'))
        predicted = row.get('return_evidence') == 6
        before_transfer = carry and arc.get('arc_approach_transfer_ms') is not None
        extended = type(base) is int and type(selected) is int and selected > base > 0
        post_aid = profile in ('left_transfer_carry_v3', 'left_transfer_carry_v4', 'left_transfer_carry_v5') and row.get('return_evidence') in (7, 8)
        priority = profile in ('left_transfer_carry_v4', 'left_transfer_carry_v5') and (arc.get('arc_priority_cap_ms') or 0) > 0
        if not (predicted or extended or before_transfer or post_aid or priority):
            continue
        if before_transfer:
            terminal, transfer = arc.get('arc_approach_terminal_deg'), arc['arc_approach_transfer_ms']
        else:
            terminal = arc.get('arc_predicted_terminal_deg') if predicted or row.get('return_evidence') == 8 else arc.get('arc_terminal_deg')
            transfer = arc.get('arc_predicted_transfer_ms')
        executed = row.get('executed') is True and bool(row.get('output_start_ms'))
        guarded = row.get('guarded_ms')
        executed_extension = bool(executed and extended and type(guarded) is int and base < guarded <= selected)
        elapsed = row.get('actual_elapsed_ms') if row.get('output_end_ms') else None
        events.append(dict(decision_ms=row['time_ms'], predicted_support=predicted, width_extended=extended,
                           forecast_before_transfer=before_transfer, terminal_deg=terminal, transfer_in_ms=transfer,
                           arc_work_j=arc.get('arc_approach_request_j'), combined_work_j=arc.get('arc_crossing_request_j'),
                           observed_deficit_j=row.get('return_work_j'),
                           requested_j=row.get('request_j'),
                           guarded_ms=guarded, main_reserve_ms=arc.get('arc_main_reserve_ms'),
                           reference_zero_ms=arc.get('arc_crossing_ref_zero_ms') or None, base_ms=base, selected_ms=selected,
                           executed_extension=executed_extension,
                           elapsed_exceeded_base=elapsed>base if executed_extension and elapsed is not None else None,
                           executed=executed, motion=None))
        if profile in ('left_transfer_carry_v2', 'left_transfer_carry_v3', 'left_transfer_carry_v4', 'left_transfer_carry_v5') and extended and (arc.get('arc_passage_reserve_ms') or 0) > 0:
            events[-1]['passage'] = dict(
                reference_speed_dps=arc.get('arc_crossing_ref_dps'),
                reference_main_ms=arc.get('arc_passage_main_ms'),
                previous_reserve_ms=arc.get('arc_main_reserve_ms'),
                selected_reserve_ms=arc['arc_passage_reserve_ms'],
                main_work_j=arc.get('arc_passage_main_j'),
                predicted_zero_speed_dps=arc.get('arc_passage_zero_dps'))
        if post_aid:
            previous = next((earlier for earlier, _ in reversed(left_aids)
                             if earlier.get('executed') and earlier.get('return_evidence') not in (7, 8)
                             and earlier.get('peak_ms') == row['peak_ms']
                             and 0 < (earlier.get('output_end_ms') or 0) <= row['time_ms']), None)
            events[-1]['post_aid'] = dict(first_end_ms=previous['output_end_ms'] if previous else None,
                first_elapsed_ms=previous.get('actual_elapsed_ms') if previous else None,
                remaining_before_ms=row.get('envelope_before_ms'), remaining_after_ms=row.get('envelope_after_ms'),
                request_j=row.get('request_j'), gross_elapsed_model_work_j=row.get('elapsed_work_pred_j'),
                cap_ms=arc.get('arc_post_cap_ms'), planned_increment_j=arc.get('arc_post_increment_j'),
                release_deceleration_dps2=arc.get('arc_post_release_decel_dps2'),
                elapsed_increment_j=arc.get('arc_post_elapsed_increment_j'),
                planned_width_ms=guarded, actual_elapsed_ms=elapsed, coast_ready_cut=row.get('completion_reason') == 7)
        if priority:
            events[-1]['center_priority'] = dict(
                before_ms=arc.get('arc_priority_base_ms'), cap_ms=arc['arc_priority_cap_ms'],
                original_work_j=arc.get('arc_priority_base_j'), center_work_j=arc.get('arc_priority_zero_j'),
                final_ms=guarded, actual_elapsed_ms=elapsed)
        if executed:
            events[-1]['motion'] = _observed_motion(folder, meta, row, terminal, complete)
    return dict(archive_complete=complete, events=events)


def observed_intervention(folder):
    """Read actual firmware decisions. A partial archive supplies lower bounds, never totals."""
    folder = Path(folder)
    meta = json.loads((folder / 'metadata.json').read_bytes())
    profile = meta.get('walking_profile')
    archive = meta.get('walking_energy') or {}
    rows = archive.get('records') or []
    stats = dict(evaluated=0, matching_donors=0, width_increased=0, executed_increased=0)
    empty = dict.fromkeys(stats)
    out = {'profile': profile, **empty, 'observed_minimum': None, 'first_executed_change': None}
    if profile not in ('left_return_history_v1', 'left_return_prediction_v1', 'left_transfer_carry_v1', 'left_transfer_carry_v2', 'left_transfer_carry_v3', 'left_transfer_carry_v4', 'left_transfer_carry_v5'):
        return dict(out, status='not_selected' if profile else 'history_not_logged')
    complete = (archive.get('archive_available') is True and archive.get('truncated') is False
                and archive.get('stored') == archive.get('total') == len(rows))
    raw_details = archive.get('arc_aids')
    details = {a['row_index']: a for a in raw_details or []}
    complete = complete and isinstance(raw_details, list)
    usable, first = 0, None
    for i, row in enumerate(rows):
        if not row.get('return_event') or row.get('side') != -1:
            continue
        arc = details.get(i, {})
        keys = ('arc_history_first_peak_ms', 'arc_history_second_peak_ms', 'arc_history_base_ms', 'arc_history_selected_ms')
        if any(type(arc.get(k)) is not int for k in keys):
            complete = False
            continue
        usable += 1
        p1, p2, base, selected = (arc[k] for k in keys)
        if base <= 0:
            continue
        stats['evaluated'] += 1
        stats['matching_donors'] += int(0 < p1 < p2 < row['peak_ms'])
        if selected <= base:
            continue
        stats['width_increased'] += 1
        if type(row.get('executed')) is not bool or (row['executed'] and
                (type(row.get('output_start_ms')) is not int or not row['output_start_ms'] or
                 type(row.get('guarded_ms')) is not int)):
            complete = False
            continue
        if row['executed'] and base < row['guarded_ms'] <= selected:
            stats['executed_increased'] += 1
            if first is None:
                first = (row, arc)
    if first is not None:
        out['first_executed_change'] = _first_intervention(folder, meta, *first, complete)
    if complete:
        return dict(out, status='complete', **stats)
    return dict(out, status='incomplete' if usable or archive else 'history_not_logged', observed_minimum=stats if usable else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted_root', type=Path)
    parser.add_argument('--runs', type=int, nargs='+', default=[*return_model.PRIOR,*return_model.RECENT])
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        binary = Path(tmp) / 'return_history_selector.exe'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-DWALKING_RETURN_HISTORY=1',
                        '-I', HERE.parents[1] / 'src', HERE / 'return_history_selector.cpp', '-o', binary], check=True)
        for run in args.runs:
            print(json.dumps(replay(args.converted_root / str(run), binary), allow_nan=False))


if __name__ == '__main__':
    main()
