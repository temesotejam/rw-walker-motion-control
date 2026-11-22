"""Measure left-aid charge delivered up to the first center crossing or reversal."""
from bisect import bisect_left, bisect_right
from pathlib import Path
import sys

import receiving_carry

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-22-return-trial'))
import pulse_release


def bracketed_charge(points, times, start, end, max_gap_ms=25, next_command_ms=float('inf')):
    """Signed mA s. Each segment must have real readback on both sides."""
    if not times or start >= end or start < times[0] or end > times[-1]:
        return None
    left = bisect_right(times, start) - 1
    right = bisect_left(times, end)
    window = points[left:right + 1]
    if times[right] > next_command_ms or any(b[0] - a[0] > max_gap_ms for a, b in zip(window, window[1:])):
        return None
    return pulse_release.charge(window, start, end)


def command_window(commands, event):
    """Match the actual driver command, its first zero, and the next nonzero."""
    expected = event['output_start_ms']
    matches = [(abs(t - expected), i) for i, (t, value) in enumerate(commands)
               if expected - 8 <= t <= expected + 8 and value < 0 and (i == 0 or commands[i-1][1] != value)]
    if not matches:
        return None
    _, index = min(matches)
    start, cmd = commands[index]
    off_index = next((i for i in range(index + 1, len(commands)) if commands[i][1] != cmd), None)
    if off_index is None:
        return None
    end, released = commands[off_index]
    if released != 0 or abs(end - event['output_end_ms']) > 8:
        return None
    next_on = next((t for t, value in commands[off_index + 1:] if value), float('inf'))
    return start, end, next_on


def first_receiving_state(rows, until):
    """Earliest CAD receiving state, indexed by the control task's snapshot time."""
    for state_ms, ready, row in sorted(rows, key=lambda p: p[0]):
        if state_ms > until:
            break
        if row.get('side') == '-1' and row.get('phase') == '2' and row.get('anchor') in ('1', '2'):
            return state_ms, ready, row
    return None


def source_image_times(packet_id, state_ms, periodic, frames, epoch):
    """Follow the publication ordinal, then image sequence, separately for each foot."""
    result = dict(packet_id=packet_id, publish_ms=None, left=None, right=None)
    samples = periodic.get('samples', [])
    if not 1 <= packet_id <= len(samples):
        return result
    # count increments once per publication even when a scheduled slot was skipped.
    packet = dict(zip(periodic['columns'], samples[packet_id-1]))
    published = packet.get('publish_us')
    if published is None or not 0 <= published <= round(state_ms * 1000):
        return result
    result['publish_ms'] = published / 1000
    for side in ('left', 'right'):
        sequence = packet.get(side + '_sequence')
        frame = frames.get(sequence)
        if not sequence or not frame or not frame.get('timestamp_valid'):
            continue
        image, received = frame.get('frame_us'), frame.get('control_observed_us')
        if image and received and epoch <= image <= received <= epoch + published:
            result[side] = dict(sequence=sequence, image_ms=(image-epoch)/1000,
                                control_received_ms=(received-epoch)/1000)
    return result


def evaluate_trial(folder, motion=None):
    motion = motion if motion is not None else receiving_carry.CarryReplay(folder)
    events = [e for e in motion.archive['records'] if e['side'] == -1 and e['executed'] and e['return_event']]
    if not events:
        return []
    if motion.clock is None:
        return [dict(peak_ms=e['peak_ms'], status='missing_measurement_epoch') for e in events]
    meta, traces = pulse_release.trace(folder)
    frames = {}
    for frame in meta.get('foot_frames', ()):
        if frame.get('state_id') == 3:
            seq = frame['sequence']
            frames[seq] = frame if seq not in frames else None
    points = traces['current']
    times = [t for t, _ in points]
    results = []
    for e in events:
        result = {'peak_ms': e['peak_ms'], 'start_ms': e['output_start_ms']}
        results.append(result)
        window = command_window(traces['command'], e)
        if window is None:
            result['status'] = 'missing_driver_edges'
            continue
        on, off, next_on = window
        actual = motion.follow({'peak_ms': e['peak_ms'], 'decision_ms': on, 'terminal_deg': 0.})
        result.update(status=actual['status'], driver_on_ms=on, driver_off_ms=off)
        if actual['status'] not in ('crossed', 'reversed'):
            continue
        end = actual['end_ms']
        result.update(end_ms=end, center_speed_dps=actual['inward_speed_dps'])
        if end < off or next_on < end:
            result['charge_status'] = 'event_during_pulse' if end < off else 'another_command_before_event'
            continue
        on_charge = bracketed_charge(points, times, on, off, next_command_ms=next_on)
        tail_charge = bracketed_charge(points, times, off, end, next_command_ms=next_on)
        first_post = bisect_right(times, off)
        if first_post < len(times) and times[first_post] <= min(end, off + 25, next_on):
            result.update(first_release_sample_ms=times[first_post], first_release_current_ma=points[first_post][1])
        result.update(charge_status='complete' if None not in (on_charge, tail_charge) else 'readback_gap',
                      on_charge_mas=-on_charge if on_charge is not None else None,
                      tail_charge_mas=-tail_charge if tail_charge is not None else None,
                      modeled_on_charge_mas=e.get('elapsed_q_pred_mas') if e.get('completion_valid') else None)
        receiving = first_receiving_state(motion.by_peak.get(e['peak_ms'], ()), end)
        if receiving:
            state_ms, ready, row = receiving
            pre = bracketed_charge(points, times, on, state_ms, next_command_ms=next_on) if state_ms > on else 0.
            q = -pre if pre is not None else None
            result.update(receiving_state_ms=state_ms, receiving_ready_ms=ready, receiving_anchor=int(row['anchor']),
                          charge_by_receiving_state_mas=q,
                          receiving_foot_sources=source_image_times(int(row['packet']), state_ms,
                              meta.get('foot_periodic', {}), frames, motion.clock),
                          # Older JSON consumers call the CAD state snapshot "capture".
                          receiving_capture_ms=state_ms, charge_by_receiving_capture_mas=q)
    return results
