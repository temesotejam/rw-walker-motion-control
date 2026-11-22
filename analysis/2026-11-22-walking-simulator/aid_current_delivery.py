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


def first_receiving_capture(rows, until):
    """Earliest recorded CAD receiving anchor for this leftward return."""
    for captured, ready, row in sorted(rows, key=lambda p: p[0]):
        if captured > until:
            break
        if row.get('side') == '-1' and row.get('phase') == '2' and row.get('anchor') in ('1', '2'):
            return captured, ready, int(row['anchor'])
    return None


def evaluate_trial(folder, motion=None):
    motion = motion if motion is not None else receiving_carry.CarryReplay(folder)
    events = [e for e in motion.archive['records'] if e['side'] == -1 and e['executed'] and e['return_event']]
    if not events:
        return []
    if motion.clock is None:
        return [dict(peak_ms=e['peak_ms'], status='missing_measurement_epoch') for e in events]
    _, traces = pulse_release.trace(folder)
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
        receiving = first_receiving_capture(motion.by_peak.get(e['peak_ms'], ()), end)
        if receiving:
            captured, ready, anchor = receiving
            pre = bracketed_charge(points, times, on, captured, next_command_ms=next_on) if captured > on else 0.
            result.update(receiving_capture_ms=captured, receiving_ready_ms=ready, receiving_anchor=anchor,
                          charge_by_receiving_capture_mas=0. - pre if pre is not None else None)
    return results
