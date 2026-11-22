"""Join a leftward replay decision to the receiving sole's CAD endpoint and measured motion."""
from bisect import bisect_right
from collections import Counter
import json
import math
from pathlib import Path
import sys

import return_model

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import score_observed_transfers as audit

NO_BUDGET = 1 << 8  # SupportTransferAudit allows a passive forecast with no pulse budget.

def latest_completed(plans, when_ms):
    # Completion can arrive after a newer capture. Never resurrect an older valid
    # plan if a newer capture has completed with a gate or invalid geometry.
    return max((p for p in plans if p[1] < when_ms),
               key=lambda p: (p[0], int(p[2]['packet'])), default=None)


class CarryReplay:
    def __init__(self, folder):
        meta = json.loads((folder / 'metadata.json').read_text(encoding='utf8'))
        self.archive = meta['walking_energy']
        self.clock = audit.measurement_epoch(meta)
        self.gyro = audit.gyro_samples(meta, return_model.read_rows(folder, 'timeseries'), self.clock)
        self.peaks = sorted(p['peak_time_ms'] for p in meta['energy_control_autonomous_peak_events'])
        self.confirmations = sorted((p['peak_telemetry_capture_time_ms'], p['peak_time_ms'], p['physical_peak_side'])
                                    for p in meta['energy_control_autonomous_peak_events'])
        self.by_peak = {}
        self.reasons = meta.get('support_transfer_shadow', {}).get('reason_names', [])
        if self.clock is not None and (folder / 'support_transfer_shadow.csv').is_file():
            for p in return_model.read_rows(folder, 'support_transfer_shadow'):
                if not p.get('capture_us32') or not p.get('ready_after_us'):
                    continue
                captured = audit.capture_us(p, self.clock) / 1000
                ready = captured + int(p['ready_after_us']) / 1000
                self.by_peak.setdefault(int(p['peak_ms']), []).append((captured, ready, p))

    def at(self, when):
        peak = next((p for p in reversed(self.confirmations) if p[0] <= when), None)
        if peak is None or peak[2] != 1:
            return {'status': 'no_confirmed_right_peak', 'terminal_deg': None}
        return self.endpoint(peak[1], when)

    def endpoint(self, peak, when):
        """Information available at the decision. All times are relative to measurement start."""
        out = {'peak_ms': peak, 'decision_ms': when, 'terminal_deg': None}
        previous = latest_completed(self.by_peak.get(peak, ()), when)
        if previous is None:
            return dict(out, status='no_completed_geometry')
        captured, ready, row = previous
        if not all(row.get(k) for k in ('phase', 'anchor')):
            return dict(out, status='receiving_endpoint_not_logged')
        phase, anchor, gate = (int(row[k]) for k in ('phase', 'anchor', 'gate_mask'))
        reason = int(row['reason'])
        out.update(captured_ms=captured, ready_ms=ready, phase=phase, anchor=anchor,
                   gate_mask=gate, baseline_flags=int(row['baseline_flags']), cap_ms=int(row['cap_ms']),
                   solver_reason=self.reasons[reason] if 0 <= reason < len(self.reasons) else reason)
        if int(row['side']) != -1 or phase != 2 or anchor not in (1, 2):
            return dict(out, status='no_left_receiving_anchor')
        target = audit.numeric(row['terminal_deg'])
        if target is None or target >= 0:
            return dict(out, status='invalid_left_endpoint')
        # The end angle is latched by the firmware for this peak. The last energy
        # margin describes its capture instant only; never re-label it as a new prediction.
        raw_margin = audit.numeric(row['coast_margin_j'])
        return dict(out, status='receiving_anchor', terminal_deg=target,
                    captured_coast_margin_mj=1000 * raw_margin if raw_margin is not None and gate in (0, NO_BUDGET) else None)

    def follow(self, plan):
        """Score the original recorded path separately from every hypothetical pulse."""
        if plan['terminal_deg'] is None:
            return None
        when = plan['decision_ms']
        i = bisect_right(self.peaks, plan['peak_ms'])
        until = min(self.peaks[i] if i < len(self.peaks) else math.inf, plan['peak_ms'] + 1000)
        times, rates, angles = self.gyro
        start = bisect_right(times, when) - 1
        stop = bisect_right(times, until)
        if start >= 0:
            stop = next((j for j in range(start, stop) if angles[j] is None), stop)
        if start < 0 or start + 1 >= stop or times[start + 1] - times[start] > 10:
            return {'status': 'censored', 'coast_scored': False, 'recorded_command': 'unknown_interval'}
        # Interpolate only to label the actual state at the query time. The prediction above
        # has already been fixed using the plans available at that time.
        f = (when - times[start]) / (times[start+1] - times[start])
        at_start = angles[start] + f * (angles[start+1] - angles[start])
        initial_v = -(rates[start] + f * (rates[start+1] - rates[start]))
        target = plan['terminal_deg']
        if at_start <= target or initial_v <= 0:
            return {'status': 'not_approaching_endpoint', 'coast_scored': False, 'recorded_command': 'unknown_interval'}
        t = [when, *times[start+1:stop]]
        x = [at_start - target, *(angle - target for angle in angles[start+1:stop])]
        v = [initial_v, *(-a for a in rates[start+1:stop])]
        status, _, end, speed, confirmed = return_model.first_outcome(
            t, x, v, 0, len(t), None, max_gap_ms=10, inward_established=True)
        exposure = audit.command_overlap(self.archive, when, confirmed)
        carry = None
        if status == 'crossed':
            carry = bool(speed >= return_model.CARRY_SPEED_DPS)
        elif status == 'reversed':
            carry = False
        return {'status': status, 'end_ms': end, 'confirmed_ms': confirmed,
                'inward_speed_dps': speed if status == 'crossed' else None, 'at_least_5_dps': carry,
                'recorded_command': exposure, 'coast_scored': carry is not None and exposure == 'no_recorded_command'}


def first_receiving_outcomes(dataset):
    """Measured speed at the CAD endpoint from the first completed receiving plan per right peak."""
    outcomes = []
    for _, peak, side in dataset.confirmations:
        if side != 1:
            continue
        plans = dataset.by_peak.get(peak, ())
        first = min((p for p in plans if p[2].get('side') == '-1' and p[2].get('phase') == '2'
                     and p[2].get('anchor') in ('1', '2')), key=lambda p: p[1], default=None)
        if first is None:
            outcomes.append(dict(peak_ms=peak, status='no_receiving_anchor'))
            continue
        captured, ready, row = first
        target = audit.numeric(row.get('terminal_deg'))
        if target is None or target >= 0:
            outcomes.append(dict(peak_ms=peak, status='invalid_left_endpoint'))
            continue
        plan = dict(peak_ms=peak, captured_ms=captured, decision_ms=ready, terminal_deg=target)
        outcomes.append(dict(plan, **dataset.follow(plan)))
    return outcomes


def evaluate_trial(folder, dataset=None):
    dataset = dataset if dataset is not None else CarryReplay(folder)
    states, _ = return_model.load_sections(folder, sections=(4.5, 3., 2.))
    events = []
    for entry in states:
        state = entry.state
        plan = dataset.endpoint(state.peak_ms, state.time_ms)
        events.append({'run': state.run, 'section_deg': state.section, 'plan': plan,
                       'recorded_zero_outcome': entry.outcome, 'observed_receiving_end': dataset.follow(plan)})
    return events


def summary(events):
    stats = {}
    for angle in (4.5, 3., 2.):
        rows = [e for e in events if e['section_deg'] == angle]
        observed = [e for e in rows if e['observed_receiving_end'] is not None]
        coast = [e for e in observed if e['observed_receiving_end']['coast_scored']]
        comparable = [e for e in coast if e['plan']['captured_coast_margin_mj'] is not None]
        stats[str(angle)] = {
            'decisions': len(rows), 'with_receiving_end': len(observed), 'coast_evaluable': len(coast),
            'coast_carries': sum(e['observed_receiving_end']['at_least_5_dps'] for e in coast),
            'coast_crossed_zero_without_carry': sum(e['recorded_zero_outcome'] == 'crossed'
                and not e['observed_receiving_end']['at_least_5_dps'] for e in coast),
            'recorded_command_counts': dict(Counter(e['observed_receiving_end']['recorded_command'] for e in observed)),
            'energy_margin_comparable': len(comparable),
            'false_positive_margin': sum(e['plan']['captured_coast_margin_mj'] > 0
                and not e['observed_receiving_end']['at_least_5_dps'] for e in comparable),
            'false_deficit': sum(e['plan']['captured_coast_margin_mj'] <= 0
                and e['observed_receiving_end']['at_least_5_dps'] for e in comparable)}
    return stats


def run(root):
    result = {}
    for split, ids in (('prior', return_model.PRIOR), ('recent', return_model.RECENT)):
        rows = [e for run_id in ids for e in evaluate_trial(root / str(run_id))]
        result[split] = {'summary': summary(rows), 'events': rows}
    return result
