"""Shadow comparison at a logged v19 return-aid decision. All energies are model J.

The first stage pays the logged local reachability demand. The second uses
previous ordinary ZERO states as scenarios for reaching the opposite peak.
The future ZERO is conditional: reserve is software time, never promised work.
"""

import math
from typing import NamedTuple

from event_foot_features import MAX_FRAME_AGE_US


# src/models/lateral_energy_control.h, coefficients(side). Q in mA s, omega in rad/s.
WORK_MODEL = {
    -1: (.03889674250, .2578004825, 35.7375, 91.4938, 12.83679),
    1: (.01259279858, .5522071518, 17.9508, 92.2064, 13.27138),
}
ENVELOPE_MS = 80
MAX_HISTORY_AGE_MS = 3000  # v19 same-side peak memory, provisional for scenarios
MAX_HISTORY_COUNT = 3
OUTWARD_RATE_DPS = 20  # existing fore/aft motion scale; this combination is experimental


def decision_rows(meta):
    """Join decision-time model inputs to audits without decoding pulse completion."""
    audit = meta['walking_energy']
    if audit.get('branch') != 24 or audit.get('revision') != 'walking_energy_admission_v19':
        raise ValueError('expected v19 branch 24 walking-energy records')
    records = audit['records']
    traces = [e for e in meta['energy_control_autonomous_zero_cross_events']
              if e.get('work_control_branch') == 24]
    if audit.get('truncated') or len(records) != audit.get('total') or len(records) != len(traces):
        raise ValueError('incomplete energy decision/trace pairing')
    result = []
    for r, e in zip(records, traces):
        kind = ('inward_stall_rescue' if r['rescue'] else
                'angle_rate_early_trial' if r['optional'] else 'energy_control_zero_cross')
        if (kind != e['event_kind'] or r['peak_ms'] != e['previous_peak_time_ms'] or r['side'] != e['physical_next_peak_side']
                or not 0 <= r['time_ms'] - e['zero_cross_time_ms'] <= 5):
            raise ValueError(f"unmatched decision at {r['time_ms']} ms")
        result.append(dict(r, event_kind=e['event_kind'], decision_i0_ma=e['aligned_initial_current_mA'],
                           decision_goal_ma=e['predicted_current_goal_mA'],
                           decision_tau_s=e['predicted_current_rise_tau_s'],
                           decision_rate_dps=e['zero_cross_abs_rate_dps']))
    return result


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _valid_model(event):
    keys = ('decision_i0_ma', 'decision_goal_ma', 'decision_tau_s', 'decision_rate_dps')
    return (event.get('side') in WORK_MODEL and all(_finite(event.get(k)) for k in keys)
            and event['decision_goal_ma'] > 0 and event['decision_tau_s'] > 0
            and event['decision_rate_dps'] >= 0)


def _work(event, width):
    """Mirror v19 positive excitation; negative signed charge is inadmissible."""
    if not width:
        return 0., 0.
    seconds = width * .001
    rise = event['decision_tau_s']
    residual = -rise * math.expm1(-seconds / rise)
    charge = event['decision_goal_ma'] * (seconds - residual) + event['decision_i0_ma'] * residual
    if charge < 0:
        return None, charge
    linear, quadratic = WORK_MODEL[event['side']][:2]
    q = .001 * charge
    return linear * math.radians(event['decision_rate_dps']) * q + quadratic * q * q, charge


def _history(event, prior_main_pulses):
    rows = []
    for r in prior_main_pulses:
        if (r.get('event_kind') != 'energy_control_zero_cross' or r.get('valid') is not True
                or r.get('assisted_cycle') is not False or r.get('side') != event['side']
                or not _finite(r.get('peak_ms')) or not 0 < r['peak_ms'] < event['peak_ms']
                or not _finite(r.get('time_ms'))
                or not 0 < event['time_ms'] - r['time_ms'] <= MAX_HISTORY_AGE_MS
                or r.get('target_j') != event['target_j'] or not _finite(r.get('request_j'))
                or r['request_j'] < 0 or type(r.get('guarded_ms')) is not int
                or not 0 <= r['guarded_ms'] <= ENVELOPE_MS):
            continue
        if r.get('coast') is True and r['request_j'] == 0 and r['guarded_ms'] == 0:
            rows.append(r)
        elif (r.get('executed') is True and r.get('completion_valid') is True
              and _finite(r.get('output_end_ms')) and r['time_ms'] <= r['output_end_ms'] < event['time_ms']
              and _valid_model(r)):
            rows.append(r)
    # One unassisted main per peak is the v19 ledger invariant.
    rows.sort(key=lambda r: r['time_ms'], reverse=True)
    distinct = {}
    for r in rows:
        distinct.setdefault(r['peak_ms'], r)
    return list(distinct.values())[:MAX_HISTORY_COUNT]


def _main_scenario(row, remaining):
    width = min(remaining, row['guarded_ms'])
    work, charge = _work(row, width)
    # A short command can still carry reverse residual current. Zero remains
    # admissible; the scenario can credit at most zero added work in that case.
    missing = max(0., row['request_j'] - (work or 0.))
    fraction = missing / row['request_j'] if row['request_j'] else 0.
    low, high, max_charge = WORK_MODEL[row['side']][2:]
    supported = (not width or (work is not None and low <= row['decision_rate_dps'] <= high
                               and charge <= max_charge))
    return fraction, missing, supported


class Allocation(NamedTuple):
    width_ms: int
    work_j: float
    charge_mas: float
    aid_unmet_j: float
    aid_fraction_unmet: float
    main_fraction_unmet: float
    main_unmet_j: float
    main_work_supported: bool

    def balance_score(self):
        return (max(self.aid_fraction_unmet, self.main_fraction_unmet),
                self.aid_fraction_unmet + self.main_fraction_unmet, self.width_ms)

    def reserve_score(self):
        return self.main_fraction_unmet, self.aid_fraction_unmet, self.width_ms


def reachability_first(event):
    """Reserve only what is left after preserving the baseline's local modeled coverage."""
    out = dict(reachability_first_aid_ms=None, reachability_first_unmet_j=None,
               baseline_model_work_j=None)
    cap, request = event.get('guarded_ms'), event.get('request_j')
    envelope = event.get('envelope_before_ms')
    if (event.get('return_event') is not True or not _valid_model(event)
            or event.get('valid') is not True or event.get('energy_admission_allowed') is not True
            or event.get('fore_aft_allowed') is not True or event.get('fore_aft_reason') not in (0, 1)
            or type(cap) is not int or type(envelope) is not int or not 0 <= cap <= envelope <= ENVELOPE_MS
            or not _finite(request) or request <= 0):
        return out
    baseline_work, _ = _work(event, cap)
    if baseline_work is None:
        return out
    target = min(request, baseline_work)
    for width in range(cap + 1):
        work, _ = _work(event, width)
        if work is not None and work + 1e-9 >= target:  # v15 kEnergyRoundoffJ
            out.update(reachability_first_aid_ms=width, reachability_first_unmet_j=max(0., request - work),
                       baseline_model_work_j=baseline_work)
            break
    return out


def evaluate_candidate(event, context, prior_main_pulses):
    """Evaluate guarded integer choices on the single recorded decision state.

    Supply history from this run only. We also check time, pulse completion,
    unassisted ordinary ZERO, direction and target stage inside the policy.
    Candidate widths are post-guard budget endpoints for offline comparison.
    """
    baseline = event.get('guarded_ms')
    result = dict(status='unavailable', reason=None, baseline_aid_ms=baseline,
                  baseline_reserve_ms=None, baseline_aid_work_j=None, baseline_aid_unmet_j=None,
                  baseline_worst_main_proxy_unmet_j=None, main_proxy_max_requested_j=None,
                  candidate_aid_ms=None, reserve_ms=None, candidate_aid_work_j=None,
                  candidate_aid_unmet_j=None, candidate_aid_work_rate_supported=None,
                  candidate_aid_charge_supported=None, main_proxy_count=0,
                  main_proxy_times_ms=[], worst_main_proxy_unmet_j=None,
                  candidate_main_proxy_work_supported=None,
                  balanced_aid_ms=None, reserve_priority_aid_ms=None,
                  balanced_aid_unmet_j=None, balanced_worst_main_proxy_unmet_j=None,
                  reserve_priority_aid_unmet_j=None, reserve_priority_worst_main_proxy_unmet_j=None,
                  leading_foot=None, leading_phase=None, outward_rate_dps=None)

    def abstain(reason):
        result['reason'] = reason
        return result

    if event.get('event_kind') != 'inward_stall_rescue' or event.get('return_event') is not True:
        return abstain('not_return_aid')
    if (event.get('valid') is not True or event.get('energy_admission_allowed') is not True
            or event.get('fore_aft_allowed') is not True or event.get('fore_aft_reason') not in (0, 1)):
        return abstain('existing_v19_gate')
    envelope = event.get('envelope_before_ms')
    if (not _valid_model(event) or not _finite(event.get('time_ms')) or not _finite(event.get('peak_ms'))
            or not 0 < event['peak_ms'] < event['time_ms']
            or not _finite(event.get('request_j')) or event['request_j'] <= 0
            or not _finite(event.get('target_j')) or event['target_j'] <= 0
            or type(envelope) is not int or type(baseline) is not int
            or not 0 < baseline <= envelope <= ENVELOPE_MS):
        return abstain('invalid_decision_model_or_budget')
    if (event.get('return_evidence') != 1 or context.get('left_valid') is not True
            or context.get('right_valid') is not True or not _finite(context.get('foot_age_ms'))
            or not 0 <= context['foot_age_ms'] <= MAX_FRAME_AGE_US / 1000):
        return abstain('no_fresh_bilateral_shape')
    left, right = context.get('left_deg'), context.get('right_deg')
    if not _finite(left) or not _finite(right) or abs(left - right) < 3 or max(left, right) <= 0:
        return abstain('no_separated_forward_foot')
    lead = 'left' if left > right else 'right'
    phase = context.get(lead + '_phase')
    result.update(leading_foot=lead, leading_phase=phase)
    angle, rate = context.get('fore_aft_angle_deg'), context.get('fore_aft_rate_dps')
    if not _finite(angle) or not _finite(rate):
        return abstain('unknown_fore_aft_motion')
    outward = rate if angle > 0 else -rate if angle < 0 else 0.
    result['outward_rate_dps'] = outward
    history = _history(event, prior_main_pulses)
    result.update(main_proxy_count=len(history), main_proxy_times_ms=[r['time_ms'] for r in history])
    if not history:
        return abstain('no_recent_same_target_ordinary_zero')

    choices = []
    for width in range(baseline + 1):
        work, charge = _work(event, width)
        if work is None:
            continue
        unmet = max(0., event['request_j'] - work)
        aid_fraction = unmet / event['request_j']
        mains = [_main_scenario(r, envelope - width) for r in history]
        choices.append(Allocation(width, work, charge, unmet, aid_fraction,
                                  max(m[0] for m in mains), max(m[1] for m in mains),
                                  all(m[2] for m in mains)))

    # Minimax normalized deficit preserves the different meanings of return
    # and arrival J. When the optima tie, prefer smaller combined fractional
    # shortfall, then less early expenditure. The second alternative gives
    # lexicographic priority to reserve; no arbitrary conversion of deg to J.
    baseline_choice = choices[-1]
    if baseline_choice.width_ms != baseline:
        return abstain('invalid_guarded_model_charge')
    balanced = min(choices, key=Allocation.balance_score)
    protect = min(choices, key=Allocation.reserve_score)
    result.update(balanced_aid_ms=balanced.width_ms, reserve_priority_aid_ms=protect.width_ms,
                  balanced_aid_unmet_j=balanced.aid_unmet_j, balanced_worst_main_proxy_unmet_j=balanced.main_unmet_j,
                  reserve_priority_aid_unmet_j=protect.aid_unmet_j,
                  reserve_priority_worst_main_proxy_unmet_j=protect.main_unmet_j,
                  baseline_reserve_ms=envelope - baseline, baseline_aid_work_j=baseline_choice.work_j,
                  baseline_aid_unmet_j=baseline_choice.aid_unmet_j,
                  baseline_worst_main_proxy_unmet_j=baseline_choice.main_unmet_j,
                  main_proxy_max_requested_j=max(r['request_j'] for r in history))
    if phase not in ('outgoing', 'returning', 'holding', 'turning'):
        return abstain('unknown_leading_foot_phase')
    protective = phase == 'outgoing' and outward >= OUTWARD_RATE_DPS
    chosen = protect if protective else balanced
    low, high, max_charge = WORK_MODEL[event['side']][2:]
    result.update(status='shadow', reason='outgoing_and_outward_reserve_priority' if protective else
                  'balance_return_and_peak_shortfalls', candidate_aid_ms=chosen.width_ms,
                  reserve_ms=envelope - chosen.width_ms, candidate_aid_work_j=chosen.work_j,
                  candidate_aid_unmet_j=chosen.aid_unmet_j,
                  candidate_aid_work_rate_supported=low <= event['decision_rate_dps'] <= high,
                  candidate_aid_charge_supported=chosen.charge_mas <= max_charge,
                  worst_main_proxy_unmet_j=chosen.main_unmet_j,
                  candidate_main_proxy_work_supported=chosen.main_work_supported)
    return result
