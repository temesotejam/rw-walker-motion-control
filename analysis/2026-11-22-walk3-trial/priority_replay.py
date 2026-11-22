"""Replay the new reserve decision on the measured arc aids from the archived runs."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent


def audit(folder, exe):
    meta = json.loads((folder / 'metadata.json').read_bytes())
    run = json.loads((folder / 'header.json').read_bytes())['run_start_us']
    records = (meta.get('walking_energy') or {}).get('records', [])
    edges = meta['energy_control_autonomous_zero_cross_events'][1:]
    if not any(r.get('return_evidence') == 5 and r['executed'] for r in records):
        return []
    with (folder / 'support_transfer_shadow.csv').open(newline='') as source:
        snapshots = {int(s['packet']): s for s in csv.DictReader(source)}
    arcs = {a['row_index']: a for a in meta['walking_energy']['arc_aids']}
    peaks = meta['energy_control_autonomous_peak_events']
    result = []
    for i, r in enumerate(records):
        if r['return_evidence'] != 5 or not r['executed']:
            continue
        e = edges[i]
        t = r['time_ms']
        prior = max(j for j, main in enumerate(records[:i]) if main['executed'] and not main['return_event'])
        prev = snapshots[r['periodic_foot_packet'] - 1]
        reserve = arcs[i]['arc_main_reserve_ms']
        envelope = r['envelope_before_ms']
        # The pre-reserve timing cap is not logged separately. In these cases
        # its lower bound (original proposal + reserve) equals its upper bound.
        if not (len(edges) == len(records) and e['event_kind'] == 'inward_stall_rescue'
                and e['zero_cross_time_ms'] == t and not e['wheel_guard_trial_eligible']
                and r['requested_ms'] + reserve == envelope and records[prior]['side'] == 1
                and int(prev['peak_ms']) == r['peak_ms'] and int(prev['time_ms']) < t):
            raise ValueError(f'{run}/{t}: archived inputs do not establish the timing cap or guard state')
        same = [p for p in peaks if p['physical_peak_side'] == -1 and 0 < t-p['peak_time_ms'] <= 3000]
        values = (t, e['detector_angle_after_deg'], r['start_rate_dps'], e['startup_target_deg'],
                  e['previous_peak_amplitude_deg'], same[-1]['peak_amplitude_deg'], r['start_i0_ma'],
                  r['start_goal_ma'], r['start_tau_s'], r['return_work_j'], envelope, reserve,
                  prev['accel_dps2'], r['return_deceleration_dps2'], records[prior]['time_ms'],
                  edges[prior]['pre_input_wheel_speed_rpm'], e['pre_input_wheel_speed_rpm'])
        line = subprocess.check_output([exe], input=' '.join(map(str, values))+'\n', text=True)
        cap, width, work = line.split()
        # Use subsequent events for outcome scoring only.
        crossed = any(edge['event_kind'] == 'energy_control_zero_cross'
                      and edge['previous_peak_time_ms'] == r['peak_ms']
                      and edge['physical_next_peak_side'] == -1 and edge['zero_cross_time_ms'] > t
                      for edge in edges[i+1:])
        result.append(dict(run=run, time_ms=t, prior_main_reserve_ms=reserve, old_ms=r['guarded_ms'],
                           relaxed=bool(int(cap)), candidate_ms=int(width), original_crossed=crossed,
                           old_work_uj=round(1e6*r['output_work_pred_j'], 2),
                           candidate_work_uj=round(1e6*float(work), 2)))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', type=Path, nargs='+', help='parents of decoded run directories')
    args = parser.parse_args()
    folders = sorted({p.resolve() for root in args.roots for p in root.iterdir() if (p/'metadata.json').is_file()})
    with tempfile.TemporaryDirectory() as scratch:
        exe = Path(scratch) / 'priority_probe.exe'
        subprocess.run(['g++','-std=c++17','-O2','-I',HERE.parents[1]/'src',HERE/'priority_probe.cpp','-o',exe], check=True)
        print(json.dumps(dict(runs=len(folders), measured=[row for f in folders for row in audit(f, exe)]), indent=2))
