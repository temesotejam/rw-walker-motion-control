"""Replay one additional aid using only previously completed physical commands."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent


def replay(folder, exe):
    meta = json.loads((folder / 'metadata.json').read_text(encoding='utf8'))
    block = meta['walking_energy']
    assert len(block['records']) == block['total'] and not block['truncated']
    arcs = {r['row_index']: r for r in block.get('arc_aids', [])}
    active = [(i, r) for i, r in enumerate(block['records']) if r['executed']]
    zeros = meta['energy_control_autonomous_zero_cross_events']
    events, ground, finished = [], {}, []

    def add(ms, order, kind, *values):
        events.append((ms, order, kind + ' ' + ' '.join(map(str, values))))

    for i, r in active:
        t, side = r['time_ms'], r['side']
        event, = [z for z in zeros if z['output_executed'] and z['previous_peak_time_ms']==r['peak_ms']
                  and 0 <= t-z['zero_cross_time_ms'] <= 5]
        if not r['return_event']:
            add(t, 1, 'M', side, t, r['requested_ms'], event['pre_input_wheel_speed_rpm'])
        else:
            arc = arcs.get(i, {})
            extended = arc.get('arc_aid_reason') == 8
            add(t, 1, 'A', t, event['work_brake_cap_ms'], arc.get('arc_base_guarded_ms', 0),
                r['guarded_ms'], arc.get('arc_main_reserve_ms', 0), r['requested_ms'], int(extended),
                r['start_rate_dps'], event['aligned_initial_current_mA'], event['predicted_current_goal_mA'],
                r['start_tau_s'], event['pre_input_wheel_speed_rpm'], r['target_j'], r['request_j'])
            if extended:
                # For these six recordings, the timing cap at the observed aid was the entire envelope.
                assert event['work_brake_cap_ms']==r['envelope_before_ms']
            valid = (extended and side==-1 and r['completion_reason']==1 and r['completion_work_valid']
                     and r['completion_width_supported'] and 0 <= r['elapsed_q_pred_mas'] <= 12.83679)
            add(r['output_end_ms'], 0, 'E', r['peak_ms'], r['output_end_ms'], r['actual_elapsed_ms'],
                arc.get('arc_terminal_deg') or 0, r['elapsed_work_pred_j'] if valid else 0)
            next_zero = next((z for z in zeros if z['event_kind']=='energy_control_zero_cross' and
                              z['previous_peak_time_ms']==r['peak_ms'] and z['zero_cross_time_ms']>=t), {})
            ground[t] = dict(trial=folder.name.rsplit('_', 1)[-1], time_ms=t, arc_extended=int(extended),
                             recorded_ms=r['guarded_ms'], reserve_ms=arc.get('arc_main_reserve_ms', ''),
                             crossed=int(bool(next_zero)), zero_rate_dps=next_zero.get('zero_cross_abs_rate_dps', ''))
        finished.append((r['output_end_ms'], r, event))

    for p in meta['energy_control_autonomous_peak_events']:
        t, side = p['peak_time_ms'], p['physical_peak_side']
        known = p['peak_telemetry_capture_time_ms']
        latest = max((x for x in finished if x[0]<=known), key=lambda x:x[0], default=(None, {}, {}))
        _, main, e = latest
        matched = bool(p['pending_command_matched'] and (p['pending_q_command_mA_s'] or 0)>0)
        ordinary = (matched and e.get('event_kind')=='energy_control_zero_cross' and
                    main['side']==side and main['assisted_cycle'] and main['completion_reason']==1 and
                    main['completion_work_valid'])
        add(known, 2, 'K', side, t, p['peak_amplitude_deg'], int(matched), main.get('peak_ms', 0),
            e.get('zero_cross_time_ms', 0), int(ordinary))
    data = '\n'.join(line for _, _, line in sorted(events))+'\n'
    answer = subprocess.check_output([str(exe)], input=data, text=True)
    for line in answer.splitlines():
        time, source, extra, raw, width, work = line.split()
        row = ground[int(time)]
        row.update(feedback_peak_ms=source, missing_j=extra, inverse_ms=raw,
                   selected_ms=width, predicted_work_j=work if row['arc_extended'] else '')
    return list(ground.values())


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', type=Path, nargs='+', help='RWLOG converter output directories')
    parser.add_argument('--out', type=Path, required=True)
    args=parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        exe=Path(tmp)/'arc.exe'
        subprocess.run(['g++','-std=c++17','-O2','-I',str(HERE.parents[1]/'src'),
                        str(HERE/'replay_arc.cpp'),'-o',str(exe)], check=True)
        rows=[r for folder in args.folders for r in replay(folder,exe)]
    with args.out.open('w',encoding='utf8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
