#!/usr/bin/env python3
"""Timing-only result. No angle fitting, no silent replacement of deadlines."""
import argparse,json
from pathlib import Path
from replay_v46s_solver_audit import load_metadata

def assess(m):
    acquisition=m.get('v46n_imu_acquisition',{})
    profile=acquisition.get('v46q_poll_profile',{})
    reader=profile.get('v46u_timing',{})
    control=m.get('v46p_control_worker',{}).get('v46u_deadline',{})
    current=m.get('v46u_current_timing',{})
    required_reader=('driver_calls','driver_failures','poll_work_count','poll_work_over_budget','host_gyro_interval_count','host_gyro_interval_over_budget')
    required_control=('count','over_budget')
    required_current=('read_count','interval_count','read_over_budget','interval_over_budget','scope')
    if (any(k not in reader for k in required_reader) or
        any(k not in control for k in required_control) or
        any(k not in current for k in required_current)):
        return {'result':'INCOMPLETE','reason':'V46u timing records missing; old logs cannot certify deadlines'}
    complete=(control.get('count',0)>0 and
              control['count']==acquisition.get('delivered')==acquisition.get('captured') and
              reader.get('driver_calls',0)==profile.get('polls') and
              reader.get('poll_work_count',0)==profile.get('polls') and
              reader.get('host_gyro_interval_count',0)==acquisition.get('captured'))
    faults=bool(acquisition.get('fault',True) or acquisition.get('queue_drops',1) or
                acquisition.get('delivery_sequence_gaps',1) or reader.get('driver_failures',1))
    violations={
        'reader_work_1000us':reader.get('poll_work_over_budget',0),
        'host_gyro_intervals_2500us':reader.get('host_gyro_interval_over_budget',0),
        'sample_to_runner_done_2500us':control.get('over_budget',0),
        'current_read_work_2000us_since_boot':current.get('read_over_budget',0),
        'current_intervals_2000us_since_boot':current.get('interval_over_budget',0),
    }
    current_observed=current.get('read_count',0)>0 and current.get('interval_count',0)>0
    return {'result':('OBSERVED_VIOLATIONS' if faults or any(violations.values()) else
                      'OBSERVED_HOST_BUDGETS_PASS' if complete and current_observed else 'INCOMPLETE'),
            'imu_control_coverage_complete':complete,
            'current_observed':current_observed,
            'current_scope':current.get('scope'),
            'violations':violations,
            'fault_or_missing_health_data':faults,
            'imu_control_observed_pass':complete and not faults and
               not any(violations[k] for k in ('reader_work_1000us','host_gyro_intervals_2500us','sample_to_runner_done_2500us')),
            'overall_hard_realtime_certified':False,
            'limits':'Host acquisition timestamps, not sensor-clock FIFO. Current totals are since boot; physical motor-apply deadlines not fully audited.'}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('input',type=Path);ap.add_argument('--out',type=Path)
    args=ap.parse_args();rows=[]
    for name,m,crc in load_metadata(args.input):rows.append({'run':name,'crc_ok':crc,**assess(m)})
    text=json.dumps(rows,ensure_ascii=False,indent=2);print(text)
    if args.out:args.out.write_text(text+'\n')
    return 0 if all(r.get('result')=='OBSERVED_HOST_BUDGETS_PASS' for r in rows) else 1
if __name__=='__main__':raise SystemExit(main())
