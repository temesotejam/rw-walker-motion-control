#!/usr/bin/env python3
"""Replay recorded decisions with the frozen V46ah helper; no future motion or hardware is simulated."""
import argparse
import csv
import json
from pathlib import Path
import struct
import subprocess
import tempfile

import replay_v46s_solver_audit as retained

ROOT = Path(__file__).resolve().parents[1]

def bits(value):
    return struct.unpack('<I', struct.pack('<f', value))[0]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metadata', type=Path, nargs='+')
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    inputs, records = [], []
    for path in args.metadata:
        m = json.loads(path.read_text())
        assert m['autonomous_timing_compensation_us'] == 3000
        zero = {z['event_index']: z for z in m['energy_control_autonomous_zero_cross_events']}
        for a in m['v46s_solver_audit']['events']:
            if not (a['solver_complete'] and a['stage'] == 4):
                continue
            z = zero[a['index']+1]
            assert a['physical_side'] == z['physical_next_peak_side']
            assert 0 <= a['t_test_ms']-z['zero_cross_time_ms'] <= 10
            raw_p1 = z.get('p1_free_peak_before_rate_deg', z['free_next_peak_amplitude_deg'])
            inputs.append(' '.join(map(str, [a['physical_side'], a['command_direction'], a['model_vbat_mV'],
                         *a['f32_bits'], a['t_test_ms'], bits(raw_p1), bits(z['previous_peak_amplitude_deg']),
                         bits(z['zero_cross_abs_rate_dps'])])))
            records.append(dict(run=path.parent.parent.name, event_index=z['event_index'],
                                t_test_ms=a['t_test_ms'], side=a['physical_side'],
                                recorded_ff_ms=a['ff_width_ms'], recorded_selected_ms=a['selected_width_ms']))
    assert records
    with tempfile.TemporaryDirectory() as directory:
        p = Path(directory)
        retained.build_driver(p)
        driver = (p/'replay.cpp').read_text()
        driver = driver.replace('struct Model {', 'struct Model { bool use_recorded_target=false;')
        driver = driver.replace('const float target=energyControlPotentialJ(',
                                'const float target=use_recorded_target ? r.corrected_target_energy_j : energyControlPotentialJ(')
        (p/'replay.cpp').write_text(driver)
        source = '#define main retained_main\n#include "replay.cpp"\n#undef main\n#include "v46ah_rate_baseline_frozen.h"\n'
        source += 'int main(){ int side,direction; unsigned vbat; while(std::cin>>side>>direction>>vbat){ Model m; m.r.physical_side=side; m.r.command_direction=direction; m.r.model_vbat_mV=vbat;\n'
        for field in retained.FLOAT_FIELDS:
            source += ' {uint32_t b; if(!(std::cin>>b))return 2; memcpy(&m.r.'+field+',&b,4);}\n'
        source += r'''
    uint32_t ms,b0,b1,b2; float p1,prev,rate; std::cin>>ms>>b0>>b1>>b2;
    memcpy(&p1,&b0,4);memcpy(&prev,&b1,4);memcpy(&rate,&b2,4);
    auto before=m.replay(false),before_full=m.replay(true);
    const float qt=fmaxf(0.f,fminf(m.r.q_available_mA_s,before.ffq+m.r.integral_mA_s));
    const float host_target=m.energyControlPotentialJ(m.energyControlAutonomousCorrectedPrediction(m.r.free_peak_deg,side,qt,nullptr));
    m.use_recorded_target=true; auto cached=m.replay(false); m.use_recorded_target=false;
    auto result=rate_baseline::evaluate(p1,rate,side,prev,m.r.target_peak_deg,ms,3000);
    if(result.reason==rate_baseline::APPLIED){
      m.r.free_peak_deg=result.adjusted_deg;
      m.r.passive_energy_j=m.energyControlPotentialJ(m.r.free_peak_deg);
    }
    auto after=m.replay(false),after_full=m.replay(true);
    std::cout<<std::setprecision(9)<<int(result.reason)<<' '<<result.p1_deg<<' '<<result.rate_deg<<' '
             <<result.correction_deg<<' '<<result.adjusted_deg;
    for(auto x:{before,before_full,after,after_full})
      std::cout<<' '<<x.valid<<' '<<x.ff<<' '<<x.selected<<' '<<x.evaluations<<' '
               <<x.q<<' '<<solver_audit::floatBits(x.q);
    std::cout<<' '<<cached.selected<<' '<<solver_audit::floatBits(host_target)<<' '
             <<solver_audit::floatBits(m.r.corrected_target_energy_j)<<'\n';
    }return 0;}
'''
        cpp = p/'full_rate.cpp'; cpp.write_text(source)
        binary = p/'full_rate'
        subprocess.run(['g++','-std=c++17','-O2','-ffp-contract=off','-Wall','-Wextra','-Werror',
                        '-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'tools/fixtures'),'-I'+str(ROOT/'src'),str(cpp),'-o',str(binary)],check=True)
        lines = subprocess.run([str(binary)],input='\n'.join(inputs)+'\n',capture_output=True,text=True,check=True).stdout.splitlines()
    assert len(lines) == len(records)
    for row,line in zip(records,lines):
        values = list(map(float,line.split()))
        assert len(values) == 32
        row.update(zip(['reason','p1_deg','rate_deg','correction_deg','adjusted_deg'], values[:5]))
        row.update(zip(['cached_target_selected_ms','host_target_bits','device_target_bits'], values[29:]))
        for i,prefix in enumerate(['before','before_full','after','after_full']):
            row.update({prefix+'_'+k:v for k,v in zip(['valid','ff_ms','selected_ms','evaluations','q_mA_s','q_bits'], values[5+i*6:11+i*6])})
        for a,b in [('before','before_full'),('after','after_full')]:
            for field in ['valid','ff_ms','selected_ms','q_bits']:
                assert row[a+'_'+field] == row[b+'_'+field], (row['run'],row['event_index'],a,field)
        assert row['after_valid'] == 1 and 0 <= row['after_selected_ms'] <= 100
        assert row['after_evaluations'] <= 60
        if row['reason'] == 0:
            assert row['adjusted_deg'] == row['rate_deg']
        else:
            assert row['before_selected_ms'] == row['after_selected_ms']
            assert row['before_q_bits'] == row['after_q_bits']
    mismatches = [{k:r[k] for k in ['run','event_index','recorded_ff_ms','before_ff_ms','recorded_selected_ms','before_selected_ms',
                                  'cached_target_selected_ms','host_target_bits','device_target_bits']}
                  for r in records if r['recorded_ff_ms'] != r['before_ff_ms'] or r['recorded_selected_ms'] != r['before_selected_ms']]
    summary = dict(method='Production V46ah helper and retained float32 width selector. Recorded current model and Ki held fixed; no closed-loop simulation.',
                   input_precision='Solver inputs use recorded float32 bits. P1, previous peak and rate are rounded event JSON values. Active direct baseline is independent of P1.',
                   cases=len(records), fast_exhaustive_matches=len(records), host_recorded_mismatches=mismatches,
                   cached_target_recorded_matches=sum(r['recorded_selected_ms']==r['cached_target_selected_ms'] for r in records),
                   active_cases=sum(r['reason']==0 for r in records),
                   bypass_cases=sum(r['reason']!=0 for r in records),
                   changed_widths=sum(r['before_selected_ms']!=r['after_selected_ms'] for r in records),
                   max_evaluations=max(r['after_evaluations'] for r in records), per_run={})
    for run in sorted({r['run'] for r in records}):
        rows=[r for r in records if r['run']==run]
        summary['per_run'][run] = dict(n=len(rows), active=sum(r['reason']==0 for r in rows), per_side={})
        for side in [-1,1]:
            active=[r for r in rows if r['side']==side and r['reason']==0]
            if not active: continue
            summary['per_run'][run]['per_side'][str(side)] = dict(n=len(active),
                correction_deg_range=[min(r['correction_deg'] for r in active),max(r['correction_deg'] for r in active)],
                old_width_ms_range=[min(r['before_selected_ms'] for r in active),max(r['before_selected_ms'] for r in active)],
                new_width_ms_range=[min(r['after_selected_ms'] for r in active),max(r['after_selected_ms'] for r in active)],
                new_Q_max_mA_s=max(r['after_q_mA_s'] for r in active))
    with (args.out_dir/'recorded_state_replay.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(records[0])); writer.writeheader(); writer.writerows(records)
    (args.out_dir/'recorded_state_replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__ == '__main__':
    main()
