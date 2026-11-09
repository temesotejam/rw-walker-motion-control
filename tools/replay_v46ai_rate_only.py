#!/usr/bin/env python3
"""Recorded-state replay of the rate-only baseline; not a closed-loop simulation."""
import argparse,csv,json,struct,subprocess,tempfile
from pathlib import Path
import replay_v46s_solver_audit as retained
ROOT=Path(__file__).resolve().parents[1]
def bits(x):return struct.unpack('<I',struct.pack('<f',x))[0]
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metadata',type=Path,nargs='+')
    parser.add_argument('--out-dir',type=Path,required=True)
    args=parser.parse_args();args.out_dir.mkdir(parents=True,exist_ok=True)
    records=[];inputs=[]
    for path in args.metadata:
        m=json.loads(path.read_text());zero={z['event_index']:z for z in m['energy_control_autonomous_zero_cross_events']}
        for a in m['v46s_solver_audit']['events']:
            if not(a['solver_complete'] and a['stage']==4):continue
            z=zero[a['index']+1]
            inputs.append(' '.join(map(str,[a['physical_side'],a['command_direction'],a['model_vbat_mV'],*a['f32_bits'],bits(z['zero_cross_abs_rate_dps'])])))
            records.append(dict(run=path.parent.parent.name,event_index=z['event_index'],t_test_ms=a['t_test_ms'],side=a['physical_side'],rate_dps=z['zero_cross_abs_rate_dps'],recorded_width_ms=a['selected_width_ms']))
    with tempfile.TemporaryDirectory() as directory:
        p=Path(directory);retained.build_driver(p)
        source='#define main retained_main\n#include "replay.cpp"\n#undef main\n#include "rate_baseline_correction.h"\n'
        source+='int main(){int side,direction;unsigned vbat;while(std::cin>>side>>direction>>vbat){Model m;m.r.physical_side=side;m.r.command_direction=direction;m.r.model_vbat_mV=vbat;\n'
        for field in retained.FLOAT_FIELDS:source+=' {uint32_t b;if(!(std::cin>>b))return 2;memcpy(&m.r.'+field+',&b,4);}\n'
        source+=r'''
        uint32_t b;float rate;std::cin>>b;memcpy(&rate,&b,4);
        auto before=m.replay(false),before_full=m.replay(true);
        const float old_baseline=m.r.free_peak_deg;
        auto result=rate_baseline::evaluate(rate,side);
        if(result.reason!=rate_baseline::APPLIED && result.reason!=rate_baseline::ZERO_FLOOR)return 3;
        m.r.free_peak_deg=result.adjusted_deg;
        m.r.passive_energy_j=m.energyControlPotentialJ(m.r.free_peak_deg);
        auto after=m.replay(false),after_full=m.replay(true);
        std::cout<<std::setprecision(9)<<int(result.reason)<<' '<<old_baseline<<' '<<result.rate_deg<<' '<<result.adjusted_deg;
        for(auto x:{before,before_full,after,after_full})std::cout<<' '<<x.valid<<' '<<x.ff<<' '<<x.selected<<' '<<x.evaluations<<' '<<x.q<<' '<<solver_audit::floatBits(x.q);
        // Check all target choices on these recorded rates/current/Ki states.
        for(float target:{8.f,10.f,12.f}){
          m.r.target_peak_deg=target;m.r.target_energy_j=m.energyControlPotentialJ(target);
          auto fast=m.replay(false),full=m.replay(true);
          if(!fast.valid||fast.valid!=full.valid||fast.ff!=full.ff||fast.selected!=full.selected||solver_audit::floatBits(fast.q)!=solver_audit::floatBits(full.q))return 4;
          if(fast.selected>100||fast.evaluations>60)return 5;
        }
        std::cout<<'\n';
    }return 0;}
'''
        cpp=p/'rate_only.cpp';cpp.write_text(source);exe=p/'rate_only'
        subprocess.run(['g++','-std=c++17','-O2','-ffp-contract=off','-Wall','-Wextra','-Werror','-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'src'),str(cpp),'-o',str(exe)],check=True)
        result=subprocess.run([str(exe)],input='\n'.join(inputs)+'\n',capture_output=True,text=True)
        if result.returncode:raise RuntimeError(f'Native replay exited {result.returncode}: {result.stderr}; completed {len(result.stdout.splitlines())} rows')
        lines=result.stdout.splitlines()
    assert len(lines)==len(records)
    for row,line in zip(records,lines):
        values=list(map(float,line.split()));assert len(values)==28
        row.update(zip(['rate_reason','recorded_baseline_deg','raw_rate_baseline_deg','new_baseline_deg'],values[:4]))
        for i,prefix in enumerate(['before','before_full','after','after_full']):
            row.update({prefix+'_'+k:v for k,v in zip(['valid','ff_ms','selected_ms','evaluations','q_mA_s','q_bits'],values[4+i*6:10+i*6])})
        for a,b in [('before','before_full'),('after','after_full')]:
            for field in ['valid','ff_ms','selected_ms','q_bits']:assert row[a+'_'+field]==row[b+'_'+field],(row,a,field)
        assert row['after_valid']==1 and 0<=row['after_selected_ms']<=100
    summary=dict(method='Production rate-only helper plus retained float32 selector; recorded current-model and Ki state held fixed; no simulated future trajectory.',
                 precision='Solver inputs: recorded float32 bits; rate: rounded event JSON.',cases=len(records),
                 fast_exhaustive_matches=len(records),all_target_choice_checks=3*len(records),
                 zero_floors=sum(r['rate_reason']==5 for r in records),P1_fallbacks=0,
                 changed_widths=sum(r['before_selected_ms']!=r['after_selected_ms'] for r in records),
                 max_evaluations=max(r['after_evaluations'] for r in records),
                 host_recorded_width_differences=[{k:r[k] for k in ['run','event_index','recorded_width_ms','before_selected_ms']} for r in records if r['recorded_width_ms']!=r['before_selected_ms']],
                 per_run={})
    for run in sorted({r['run'] for r in records}):
        rows=[r for r in records if r['run']==run]
        summary['per_run'][run]=dict(cases=len(rows),zero_floors=sum(r['rate_reason']==5 for r in rows),changed_widths=sum(r['before_selected_ms']!=r['after_selected_ms'] for r in rows),
            width_range_ms=[min(r['after_selected_ms'] for r in rows),max(r['after_selected_ms'] for r in rows)])
    with (args.out_dir/'recorded_state_replay.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    (args.out_dir/'recorded_state_replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
