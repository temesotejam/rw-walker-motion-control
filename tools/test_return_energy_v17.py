"""Native v17 policy tests and causal probes on recorded v16 states.

Replay cannot predict the physical trajectory after changing an input. Foot
frames are delivered only after their ready time, with extra-delay sensitivity.
"""
from pathlib import Path
import argparse
import bisect
import csv
import json
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CPP = r'''
#include "walking_return_energy.h"
#include <cassert>
#include <iostream>
#include <iomanip>
namespace e=walking_return_energy;
void tests(){
 lateral_energy::Geometry g;lateral_energy::CurrentTable table;assert(table.prepare(.031652216f));
 e::Input in;in.side=1;in.rate_dps=42;in.body_angle_deg=-1.2;in.target_deg=8;
 in.previous_peak_deg=5.7;in.same_side_peak_deg=7.8;in.initial_current_ma=0;in.goal_current_ma=328;
 const auto base=e::decide(g,table,in);assert(base.valid);
 in.arrival_bias_j=.00015f;auto lower=e::decide(g,table,in);assert(lower.width_ms<base.width_ms);
 in.arrival_bias_j=-.00015f;auto higher=e::decide(g,table,in);assert(higher.width_ms>=base.width_ms);
 in.arrival_bias_j=0;in.side=-1;in.body_angle_deg=9.4f;in.rate_dps=15;
 auto coast=e::decide(g,table,in);assert(coast.coast);
 in.return_work_j=.0001f;auto drive=e::decide(g,table,in);
 assert(drive.valid && drive.width_ms>15 && !drive.coast && drive.request_j>=.00009999f);
 assert(drive.saturated || drive.predicted_work_j+1e-9f>=drive.request_j);
 if(!drive.saturated){auto previous=e::evaluateWidth(g,table,in,drive.width_ms-1);
   assert(previous.predicted_work_j<drive.request_j);}
 auto blocked=e::evaluateWidth(g,table,in,0);assert(!blocked.valid);
 assert(!e::earlyAdmissible(e::evaluateWidth(g,table,in,1)));
 in.return_work_j=0;assert(!e::evaluateWidth(g,table,in,10).valid);
 e::ArrivalFeedback f;f.observe(1,100,.0009f,.0007f,0,.0007f,true);
 assert(f.bias(1,110,.0007f)==0);f.observe(1,200,.0009f,.0007f,0,.0007f,true);
 const float b=f.bias(1,210,.0007f);assert(b>0 && b<=.25f*.0007f);
 assert(f.bias(-1,210,.0007f)==0 && f.bias(1,3300,.0007f)==0 && f.bias(1,100,.0007f)==0);
 f.observe(1,300,.0009f,.0008f,.0001f,.0007f,true);
 assert(std::fabs(f.bias(1,310,.0007f)-b)<1e-9f); // undo already applied bias
 f.observe(1,400,0,.001f,0,.0007f,false);assert(f.bias(1,410,.0007f)==b);
 f.reset();assert(f.bias(1,420,.0007f)==0);
 for(int sign:{1,-1})for(int mode=0;mode<5;++mode){
   e::ReturnDetector d;bool selected=false;
   for(unsigned t=200;t<=260;t+=2){e::ReturnInput x;x.active=true;x.side=sign;x.now=1000+t;x.peak_ms=1000;
    x.peak_deg=10.6f;x.angle_deg=sign*8.5f;x.rate_dps=-sign*(30.f-(t-200)*.35f);
    x.foot_frame_ms=1150;x.foot_ready_ms=1210;x.left_deg=15;x.right_deg=0;x.foot_valid=true;x.target_j=.000733f;
    if(mode==1)x.foot_valid=false;if(mode==2)x.foot_frame_ms=900;
    if(mode==3)x.rate_dps=-sign*(10.f+(t-200)*.3f);
    if(mode==4)x.foot_ready_ms=x.now+1;
    const auto q=d.observe(x);if(q.selected){selected=true;assert(q.request_j>0 && q.request_j<=.5f*x.target_j);}
   }
   assert(selected==(mode==0));
 }
 std::cout<<"policy PASS\n";
}
int main(){tests();std::cout<<std::setprecision(12);lateral_energy::Geometry g;
 lateral_energy::CurrentTable table;assert(table.prepare(.031652216f));
 e::ReturnDetector detector;e::ArrivalFeedback feedback;char type;unsigned id;
 while(std::cin>>type>>id){
  if(type=='D'){e::ReturnInput in;int active,valid;std::cin>>in.now>>in.peak_ms>>in.side>>in.peak_deg>>in.angle_deg>>in.rate_dps
    >>in.foot_frame_ms>>in.foot_ready_ms>>in.left_deg>>in.right_deg>>in.target_j>>active>>valid;
    in.active=active;in.foot_valid=valid;auto d=detector.observe(in);
    if(d.selected)std::cout<<"D "<<id<<' '<<in.now<<' '<<in.peak_ms<<' '<<d.request_j<<' '<<d.stopping_distance_deg<<' '<<d.deceleration_dps2<<'\n';
  }else if(type=='F'){int side;unsigned now;float measured,predicted,target;std::cin>>side>>now>>measured>>predicted>>target;
    feedback.observe(side,now,measured,predicted,0,target,true);
  }else if(type=='S'){e::Input in;unsigned now;int side;std::cin>>now>>side>>in.rate_dps>>in.body_angle_deg>>in.target_deg
    >>in.previous_peak_deg>>in.same_side_peak_deg>>in.initial_current_ma>>in.goal_current_ma>>in.cap_ms;
    in.side=side;if(in.same_side_peak_deg<0)in.same_side_peak_deg=NAN;
    const auto old=e::decide(g,table,in);in.arrival_bias_j=feedback.bias(in.side,now,lateral_energy::potential(g,in.target_deg));
    const auto adjusted=e::decide(g,table,in);
    std::cout<<"S "<<id<<' '<<in.arrival_bias_j<<' '<<old.width_ms<<' '<<adjusted.width_ms<<' '<<adjusted.request_j
      <<' '<<adjusted.saturated<<' '<<e::earlyAdmissible(adjusted)<<'\n';
  }
 }
}
'''


def records(path):
    with path.open(encoding='utf8', newline='') as stream:
        return list(csv.DictReader(stream))


def probe(exe, path, delay):
    meta = json.loads((path / 'metadata.json').read_text(encoding='utf8'))
    prior = records(path / 'v16_energy_events.csv')
    series = [r for r in records(path / 'timeseries.csv') if r['state_id'] == '3']
    peaks = meta['energy_control_autonomous_peak_events']
    confirmations = [max(p['peak_time_ms'], p['peak_telemetry_capture_time_ms']) for p in peaks]
    ready = {r['sequence']: int(r['ready_ms']) for r in records(path / 'foot_pose_prediction.csv')}
    feet = []
    for f in records(path / 'foot_angles.csv'):
        if not f['measurement_time_us'] or f['sequence'] not in ready:
            continue
        valid = all(f[k] in ['True', 'true', '1'] for k in
                    ['timestamp_valid', 'frame_valid', 'zero_ready', 'left_valid', 'right_valid', 'left_in_range', 'right_in_range'])
        feet.append((ready[f['sequence']] + delay, int(float(f['measurement_time_us']) / 1000),
                     float(f['left_deg']) if valid else 0, float(f['right_deg']) if valid else 0, valid))
    feet.sort()
    foot_times = [f[0] for f in feet]
    sent = [a for a in meta['walking_energy']['records'] if a['executed']]
    consumed = {a['peak_ms']: a['time_ms'] for a in sent}
    events = []
    for i, r in enumerate(prior):
        values = [r[k] for k in ['side', 'rate_dps', 'model_body_deg', 'target_deg', 'previous_peak_deg']]
        values += [r['same_side_peak_deg'] or '-1']
        values += [r[k] for k in ['aligned_i0_ma', 'goal_current_ma', 'cap_ms']]
        events.append((float(r['time_ms']), 'S ' + str(i) + ' ' + r['time_ms'] + ' ' + ' '.join(values)))
        if r['prediction_usable'] == 'True':
            p = next(p for p in peaks if p['peak_time_ms'] == int(r['next_peak_ms']))
            events.append((max(p['peak_time_ms'], p['peak_telemetry_capture_time_ms']),
                           'F ' + str(i) + ' ' + ' '.join(str(x) for x in
                            [r['side'], p['peak_time_ms'], r['observed_nominal_peak_j'], r['elapsed_arrival_pred_j'], r['target_j']])))
    seen_samples = set()
    for i, r in enumerate(series):
        if r['body_rate_sample_time_us'] in seen_samples:
            continue
        seen_samples.add(r['body_rate_sample_time_us'])
        t = int(r['t_test_ms'])
        pi = bisect.bisect_right(confirmations, t) - 1
        fi = bisect.bisect_right(foot_times, t) - 1
        if pi < 0 or fi < 0:
            continue
        p, f = peaks[pi], feet[fi]
        active = r['pulse_active'] == '0' and consumed.get(p['peak_time_ms'], 10**9) > t
        values = [t, p['peak_time_ms'], p['physical_peak_side'], p['peak_amplitude_deg'],
                  r['pitch_mekf_detector_relative_deg'], r['physical_roll_rate_dps'], f[1], f[0],
                  f[2], f[3], p['target_energy_j'], int(active), int(f[4])]
        events.append((t, 'D ' + str(i) + ' ' + ' '.join(map(str, values))))
    result = subprocess.run([str(exe)], input='\n'.join(s for _, s in sorted(events)) + '\n',
                            text=True, capture_output=True, check=True)
    seen = set()
    candidates, corrections = [], []
    for line in result.stdout.splitlines():
        v = line.split()
        if v[0] == 'D' and v[3] not in seen:
            seen.add(v[3])
            candidates.append(dict(zip(['time_ms', 'peak_ms', 'request_j', 'stopping_distance_deg', 'deceleration_dps2'], map(float, v[2:]))))
        if v[0] == 'S':
            r = prior[int(v[1])]
            corrections.append({'time_ms': int(r['time_ms']), 'side': int(r['side']), 'bias_j': float(v[2]),
                                'original_selector_ms': int(v[3]), 'corrected_selector_ms': int(v[4]),
                                'cap_limited': bool(int(v[6])), 'early_admissible_before_wheel_guard': bool(int(v[7]))})
    assert len(corrections) == len(prior), 'Incomplete selector replay'
    return {'extra_foot_ready_delay_ms': delay, 'return_candidates': candidates, 'candidate_corrections': corrections}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='return_energy_v17_') as folder:
        temp = Path(folder)
        source = temp / 'test.cpp'
        source.write_text(CPP, encoding='utf8')
        exe = temp / 'test.exe'
        subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-misleading-indentation',
                        '-I' + str(ROOT / 'src'), str(source), '-o', str(exe)], check=True)
        print(subprocess.run([str(exe)], text=True, capture_output=True, check=True).stdout.strip())
        if args.trial:
            probes = [probe(exe, args.trial, delay) for delay in [0, 10, 25]]
            for p in probes:
                final = [x for x in p['return_candidates'] if x['peak_ms'] == 20761]
                assert final and final[0]['time_ms'] < 20966, 'Missed final recorded inward stall'
                print('extra delay', p['extra_foot_ready_delay_ms'], 'final return probe', final[0])
            if args.out:
                args.out.parent.mkdir(parents=True, exist_ok=True)
                args.out.write_text(json.dumps({'semantics': 'recorded_state_probes_only_not_changed_plant_trajectory',
                                                'probes': probes}, indent=2) + '\n', encoding='utf8')


if __name__ == '__main__':
    main()
