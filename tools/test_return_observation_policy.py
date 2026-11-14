"""Compile v18's portable observer and verify its observation/IMU contracts.

Optional recorded-state replay uses only frames available by ready/control
receipt time. It compares candidate detection, not the changed plant response.
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
#include "return_observation_policy.h"
#include <cassert>
#include <cstdlib>
#include <iomanip>
#include <iostream>
namespace p=return_observation_policy;
#undef assert
#define assert(test) do {if(!(test)){std::cerr<<"assertion failed: "<<#test<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
static p::Input input(unsigned elapsed=0) {
 p::Input in;in.run_id=1;in.now=1200+elapsed;in.peak_ms=1000;in.sample_us=1000000+elapsed*1000;
 in.side=1;in.peak_deg=10.6f;in.angle_deg=8.5f;in.rate_dps=-(30.f-elapsed*.35f);in.target_j=.000733f;in.active=true;
 in.foot.run_id=1;in.foot.frame_ms=1150;in.foot.ready_ms=1190;
 in.foot.timestamp_valid=in.foot.frame_valid=in.foot.zero_ready=true;
 in.foot.left_detected=in.foot.right_detected=true;in.foot.left_range_ok=in.foot.right_range_ok=true;
 in.foot.left_deg=15;in.foot.right_deg=0;return in;
}
static void tests() {
 for(int side:{1,-1}) {
  p::Observer observed,dropout;bool first=false,second=false;
  for(unsigned t=0;t<=60;t+=2) {
   auto in=input(t);in.side=side;in.angle_deg*=side;in.rate_dps*=side;
   auto q=observed.observe(in);first|=q.selected;
   in.foot.left_detected=false;in.foot.left_range_ok=false;in.foot.left_deg=NAN;
   auto r=dropout.observe(in);second|=r.selected;
   if(r.selected){assert(r.evidence==p::Evidence::RecognitionDropout);assert(r.deceleration_span_ms>=40);
    assert(r.request_j>0 && r.request_j<=.25f*in.target_j);}
  }
  assert(first && second);
 }
 // Camera invalidation keeps independent IMU history. No prior shape amount
 // is required for dropout; the final trial had unseparated prior markers.
 {p::Observer d;bool selected=false;
  for(unsigned t=0;t<=60;t+=2){auto in=input(t);in.foot.left_deg=1;
   if(t>=30){in.foot.left_detected=false;in.foot.left_deg=NAN;in.foot.left_range_ok=false;}
   const auto q=d.observe(in);if(q.selected){selected=true;assert(q.evidence==p::Evidence::RecognitionDropout);assert(in.now<=1260);}}
  assert(selected);}
 // Invalid angle storage is ignored, even when it contains an old angle.
 {auto in=input();in.foot.left_detected=false;in.foot.left_range_ok=false;in.foot.left_deg=500;
  assert(p::assessFoot(in)==p::Evidence::RecognitionDropout);}
 for(int mode=0;mode<15;++mode) {
  p::Observer d;bool selected=false;
  for(unsigned t=0;t<=60;t+=2){auto in=input(t);
   in.foot.left_detected=false;in.foot.left_range_ok=false;in.foot.left_deg=NAN;
   if(mode==0)in.foot.run_id=2;
   if(mode==1)in.foot.frame_ms=900;
   if(mode==2)in.foot.ready_ms=in.now+1;
   if(mode==3)in.foot.timestamp_valid=false;
   if(mode==4)in.foot.frame_valid=false;
   if(mode==5)in.foot.zero_ready=false;
   if(mode==6){in.foot.right_range_ok=false;in.foot.right_deg=40;}
   if(mode==7)in.rate_dps=-in.rate_dps; // outward
   if(mode==8)in.rate_dps=-(10.f+t*.3f); // inward acceleration
   if(mode==9)in.sample_age_us=10001;
   if(mode==10)in.sample_us=1000000; // duplicate IMU
   if(mode==11)in.sample_us+=t*6000; // acquisition gaps >10ms
   if(mode==12)in.foot.right_deg=NAN; // claimed detected but no numeric angle
   if(mode==13)in.now+=t*6; // delivery gaps >10ms
   if(mode==14)in.rate_dps=-(30.f-t*.05f); // weak deceleration
   selected|=d.observe(in).selected;
  }assert(!selected);
 }
 // Known range rejection cannot turn into a dropout exception one frame later.
 {p::Observer d;bool selected=false;
  for(unsigned t=0;t<=60;t+=2){auto in=input(t);
   if(t==0){in.foot.left_range_ok=false;in.foot.left_deg=40;}
   else{in.foot.left_detected=false;in.foot.left_deg=NAN;in.foot.left_range_ok=false;}
   const auto q=d.observe(in);selected|=q.selected;assert(q.range_veto);}
  assert(!selected);}
 // A new accepted range observation clears the veto but still requires shape.
 {p::Observer d;auto in=input();in.foot.left_range_ok=false;assert(d.observe(in).range_veto);
  in=input(2);in.foot.left_deg=1;auto q=d.observe(in);assert(!q.range_veto && !q.selected);}
 // Consume survives inactive samples/gaps. A genuinely new peak/run rearms.
 {p::Observer d;d.consume(1,1000);
  for(unsigned t=0;t<=60;t+=2){auto in=input(t);assert(!d.observe(in).selected);}
  auto in=input(62);in.active=false;assert(!d.observe(in).selected);
  in=input(64);in.sample_us+=50000;assert(!d.observe(in).selected);
  in=input(66);in.run_id=0;assert(!d.observe(in).selected);
  in=input(68);in.peak_ms=0;assert(!d.observe(in).selected);
  for(unsigned t=0;t<=60;t+=2){in=input(t);assert(!d.observe(in).selected);}
  bool selected=false;for(unsigned t=0;t<=60;t+=2){in=input(t);in.run_id=2;in.foot.run_id=2;selected|=d.observe(in).selected;}assert(selected);
  d.consume(2,1000);selected=false;for(unsigned t=0;t<=60;t+=2){in=input(t);in.run_id=2;in.foot.run_id=2;in.peak_ms=1001;selected|=d.observe(in).selected;}assert(selected);}
 // A new run/peak or a gap must rebuild motion evidence, not reuse a derivative.
 for(int mode=0;mode<3;++mode){p::Observer d;
  for(unsigned t=0;t<=24;t+=2)d.observe(input(t));
  auto in=input(26);if(mode==0){in.run_id=2;in.foot.run_id=2;}if(mode==1)in.peak_ms=1001;if(mode==2){in.now+=20;in.sample_us+=20000;}
  auto q=d.observe(in);assert(!q.selected && q.deceleration_dps2==0);}
 std::cout<<"observation policy: fresh shape, dropout, range veto, time/run/IMU gaps, outward exclusion, consume PASS\n";
}
int main(){tests();std::cout<<std::setprecision(12);p::Observer d;unsigned id;
 while(std::cin>>id){p::Input in;int active,timestamp,frame,zero,left,right,lr,rr;
  std::cin>>in.run_id>>in.now>>in.peak_ms>>in.sample_us>>in.sample_age_us>>in.side>>in.peak_deg>>in.angle_deg>>in.rate_dps>>in.target_j>>active
    >>in.foot.run_id>>in.foot.frame_ms>>in.foot.ready_ms>>timestamp>>frame>>zero>>left>>right>>lr>>rr>>in.foot.left_deg>>in.foot.right_deg;
  in.active=active;in.foot.timestamp_valid=timestamp;in.foot.frame_valid=frame;in.foot.zero_ready=zero;
  in.foot.left_detected=left;in.foot.right_detected=right;in.foot.left_range_ok=lr;in.foot.right_range_ok=rr;
  const auto q=d.observe(in);if(q.selected){std::cout<<"D "<<id<<' '<<in.now<<' '<<in.peak_ms<<' '<<q.request_j<<' '<<q.stopping_distance_deg<<' '<<q.deceleration_dps2<<' '<<unsigned(q.evidence)<<' '<<q.foot_age_ms<<' '<<q.deceleration_span_ms<<'\n';d.consume(in.run_id,in.peak_ms);}
 }
}
'''


def records(path):
    with path.open(encoding='utf8', newline='') as stream:
        return list(csv.DictReader(stream))


def truth(value):
    return value in ['True', 'true', '1']


def replay(exe, path, delay):
    meta = json.loads((path / 'metadata.json').read_text(encoding='utf8'))
    peaks = meta['energy_control_autonomous_peak_events']
    confirmations = [max(p['peak_time_ms'], p['peak_telemetry_capture_time_ms']) for p in peaks]
    ready = {r['sequence']: int(r['ready_ms']) for r in records(path / 'foot_local_prediction.csv')}
    # These are availability probes. Use observed receipt when recorded;
    # otherwise ready time plus specified delivery delay is an explicit model.
    feet = []
    for f in records(path / 'foot_angles.csv'):
        if int(f['measurement_time_us']) < 0 or f['sequence'] not in ready:
            continue
        frame = int(f['measurement_time_us']) // 1000
        receipt = ready[f['sequence']] + delay
        if f.get('control_observed_us'):
            # Both frame_us and control_observed_us share the monotonic clock.
            receipt = max(receipt, frame + (int(f['control_observed_us']) - int(f['frame_us'])) // 1000)
        feet.append((receipt, frame, f))
    feet.sort(key=lambda f: f[0])
    foot_times = [f[0] for f in feet]
    consumed = {a['peak_ms']: a['time_ms'] for a in meta['walking_energy']['records'] if a['executed']}
    seen = set()
    lines = []
    for i, r in enumerate(records(path / 'timeseries.csv')):
        if r['state_id'] != '3' or not r['body_rate_sample_time_us'] or r['body_rate_sample_time_us'] in seen:
            continue
        seen.add(r['body_rate_sample_time_us'])
        t = int(r['t_test_ms'])
        pi, fi = bisect.bisect_right(confirmations, t) - 1, bisect.bisect_right(foot_times, t) - 1
        if pi < 0 or fi < 0:
            continue
        p, (receipt, frame, f) = peaks[pi], feet[fi]
        values = [i, f['run_id'], t, p['peak_time_ms'], r['body_rate_sample_time_us'], r['imu_sample_age_us'],
                  p['physical_peak_side'], p['peak_amplitude_deg'], r['pitch_mekf_detector_relative_deg'],
                  r['physical_roll_rate_dps'], p['target_energy_j'],
                  int(r['pulse_active'] == '0' and consumed.get(p['peak_time_ms'], 10**9) > t),
                  f['run_id'], frame, receipt]
        values += [int(truth(f[k])) for k in ['timestamp_valid', 'frame_valid', 'zero_ready', 'left_valid', 'right_valid',
                                            'left_control_range_ok', 'right_control_range_ok']]
        # Missing angles are placeholders only; assessFoot never uses them.
        values += [f['left_deg'] or 0, f['right_deg'] or 0]
        lines.append(' '.join(map(str, values)))
    result = subprocess.run([str(exe)], input='\n'.join(lines) + '\n', text=True, capture_output=True, check=True)
    found = []
    for line in result.stdout.splitlines():
        if not line.startswith('D '):
            continue
        v = line.split()
        names = ['time_ms', 'peak_ms', 'request_j', 'stopping_distance_deg', 'deceleration_dps2',
                 'evidence', 'foot_age_ms', 'deceleration_span_ms']
        found.append(dict(zip(names, map(float, v[2:]))))
    return {'extra_ready_delay_ms': delay, 'candidates': found}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='return_observation_policy_') as folder:
        temp = Path(folder)
        source, exe = temp / 'test.cpp', temp / 'test.exe'
        source.write_text(CPP, encoding='utf8')
        subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(ROOT / 'src'), str(source), '-o', str(exe)], check=True)
        print(subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.strip())
        if args.trial:
            probes = [replay(exe, args.trial, delay) for delay in [0, 10, 25]]
            for p in probes:
                final = [c for c in p['candidates'] if c['peak_ms'] == 24214]
                if args.trial.name == '39825069':
                    assert final and final[0]['time_ms'] < 24427, 'Missed recorded final inward dropout'
                print('delay', p['extra_ready_delay_ms'], 'final', final)
            if args.out:
                args.out.write_text(json.dumps({'semantics': 'recorded_state_candidate_probes_not_changed_plant_trajectory',
                                                'probes': probes}, indent=2) + '\n', encoding='utf8')


if __name__ == '__main__':
    main()
