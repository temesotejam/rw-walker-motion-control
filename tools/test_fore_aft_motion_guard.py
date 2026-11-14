"""Native v19 motion-envelope tests and fixed-record-state replay.

Replay reconstructs gravity from rounded MEKF Euler roll/pitch, with yaw=0;
the gravity projection is yaw-independent. Logged body gyro can clip. This is
not a quaternion-exact hardware replay or a plant trajectory after stopping.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

from analyze_v14_trial_energy import ROOT, plt, stats

CPP = r'''
#include "fore_aft_motion_guard.h"
#include <cassert>
#include <iostream>
#include <iomanip>
namespace f=fore_aft_motion_guard;
mekf6::Quaternion quat(float roll,float pitch,float yaw=0){
 const float r=roll*.008726646259971648f,p=pitch*.008726646259971648f,y=yaw*.008726646259971648f;
 return {std::cos(r)*std::cos(p)*std::cos(y)+std::sin(r)*std::sin(p)*std::sin(y),
 std::sin(r)*std::cos(p)*std::cos(y)-std::cos(r)*std::sin(p)*std::sin(y),
 std::cos(r)*std::sin(p)*std::cos(y)+std::sin(r)*std::cos(p)*std::sin(y),
 std::cos(r)*std::cos(p)*std::sin(y)-std::sin(r)*std::sin(p)*std::cos(y)};
}
f::Input input(uint32_t us,float roll,float pitch=0,float yaw=0){f::Input x;
 x.now_us=x.sample_us=us;x.valid=true;x.quaternion=quat(roll,pitch,yaw);return x;}
void tests(){
 {f::Guard g;assert(!g.result().initialized && !g.result().allow_output);
 auto a=g.observe(input(1000,7));assert(a.initialized && a.allow_output && a.angle_deg==0);
 auto b=g.observe(input(3500,8));assert(std::fabs(b.angle_deg-1)<.001f && b.allow_output);
 assert(b.start_sample_us==1000 && std::fabs(b.start_roll_deg-7)<.001f);}
 for(int sign:{-1,1}){f::Guard g;bool stopped=false;uint32_t stop_us=0;
  for(uint32_t t=1000;t<700000;t+=2500){auto x=input(t,sign*40.f*(t-1000)*1e-6f);
   x.gyro_rad_s.x=sign*40.f*.0174532925199433f;const auto a=g.observe(x);
   if(a.reason==f::Reason::Candidate)assert(!a.allow_output && !a.stop);
   if(a.stop){stopped=true;stop_us=t;assert(a.reason==f::Reason::Diverging);break;}}
  assert(stopped && stop_us>=188000 && stop_us<=200000);
  assert(g.observe(input(stop_us+2500,0)).stop);g.reset();assert(!g.observe(input(1000,0)).stop);
 }
 {f::Guard g;for(uint32_t t=1000;t<2000000;t+=2500){
   const float seconds=(t-1000)*1e-6f;auto x=input(t,5.8f*std::sin(12.566370614f*seconds));
   x.gyro_rad_s.x=5.8f*12.566370614f*std::cos(12.566370614f*seconds)*.0174532925199433f;
   auto a=g.observe(x);assert(!a.stop && a.allow_output);}}
 {f::Guard g;for(uint32_t t=1000;t<2000000;t+=2500){
   const float seconds=(t-1000)*1e-6f;auto x=input(t,8.f*std::sin(6.283185307f*seconds));
   x.gyro_rad_s.x=8.f*6.283185307f*std::cos(6.283185307f*seconds)*.0174532925199433f;
   assert(!g.observe(x).stop);}}
 {f::Guard g;g.observe(input(1000,0));auto x=input(3500,7);x.gyro_rad_s.x=100*.0174532925199433f;
  auto a=g.observe(x);assert(a.reason==f::Reason::Candidate && !a.allow_output);
  for(unsigned t=6000;t<30000;t+=2500){x=input(t,7.2f);x.gyro_rad_s.x=100*.0174532925199433f;assert(!g.observe(x).stop);}
  assert(g.observe(input(31000,7.2f)).allow_output);}
 {f::Guard g;g.observe(input(1000,0));
  for(unsigned t=3500;t<110000;t+=2500){auto x=input(t,7);x.gyro_rad_s.x=100*.0174532925199433f;
   assert(!g.observe(x).stop);} // No measured angle gain: gyro alone cannot confirm.
  g.reset();auto x=input(1000,0);g.observe(x);x=input(3500,7);x.gyro_rad_s.x=100*.0174532925199433f;
  g.observe(x);for(unsigned t=4000;t<13000;t+=1000){x.now_us=t;const auto a=g.observe(x);
   assert(!a.stop && !a.sample_advanced && a.candidate_us==0);}}
 for(int sign:{-1,1}){f::Guard g;g.observe(input(1000,3));
  for(unsigned i=1;i<=880;++i){auto x=input(1000+i*2500,3,sign*i*.125f,37);
   x.gyro_rad_s.y=sign*50.f*.0174532925199433f*std::cos(3.f*.0174532925199433f);
   x.gyro_rad_s.z=-sign*50.f*.0174532925199433f*std::sin(3.f*.0174532925199433f);
   auto a=g.observe(x);assert(!a.stop);if(a.valid)assert(std::fabs(a.angle_deg)<.002f);}}
 // Verify derivative independently by advancing a quaternion with body rates.
 {f::Guard g;auto x=input(1000,0);g.observe(x);x=input(3500,12,30,41);
  x.gyro_rad_s={.4f,.7f,-.3f};auto a=g.observe(x);
  const auto q=x.quaternion;const float dt=.00001f;
  const mekf6::Quaternion d{1,.5f*dt*.4f,.5f*dt*.7f,.5f*dt*-.3f};
  x.quaternion={q.w*d.w-q.x*d.x-q.y*d.y-q.z*d.z,q.w*d.x+q.x*d.w+q.y*d.z-q.z*d.y,
    q.w*d.y-q.x*d.z+q.y*d.w+q.z*d.x,q.w*d.z+q.x*d.y-q.y*d.x+q.z*d.w};
  x.now_us=x.sample_us=3510;const auto b=g.observe(x);
  assert(std::fabs((b.angle_deg-a.angle_deg)/dt-a.rate_dps)<.2f);
  assert(std::fabs(a.rate_dps-.4f*57.2957795f)>1);}
 {f::Guard a;auto x=input(1000,0);x.now_us=11001;assert(a.observe(x).reason==f::Reason::Stale);
  f::Guard b;x=input(1000,0);x.valid=false;assert(b.observe(x).reason==f::Reason::Invalid);
  f::Guard c;c.observe(input(1000,0));auto first=c.observe(input(3500,0));
  assert(first.sample_advanced && !c.observe(input(3500,0)).sample_advanced);}
 {f::Guard g;g.observe(input(1000,3,20,41));auto x=input(3500,3,20,41);
  x.quaternion.w=-x.quaternion.w;x.quaternion.x=-x.quaternion.x;
  x.quaternion.y=-x.quaternion.y;x.quaternion.z=-x.quaternion.z;
  const auto a=g.observe(x);assert(a.allow_output && std::fabs(a.angle_deg)<.001f);}
 {f::Guard g;const uint32_t origin=UINT32_MAX-100000;
  bool stopped=false;for(uint32_t dt=0;dt<300000;dt+=2500){auto x=input(origin+dt,40.f*dt*1e-6f);
   x.gyro_rad_s.x=40*.0174532925199433f;if(g.observe(x).stop){stopped=true;break;}}
  assert(stopped);}
 std::cout<<"portable PASS\n";
}
int main(){tests();std::cout<<std::setprecision(12);f::Guard guard;char command;unsigned id;
 while(std::cin>>command>>id){
  if(command=='R'){guard.reset();continue;}
  f::Input x;float roll,pitch,gx,gy,gz,bx,by,bz,scale;
  std::cin>>x.now_us>>x.sample_us>>roll>>pitch>>gx>>gy>>gz>>bx>>by>>bz>>scale;
  x.valid=true;x.quaternion=quat(roll,pitch);
  x.gyro_rad_s={-gx*.0174532925199433f,gy*scale*.0174532925199433f,-gz*.0174532925199433f};
  x.bias_rad_s={-bx*.0174532925199433f,by*scale*.0174532925199433f,-bz*.0174532925199433f};
  auto a=guard.observe(x);std::cout<<id<<' '<<a.stop<<' '<<a.allow_output<<' '<<a.valid<<' '
   <<unsigned(a.reason)<<' '<<a.angle_deg<<' '<<a.rate_dps<<' '<<a.time_to_boundary_ms<<' '<<a.candidate_us<<'\n';
 }
}
'''


def read_csv(path):
    with path.open(encoding='utf8', newline='') as stream:
        return list(csv.DictReader(stream))


def compile_fixture(temp):
    hashes = {}
    for name in ['fore_aft_motion_guard.h', 'mekf6.hpp', 'realtime_code.h']:
        blob = (ROOT / 'src' / name).read_bytes(); (temp / name).write_bytes(blob)
        hashes[name] = hashlib.sha256(blob).hexdigest()
    (temp / 'guard.cpp').write_text(CPP, encoding='utf8'); exe = temp / 'guard.exe'
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror', '-I' + str(temp),
                    str(temp / 'guard.cpp'), '-o', str(exe)], check=True)
    return exe, hashes


def replay(path, exe):
    meta = json.loads((path / 'metadata.json').read_text(encoding='utf8'))
    source = [r for r in read_csv(path / 'timeseries.csv') if r['state_id'] == '3' and r['roll_mekf_abs_deg']]
    commands = ['R 0']
    # Sensor samples are in a separate clock from t_test_ms. Use their recorded
    # timestamps and age; do not turn CSV row spacing into a fresh IMU clock.
    for i, r in enumerate(source):
        sample = int(r['body_rate_sample_time_us']); now = sample + int(r['imu_sample_age_us'])
        values = [i, now & 0xffffffff, sample, r['roll_mekf_abs_deg'], r['pitch_mekf_abs_deg'],
                  r['gx_dps'], r['gy_dps'], r['gz_dps'], r['mekf_bias_x_dps'], r['mekf_bias_y_dps'],
                  r['mekf_bias_z_dps'], meta['mekf_gyro_y_scale']]
        commands.append('S ' + ' '.join(map(str, values)))
    result = subprocess.run([str(exe)], input='\n'.join(commands) + '\n', text=True, capture_output=True, check=True)
    lines = result.stdout.splitlines(); assert lines.pop(0) == 'portable PASS'
    assert len(lines) == len(source)
    rows = []
    for line in lines:
        values = line.split(); r = source[int(values[0])]
        floats = [float(value) for value in values[5:8]]
        row = {'time_ms': float(r['t_test_ms']), 'sample_us': int(r['body_rate_sample_time_us']),
               'stop': bool(int(values[1])), 'allow_output': bool(int(values[2])), 'valid': bool(int(values[3])),
               'reason': int(values[4]), 'fore_aft_relative_deg': floats[0] if math.isfinite(floats[0]) else None,
               'projected_fore_aft_rate_dps': floats[1] if math.isfinite(floats[1]) else None,
               'time_to_20deg_ms': floats[2] if math.isfinite(floats[2]) else None,
               'candidate_us': int(values[8]), 'raw_roll_deg': float(r['roll_mekf_abs_deg']),
               'lateral_pitch_deg': float(r['pitch_mekf_abs_deg']), 'command_ma': int(r['motor_cmd_mA'])}
        rows.append(row)
    stopped = next((r for r in rows if r['stop']), None)
    candidate = next((r for r in rows if r['reason'] == 3), None)
    outputs = [a for a in meta.get('walking_energy', {}).get('records', []) if a['executed']]
    return {'case': path.name, 'path': str(path.relative_to(ROOT)), 'build': meta['diagnostic_build_id'],
            'samples': len(rows), 'first_candidate': candidate, 'first_stop': stopped,
            'vetoed_recorded_pulses': [a['time_ms'] for a in outputs if
                not min(rows, key=lambda r: abs(r['time_ms'] - a['time_ms']))['allow_output']],
            'angle_relative_deg': stats([r['fore_aft_relative_deg'] for r in rows if r['valid'] and not r['stop']]),
            'reason_counts': dict(Counter(r['reason'] for r in rows)),
            'replay_limit': 'Rounded Euler-derived gravity; exact yaw not needed for projection; archived bias/rates and sensor ages used. Not a new plant trajectory.'}, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT / 'analysis/2026-11-14-walking')
    args = parser.parse_args()
    cases = [ROOT / 'analysis/2026-11-14-walking' / name for name in ['62788986', '39825069', '28820005']]
    cases += [ROOT / 'analysis/2026-11-13-walking' / name for name in ['172594684', '294107213']]
    reviews = []; traces = {}
    with tempfile.TemporaryDirectory(prefix='v19_fore_aft_') as folder:
        exe, hashes = compile_fixture(Path(folder))
        for path in cases:
            review, rows = replay(path, exe); reviews.append(review); traces[path.name] = rows
    assert reviews[0]['first_stop'] and reviews[0]['first_stop']['reason'] == 4
    assert all(r['first_stop'] is None for r in reviews[1:]), 'Comparison trace STOP requires investigation'
    assert all(r['first_candidate'] is None for r in reviews[1:]), 'Comparison new-output veto requires investigation'
    report = {'revision': 'fore_aft_motion_guard_v19', 'portable_tests': 'PASS', 'source_sha256': hashes,
        'trial_envelope': {'candidate_angle_deg': 6, 'minimum_outward_rate_dps': 20, 'prediction_boundary_deg': 20,
                           'maximum_time_to_boundary_ms': 350, 'confirmation_ms': 40, 'minimum_outward_gain_deg': 1},
        'basis': 'Motion-only trial limits selected to separate the sustained recorded fall from observed few-degree oscillations; no identified stability, contact, inertia, J or physical recovery guarantee.',
        'candidate_action': 'Veto new output; confirmed divergence latches STOP and applies during an active pulse.',
        'rates': 'Projection derivative from g cross (mapped gyro minus bias); body X alone is not Euler roll derivative.',
        'cases': reviews, 'limits': ['No physical trajectory is predicted after an earlier stop or veto.',
            'Comparison no-STOP is a recorded-state false-positive check, not proof of walking or safety.',
            'Near sideways +/-90 the projection is unobservable: veto output without inventing fore/aft divergence.',
            'The independent existing 90-degree stop remains necessary; this guard detects outward growth rather than all unsafe static postures.']}
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'v19_fore_aft_replay.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf8')
    with (args.out / 'v19_fore_aft_replay.csv').open('w', encoding='utf8', newline='') as stream:
        fields = ['case'] + list(next(iter(traces.values()))[0]); writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case, rows in traces.items(): writer.writerows(dict(case=case, **r) for r in rows)
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
    for case, rows in traces.items():
        q = [r for r in rows if not r['stop']]
        axes[0].plot([r['time_ms'] / 1000 for r in q], [r['fore_aft_relative_deg'] for r in q], lw=.7, label=case)
    fall = traces['62788986']; stop = reviews[0]['first_stop']['time_ms']; candidate = reviews[0]['first_candidate']['time_ms']
    axes[1].plot([r['time_ms'] / 1000 for r in fall], [r['raw_roll_deg'] for r in fall], color='tab:red', label='recorded raw fore/aft roll')
    axes[2].plot([r['time_ms'] / 1000 for r in fall if r['time_ms'] <= stop],
                 [r['projected_fore_aft_rate_dps'] for r in fall if r['time_ms'] <= stop], label='projected fore/aft rate before STOP')
    for ax in axes:
        ax.axvline(candidate / 1000, color='tab:orange', ls=':', label='first candidate / new-output veto')
        ax.axvline(stop / 1000, color='tab:red', ls='--', label='confirmed STOP')
        ax.grid(alpha=.2); ax.legend(fontsize=7, ncol=2); ax.set_xlim(0, 30)
    axes[0].set_ylabel('Run-relative fore/aft [deg]'); axes[1].set_ylabel('Fall trace roll [deg]')
    axes[2].set_ylabel('Projected rate [deg/s]'); axes[2].set_xlabel('Measurement time [s]')
    fig.suptitle('v19 motion-envelope replay on recorded states\nNo changed physical trajectory or recovered walking is predicted')
    fig.savefig(args.out / 'v19_fore_aft_replay.png', dpi=150, bbox_inches='tight'); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True, constrained_layout=True)
    origin = fall[0]['raw_roll_deg']
    axes[0].plot([r['time_ms'] / 1000 for r in fall], [r['raw_roll_deg'] - origin for r in fall],
                 color='black', label='recorded fore/aft roll minus start roll (unmodified trace)')
    before = [r for r in fall if r['time_ms'] <= stop]
    axes[1].plot([r['time_ms'] / 1000 for r in before],
                 [r['projected_fore_aft_rate_dps'] for r in before], label='projected angular rate until STOP')
    axes[2].plot([r['time_ms'] / 1000 for r in fall], [r['command_ma'] for r in fall], color='grey',
                 label='recorded motor command; unchanged input trace')
    for ax in axes:
        ax.axvline(candidate / 1000, color='tab:orange', ls=':', label='new-output veto')
        ax.axvline(stop / 1000, color='tab:red', ls='--', label='confirmed STOP')
        ax.grid(alpha=.2); ax.legend(fontsize=7, ncol=2); ax.set_xlim(16.15, 17.0)
    axes[0].set_ylabel('Relative fore/aft [deg]'); axes[1].set_ylabel('Projected rate [deg/s]')
    axes[2].set_ylabel('Command [mA]'); axes[2].set_xlabel('Measurement time [s]')
    fig.suptitle('62788986 / v19 guard on recorded states\n'
                 'Motion after an earlier veto/STOP is not predicted; these curves remain the original trial')
    fig.savefig(args.out / 'v19_fore_aft_detail.png', dpi=150, bbox_inches='tight'); plt.close(fig)
    print(json.dumps({'portable_tests': 'PASS', 'cases': [{'case': r['case'], 'candidate': r['first_candidate'],
                      'stop': r['first_stop']} for r in reviews]}, indent=2))


if __name__ == '__main__':
    main()
