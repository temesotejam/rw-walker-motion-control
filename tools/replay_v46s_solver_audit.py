#!/usr/bin/env python3
"""Replay V46s RWLOG audit offline. g++ is required; the motor is never accessed."""
import argparse
import csv
import hashlib
import json
import re
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path
from convert_rwlog_to_csv import parse_header, verify_crc
from v46t_current_observation_contract import normalize_current_observation
from v46u_timing_contract import original_timing_file
from v46ac_delay_comp_contract import normalize_runner as normalize_v46ac_runner
from v46ab_no_prediction_contract import normalize_runner as normalize_v46ab_runner
from v46aa_control_zero_contract import normalize_runner as normalize_v46aa_runner
from v46z_comparison_zero_contract import normalize_runner as normalize_v46z_runner
from v46ak_observation_contract import normalize_runner as normalize_v46ak_runner
from v46al_control_contract import normalize_runner as normalize_v46al_runner

ROOT = Path(__file__).resolve().parents[1]
FLOAT_FIELDS = 'i0_mA free_peak_deg target_peak_deg target_energy_j passive_energy_j q_available_mA_s integral_mA_s signed_target_current_mA tau_s base_gain correction_c correction_gain correction_limit ff_q_mA_s selected_q_mA_s corrected_target_energy_j'.split()

def original_runner():
    text = normalize_v46ak_runner((ROOT / 'src/experiment_runner.cpp').read_text())
    text = normalize_v46al_runner(text)
    text = normalize_current_observation(text)
    text = normalize_v46ac_runner(text)
    text = normalize_v46ab_runner(text)
    text = normalize_v46aa_runner(text)
    text = normalize_v46z_runner(text)
    return re.sub(r'\n  // V46s audit begin\n.*?\n  // V46s audit end', '', text, flags=re.S)

def method(text, name):
    m = re.search(r'(?:float|void) ExperimentRunner::' + re.escape(name) + r'\(', text)
    if not m:
        raise ValueError('missing model method: ' + name)
    start = text.index('{', m.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[m.start():end].replace('ExperimentRunner::', '')

def build_driver(directory):
    """Extract the actual firmware evaluator and fast search, not a Python imitation."""
    text = original_runner()
    baseline = json.loads((ROOT / 'tools/v46s_audit_baseline.json').read_text())
    if hashlib.sha256(text.encode()).hexdigest() != baseline['runner_sha256']:
        raise ValueError('controller changed: update the versioned replay model before using this tool')
    config = original_timing_file('src/config.h').decode().replace('v46t_current_observation_20260915', 'v46s_solver_audit_20260915').replace('v46s_solver_audit_20260915', 'v46r_fast_solver_control_20260915')
    if hashlib.sha256(config.encode()).hexdigest() != baseline['config_sha256']:
        raise ValueError('configuration changed: replay model requires review')
    control = text[text.index('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross'):]
    block = control[control.index('  struct FastCandidate {'):control.index('  const uint32_t v46r_fast_solver_t0_us')]
    # Recorded cached float32 values avoid repeating on-device powf on another CPU.
    block = re.sub(r'  const float fast_v =.*?  auto evaluate_width',
                   '  const float fast_signed_target_current_mA = r.signed_target_current_mA;\n  const float fast_tau_s = r.tau_s;\n\n  auto evaluate_width',block,flags=re.S)
    potential = method(text, 'energyControlPotentialJ')
    correction = method(text, 'energyControlAutonomousCorrectedPrediction')
    source = r'''#include <iostream>
#include <iomanip>
#include <limits>
#include <math.h>
#include "config.h"
#include "solver_audit.h"
struct Result { bool valid=false; unsigned ff=65535, selected=65535, candidate=65535, evaluations=0; float ffq=NAN, q=NAN; };
struct Model {
  solver_audit::Record r;
  float energyControlAutonomousGainForSide(int8_t) const { return r.base_gain; }
  void energyControlAutonomousCorrectionParameters(int8_t, float* c, float* g) const { *c=r.correction_c; *g=r.correction_gain; }
'''+potential+'\n'+correction+r'''
  Result replay(bool exhaustive) {
    struct Event { float i0_estimated_mA, free_next_peak_amplitude_deg; int8_t physical_next_peak_side, q_command_direction; };
    const Event event{r.i0_mA, r.free_peak_deg, r.physical_side, r.command_direction};
'''+block+r'''
    auto legacy_pick = [&](float target, float zero_energy) -> FastCandidate {
      FastCandidate best; best.valid=true; best.width_ms=0; best.q_mA_s=0;
      best.energy_j=zero_energy; best.error_j=fabsf(target-zero_energy);
      for (uint16_t w=Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS; w<=Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS; ++w) {
        const FastCandidate c=evaluate_width(w,target);
        if (!c.valid) break;
        if(c.error_j<best.error_j) best=c;
      }
      return best;
    };
    Result out;
    const float zero_energy=energyControlPotentialJ(energyControlAutonomousCorrectedPrediction(r.free_peak_deg,r.physical_side,0,nullptr));
    const FastCandidate ff=exhaustive ? legacy_pick(r.target_energy_j,zero_energy) : fast_pick_width(r.target_energy_j);
    if(!ff.valid) return out;
    out.ff=ff.width_ms; out.ffq=ff.q_mA_s;
    const float target_q=fmaxf(0.0f,fminf(r.q_available_mA_s,ff.q_mA_s+r.integral_mA_s));
    const float target=energyControlPotentialJ(energyControlAutonomousCorrectedPrediction(r.free_peak_deg,r.physical_side,target_q,nullptr));
    if(!isfinite(target)) return out;
    const FastCandidate s=exhaustive ? legacy_pick(target,r.passive_energy_j) : fast_pick_width(target);
    if(!s.valid) return out;
    out.candidate=s.width_ms;
    out.selected=0; out.q=0;
    if(s.error_j<fabsf(target-r.passive_energy_j)) { out.selected=s.width_ms;out.q=s.q_mA_s; }
    out.evaluations=fast_eval_count;out.valid=true;
    return out;
  }
};
int main() {
  int side,direction; unsigned vbat;
  while(std::cin>>side>>direction>>vbat) {
    Model m; m.r.physical_side=side; m.r.command_direction=direction; m.r.model_vbat_mV=vbat;
'''
    for f in FLOAT_FIELDS:
        source += '    {uint32_t bits; if(!(std::cin>>bits)) return 2; memcpy(&m.r.'+f+', &bits, 4);}\n'
    source += r'''    const Result fast=m.replay(false), legacy=m.replay(true);
    for(const auto& x:{fast,legacy}) {
      std::cout<<x.valid<<' '<<x.ff<<' '<<x.selected<<' '<<x.candidate<<' '<<x.evaluations<<' '<<solver_audit::floatBits(x.ffq)<<' '<<solver_audit::floatBits(x.q)<<' ';
    }
    std::cout<<'\n';
  }
  return 0;
}
'''
    cpp = Path(directory)/'replay.cpp'; binary=Path(directory)/'replay'
    cpp.write_text(source)
    compiler=shutil.which('g++')
    if not compiler: raise ValueError('g++ is required for the native float32 replay')
    subprocess.run([compiler,'-std=c++17','-O2','-ffp-contract=off','-Wall','-Wextra','-Werror',
                    '-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'src'),str(cpp),'-o',str(binary)],check=True,capture_output=True,text=True)
    return binary

def native_replay(binary, records):
    rows=[]
    for e in records:
        bits=e['f32_bits']
        if len(bits)!=len(FLOAT_FIELDS) or any(type(x) is not int or not 0<=x<=0xffffffff for x in bits):
            raise ValueError('invalid float32 replay payload')
        rows.append(' '.join(map(str,[e['physical_side'],e['command_direction'],e['model_vbat_mV'],*bits])))
    output=subprocess.run([str(binary)],input='\n'.join(rows)+'\n',capture_output=True,text=True,check=True).stdout.splitlines()
    if len(output)!=len(records):raise ValueError('native replay returned the wrong record count')
    result=[]
    for e,line in zip(records,output):
        x=list(map(int,line.split()))
        if len(x)!=14:raise ValueError('invalid native replay result')
        result.append(dict(index=e['index'],t_test_ms=e['t_test_ms'],pulse_id=e['pulse_id_after'],
               recorded_ff=e['ff_width_ms'],recorded_selected=e['selected_width_ms'],
               fast_valid=bool(x[0]),fast_ff=x[1],fast_selected=x[2],fast_evaluations=x[4],
               legacy_valid=bool(x[7]),legacy_ff=x[8],legacy_selected=x[9],
               fast_replay_match=bool(x[0] and x[1]==e['ff_width_ms'] and x[2]==e['selected_width_ms']),
               legacy_ff_match=bool(x[7] and x[8]==e['ff_width_ms']),
               legacy_selected_match=bool(x[7] and x[9]==e['selected_width_ms']),
               decision_us=e['decision_us'],solver_us=e['solver_us'],entry_sample_age_us=e['entry_sample_age_us'],exit_sample_age_us=e['exit_sample_age_us']))
    return result

def load_metadata(path):
    path=Path(path)
    blobs=[]
    if path.suffix.lower()=='.zip':
        with zipfile.ZipFile(path) as archive:
            for info in archive.infolist():
                if info.filename.lower().endswith('.rwlog'):
                    if info.file_size>100_000_000:raise ValueError('RWLOG larger than 100 MB')
                    blobs.append((info.filename,archive.read(info)))
        if not blobs:raise ValueError('ZIP contains no RWLOG files')
    elif path.suffix.lower()=='.rwlog':blobs=[(path.name,path.read_bytes())]
    else:
        data=json.loads(path.read_text())
        return [(path.name,data.get('metadata',data),None)]
    loaded=[]
    for name,raw in blobs:
        h=parse_header(raw)
        if not verify_crc(raw,h):raise ValueError(name+': RWLOG CRC mismatch')
        s=h['header_size'];end=s+h['metadata_json_size']
        if end>len(raw) or end>h['samples_offset']:raise ValueError('invalid metadata range')
        loaded.append((name,json.loads(raw[s:end]),True))
    return loaded

def quantiles(values):
    if not values:return {'n':0,'median_us':None,'p95_us':None,'max_us':None}
    values=sorted(values)
    def q(f):
        t=f*(len(values)-1);i=int(t)
        return values[i]+(values[min(i+1,len(values)-1)]-values[i])*(t-i)
    return dict(n=len(values),median_us=q(.5),p95_us=q(.95),max_us=values[-1])

def analyze(metadata,binary,crc_ok=None):
    audit=metadata.get('v46s_solver_audit')
    if not audit:raise ValueError('no V46s audit in this file; earlier firmware cannot supply missing diagnostics')
    if audit.get('available') is False:raise ValueError('audit unavailable: '+str(audit.get('reason', 'unknown')))
    if audit.get('schema_version')!=1 or audit.get('float_fields')!=FLOAT_FIELDS or audit.get('solver_revision')!='v46r_fast_solver_control_20260915':
        raise ValueError('unsupported audit schema or solver revision')
    events=audit['events']
    if audit['count']!=len(events):raise ValueError('audit count mismatch')
    for i,e in enumerate(events):
        if i and e['index']!=events[i-1]['index']+1:raise ValueError('nonconsecutive audit indices')
    complete=[e for e in events if e['solver_complete'] and e['stage']==4]
    rows=native_replay(binary,complete) if complete else []
    mismatches=[r for r in rows if not (r['fast_replay_match'] and r['legacy_ff_match'] and r['legacy_selected_match'])]
    summary=dict(schema_version=1,crc_ok=crc_ok,recorded=len(events),replayed=len(rows),
        not_replayed=len(events)-len(rows),overwritten=audit['overwritten'],
        output_count=sum(bool(e['output_executed']) for e in events),
        fast_replay_matches=sum(r['fast_replay_match'] for r in rows),
        legacy_ff_matches=sum(r['legacy_ff_match'] for r in rows),
        legacy_selected_matches=sum(r['legacy_selected_match'] for r in rows),
        mismatch_indices=[r['index'] for r in mismatches],
        full_comparison_pass=bool(rows) and not mismatches and not audit['overwritten'] and len(rows)==len(events),
        timing={k:quantiles([e[k] for e in complete]) for k in ('decision_us','solver_us','fast_solver_us','ff_search_us','selected_search_us','entry_sample_age_us','exit_sample_age_us')},
        caveat='Native replay is offline model validation, not hardware timing validation. float32 bits preserve inputs; host/ESP32 libm rounding and objective ties can differ. Every mismatch requires review; no tolerance hides width mismatches.')
    return summary,rows

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('input',type=Path);p.add_argument('--out-dir',type=Path,default=Path('solver-audit-report'))
    args=p.parse_args();args.out_dir.mkdir(parents=True,exist_ok=True)
    try:
        with tempfile.TemporaryDirectory() as td:
            binary=build_driver(td)
            all_pass=True
            for n,(name,meta,crc) in enumerate(load_metadata(args.input)):
                summary,rows=analyze(meta,binary,crc)
                all_pass=all_pass and summary['full_comparison_pass']
                target=args.out_dir/('run_'+str(n+1)+'_'+Path(name).stem)
                target.with_suffix('.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
                if rows:
                    with target.with_suffix('.csv').open('w',newline='') as f:
                        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
                print(json.dumps(summary,indent=2,ensure_ascii=False))
        return 0 if all_pass else 1
    except (OSError,ValueError,KeyError,subprocess.CalledProcessError,zipfile.BadZipFile) as e:
        print('ERROR:',e)
        if isinstance(e,subprocess.CalledProcessError):print(e.stderr)
        return 2
if __name__=='__main__':raise SystemExit(main())
