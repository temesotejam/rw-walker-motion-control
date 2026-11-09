#!/usr/bin/env python3
"""Replay the actual production decision block and a frozen 0.47.17 selector.

The exhaustive 0..100 ms oracle checks the new minimum-Q-error specification.
Old/new deltas are counterfactual arithmetic on the SAME recorded states, not a
prediction of a closed-loop physical trajectory or target-board execution time.
"""
import argparse
import collections
import json
import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--out', type=Path)
args = parser.parse_args()
fixture = json.loads((ROOT/'tools/fixtures/control_math_04717.json').read_text())
inputs = json.loads((ROOT/'tools/fixtures/control_inputs_04717.json').read_text())
source = (ROOT/'src/experiment_runner.cpp').read_text()
fields = inputs['float_fields']


def method(name):
    start = source.index('float ExperimentRunner::'+name+'(')
    end = source.index('{', start)+1
    depth = 1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}')
        end += 1
    return source[start:end]


def clean(block):
    block = re.sub(r'  // V46s audit begin\n.*?  // V46s audit end\n', '', block, flags=re.S)
    a = block.index('  const float fast_v =')
    b = block.index('  const float fast_tau_s =', a)
    b = block.index(';', b)+1
    block = block[:a]+'''  const float fast_signed_target_current_mA = r.signed_target_current_mA;
  const float fast_tau_s = r.tau_s;'''+block[b:]
    block = block.replace('    logger_->addEnergyControlAutonomousZeroCrossEvent(event);\n','')
    block = block.replace('    rearm_for_next_peak();\n','').replace('return;', 'return out;')
    return block


a = source.index('  // Direct-Q control begin.')
b = source.index('  // Direct-Q control end.',a)
actual = clean(source[a:b])
legacy = clean(fixture['solver_block']+fixture['solver_tail'])
legacy = legacy.replace('float q_mA_s = 0.0f;', '++computed; float q_mA_s = 0.0f;')
for name in ('energyControlPotentialJ','energyControlAutonomousCorrectedPrediction'):
    assert method(name)==fixture['methods'][name], name+' changed alongside the inverse'

cpp='''#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "config.h"
#include "psram_logger.h"
#include "control_math_cache.h"
#include "direct_q_solver.h"
struct Input { int8_t physical_side=1;
'''+''.join('float '+f+'=0;\n' for f in fields)+'''};
struct Result { bool valid=false, upper=false, lower=false; unsigned width=0, evaluations=0, computed=0;
float ff=0, q=0, requested=0, energy=0; };
static float scalar(uint32_t bits) { float f;memcpy(&f,&bits,4);return f; }
'''
setup='''    Result out;
    unsigned computed=0;
    PsramLogger::EnergyControlAutonomousZeroCrossEvent event;
    event.i0_estimated_mA=r.i0_mA;event.free_next_peak_amplitude_deg=r.free_peak_deg;
    event.physical_next_peak_side=r.physical_side;event.target_energy_j=r.target_energy_j;
    event.target_peak_deg=r.target_peak_deg;event.passive_energy_j=r.passive_energy_j;
    event.q1_gain_deg_per_mA_s=r.base_gain;event.q_available_mA_s=r.q_available_mA_s;
    event.g_side_base_deg_per_mA_s=r.base_gain;event.c_side_used_deg=r.correction_c;
    event.g_side_corrected_deg_per_mA_s=r.correction_gain;
    const float energy_control_autonomous_integral_plus_mA_s_=r.integral_mA_s;
    const float energy_control_autonomous_integral_minus_mA_s_=r.integral_mA_s;
'''
for name,block in [('Legacy',legacy),('Actual',actual)]:
    cpp+='struct '+name+''' {
  Input r;
  float energyControlAutonomousGainForSide(int8_t) const { return r.base_gain; }
  void energyControlAutonomousCorrectionParameters(int8_t,float* c,float* g) const {
    *c=r.correction_c;*g=r.correction_gain;
  }
'''+''.join(method(n).replace('ExperimentRunner::','') for n in
            ['energyControlPotentialJ','energyControlAutonomousCorrectedPrediction'])
    cpp+='\nResult replay() {\n'+setup+block+'''
    out.valid=true;out.width=selected_width_ms;out.ff=event.q_ff_energy_mA_s;
    out.upper=event.q_saturated_upper;out.lower=event.q_saturated_lower;
    out.q=selected_q_mA_s;out.requested=corrected_q_target_mA_s;out.energy=selected_energy_j;
'''+('out.evaluations=fast_eval_count;out.computed=computed;' if name=='Legacy' else 'out.evaluations=selected_inverse.evaluations;out.computed=selected_inverse.evaluations;')+'\nreturn out;\n}\n};\n'

cpp+='''static void oracle(const Input& r,const Result& actual) {
  float best=actual.requested;unsigned ms=0;
  for(unsigned w=1;w<=100;++w) {
    const float t=static_cast<float>(w)/1000.0f;
    const float q=fabsf(r.signed_target_current_mA*t+(r.i0_mA-r.signed_target_current_mA)*r.tau_s*(1-expf(-t/r.tau_s)));
    const float e=fabsf(q-actual.requested);
    if(e<best){best=e;ms=w;}
  }
  assert(actual.width==ms);
  assert(fabsf(actual.q-actual.requested)==best);
}
struct Row { unsigned run,index,old_width,old_ff;int8_t side;uint32_t bits[16]; };
static const Row rows[]={
'''
for e in inputs['records']:
    cpp+='{'+','.join(map(str,[e['run'],e['index'],e['selected_width_ms'],e['ff_width_ms'],e['physical_side']]))+',{'+','.join(str(v)+'U' for v in e.get('f32_bits',e.get('bits')))+'}},\n'
cpp+='''};
int main(){
  for(const Row& row:rows){
    Input r;r.physical_side=row.side;
'''+''.join(f'r.{name}=scalar(row.bits[{i}]);\n' for i,name in enumerate(fields))+'''
    Legacy old;Actual now;old.r=now.r=r;
    const auto before=old.replay(),after=now.replay();
    assert(before.valid && before.width==row.old_width);
    assert(after.valid && after.width<=100 && after.evaluations<=25);
    oracle(r,after);
    if(row.index==1) {
      Input bad=r;bad.integral_mA_s=NAN;now.r=bad;assert(!now.replay().valid);
      bad=r;bad.i0_mA=NAN;now.r=bad;assert(!now.replay().valid);
      bad=r;bad.tau_s=0;now.r=bad;assert(!now.replay().valid);
      bad=r;bad.correction_gain=0;now.r=bad;assert(!now.replay().valid);
      bad=r;bad.free_peak_deg=100;now.r=bad;assert(!now.replay().valid);
      bad=r;bad.q_available_mA_s=-1;now.r=bad;assert(!now.replay().valid);
      now.r=r;
    }
    std::printf("%u,%u,%u,%u,%.9g,%.9g,%u,%u,%.9g,%u,%u,%u,%u,%u\\n",row.run,row.index,
      before.width,after.width,before.q,after.q,before.evaluations,after.evaluations,
      after.requested,before.upper,after.upper,before.lower,after.lower,before.computed);
  }
}
'''
with tempfile.TemporaryDirectory(prefix='direct-q-replay-') as tmp:
    p=Path(tmp);(p/'replay.cpp').write_text(cpp)
    subprocess.run(['g++','-std=c++17','-Os','-ffp-contract=off','-Wall','-Wextra','-Werror',
                    '-Wno-unused-variable','-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'src'),
                    str(p/'replay.cpp'),'-o',str(p/'replay')],check=True)
    text=subprocess.check_output([str(p/'replay')],text=True)
records=[]
for row in text.splitlines():
    values=row.split(',')
    records.append(dict(zip(['run','index','old_ms','new_ms','old_q','new_q','old_evaluations',
                             'new_evaluations','requested_q','old_upper','new_upper','old_lower','new_lower','old_computed'],
                            [int(v) if i not in (4,5,8) else float(v) for i,v in enumerate(values)])))
deltas=collections.Counter(r['new_ms']-r['old_ms'] for r in records)
result={'source':'five 0.47.17 runs; same recorded state; host arithmetic, no physical replay',
        'count':len(records),'width_delta_counts':dict(sorted(deltas.items())),
        'old_logical_evaluations':sum(r['old_evaluations'] for r in records),
        'old_cached_charge_energy_computations':sum(r['old_computed'] for r in records),
        'new_charge_integral_evaluations':sum(r['new_evaluations'] for r in records),
        'upper_flag_changes':sum(r['old_upper']!=r['new_upper'] for r in records),
        'lower_flag_changes':sum(r['old_lower']!=r['new_lower'] for r in records),
        'records':records}
if args.out:
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='records'},indent=2))
print('Production decision replay: frozen 0.47.17 matches recorded widths; new inverse matches exhaustive 101-width Q-error oracle PASS')
