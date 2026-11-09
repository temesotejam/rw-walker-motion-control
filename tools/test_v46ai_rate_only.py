#!/usr/bin/env python3
"""Exercise the actual rate-only helper, integration and nullable legacy logs."""
from pathlib import Path
import csv,json,re,subprocess,tempfile
from convert_rwlog_to_csv import write_energy_control_autonomous_events
ROOT=Path(__file__).resolve().parents[1]
runner=(ROOT/'src/experiment_runner.cpp').read_text()
header=(ROOT/'src/experiment_runner.h').read_text()
logger=(ROOT/'src/psram_logger.cpp').read_text()
helper=(ROOT/'src/rate_baseline_correction.h').read_text()
config=(ROOT/'src/config.h').read_text()
assert 'energyControlAutonomousFreeNextPeakAmplitude' not in runner+header
assert 'ENERGY_CONTROL_AUTONOMOUS_P1_FREE_DECAY_' not in runner+config
call='const auto baseline = rate_baseline::evaluate(\n      event.zero_cross_abs_rate_dps, event.physical_next_peak_side);'
assert runner.count(call)==1
start=runner.index(call);layer=runner.index('// V46al-R2 previous-peak active control begin',start)
assert start<layer<runner.index('event.passive_energy_j = energyControlPotentialJ',layer)
assert 'event.p1_free_peak_before_rate_deg = NAN;' in runner[start:layer]
assert 'event.rate_baseline_correction_deg = NAN;' in runner[start:layer]
assert 'previous_peak_control::evaluate(' in runner[layer:]
assert 'event.free_next_peak_amplitude_deg = previous_peak_result.corrected_free_peak_deg;' in runner[layer:]
for forbidden in ['previous_peak_deg','target_deg','t_test_ms','compensation_us','OUTSIDE_STATE','WARMUP','OTHER_SETTINGS']:
    assert forbidden not in helper,forbidden
parts=[]
for line in logger.splitlines():
    m=re.fullmatch(r'\s*json \+= ("(?:[^"\\]|\\.)*");',line)
    if m:
        value=json.loads(m.group(1))
        if value.startswith('"rate_baseline_'):parts.append(value)
metadata=json.loads('{'+''.join(parts).rstrip(',')+'}')
assert metadata['rate_baseline_revision']=='v46ai_rate_only_all_zero_crosses_20260919'
assert metadata['rate_baseline_blend']==1
assert metadata['rate_baseline_delta_cap_enabled'] is False
assert metadata['rate_baseline_delta_cap_deg'] is None
assert metadata['rate_baseline_p1_fallback_enabled'] is False
assert metadata['rate_baseline_previous_peak_used'] is False
assert metadata['rate_baseline_zero_floor_deg']==0
assert 'RWLOG_FORMAT_VERSION = 51' in logger
with tempfile.TemporaryDirectory() as directory:
    p=Path(directory)
    event=dict(p1_free_peak_before_rate_deg=None,rate_baseline_correction_deg=None,
               rate_baseline_peak_deg=-5.78123,free_next_peak_amplitude_deg=0,rate_baseline_reason=5)
    assert write_energy_control_autonomous_events({'energy_control_autonomous_zero_cross_events':[event,{}]},p)==(0,2)
    rows=list(csv.DictReader((p/'energy_control_autonomous_zero_cross_events.csv').open()))
    for key in ['p1_free_peak_before_rate_deg','rate_baseline_correction_deg']:
        assert rows[0][key]==rows[1][key]==''
    assert float(rows[0]['rate_baseline_peak_deg'])==-5.78123
    assert float(rows[0]['free_next_peak_amplitude_deg'])==0
code=r'''
#include <cassert>
#include <cmath>
#include <cfloat>
#include <iostream>
#include "rate_baseline_correction.h"
using namespace rate_baseline;
void near(float a,float b,float tol=1e-4f){assert(std::fabs(a-b)<tol);}
int main(){
  unsigned cases=0;
  for(int side:{-1,1}){
    float last=-1;
    for(int i=0;i<=16000;++i){
      float rate=i*.01f;auto r=evaluate(rate,side);
      float expected=side>0?7.217460941f+.286814471f*(rate-65):8.399746959f+.130807354f*(rate-65);
      near(r.rate_deg,expected);near(r.adjusted_deg,std::fmax(0.f,expected));
      assert(r.reason==(expected<0?ZERO_FLOOR:APPLIED));
      assert(r.adjusted_deg>=last);last=r.adjusted_deg;++cases;
    }
    // Former boundaries do not change formulas, including one-float outside.
    for(float boundary:{59.f,61.5f,70.f,72.f}){
      auto lo=evaluate(std::nextafter(boundary,-INFINITY),side);
      auto hi=evaluate(std::nextafter(boundary,INFINITY),side);
      assert(lo.reason==APPLIED&&hi.reason==APPLIED);
      near(lo.adjusted_deg,hi.adjusted_deg);++cases;
    }
    for(float rate:{NAN,INFINITY,-INFINITY,-1.f}){
      auto r=evaluate(rate,side);assert(r.reason==INVALID_INPUT&&std::isnan(r.adjusted_deg));++cases;
    }
    auto huge=evaluate(FLT_MAX,side);assert(std::isfinite(huge.adjusted_deg));++cases;
  }
  for(int side:{0,2,-2}){auto r=evaluate(65,side);assert(r.reason==INVALID_INPUT&&std::isnan(r.adjusted_deg));++cases;}
  // 9158 event 55: no P1 branch just above the former 70 deg/s boundary.
  auto e=evaluate(70.0565f,-1);near(e.adjusted_deg,9.061174f);
  near(e.adjusted_deg+.25455f*.13010f,9.094291f);
  // Both early positive crossings now use the same formula and zero floor.
  auto early=evaluate(19.6791f,1);assert(early.reason==ZERO_FLOOR&&early.adjusted_deg==0);
  assert(evaluate(0,-1).adjusted_deg==0);
  std::cout<<"V46ai production rate-only baseline: "<<cases<<" cases; full rate sweep, former boundaries, zero floor, invalid values, recorded outlier PASS\n";
}
'''
with tempfile.TemporaryDirectory() as directory:
    p=Path(directory);(p/'test.cpp').write_text(code)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
