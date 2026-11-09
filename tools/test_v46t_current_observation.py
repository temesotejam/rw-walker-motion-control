#!/usr/bin/env python3
"""Execute the actual production snapshot/clock/age statements with deterministic preemption."""
from pathlib import Path
import hashlib, subprocess, tempfile
from v46t_current_observation_contract import NEW_PREFIX, NEW_AGE, normalize_current_observation
from v46ac_delay_comp_contract import normalize_runner as normalize_v46ac_runner
from v46ab_no_prediction_contract import normalize_runner as normalize_v46ab_runner
from v46aa_control_zero_contract import normalize_runner as normalize_v46aa_runner
from v46z_comparison_zero_contract import normalize_runner as normalize_v46z_runner
from v46ak_observation_contract import normalize_runner as normalize_v46ak_runner
from v46al_control_contract import normalize_runner as normalize_v46al_runner
R=Path(__file__).resolve().parents[1]
s=(R/'src/experiment_runner.cpp').read_text()
base='abed74514dd0c29d494c3172b60efa466553b8015c649e4f5306b472a289d893'
assert hashlib.sha256(normalize_current_observation(normalize_v46z_runner(normalize_v46aa_runner(normalize_v46ab_runner(normalize_v46ac_runner(normalize_v46al_runner(normalize_v46ak_runner(s))))))).encode()).hexdigest()==base
start=s.index('void ExperimentRunner::logSampleNow() {')
body=s[start:s.index('void ExperimentRunner::finishRun()',start)]
assert body.count('telemetrySnapshot()')==1
assert 'currentAgeUs(now_us)' not in body
assert NEW_PREFIX in body and NEW_AGE in body
prefix=NEW_PREFIX[:NEW_PREFIX.index('  if (logger_->full()) {')]
assignment='  row.roller_current_age_us = '+body.split('  row.roller_current_age_us = ',1)[1].split(';',1)[0]+';'
cpp=r'''#include <cstdint>
#include <cassert>
#include <iostream>
struct RollerTelemetry { uint32_t current_sample_time_us=0, current_sequence=0; int actual_current_mA=0; bool current_valid=false; };
struct FakeRoller {
  RollerTelemetry published, replacement;
  unsigned copies=0; bool publish_after_copy=false;
  RollerTelemetry telemetrySnapshot() {
    ++copies; const auto out=published;
    if(publish_after_copy) published=replacement;
    return out;
  }
};
FakeRoller obj; FakeRoller* roller_=&obj;
uint32_t clock_value=0; unsigned clocks=0; bool publish_after_clock=false;
uint32_t micros() {++clocks;auto v=clock_value;if(publish_after_clock)obj.published=obj.replacement;return v;}
struct Row {uint32_t roller_current_age_us=0;RollerTelemetry data;uint32_t reference_us;};
Row observe() {
'''+prefix+'\n  Row row;\n'+assignment+r'''
  row.data=roller_telemetry;row.reference_us=now_us;return row;
}
int main() {
  // Old code: clock first, then a snapshot containing a newer sample.
  assert(uint32_t(1000U-1100U)==4294967196U);
  // Old code: the age helper independently re-read a different sample.
  assert(uint32_t(1300U-1200U)!=uint32_t(1300U-1250U));
  unsigned checks=0;uint32_t seed=0x46f35aU;
  for(unsigned mode=0;mode<4;++mode)for(unsigned i=0;i<25000;++i) {
    seed=1664525U*seed+1013904223U;
    uint32_t sample=seed; if(sample==0)sample=1;
    const uint32_t elapsed=1+(seed%20000U);
    obj={};obj.published={sample,123,17,true};obj.replacement={sample+elapsed-1,124,99,true};
    obj.publish_after_copy=(mode&1)!=0; publish_after_clock=(mode&2)!=0;
    clock_value=sample+elapsed;clocks=0;
    auto row=observe();
    assert(obj.copies==1 && clocks==1);
    assert(row.data.current_sequence==123 && row.data.actual_current_mA==17);
    assert(row.roller_current_age_us==elapsed);
    assert(row.roller_current_age_us==uint32_t(row.reference_us-row.data.current_sample_time_us));
    ++checks;
  }
  publish_after_clock=false;obj={};clock_value=900;auto missing=observe();assert(missing.roller_current_age_us==UINT32_MAX);
  obj={};obj.published={0xfffffff0U,2,-300,true};clock_value=8;assert(observe().roller_current_age_us==24);
  obj={};obj.published={100,3,2,false};clock_value=200;assert(observe().roller_current_age_us==100);
  obj={};obj.published={200,4,2,true};clock_value=200;assert(observe().roller_current_age_us==0);
  std::cout<<checks<<" preemption schedules + missing/reset/wrap/read-failure/zero-age PASS\n";
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d);(p/'test.cpp').write_text(cpp)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
print('V46t exact logging delta and native observation tests PASS')
