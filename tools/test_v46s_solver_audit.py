#!/usr/bin/env python3
"""V46s audit-only regression: unchanged controller + executable native logging/replay."""
from pathlib import Path
from v46u_timing_contract import original_timing_file
import copy
import hashlib
import json
import subprocess
import tempfile
from replay_v46s_solver_audit import ROOT, original_runner, build_driver, method, analyze

def run(cmd,**kw):
    try:return subprocess.run(cmd,check=True,capture_output=True,text=True,**kw).stdout
    except subprocess.CalledProcessError as e:
        print(e.stdout,e.stderr);raise

def main():
    baseline=json.loads((ROOT/'tools/v46s_audit_baseline.json').read_text())
    assert hashlib.sha256(original_runner().encode()).hexdigest()==baseline['runner_sha256']
    cfg=original_timing_file('src/config.h').decode().replace('v46t_current_observation_20260915', 'v46s_solver_audit_20260915').replace('v46s_solver_audit_20260915','v46r_fast_solver_control_20260915')
    assert hashlib.sha256(cfg.encode()).hexdigest()==baseline['config_sha256']
    for path,digest in baseline['protected'].items():
        assert hashlib.sha256(original_timing_file(path)).hexdigest()==digest,path
    logger=(ROOT/'src/psram_logger.cpp').read_text()
    assert logger.count('solver_audit_->clear();')==2
    assert 'solver_audit_->appendJson(json)' in logger
    assert 'ps_malloc(sizeof(solver_audit::Buffer<128>))' in logger
    assert 'solver_audit::Buffer<128>* solver_audit_ = nullptr;' in (ROOT/'src/psram_logger.h').read_text()
    assert 'audit_psram_allocation_failed' in logger
    assert 'legacy_exhaustive_solver_controls_motor' not in logger
    assert 'runEnergyControlAutonomousSolverShadow();' not in original_runner()
    print('PASS: retained baseline after reversing explicit migration/audit hunks; IMU/motor/MEKF/log layout unchanged')
    with tempfile.TemporaryDirectory() as td:
        td=Path(td);binary=build_driver(td)
        driver=td/'audit_test.cpp'
        context='\n'.join(method(original_runner(),x) for x in ('energyControlAutonomousGainForSide','energyControlAutonomousCorrectionParameters','predictCurrentGoalMa','predictRiseTauS'))
        code=r'''#define main replay_cli_main
#include "replay.cpp"
#undef main
#include <cassert>
struct Context {
'''+context+r'''
};
int main() {
  solver_audit::Buffer<3> ring;
  solver_audit::Record r;
  for(unsigned i=0;i<5;++i){r.t_test_ms=i;ring.push(r);}
  assert(ring.count()==3 && ring.overwritten()==2);
  assert(ring.at(0).t_test_ms==2 && ring.at(2).t_test_ms==4);
  assert(ring.at(0).index==3 && ring.at(2).index==5);
  ring.clear();assert(ring.count()==0 && ring.overwritten()==0);
  int exits=0;
  auto early=[&]() { auto finish=[&](){++exits;}; solver_audit::ScopeExit<decltype(finish)> scope(finish); return; };
  early();early();assert(exits==2);
  assert(solver_audit::floatBits(-0.0f)==0x80000000U);
  assert(static_cast<uint32_t>(5U-0xfffffffeU)==7U);
  assert(sizeof(solver_audit::Record)*128<=32768);
  solver_audit::Buffer<128> records;
  float free_peak,target,i0,integral; int side; unsigned vbat,expected_ff,expected_selected;
  Context c;
  unsigned index=0;
  while(std::cin>>free_peak>>target>>i0>>integral>>side>>vbat>>expected_ff>>expected_selected) {
    Model m; auto& e=m.r;
    e.t_test_ms=++index*100;
    e.physical_side=e.command_direction=side;
    e.model_vbat_mV=vbat; e.free_peak_deg=free_peak; e.target_peak_deg=target;
    e.i0_mA=i0; e.integral_mA_s=integral;
    e.target_energy_j=m.energyControlPotentialJ(target);
    e.passive_energy_j=m.energyControlPotentialJ(free_peak);
    e.base_gain=c.energyControlAutonomousGainForSide(side);
    c.energyControlAutonomousCorrectionParameters(side,&e.correction_c,&e.correction_gain);
    e.correction_limit=Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG;
    e.signed_target_current_mA=side*c.predictCurrentGoalMa(300,vbat/1000.0f);
    e.tau_s=c.predictRiseTauS(300);
    const float t=Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS/1000.0f;
    e.q_available_mA_s=fabsf(e.signed_target_current_mA*t+(i0-e.signed_target_current_mA)*e.tau_s*(1-expf(-t/e.tau_s)));
    const auto fast=m.replay(false);
    assert(fast.valid && fast.evaluations<=60);
    e.ff_width_ms=fast.ff; e.selected_width_ms=fast.selected;
    e.fast_selected_width_ms=fast.candidate;
    e.ff_q_mA_s=fast.ffq;e.selected_q_mA_s=fast.q;
    e.eval_count=fast.evaluations;e.stage=4;e.solver_started=e.solver_complete=1;
    records.push(e);
  }
  String json;records.appendJson(json);std::cout<<json.c_str()<<'\n';
}
'''
        driver.write_text(code)
        exe=td/'audit_test'
        run(['g++','-std=c++17','-O2','-ffp-contract=off','-Wall','-Wextra','-Werror','-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'src'),str(driver),'-o',str(exe)])
        fixture=json.loads((ROOT/'tools/fixtures/v46q_solver56_rounded.json').read_text())
        data='\n'.join(' '.join(map(str,x)) for x in fixture['cases'])+'\n'
        audit=json.loads(run([str(exe)],input=data))
        assert audit['count']==56 and audit['overwritten']==0
        summary,rows=analyze({'v46s_solver_audit':audit},binary,True)
        assert summary['fast_replay_matches']==56,summary
        print('PASS: native ring/reset/overflow/early-return/float32 serialization/60-evaluation bound')
        print('56 rounded historical inputs: native fast-vs-exhaustive matches:',summary['legacy_ff_matches'],summary['legacy_selected_matches'])
        historical=sum(row['recorded_ff']==case[-2] and row['recorded_selected']==case[-1] for row,case in zip(rows,fixture['cases']))
        print('Rounded replay matches historical recorded widths:',historical,'/ 56 (not exact-input certification)')
        bad=copy.deepcopy(audit);bad['events'][0]['selected_width_ms']=999
        s,_=analyze({'v46s_solver_audit':bad},binary)
        assert not s['full_comparison_pass'] and s['mismatch_indices']==[1]
        lost=copy.deepcopy(audit);lost['overwritten']=1
        s,_=analyze({'v46s_solver_audit':lost},binary)
        assert not s['full_comparison_pass']
        partial=copy.deepcopy(audit);partial['events'][0]['stage']=1;partial['events'][0]['solver_complete']=0
        s,_=analyze({'v46s_solver_audit':partial},binary)
        assert s['not_replayed']==1 and not s['full_comparison_pass']
        empty=copy.deepcopy(audit);empty['events']=[];empty['count']=0
        s,_=analyze({'v46s_solver_audit':empty},binary)
        assert not s['full_comparison_pass']
        try:analyze({},binary)
        except ValueError:pass
        else:raise AssertionError('old metadata incorrectly accepted')
        try:analyze({'v46s_solver_audit':{'available':False,'reason':'audit_psram_allocation_failed'}},binary)
        except ValueError:pass
        else:raise AssertionError('unavailable audit incorrectly accepted')
        print('PASS: wrong width, lost records, failed decisions, empty runs, allocation failure and old logs cannot report a full pass')
    run(['python','tools/test_v46r_fast_solver_control.py'],cwd=ROOT)
    print('V46s audit and replay regressions PASS')
if __name__=='__main__':main()
