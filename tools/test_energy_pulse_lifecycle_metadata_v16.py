"""Actual v16 compact completion JSON and CSV; selector remains the v15 model.

Uses the established real-source serializer fixture, then supplies distinct
completed, interrupted, negative-Q, pending, unqueued and late-stop records.
Elapsed predictions describe software queue timing, not measured shaft work.
"""
from pathlib import Path
import csv
import subprocess
import tempfile

import convert_rwlog_to_csv as converter

ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'tools/test_early_energy_v15_metadata.py'
context={'__file__':str(base)}
exec(compile(base.read_text(encoding='utf8').split('\nwith tempfile.TemporaryDirectory',1)[0],str(base),'exec'),context)
fixture='#include "energy_pulse_completion.h"\n'+context['fixture']
parse=context['parse']
start=fixture.index('  for(unsigned i=0;',fixture.index('int main('))
end=fixture.index('  PsramString json;',start)
records=r'''
  for(unsigned i=0;i<7 && !empty;++i){
    active_energy::Input in;in.side=1;in.rate_dps=35.f;in.body_angle_deg=0;
    in.target_deg=8;in.previous_peak_deg=8;in.same_side_peak_deg=6;
    in.initial_current_ma=i==2?-150.f:0.f;in.goal_current_ma=329.547119f;in.cap_ms=80;
    auto decision=active_energy::decide(geometry,table,in);
    CHECK(decision.valid && !decision.coast && decision.width_ms==80);
    auto& a=runner.walking_energy_audit_[i];a.decision=decision;
    a.time_ms=1200+i*300;a.peak_ms=a.time_ms-250;a.trigger_ms=a.time_ms;a.side=1;
    a.requested_ms=a.guarded_ms=80;a.executed=i!=4;
    a.optional=i==4;a.early_budget_checked=i==4;a.early_budget_admitted=false;
    if(a.executed){
      a.output_start_ms=a.time_ms+7;
      a.start_i0_ma=in.initial_current_ma;a.start_goal_ma=in.goal_current_ma;
      a.start_tau_s=.0316f;a.start_rate_dps=in.rate_dps;
      if(i!=3){
        energy_pulse_completion::Input done;done.side=a.side;done.rate_dps=a.start_rate_dps;
        done.initial_current_ma=a.start_i0_ma;done.goal_current_ma=a.start_goal_ma;done.tau_s=a.start_tau_s;
        done.available_j=decision.available_j;done.request_j=decision.request_j;done.planned_ms=80;
        done.actual_elapsed_ms=i==0?80:i==5?123:i==6?0:17;
        a.completion=energy_pulse_completion::evaluate(done);
        a.completion_reason=i==0 || i==5?1:i==1 || i==6?2:3;
        a.output_end_ms=a.output_start_ms+done.actual_elapsed_ms;
      }
    }
    ++runner.walking_energy_count_;++runner.walking_energy_total_;
  }
'''
fixture=fixture[:start]+records+fixture[end:]
fixture=fixture.replace('for(unsigned branch:{0u,15u,19u,20u})',
                        'for(unsigned branch:{0u,15u,19u,20u,21u})')
marker='    number("redistribution_j",d.redistribution_j);number("request_j",d.request_j);'
assert fixture.count(marker)==1
fixture=fixture.replace(marker,marker+r'''
    const auto& c=runner.walking_energy_audit_[i].completion;
    number("elapsed_q_pred_mas",c.q_mas);number("elapsed_current_end_pred_ma",c.current_end_ma);
    number("elapsed_work_pred_j",c.work_j);number("elapsed_arrival_pred_j",c.arrival_j);
    number("elapsed_unmet_pred_j",c.unmet_j);
''')

with tempfile.TemporaryDirectory(prefix='pulse_lifecycle_metadata_v16_') as temporary:
    tmp=Path(temporary)
    (tmp/'test.cpp').write_text(fixture,encoding='utf8')
    exe=tmp/'test.exe'
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                    '-DEARLY_ENERGY_MAIN=1','-DPULSE_LIFECYCLE_FIX=1',
                    '-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'src'),
                    str(tmp/'test.cpp'),'-o',str(exe)],check=True)
    for mode,count in [('',7),('empty',0)]:
        output=subprocess.check_output([str(exe)]+([mode] if mode else []),text=True).splitlines()
        assert len(output)==2
        raw,reference=map(parse,output)
        selector='walking_early_energy_main_v15'
        assert raw['energy_model']==raw['wheel_work_model_revision']==selector
        assert raw['lateral_energy_revision']==raw['energy_control_autonomous_revision']==selector
        assert raw['work_control_branch_codes']['21']=='energy_pulse_lifecycle_v16'
        lifecycle=raw['pulse_lifecycle']
        assert lifecycle['active'] is True and lifecycle['revision']=='energy_pulse_lifecycle_v16'
        assert lifecycle['selector_revision']==selector and lifecycle['energy_model_changed'] is False
        assert lifecycle['physical_delivery_measured'] is False
        assert 'not_bus_ACK_or_physical_current_edge' in lifecycle['setCurrentMa_success']
        assert 'actual_software_elapsed_no_duration_clamp' in lifecycle['completion']
        for event in raw['event_revision_checks']:
            if event['branch'] in (19,20,21):
                expected='walking_peak_rate_energy_v14' if event['branch']==19 else selector
                assert event['free_model_revision']==event['arrival_energy_model_revision']==expected
                assert event['maximum_input_work_j']>0
            else:
                assert event['arrival_energy_model_revision'] is None
        block=raw['walking_energy']
        assert block['branch']==21 and block['revision']=='energy_pulse_lifecycle_v16'
        assert block['selector_revision']==selector
        assert block['completion_revision']=='actual_elapsed_pulse_completion_v16'
        assert block['elapsed_semantics']=='software_queue_accept_to_stop_request_not_physical_current_edges'
        assert block['completion_independent_of_log_capacity'] is True
        assert block['energy_neutral_model'].startswith('nominal_only;')
        assert block['stored']==block['total']==count and block['truncated'] is False
        compact=block['records']
        assert len(compact['fields'])==len(set(compact['fields']))
        assert len(compact['rows'])==count
        assert all(len(row)==len(compact['fields']) for row in compact['rows'])
        metadata=converter.expand_tables(raw)
        rows=metadata['walking_energy']['records']
        for row,original in zip(rows,reference):
            for key,value in original.items():
                if value is None:assert row[key] is None
                else:assert abs(row[key]-value)<=5.01e-10,(key,row[key],value)
        if rows:
            full,early,negative,pending,nooutput,late,zero=rows
            assert full['completion_reason']==1 and full['actual_elapsed_ms']==80
            assert full['completion_valid'] and full['completion_work_valid'] and full['planned_duration_completed']
            assert full['output_end_ms']-full['output_start_ms']==80
            assert early['completion_reason']==2 and early['actual_elapsed_ms']==17
            assert not early['planned_duration_completed'] and early['completion_work_valid']
            assert early['elapsed_q_pred_mas']<full['elapsed_q_pred_mas']
            assert early['elapsed_work_pred_j']<full['elapsed_work_pred_j']
            assert early['elapsed_unmet_pred_j']>full['elapsed_unmet_pred_j']
            assert negative['completion_reason']==3 and negative['completion_valid']
            assert negative['elapsed_q_pred_mas']<0 and not negative['completion_work_valid']
            assert negative['elapsed_work_pred_j'] is negative['elapsed_arrival_pred_j'] is negative['elapsed_unmet_pred_j'] is None
            assert pending['executed'] and pending['output_start_ms']>0 and pending['output_end_ms']==0
            assert pending['completion_reason']==0 and not pending['completion_valid']
            assert pending['elapsed_q_pred_mas'] is None and pending['elapsed_work_pred_j'] is None
            assert not nooutput['executed'] and nooutput['output_start_ms']==nooutput['output_end_ms']==0
            assert nooutput['output_q_pred_mas']==nooutput['output_work_pred_j']==0
            assert nooutput['elapsed_q_pred_mas'] is None and nooutput['elapsed_work_pred_j'] is None
            assert nooutput['early_budget_checked'] and not nooutput['early_budget_admitted']
            assert late['actual_elapsed_ms']==123 and late['planned_duration_completed']
            assert not late['completion_width_supported'] and late['completion_extrapolated']
            assert late['elapsed_q_pred_mas']>full['elapsed_q_pred_mas']
            assert late['elapsed_work_pred_j']>full['elapsed_work_pred_j']
            assert zero['completion_valid'] and zero['completion_work_valid']
            assert zero['executed'] and zero['actual_elapsed_ms']==0 and not zero['planned_duration_completed']
            assert zero['output_start_ms']==zero['output_end_ms']>0
            assert zero['elapsed_q_pred_mas']==zero['elapsed_work_pred_j']==0
            assert zero['elapsed_unmet_pred_j']==zero['request_j']
        out=tmp/('csv_'+(mode or 'records'));out.mkdir()
        assert converter.write_walking_energy_diagnostics(metadata,out)==count
        csv_path=out/'walking_energy.csv'
        if not count:
            assert not csv_path.exists()
            continue
        with csv_path.open(newline='',encoding='utf8') as stream:
            csv_rows=list(csv.DictReader(stream))
        assert len(csv_rows)==count
        for csv_row,row in zip(csv_rows,rows):
            assert set(csv_row)==set(row)
            for key,value in row.items():
                if value is None:assert csv_row[key]==''
                elif isinstance(value,bool):assert csv_row[key]==str(value)
                elif isinstance(value,(int,float)):assert float(csv_row[key])==value
                else:assert csv_row[key]==value
    defines=['FOOT_EARLY_TRIAL','ANGLE_EARLY_TRIAL','WALK_SMOOTH_TRIAL','SYMMETRIC_RESCUE_TRIAL',
             'EARLY_RESCUE_TRIAL','SWAY_BALANCE_TRIAL','FOOT_SUPPORT_ASSIST','FOOT_SUPPORT_LOCAL',
             'FOOT_SUPPORT_TRANSFER','FOOT_SUPPORT_ENERGY','EARLY_ENERGY_MAIN','PULSE_LIFECYCLE_FIX']
    macros=subprocess.check_output(['g++','-E','-dM','-x','c++',str(ROOT/'src/runtime_diagnostics.h')]+
                                   ['-D'+name+'=1' for name in defines],text=True)
    assert '#define RUNTIME_DIAGNOSTIC_BUILD_ID "energy-pulse-lifecycle-v16"' in macros

print('v16 actual metadata: branch21/unchanged model/queue-clock/0-17-80-123ms completion/negative-Q/open/CSV PASS')
