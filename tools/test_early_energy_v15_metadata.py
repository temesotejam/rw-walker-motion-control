"""Actual v14/v15 metadata, candidate-vs-output records and CSV round-trip.

Reuses the existing host fixture without changing the v14 test. Serialization,
record members, policy decisions and selected generic metadata statements come
from current firmware sources. This does not validate physical energy delivery.
"""
from pathlib import Path
import csv
import subprocess
import tempfile

import convert_rwlog_to_csv as converter

ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'tools/test_walking_energy_metadata.py'
context={'__file__':str(base)}
exec(compile(base.read_text(encoding='utf8').split('\nwith tempfile.TemporaryDirectory',1)[0],str(base),'exec'),context)
fixture=context['fixture']
parse=context['parse']
logger=context['logger']


def statement(field):
    marker='  json += "\\\"'+field+'\\\"'
    start=logger.index(marker)
    return logger[start:logger.index(';\n',start)+2]


fields=['energy_model','energy_control_autonomous_policy','lateral_energy_revision',
        'lateral_energy_formula','lateral_energy_coefficient_status',
        'energy_control_autonomous_revision','energy_request_audit_revision',
        'energy_request_audit_semantics','energy_control_autonomous_free_model',
        'energy_control_autonomous_q_policy']
generic=r'''
  const bool walking_energy_mode=true,early_energy_mode=EARLY_ENERGY_MAIN;
  const bool work_mode=true,inverse_mode=false,energy_bridge_enabled_=true;
  const bool positive_current_batch_mode=false,negative_probe_extended_mode=false;
  const bool negative_probe_batch_mode=false,negative_probe_mode_=false;
  const char* walking_energy_revision=early_energy_mode?walking_energy_policy_v15::kRevision:walking_energy_policy::kRevision;
'''+''.join(statement(key) for key in fields)
generic+=r'''  json+="\"event_revision_checks\":[";
'''
generic+=r'''
  for(unsigned branch:{0u,15u,19u,20u})for(bool coast:{false,true}){
    if(branch!=0 || coast)json+=",";
    json+=eventRevisions(branch,coast);
  }
  json+="],";
'''
start=logger.index('    detail += ",\\\"free_model_revision\\\"')
end=logger.index('    detail += ",\\\"work_control_branch\\\"',start)
revision_code=logger[start:end]
maximum_marker='    appendAutonomousNullable("maximum_input_work_j",'
maximum_start=logger.index(maximum_marker)
maximum_code=logger[maximum_start:logger.index(';\n',maximum_start)+2]
event_function=r'''
String eventRevisions(unsigned branch,bool coast){
  struct Event {unsigned work_control_branch;float maximum_input_work_j;} e{branch,.000045678f};
  const bool work_event=branch!=0,inverse_event=false,energy_bridge_enabled_=true;
  const unsigned control_target_cdeg_=800;
  String detail="{\"branch\":"+String(branch)+",\"coast\":"+String(coast?"true":"false");
REVISION_CODE
  auto appendAutonomousNullable=[&](const char* name,float value,unsigned digits){
    detail+=",\""+String(name)+"\":";
    detail+=std::isfinite(value)?String(value,static_cast<int>(digits)):String("null");};
MAXIMUM_CODE
  detail+="}";return detail;
}
'''.replace('REVISION_CODE',revision_code).replace('MAXIMUM_CODE',maximum_code)

prefix=r'''
#include "walking_energy_policy_v15.h"
#include "walking_energy_loss.h"
#include "one_peak_energy_inverse.h"
#include <Arduino.h>
#if EARLY_ENERGY_MAIN
namespace active_energy=walking_energy_policy_v15;
#else
namespace active_energy=walking_energy_policy;
#endif
'''
fixture=prefix+event_function+fixture
fixture=fixture.replace('walking_energy_policy::decide(', 'active_energy::decide(')
fixture=fixture.replace('walking_energy_policy::evaluateWidth(', 'active_energy::evaluateWidth(')
fixture=fixture.replace('i<4 && !empty','i<5 && !empty')
fixture=fixture.replace('in.initial_current_ma=i==2?-150.f:-5.f;', 'in.initial_current_ma=i==2?0.f:-5.f;')
fixture=fixture.replace('a.optional=i==2;a.rescue=i==2;', 'a.optional=i==0 || i==4;a.rescue=i==2;')
fixture=fixture.replace('    ++runner.walking_energy_count_;',r'''
#if EARLY_ENERGY_MAIN
    a.early_budget_checked=a.optional;
    a.early_budget_admitted=a.optional && active_energy::earlyAdmissible(a.decision);
    if(a.optional && !a.early_budget_admitted)a.executed=false;
#endif
    ++runner.walking_energy_count_;''')
sentinel=r'  json+="\"sentinel\":0";'
assert fixture.count(sentinel)==1
fixture=fixture.replace(sentinel,generic+sentinel)

with tempfile.TemporaryDirectory(prefix='early_energy_metadata_') as temporary:
    tmp=Path(temporary)
    (tmp/'test.cpp').write_text(fixture,encoding='utf8')
    for version in [14,15]:
        early=version==15
        revision='walking_early_energy_main_v15' if early else 'walking_peak_rate_energy_v14'
        exe=tmp/f'v{version}.exe'
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra',f'-DEARLY_ENERGY_MAIN={int(early)}',
                        '-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'src'),
                        str(tmp/'test.cpp'),'-o',str(exe)],check=True)
        flags=['-D'+name+'=1' for name in ['FOOT_EARLY_TRIAL','ANGLE_EARLY_TRIAL','WALK_SMOOTH_TRIAL',
               'SYMMETRIC_RESCUE_TRIAL','EARLY_RESCUE_TRIAL','SWAY_BALANCE_TRIAL',
               'FOOT_SUPPORT_ASSIST','FOOT_SUPPORT_LOCAL','FOOT_SUPPORT_TRANSFER','FOOT_SUPPORT_ENERGY']]
        flags += [f'-DEARLY_ENERGY_MAIN={int(early)}']
        macros=subprocess.check_output(['g++','-E','-dM','-x','c++',
                                        str(ROOT/'src/runtime_diagnostics.h')]+flags,text=True)
        build_id='early-energy-main-v15' if early else 'foot-support-energy-v14'
        assert '#define RUNTIME_DIAGNOSTIC_BUILD_ID "'+build_id+'"' in macros
        for mode,count in [('',5),('empty',0)]:
            output=subprocess.check_output([str(exe)]+([mode] if mode else []),text=True).splitlines()
            assert len(output)==2
            raw,reference=map(parse,output)
            assert raw['wheel_work_model_revision']==raw['energy_model']==revision
            for key in ['lateral_energy_revision','energy_control_autonomous_revision','energy_request_audit_revision']:
                assert raw[key]==revision
            assert raw['work_control_branch_codes'][str(20 if early else 19)]==revision
            assert raw['early_energy_main']['active'] is early
            assert raw['early_energy_main']['postguard_budget_required'] is early
            assert raw['early_energy_main']['experimental_ceiling_ms']==80
            assert raw['early_energy_main']['ceiling_physics_derived'] is False
            assert raw['early_energy_main']['physical_zero_deficit_guaranteed'] is False
            assert raw['early_energy_main']['reversal_cutoff_active'] is False
            if early:
                assert 'least_integer' in raw['early_energy_main']['selection']
                assert 'postguard' in raw['energy_control_autonomous_policy']
                assert 'min_integer_with_W_at_least_request' in raw['lateral_energy_formula']
                assert 'rescue15ms' in raw['foot_timing']['policy']
                assert 'experimental80ms' in raw['foot_timing']['policy']
            else:
                assert 'argmin_integer' in raw['lateral_energy_formula']
            for event in raw['event_revision_checks']:
                if event['branch'] in (19,20):
                    expected='walking_early_energy_main_v15' if event['branch']==20 else 'walking_peak_rate_energy_v14'
                    assert event['free_model_revision']==event['arrival_energy_model_revision']==expected
                    assert event['maximum_input_work_j']>0
                else:
                    assert event['arrival_energy_model_revision'] is None
                    assert event['maximum_input_work_j'] is None
            for key in ['tilt_priority','foot_span_control','foot_span_high_current','positive_sway_probe','weak_side_assist']:
                assert raw[key]['active'] is False and raw[key]['width_sizing_active'] is False
            block=raw['walking_energy']
            assert block['revision']==revision and block['branch']==(20 if early else 19)
            assert block['physical_body_inertia_identified'] is False
            assert block['energy_neutral_model'].startswith('nominal_only;')
            assert block['paired_feedback_is_energy_neutral_estimation'] is False
            records=block['records']
            assert len(records['fields'])==len(set(records['fields']))
            assert len(records['rows'])==count
            assert all(len(row)==len(records['fields']) for row in records['rows'])
            metadata=converter.expand_tables(raw)
            rows=metadata['walking_energy']['records']
            for row,original in zip(rows,reference):
                for key,value in original.items():
                    if value is None:assert row[key] is None
                    else:assert abs(row[key]-value)<=5.01e-10
            if early:
                assert block['generic_early_15ms_cap_active'] is False
                assert block['main_ceiling_ms']==80 and block['rescue_ceiling_ms']==15
                assert block['online_reversal_cutoff_active'] is False
                assert 'model_error' in block['energy_fulfillment_scope']
                if rows:
                    assert rows[0]['early_budget_checked'] and not rows[0]['early_budget_admitted']
                    assert not rows[0]['executed'] and rows[0]['predicted_work_j']>0
                    assert rows[0]['output_work_pred_j']==rows[0]['output_q_pred_mas']==0
                    assert rows[0]['unmet_after_command_j']==rows[0]['request_j']
                    assert rows[4]['early_budget_checked'] and rows[4]['early_budget_admitted']
                    assert rows[4]['executed'] and rows[4]['predicted_work_j']>=rows[4]['request_j']
                    assert rows[4]['output_work_pred_j']==rows[4]['predicted_work_j']
            out=tmp/f'csv_{version}_{mode or "rows"}'
            out.mkdir()
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

print('v14/v15 metadata: actual C++ revisions/ceil-budget flags/compact J/candidate-output/CSV PASS')
