"""Real v19 JSON/CSV with support admission and motion-only guard diagnostics.

The same C++ fixture is compiled as v18 and v19 to check that adding admission
metadata does not relabel local aid fulfillment as next-peak energy or change
the model's J/width values. Synthetic monitor state verifies serialization,
not physical fore-aft limits, delivery, or walking recovery.
"""
from pathlib import Path
import csv
import subprocess
import tempfile

import _project  # Add the project tools directory for direct test runs.
import convert_rwlog_to_csv as converter

ROOT = Path(__file__).resolve().parents[1]
base = ROOT / 'test/fixtures/energy/test_return_energy_metadata_v18.py'
context = {'__file__': str(base)}
exec(compile(base.read_text(encoding='utf8').split('\nwith tempfile.TemporaryDirectory', 1)[0], str(base), 'exec'), context)
parse = context['context']['parse']
fixture = '#include "fore_aft_motion_guard.h"\n#include "energy_admission_v19.h"\n#include "receiving_arc_aid.h"\n#if WALKING_RETURN_HISTORY\n#include "return_crossing_work.h"\n#endif\n#if WALKING_RIGHT_ARRIVAL\n#include "right_arrival_energy.h"\n#endif\n' + context['fixture']
fixture = fixture.replace('for(unsigned branch:{0u,15u,19u,20u,21u,22u,23u})',
                          'for(unsigned branch:{0u,15u,19u,20u,21u,22u,23u,24u})')
fixture = '#if WALKING_RETURN_HISTORY\n#include "return_observation_policy.h"\n#endif\n' + fixture
marker = 'class ExperimentRunner {public:\n'
assert fixture.count(marker) == 1
fixture = fixture.replace(marker, marker + r'''
#if WALKING_SMALL_STEP_AB
  bool baseline=false;
  bool smallStepBaselineSelected() const { return baseline; }
#if WALKING_RETURN_HISTORY
  bool history_trial=false;
  bool passage=false;
  bool post_aid=false;
  bool prediction_trial=false;
  bool transfer_carry=false;
  bool isLeftTransferCarryProfile() const { return transfer_carry; }
  bool isTransferPriorityProfile() const { return transfer_carry; }
  bool isContinuationPriorityProfile() const { return transfer_carry; }
  bool returnPredictionTrialSelected() const { return prediction_trial || transfer_carry; }
  bool carryHistoryTrialSelected() const { return returnPredictionTrialSelected() || history_trial; }
#endif
#endif
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  fore_aft_motion_guard::Guard walking_fore_aft_guard_;
  fore_aft_motion_guard::Result walking_fore_aft_last_;
  uint32_t walking_fore_aft_candidates_=0;
#endif
''')
marker = 'a.optional=i==4;a.early_budget_checked=i==4;a.early_budget_admitted=false;'
assert fixture.count(marker) == 1
fixture = fixture.replace(marker, marker + r'''
#if WALKING_RIGHT_ARRIVAL
    if(i==0){
      a.right_arrival_checked=true;a.right_base_ms=54;a.right_base_q=10.f;
      a.right_arrival=right_arrival_energy::limit(geometry,table,25.5f,3.86f,4.5f,0.f,345.f,54);
      a.right_final_j=a.right_arrival.selected_j;
    }
#endif
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
    const auto scope=a.return_event?energy_admission_v19::Scope::ExplicitReturnAid:
        a.optional?energy_admission_v19::Scope::OptionalMain:energy_admission_v19::Scope::OrdinaryZero;
    const auto admission=energy_admission_v19::assess(scope,decision);
    a.energy_admission_checked=true;a.energy_admission_applies=admission.applies;a.energy_admission_allowed=admission.allowed;
    a.energy_unsupported_mask=admission.unsupported_mask;a.energy_admission_reason=static_cast<uint8_t>(admission.reason);
    a.fore_aft_allowed=i!=4;a.fore_aft_reason=static_cast<uint8_t>(i==4?fore_aft_motion_guard::Reason::Candidate:fore_aft_motion_guard::Reason::Healthy);
    a.fore_aft_angle_deg=i==4?7.5f:.25f;a.fore_aft_rate_dps=i==4?35.f:1.f;
#if SUPPORT_TRANSFER_CARRY2
    if(a.return_event){
      a.arc_aid.reason=receiving_arc_aid::Reason::Applied;a.arc_aid.extra_j=.00004f;
#if SUPPORT_TRANSFER_STALL_V3
      a.arc_aid.extension_pivot_only=true;
#endif
      a.arc_foot_source=2;a.base_aid_ms=28;a.arc_candidate_ms=33;a.main_reserve_ms=50;
#if SUPPORT_TRANSFER_ARC_FEEDBACK
      a.arc_feedback_peak_ms=1250;a.arc_feedback_base_ms=30;
      a.arc_feedback_available_j=.000013f;a.arc_feedback_added_j=.000013f;
#endif
#if WALKING_RETURN_HISTORY
      a.history.first_peak_ms=2519;a.history.second_peak_ms=4242;a.history.limit_ms=28;
      a.history.deceleration=77.21f;a.history.request_j=.000044794f;
      a.history_base_ms=19;a.history_final_ms=24;
      a.history_state.body_deg=4.8134f;a.history_state.left_deg=3.66559f;a.history_state.right_deg=-.10185f;
      if (runner.prediction_trial) {
        a.return_evidence=6;a.predicted_transfer_ms=28.75f;a.predicted_terminal_deg=-1.91f;
        a.crossing_work={3132,3309,25.25f,.000079f,28,31};
      }
      if (runner.post_aid) {
        a.return_evidence=8;a.post_cap_ms=38;a.post_release_deceleration_dps2=172.43f;
        a.post_increment_j=.000051f;a.post_elapsed_increment_j=.000049f;
      } else if (runner.passage) {
        a.return_evidence=1;
        a.crossing_work={1351,1512,19.134f,.00003209f,11,21};
        a.crossing_work.passage_main_ms=40;
        a.crossing_work.passage_reserve_ms=54;
        a.crossing_work.passage_main_j=.00012f;
        a.crossing_work.passage_zero_dps=17.03f;
      } else if (runner.transfer_carry) {
        a.return_evidence=1;
        a.crossing_work={3168,3321,31.3949f,.000079f,15,32};
        a.crossing_work.transfer_ms=39.f;
        a.crossing_work.terminal_deg=-1.91f;
        a.crossing_work.arc_request_j=.000036f;
        a.arc_priority.base_ms=2;a.arc_priority.cap_ms=30;
        a.arc_priority.base_work_j=.00000018f;a.arc_priority.zero_work_j=.00000936f;
      }
#endif
    }
#endif
#if WALKING_TURN_OBSERVER
    if(i==0 || i==4){
      a.turn_cycle_end_ms=900;a.turn_base_ms=66;a.turn_candidate_ms=63;
      a.turn_adjustment_j=-.0000197f;
    }
#endif
#endif
''')
marker = '  PsramString json;'
assert fixture.count(marker) == 1
fixture = fixture.replace(marker, r'''
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  if(!empty){
    auto& motion=runner.walking_fore_aft_last_;
    motion.valid=motion.initialized=motion.reference_ready=true;motion.allow_output=false;motion.stop=true;
    motion.start_sample_us=1000000;motion.start_roll_deg=4.f;motion.sample_us=1260000;
    motion.angle_deg=8.5f;motion.rate_dps=35.f;motion.time_to_boundary_ms=328.5f;motion.candidate_us=40000;
    motion.reason=fore_aft_motion_guard::Reason::Diverging;runner.walking_fore_aft_candidates_=3;
  }
#endif
''' + marker)

flags = ['-DEARLY_ENERGY_MAIN=1', '-DPULSE_LIFECYCLE_FIX=1', '-DWALKING_RETURN_ENERGY=1', '-DWALKING_TRANSFER_V18=1']
fixture = '#include "walking_trial_profile.h"\n' + fixture
fixture = fixture.replace('const bool small_step=false;', '''
#if WALKING_SMALL_STEP_AB
  runner.baseline=argc>1 && std::string(argv[1])=="small-step";
#if WALKING_RETURN_HISTORY
  runner.history_trial=argc>1 && std::string(argv[1])=="history";
  runner.passage=argc>1 && std::string(argv[1])=="passage";
  runner.post_aid=argc>1 && std::string(argv[1])=="post-aid";
  runner.prediction_trial=argc>1 && std::string(argv[1])=="prediction";
  runner.transfer_carry=!runner.baseline && !runner.history_trial && !runner.prediction_trial;
#endif
#endif
  const bool small_step=argc>1 && std::string(argv[1])=="small-step";''')
fixture = fixture.replace('const bool return_energy_mode=true;', 'const bool return_energy_mode=!small_step;')
fixture = fixture.replace('early_energy_mode=EARLY_ENERGY_MAIN;', 'early_energy_mode=EARLY_ENERGY_MAIN && !small_step;')
profile_line = next(line for line in context['logger'].splitlines() if 'String(walking_trial::name(walking_profile_))' in line)
fixture = fixture.replace('json+="{";', 'json+="{";\n#if WALKING_SMALL_STEP_AB\n'
                          '  auto walking_profile_=small_step ? walking_trial::Profile::SmallStep : walking_trial::forRun(true,8.f,0.f);\n'
                          '#if WALKING_RETURN_HISTORY\n  if(runner.history_trial) walking_profile_=walking_trial::Profile::CarryHistory;\n#endif\n'
                          '#if WALKING_RETURN_HISTORY\n  if(runner.prediction_trial) walking_profile_=walking_trial::Profile::ReturnPrediction;\n#endif\n'
                          + profile_line + '\n#endif\n'
                          '#if WALKING_SMALL_STEP_AB\n  const bool post_aid_recheck=walking_profile_==walking_trial::Profile::LeftTransferCarryV3 || walking_profile_==walking_trial::Profile::LeftTransferCarryV4 || walking_profile_==walking_trial::Profile::LeftTransferCarryV5;\n'
                          '  const bool continuation_recheck=walking_profile_==walking_trial::Profile::LeftTransferCarryV5;\n'
                          '#else\n  const bool post_aid_recheck=false, continuation_recheck=false;\n#endif\n')
with tempfile.TemporaryDirectory(prefix='admission_metadata_v19_') as folder:
    temp = Path(folder)
    (temp / 'test.cpp').write_text(fixture, encoding='utf8')
    previous_rows = {}
    for admission, carry, turn, arrival in [(False,False,False,False), (True,False,False,False),
                                           (True,True,False,False), (True,True,True,False),
                                           (True,True,True,True)]:
        exe = temp / ('v19.exe' if admission else 'v18.exe')
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror'] + flags +
                       ['-DWALKING_ADMISSION_V19=' + str(int(admission)), '-DSUPPORT_TRANSFER_CARRY2=' + str(int(carry)),
                        '-DWALKING_TURN_OBSERVER=' + str(int(turn)), '-DSUPPORT_TRANSFER_ARC_FEEDBACK=' + str(int(turn)),
                        '-DSUPPORT_TRANSFER_STALL_V3=' + str(int(turn)),
                        '-DWALKING_SMALL_STEP_AB=' + str(int(turn)),
                        '-DWALKING_RIGHT_ARRIVAL=' + str(int(arrival)),
                        '-DSUPPORT_TRANSFER_V39=' + str(int(arrival)),
                        '-DWALKING_RETURN_HISTORY=' + str(int(arrival)),
                        '-I' + str(ROOT / 'test/fixtures/arduino'),
                        '-I' + str(ROOT / 'src'), str(temp / 'test.cpp'), '-o', str(exe)], check=True)
        modes = [('', 7), ('empty', 0)] + ([('small-step', 7)] if turn else []) + ([('history', 7),('prediction', 7),('passage', 7),('post-aid', 7)] if arrival else [])
        for mode, count in modes:
            output = subprocess.check_output([str(exe)] + ([mode] if mode else []), text=True).splitlines()
            raw, _ = map(parse, output)
            metadata = converter.expand_tables(raw)
            rows = metadata['walking_energy']['records']
            if turn:
                assert raw['walking_profile'] == ('v14_small_step' if mode=='small-step' else
                                                  'left_return_history_v1' if mode=='history' else
                                                  'left_return_prediction_v1' if mode=='prediction' else
                                                  'left_transfer_carry_v5' if arrival else 'turn3')
            if arrival and mode!='small-step':
                model=metadata['walking_energy']
                assert model['right_arrival_model']==('positive_unassisted_real_zero_startup_4p5_5_v1'
                       if raw['walking_profile']=='left_transfer_carry_v5' else 'positive_unassisted_real_zero_startup_4p5_v1')
                if rows:
                    r=rows[0]
                    assert r['right_arrival_checked'] and r['right_arrival_eligible']
                    assert model['right_arrival_reason_names'][r['right_arrival_reason']]=='reduced'
                    assert r['right_arrival_base_ms']==54 and 0<r['right_arrival_limit_ms']<54
                    assert r['right_arrival_base_j']>r['right_arrival_final_j']
                    assert not rows[1]['right_arrival_checked'] and rows[1]['right_arrival_final_j'] is None
            else:
                assert 'right_arrival_model' not in metadata['walking_energy']
            if mode=='small-step':
                revision = 'walking_peak_rate_energy_v14'
                assert all(raw[key]==revision for key in ('energy_model','wheel_work_model_revision','lateral_energy_revision',
                                                          'energy_control_autonomous_revision','energy_request_audit_revision'))
                assert raw['foot_timing']['active'] is False and raw['walking_transfer']['active'] is False
                assert raw['fore_aft_motion_guard']['active'] and raw['energy_admission']['early_cap_ms']==15
                assert raw['energy_admission']['reason_names'][9]=='no_positive_small_step_work'
                assert 'v14_nearest_energy_error' in raw['energy_control_autonomous_policy']
                assert revision in raw['energy_control_autonomous_free_model']
                block=metadata['walking_energy']
                assert block['revision']=='small_step_v14_current_guards' and block['selector_revision']==revision
                assert block['branch']==19 and block['generic_early_15ms_cap_active'] is True
                assert block['completion_revision']=='actual_elapsed_pulse_completion_v16'
                assert block['foot_role']=='observation_only'
                continue
            if not admission:
                assert raw['energy_model'] == 'walking_energy_transfer_v18'
                assert 'energy_admission' not in raw and 'fore_aft_motion_guard' not in raw
                previous_rows[mode] = rows
                continue
            revision = 'walking_energy_admission_v19'
            for key in ['energy_model', 'wheel_work_model_revision', 'lateral_energy_revision',
                        'energy_control_autonomous_revision', 'energy_request_audit_revision']:
                assert raw[key] == revision, (key, raw[key])
            assert raw['work_control_branch_codes']['23'] == 'walking_energy_transfer_v18'
            assert raw['work_control_branch_codes']['24'] == revision
            assert raw['walking_transfer']['revision'] == revision and raw['walking_transfer']['branch'] == 24
            assert raw['walking_transfer']['combined_halfcycle_ceiling_ms'] == 80
            assert raw['walking_transfer']['assisted_halfcycle_used_for_arrival_bias_learning'] is False
            assert 'v19_optional_main' in raw['energy_control_autonomous_policy']
            assert 'walking_energy_admission_v19:' in raw['energy_control_autonomous_free_model']
            gate = raw['energy_admission']
            for key in ['active', 'optional_only', 'passive_rate_support_required', 'work_rate_support_required',
                        'charge_support_required', 'ordinary_zero_and_aid_exempt', 'negative_no_joint_support']:
                assert gate[key] is True, key
            for key in ['request_j_changed', 'selected_width_changed', 'selected_current_changed', 'rejected_optional_consumes_ZERO']:
                assert gate[key] is False, key
            assert gate['unsupported_mask_bits'] == {'1': 'passive_rate', '2': 'work_rate', '4': 'charge'}
            assert gate['reason_names'][1:3] == ['ordinary_zero_exempt', 'explicit_return_aid_exempt']
            assert 'not_joint_validation' in gate['coverage_semantics']
            guard = raw['fore_aft_motion_guard']
            assert guard['revision'] == 'fore_aft_motion_guard_v19'
            for key in ['physical_limit_identified', 'inertia_identified', 'energy_model_identified',
                        'corrective_work_commanded', 'fall_prevention_guaranteed', 'few_degree_static_pose_rejected']:
                assert guard[key] is False, key
            assert 'provisional_motion_envelope' in guard['interpretation']
            block = raw['walking_energy']
            if carry:
                aids = metadata['walking_energy']['arc_aids']
                assert len(aids) == bool(count)
                assert 'arc_aid_reason' not in block['records']['fields']
                if count:
                    assert aids[0]['row_index'] == 1 and aids[0]['arc_foot_source'] == 2
                    assert block['arc_aid_reason_names'][aids[0]['arc_aid_reason']] == 'applied'
                    assert block['arc_aid_reason_names'][10] == 'command_veto'
                    if turn:
                        assert block['return_evidence_names'][5] == 'measured_arc_shortfall'
                        assert aids[0]['arc_extension_pivot_only'] is True
                        assert aids[0]['arc_feedback_peak_ms']==1250 and aids[0]['arc_feedback_base_ms']==30
                        assert abs(aids[0]['arc_feedback_available_j']-.000013)<1.e-10
                        assert aids[0]['arc_feedback_command_j']==(aids[0]['arc_feedback_available_j'] if block['records']['rows'][1][block['records']['fields'].index('executed')] else 0)
            else:
                assert 'arc_aids' not in block
            assert block['revision'] == block['selector_revision'] == revision and block['branch'] == 24
            assert block['combined_half_cycle_envelope_ms'] == 80 and block['main_at_real_zero_after_aid'] is True
            assert block['completion_reason_names'][5] == 'mechanical_zero_cutoff'
            assert 'not_next_peak' in block['aid_prediction_semantics']
            assert block['stored'] == block['total'] == count and block['truncated'] is False
            detail = block['fore_aft_motion_guard']
            assert detail['motion_only_not_J'] is True and detail['physical_stability_limit_identified'] is False
            expected = {'candidate_angle_deg': 6, 'minimum_outward_rate_dps': 20, 'predicted_boundary_deg': 20,
                        'maximum_time_to_boundary_ms': 350, 'confirmation_us': 40000,
                        'minimum_outward_gain_deg': 1, 'maximum_sample_age_us': 10000}
            for key, value in expected.items():
                assert detail[key] == value, (key, detail[key])
            for event in raw['event_revision_checks']:
                if event['branch'] == 24:
                    assert event['free_model_revision'] == event['arrival_energy_model_revision'] == revision
                    assert event['maximum_input_work_j'] > 0
                if event['branch'] == 23:
                    assert event['arrival_energy_model_revision'] == 'walking_energy_transfer_v18'
            fields = raw['walking_energy']['records']['fields']
            assert len(fields) == len(set(fields))
            assert all(len(row) == len(fields) for row in raw['walking_energy']['records']['rows'])
            for prior, current in zip(previous_rows['' if mode in ('history','prediction','passage','post-aid') else mode], rows):
                for key in ['available_j', 'target_j', 'potential_j', 'request_j', 'predicted_work_j',
                            'predicted_arrival_j', 'unmet_j', 'q_mas', 'requested_ms', 'guarded_ms']:
                    assert current[key] == prior[key], (key, current[key], prior[key])
                expected_mask = sum(bit for bit, key in [(1, 'passive_rate_supported'), (2, 'work_rate_supported'),
                                                       (4, 'charge_supported')] if not current[key])
                assert current['energy_unsupported_mask'] == expected_mask
            out = temp / (('arrival_' if arrival else 'turn_' if turn else 'carry_' if carry else 'v19_') + (mode or 'rows'))
            out.mkdir()
            assert converter.write_walking_energy_diagnostics(metadata, out) == count
            if count:
                full, aid, _, _, optional, _, _ = rows
                assert full['energy_admission_checked'] and full['energy_admission_allowed'] and not full['energy_admission_applies']
                assert full['energy_admission_reason'] == 1
                assert aid['energy_admission_allowed'] and not aid['energy_admission_applies'] and aid['energy_admission_reason'] == 2
                assert aid['return_event'] and aid['available_j'] == 0
                assert aid['predicted_arrival_j'] == aid['predicted_work_j']
                assert optional['energy_admission_checked'] and optional['energy_admission_applies'] and not optional['energy_admission_allowed']
                assert optional['energy_admission_reason'] in [5, 6, 7, 8] and not optional['executed']
                assert optional['fore_aft_allowed'] is False and optional['fore_aft_reason'] == 3
                assert optional['fore_aft_angle_deg'] == 7.5 and optional['fore_aft_rate_dps'] == 35
                assert detail['initialized'] is True and detail['stop'] is True and detail['allow_output'] is False
                assert detail['reason'] == 'fore_aft_outward_divergence' and detail['candidate_entries'] == 3
                assert detail['start_sample_us'] == 1000000 and detail['start_roll_deg'] == 4
                assert detail['last_sample_us'] == 1260000 and detail['candidate_us'] == 40000
                assert detail['angle_deg'] == 8.5 and detail['rate_dps'] == 35
                with (out / 'walking_energy.csv').open(newline='', encoding='utf8') as stream:
                    csv_rows = list(csv.DictReader(stream))
                if arrival:
                    assert csv_rows[0]['right_arrival_base_ms']=='54'
                    assert int(csv_rows[0]['right_arrival_limit_ms'])<54
                    assert csv_rows[0]['right_arrival_eligible']=='True'
                if carry:
                    assert csv_rows[1]['arc_base_guarded_ms'] == '28' and csv_rows[1]['arc_main_reserve_ms'] == '50'
                    assert all(row['arc_aid_reason'] == '' for i, row in enumerate(csv_rows) if i != 1)
                    if arrival and mode in ('','history','prediction','passage','post-aid'):
                        assert csv_rows[1]['arc_history_base_ms']=='19' and csv_rows[1]['arc_history_selected_ms']=='24'
                        assert csv_rows[1]['arc_history_first_peak_ms']=='2519' and csv_rows[1]['arc_history_second_peak_ms']=='4242'
                        assert csv_rows[1]['arc_history_limit_ms']=='28' and float(csv_rows[1]['arc_history_request_j'])>0
                        assert all(row['arc_history_selected_ms']=='' for i,row in enumerate(csv_rows) if i!=1)
                    else:
                        assert 'arc_history_selected_ms' not in csv_rows[1]
                    if mode=='prediction':
                        assert block['return_evidence_names'][int(csv_rows[1]['return_evidence'])]=='predicted_arc_shortfall'
                        assert float(csv_rows[1]['arc_predicted_transfer_ms'])==28.75
                        assert abs(float(csv_rows[1]['arc_predicted_terminal_deg'])+1.91)<1e-5
                        assert int(csv_rows[1]['arc_crossing_ref_zero_ms'])==3132
                        assert int(csv_rows[1]['arc_crossing_selected_ms'])==31
                    if arrival and mode=='':
                        assert (int(csv_rows[1]['arc_priority_base_ms']),int(csv_rows[1]['arc_priority_cap_ms']))==(2,30)
                        assert abs(float(csv_rows[1]['arc_priority_base_j'])-.00000018)<1e-9
                        assert abs(float(csv_rows[1]['arc_priority_zero_j'])-.00000936)<1e-9
                        assert float(csv_rows[1]['arc_approach_transfer_ms'])==39
                        assert abs(float(csv_rows[1]['arc_approach_terminal_deg'])+1.91)<1e-5
                        assert abs(float(csv_rows[1]['arc_approach_request_j'])-.000036)<1e-9
                        assert int(csv_rows[1]['arc_crossing_ref_zero_ms'])==3168
                        assert int(csv_rows[1]['arc_crossing_selected_ms'])==32
                    elif arrival and mode=='post-aid':
                        row = csv_rows[1]
                        assert 'reopen_aid_once' in raw['foot_timing']['policy']
                        assert 'observed_carry_can_end_continuation_early' in raw['foot_timing']['policy']
                        assert block['return_evidence_names'][8]=='continue_after_observed_loss'
                        assert block['completion_reason_names'][7]=='observed_carry_cutoff'
                        assert block['reversal_cutoff_scope']=='post_aid_or_priority_assist_two_fresh_outward_rates_before_zero'
                        assert abs(block['post_aid_coast_fall_tau_s']-.070)<1e-5
                        assert block['return_evidence_names'][7]=='post_aid_shortfall'
                        assert block['return_evidence_names'][int(row['return_evidence'])]=='continue_after_observed_loss'
                        assert abs(float(row['arc_post_release_decel_dps2'])-172.43)<1e-4
                        assert int(row['arc_post_cap_ms']) == 38
                        assert abs(float(row['arc_post_increment_j'])-51e-6) < 1e-9
                        assert abs(float(row['arc_post_elapsed_increment_j'])-49e-6) < 1e-9
                    elif arrival and mode=='passage':
                        row=csv_rows[1]
                        assert row['arc_approach_transfer_ms']==''
                        assert int(row['arc_crossing_ref_zero_ms'])==1351
                        assert (int(row['arc_crossing_base_ms']),int(row['arc_crossing_selected_ms']))==(11,21)
                        assert (int(row['arc_passage_main_ms']),int(row['arc_passage_reserve_ms']))==(40,54)
                        assert abs(float(row['arc_passage_main_j'])-.00012)<1e-9
                        assert abs(float(row['arc_passage_zero_dps'])-17.03)<1e-4
                    else:
                        assert 'arc_approach_transfer_ms' not in csv_rows[1]
                if turn:
                    assert csv_rows[1]['arc_extension_pivot_only'] == 'True'
                    for index in (0,4):
                        event = csv_rows[index]
                        assert event['turn_cycle_end_ms'] == '900'
                        assert event['turn_base_ms'] == '66' and event['turn_candidate_ms'] == '63'
                        assert abs(float(event['turn_adjustment_j']) + .0000197) < 1.e-9
                    assert float(csv_rows[0]['turn_command_adjustment_j']) == float(csv_rows[0]['turn_adjustment_j'])
                    assert float(csv_rows[4]['turn_command_adjustment_j']) == 0
                assert csv_rows[4]['energy_admission_applies'] == 'True'
                assert csv_rows[4]['energy_admission_allowed'] == csv_rows[4]['fore_aft_allowed'] == 'False'
                assert float(csv_rows[4]['fore_aft_angle_deg']) == 7.5
            else:
                assert detail['initialized'] is False and detail['angle_deg'] is None
    defines = ['FOOT_EARLY_TRIAL', 'ANGLE_EARLY_TRIAL', 'WALK_SMOOTH_TRIAL', 'SYMMETRIC_RESCUE_TRIAL',
               'EARLY_RESCUE_TRIAL', 'SWAY_BALANCE_TRIAL', 'FOOT_SUPPORT_ASSIST', 'FOOT_SUPPORT_LOCAL',
               'FOOT_SUPPORT_TRANSFER', 'FOOT_SUPPORT_ENERGY', 'EARLY_ENERGY_MAIN', 'PULSE_LIFECYCLE_FIX',
               'WALKING_RETURN_ENERGY', 'WALKING_TRANSFER_V18', 'WEB_PAGE_TRANSPORT_FIX', 'WALKING_AUDIT_PSRAM',
               'WALKING_ADMISSION_V19']
    macros = subprocess.check_output(['g++', '-E', '-dM', '-x', 'c++', str(ROOT / 'src/runtime_diagnostics.h')] +
                                     ['-D' + name + '=1' for name in defines], text=True)
    assert '#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-energy-admission-v19"' in macros

print('v19 real metadata: branch24/support admission/motion envelope/audit reasons/unchanged v18 J-width/CSV PASS')
