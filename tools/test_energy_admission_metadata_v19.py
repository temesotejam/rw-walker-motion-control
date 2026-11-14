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

import convert_rwlog_to_csv as converter

ROOT = Path(__file__).resolve().parents[1]
base = ROOT / 'tools/test_return_energy_metadata_v18.py'
context = {'__file__': str(base)}
exec(compile(base.read_text(encoding='utf8').split('\nwith tempfile.TemporaryDirectory', 1)[0], str(base), 'exec'), context)
parse = context['context']['parse']
fixture = '#include "fore_aft_motion_guard.h"\n#include "energy_admission_v19.h"\n' + context['fixture']
fixture = fixture.replace('for(unsigned branch:{0u,15u,19u,20u,21u,22u,23u})',
                          'for(unsigned branch:{0u,15u,19u,20u,21u,22u,23u,24u})')
marker = 'class ExperimentRunner {public:\n'
assert fixture.count(marker) == 1
fixture = fixture.replace(marker, marker + r'''
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  fore_aft_motion_guard::Guard walking_fore_aft_guard_;
  fore_aft_motion_guard::Result walking_fore_aft_last_;
  uint32_t walking_fore_aft_candidates_=0;
#endif
''')
marker = 'a.optional=i==4;a.early_budget_checked=i==4;a.early_budget_admitted=false;'
assert fixture.count(marker) == 1
fixture = fixture.replace(marker, marker + r'''
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
    const auto scope=a.return_event?energy_admission_v19::Scope::ExplicitReturnAid:
        a.optional?energy_admission_v19::Scope::OptionalMain:energy_admission_v19::Scope::OrdinaryZero;
    const auto admission=energy_admission_v19::assess(scope,decision);
    a.energy_admission_checked=true;a.energy_admission_applies=admission.applies;a.energy_admission_allowed=admission.allowed;
    a.energy_unsupported_mask=admission.unsupported_mask;a.energy_admission_reason=static_cast<uint8_t>(admission.reason);
    a.fore_aft_allowed=i!=4;a.fore_aft_reason=static_cast<uint8_t>(i==4?fore_aft_motion_guard::Reason::Candidate:fore_aft_motion_guard::Reason::Healthy);
    a.fore_aft_angle_deg=i==4?7.5f:.25f;a.fore_aft_rate_dps=i==4?35.f:1.f;
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
with tempfile.TemporaryDirectory(prefix='admission_metadata_v19_') as folder:
    temp = Path(folder)
    (temp / 'test.cpp').write_text(fixture, encoding='utf8')
    previous_rows = {}
    for admission in [False, True]:
        exe = temp / ('v19.exe' if admission else 'v18.exe')
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror'] + flags +
                       ['-DWALKING_ADMISSION_V19=' + str(int(admission)), '-I' + str(ROOT / 'tools/host_v46o'),
                        '-I' + str(ROOT / 'src'), str(temp / 'test.cpp'), '-o', str(exe)], check=True)
        for mode, count in [('', 7), ('empty', 0)]:
            output = subprocess.check_output([str(exe)] + ([mode] if mode else []), text=True).splitlines()
            raw, _ = map(parse, output)
            metadata = converter.expand_tables(raw)
            rows = metadata['walking_energy']['records']
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
            for prior, current in zip(previous_rows[mode], rows):
                for key in ['available_j', 'target_j', 'potential_j', 'request_j', 'predicted_work_j',
                            'predicted_arrival_j', 'unmet_j', 'q_mas', 'requested_ms', 'guarded_ms']:
                    assert current[key] == prior[key], (key, current[key], prior[key])
                expected_mask = sum(bit for bit, key in [(1, 'passive_rate_supported'), (2, 'work_rate_supported'),
                                                       (4, 'charge_supported')] if not current[key])
                assert current['energy_unsupported_mask'] == expected_mask
            out = temp / ('v19_' + (mode or 'rows'))
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
