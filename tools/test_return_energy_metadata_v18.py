"""Real v18 metadata/compact rows/CSV: distinguish aid demand from arrival J.

Completion values describe software queue timing and provisional predictions.
The fixture does not establish physical delivery or walking recovery.
"""
from pathlib import Path
import csv
import subprocess
import tempfile

import convert_rwlog_to_csv as converter

ROOT = Path(__file__).resolve().parents[1]
base = ROOT / 'tools/test_energy_pulse_lifecycle_metadata_v16.py'
context = {'__file__': str(base)}
exec(compile(base.read_text(encoding='utf8').split('\nwith tempfile.TemporaryDirectory', 1)[0], str(base), 'exec'), context)
logger = (ROOT / 'src/psram_logger.cpp').read_text(encoding='utf8')
fixture = '#include "half_cycle_energy_budget.h"\n' + context['fixture']

# Exercise the real revision declaration too, rather than the older fixture's
# deliberately frozen v14/v15/v17 declaration.
revision_start = logger.index('#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18\n  const char* walking_energy_revision=')
revision_end = logger.index('#endif', revision_start) + len('#endif')
old_revision = '  const char* walking_energy_revision=return_energy_mode?"walking_return_energy_v17":early_energy_mode?walking_energy_policy_v15::kRevision:walking_energy_policy::kRevision;'
assert fixture.count(old_revision) == 1
fixture = fixture.replace(old_revision, logger[revision_start:revision_end])
fixture = fixture.replace('for(unsigned branch:{0u,15u,19u,20u,21u})',
                          'for(unsigned branch:{0u,15u,19u,20u,21u,22u,23u})')

original = '    CHECK(decision.valid && !decision.coast && decision.width_ms==80);'
assert fixture.count(original) == 1
fixture = fixture.replace(original, r'''
    if(i==1){
      walking_return_energy::Input aid;
      static_cast<walking_energy_policy::Input&>(aid)=in;
      aid.body_angle_deg=9.4f;aid.return_work_j=.00015f;
      decision=half_cycle_energy_budget::decideReturnAid(geometry,table,aid);
    }
    CHECK(decision.valid && !decision.coast);
    CHECK(i==1 || decision.width_ms==80);
''')
original = 'a.requested_ms=a.guarded_ms=80;'
assert fixture.count(original) == 1
fixture = fixture.replace(original,
    'a.arrival_bias_j=i==1?0:.000123f;a.return_work_j=i==1?.00015f:0;a.return_event=i==1;'
    'a.return_deceleration_dps2=154.68f;a.return_stopping_distance_deg=.485f;a.return_foot_age_ms=131;'
    'a.return_evidence=i==1?2:0;a.assisted_cycle=i==1 || i==2;'
    'a.envelope_before_ms=i==2?63:80;a.envelope_after_ms=i==1?63:0;'
    'a.requested_ms=a.guarded_ms=decision.width_ms;')
fixture = fixture.replace('done.planned_ms=80;', 'done.planned_ms=a.guarded_ms;')
fixture = fixture.replace('a.completion_reason=i==0 || i==5?1:i==1 || i==6?2:3;',
                          'a.completion_reason=i==0 || i==5?1:i==1?5:i==6?2:3;')

with tempfile.TemporaryDirectory(prefix='return_metadata_v18_') as folder:
    temp = Path(folder)
    (temp / 'test.cpp').write_text(fixture, encoding='utf8')
    exe = temp / 'test.exe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DEARLY_ENERGY_MAIN=1', '-DPULSE_LIFECYCLE_FIX=1', '-DWALKING_RETURN_ENERGY=1',
                    '-DWALKING_TRANSFER_V18=1', '-I' + str(ROOT / 'tools/host_v46o'),
                    '-I' + str(ROOT / 'src'), str(temp / 'test.cpp'), '-o', str(exe)], check=True)
    for mode, count in [('', 7), ('empty', 0)]:
        output = subprocess.check_output([str(exe)] + ([mode] if mode else []), text=True).splitlines()
        raw, _ = map(context['parse'], output)
        revision = 'walking_energy_transfer_v18'
        for key in ['energy_model', 'wheel_work_model_revision', 'lateral_energy_revision',
                    'energy_control_autonomous_revision', 'energy_request_audit_revision']:
            assert raw[key] == revision, (key, raw[key])
        assert raw['work_control_branch_codes']['22'] == 'walking_return_energy_v17'
        assert raw['work_control_branch_codes']['23'] == revision
        assert raw['pulse_lifecycle']['selector_revision'] == revision
        assert raw['pulse_lifecycle']['physical_delivery_measured'] is False
        assert raw['early_energy_main']['reversal_cutoff_active'] is False
        assert raw['early_energy_main']['mechanical_zero_cutoff_active'] is True
        transfer = raw['walking_transfer']
        assert transfer['branch'] == 23 and transfer['combined_halfcycle_ceiling_ms'] == 80
        assert transfer['ceiling_physics_derived'] is False
        assert transfer['assisted_halfcycle_used_for_arrival_bias_learning'] is False
        assert transfer['dropout_budget_limit_target_fraction'] == .25
        assert transfer['missing_angles_forward_filled'] is False
        assert transfer['physical_recovery_validated'] is False
        assert transfer['mechanical_zero_cutoff_reason'] == 5
        assert 'not_measured_physical_current_edge' in transfer['cutoff_semantics']
        assert 'nominal_potential_not_credited' in transfer['return_aid_request']
        assert 'not_next_peak_arrival_energy' in transfer['return_aid_fulfillment']
        assert 'fresh_rate' in transfer['zero_main']
        assert 'two_20ms_deceleration_windows' in transfer['dropout_evidence']
        assert 'actual_ZERO_recomputes_full_arrival_request' in raw['energy_control_autonomous_policy']
        assert 'aid_available=0' in raw['lateral_energy_formula']
        assert 'nominal_U_diagnostic_not_aid_credit' in raw['lateral_energy_formula']
        assert 'not_next_peak_arrival' in raw['energy_request_audit_semantics']
        for event in raw['event_revision_checks']:
            if event['branch'] == 23:
                assert event['free_model_revision'] == event['arrival_energy_model_revision'] == revision
                assert event['maximum_input_work_j'] > 0
            if event['branch'] == 22:
                assert event['arrival_energy_model_revision'] == 'walking_return_energy_v17'
        block = raw['walking_energy']
        assert block['revision'] == block['selector_revision'] == revision and block['branch'] == 23
        assert block['combined_half_cycle_envelope_ms'] == 80
        assert block['main_at_real_zero_after_aid'] is True
        assert block['arrival_feedback'] == 'causal_completed_unassisted_main_only'
        assert block['completion_reason_names'][5] == 'mechanical_zero_cutoff'
        assert block['return_evidence_names'][2] == 'recognition_dropout'
        assert 'not_next_peak' in block['aid_prediction_semantics']
        assert block['elapsed_semantics'] == 'software_queue_accept_to_stop_request_not_physical_current_edges'
        assert block['energy_neutral_model'].startswith('nominal_only;')
        assert block['stored'] == block['total'] == count and block['truncated'] is False
        fields = block['records']['fields']
        assert len(fields) == len(set(fields))
        assert all(len(row) == len(fields) for row in block['records']['rows'])
        metadata = converter.expand_tables(raw)
        out = temp / (mode or 'rows')
        out.mkdir()
        assert converter.write_walking_energy_diagnostics(metadata, out) == count
        if count:
            with (out / 'walking_energy.csv').open(newline='', encoding='utf8') as stream:
                rows = list(csv.DictReader(stream))
            aid = metadata['walking_energy']['records'][1]
            assert aid['return_event'] and aid['assisted_cycle'] and aid['return_evidence'] == 2
            assert aid['available_j'] == aid['arrival_bias_j'] == 0
            assert aid['potential_j'] > 0  # Diagnostic retained without credit.
            assert abs(aid['request_j'] - .00015) < 1e-9
            assert aid['predicted_arrival_j'] == aid['predicted_work_j']
            assert aid['elapsed_arrival_pred_j'] == aid['elapsed_work_pred_j']
            assert aid['completion_reason'] == 5 and aid['actual_elapsed_ms'] == 17
            assert aid['envelope_before_ms'] == 80 and aid['envelope_after_ms'] == 63
            assert rows[1]['return_event'] == rows[1]['assisted_cycle'] == 'True'
            assert rows[1]['return_evidence'] == '2' and rows[1]['completion_reason'] == '5'
            assert float(rows[1]['available_j']) == 0 and float(rows[1]['potential_j']) > 0
    defines = ['FOOT_EARLY_TRIAL', 'ANGLE_EARLY_TRIAL', 'WALK_SMOOTH_TRIAL', 'SYMMETRIC_RESCUE_TRIAL',
               'EARLY_RESCUE_TRIAL', 'SWAY_BALANCE_TRIAL', 'FOOT_SUPPORT_ASSIST', 'FOOT_SUPPORT_LOCAL',
               'FOOT_SUPPORT_TRANSFER', 'FOOT_SUPPORT_ENERGY', 'EARLY_ENERGY_MAIN', 'PULSE_LIFECYCLE_FIX',
               'WALKING_RETURN_ENERGY', 'WALKING_TRANSFER_V18', 'WEB_PAGE_TRANSPORT_FIX', 'WALKING_AUDIT_PSRAM']
    macros = subprocess.check_output(['g++', '-E', '-dM', '-x', 'c++', str(ROOT / 'src/runtime_diagnostics.h')] +
                                     ['-D' + name + '=1' for name in defines], text=True)
    assert '#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-energy-transfer-v18"' in macros

print('v18 real metadata: revisions/branch23/aid-only fulfillment/ZERO recompute/shared envelope/reason5/CSV PASS')
