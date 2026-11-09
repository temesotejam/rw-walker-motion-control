#!/usr/bin/env python3
"""Validate real C++ maximum-capacity serialization and corrupted-file handling."""
from pathlib import Path
import csv, json, struct, tempfile, shutil, subprocess, sys
import convert_rwlog_to_csv as converter
work=json.loads(Path('/tmp/control-work-fixture.json').read_text())
assert work['pulse_on']['log_row']==dict(count=1,max_us=251,sum_us=251,mean_us=251)
assert work['pulse_on']['log_encode']['mean_us']==100
assert work['pulse_on']['log_store']['mean_us']==151
assert work['log_substages']=='encode_includes_beta_ceilings;store_is_synchronous_psram_addSample'
assert work['target_code_placement']=='IRAM_MEKF_log_encoder_normal_update_and_Madgwick_IMU;external_calls_and_data_may_use_flash'
assert work['normal_update_compiler']=='selected_routines_GCC_O2_no_fast_math;upstream_Madgwick_2.4.0_function_bodies_unchanged'
assert work['pulse_off']['filter']['mean_us']==700
assert work['pulse_on']['filter']['count']==0
diagnostics=json.loads(Path('/tmp/mekf-diagnostics-fixture.json').read_text(),
    parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
assert diagnostics['invalid']['valid'] is False and diagnostics['invalid']['fresh'] is False
assert diagnostics['invalid']['roll_deg'] is None and diagnostics['invalid']['quaternion']['w'] is None
for axis, attitude in enumerate(diagnostics['axes']):
    assert attitude['valid'] and attitude['fresh'] and attitude['age_us'] == 2500
    assert attitude['sample_us'] == 1000000
    assert 'yaw_deg' not in attitude
    for index, key in enumerate(('roll_deg','pitch_deg')):
        assert abs(attitude[key] - (20 if axis == index else 0)) < 0.001
    assert attitude['estimate'] == 'posterior' and attitude['euler_order'] == 'ZYX'
assert diagnostics['stale']['valid'] and not diagnostics['stale']['fresh']
assert not diagnostics['sensor_failed']['fresh']
assert diagnostics['wrapped']['age_us'] == 251 and diagnostics['wrapped']['fresh']
assert diagnostics['invalid']['inputs']['valid'] is False
assert diagnostics['invalid']['inputs']['accel_g'] == [None,None,None]
inputs=diagnostics['inputs']['inputs']
assert inputs['valid'] and inputs['frame']=='mekf' and inputs['axis_order']=='xyz'
assert inputs['accel_age_us']==3500 and inputs['accel_sample_us']==999000
assert inputs['accel_g']==[.01,.02,1]
for actual,expected in zip(inputs['gyro_dps'],[1,2,3]): assert abs(actual-expected)<.00001
for actual,expected in zip(inputs['gyro_bias_dps'],[.1,-.2,.3]): assert abs(actual-expected)<.00001
assert inputs['last_accel_update']['used'] and inputs['last_accel_update']['confidence']>.99
evidence=json.loads(Path('tools/fixtures/foot_calibration_20261101.json').read_text())
calibration=diagnostics['calibration']
assert calibration['revision']=='fixed_leg_five_pose_repeat_20261101' and calibration['provisional']
assert calibration['fit_poses']==5 and calibration['heldout_hand_supported_poses']==0
assert calibration['outside_range_policy']=='extrapolate_nearest_endpoint_segment_and_save'
assert not calibration['independent_angular_accuracy_validated']
import hashlib
assert calibration['source_sha256']==hashlib.sha256(Path('tools/fixtures/foot_calibration_20261101.json').read_bytes()).hexdigest()
for side in ('right','left'):
    records=evidence['original_records']
    repeat=evidence['repeat_records']
    endpoint=(records[-1][side+'_x']+repeat[-1][side+'_x']+records[0][side+'_x']-repeat[0][side+'_x'])/2
    angle=(records[-1]['reference_angle_deg']+repeat[0]['roll_deg']-repeat[-1]['roll_deg'])/2
    assert abs(diagnostics['range'][side+'_support_x'][0]-endpoint)<.0001
    assert abs(diagnostics['range'][side+'_support_x'][1]-records[0][side+'_x'])<.0001
    assert abs(calibration[side+'_points'][0][0]-endpoint)<.0001
    assert abs(calibration[side+'_points'][0][1]-angle)<.00001
    for point, record in zip(calibration[side+'_points'][1:],reversed(records[:-1])):
        assert abs(point[0]-record[side+'_x'])<.0001
        assert abs(point[1]-record['reference_angle_deg'])<.00001
source = Path('/tmp/runtime-fixture.rwlog')
data = source.read_bytes()
header = converter.parse_header(data)
metadata = converter.expand_tables(json.loads(data[110:110+header['metadata_json_size']], parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value))))
assert metadata['metadata_json_final_bytes'] == header['metadata_json_size']
assert not metadata['metadata_event_detail_truncated']
assert metadata['firmware_revision']=='1.0.99-alternate-foot-trial'
for manifest in ('site/manifest.json','site/manifest-freefoot-runtime-v2.json'):
    cfg=json.loads(Path(manifest).read_text())
    assert cfg['version']==metadata['firmware_revision']
    assert cfg['new_install_improv_wait_time']==0
assert metadata['energy_model']=='angle_energy_bridge'
assert metadata['energy_control_autonomous_revision']=='lateral_energy_angle_bridge_v8_20261015'
assert metadata['energy_control_autonomous_negative_side_brake_width_max_ms']==0
assert metadata['energy_control_autonomous_integral_Ki_J_per_J'] is None
assert abs(metadata['energy_control_autonomous_integral_Ki_plus_J_per_J']-.029032)<1e-6
assert metadata['upright_reference_valid'] is True
assert metadata['upright_reference_revision'] == 3
assert abs(metadata['upright_reference_abs_pitch_deg'] - 2.2) < 1e-6
assert abs(metadata['measurement_start_abs_pitch_deg'] - 3.2) < 1e-6
assert abs(metadata['measurement_start_upright_relative_deg'] - 1.0) < 1e-6
assert 'no_work_rezero' in metadata['angle_reference_policy']
assert metadata['peak_error_adaptation_enabled'] is False
assert metadata['energy_request_audit_revision']=='unrestricted_inverse_v1_20261014'
audit=metadata['energy_control_autonomous_zero_cross_events'][0]
assert abs(audit['unclamped_energy_request_j']-.00040325)<1e-8
assert abs(audit['q_unclamped_mA_s']-9.01853)<1e-5
assert audit['unclamped_inverse_valid'] is True
assert audit['maximum_input_work_j'] is None
assert abs(audit['maximum_effective_arrival_energy_increment_j']-.00003841)<1e-8
assert 'signed_wheel_speed_bin' in metadata['peak_error_adaptation_policy']
assert metadata['legacy_rate_and_Q1_models_active'] is False
assert metadata['lateral_energy_coefficient_status'].startswith('historical_04731_positive_Q_peak_angle_fit')
assert metadata['foot_camera_required_for_autonomous_start'] is False
assert metadata['motor_driver_polarity']==-1
assert metadata['normal_autonomous_motor_output_sign']==-1
assert metadata['energy_control_autonomous_brake_width_max_ms']==0
assert metadata['energy_control_autonomous_negative_side_brake_width_max_ms']==0
assert 'motor_cmd_mA_is_signed_current_sent_to_Roller485' in metadata['motor_direction_semantics']
assert metadata['autonomous_input_advance_us']==0
assert metadata['autonomous_total_timing_projection_us']==0
assert metadata['autonomous_timing_compensation_us']==0
assert metadata['autonomous_input_peak_percent']==65
assert metadata['autonomous_duration_ms']==15000
assert metadata['autonomous_control_prediction_enabled'] is False
assert metadata['rate_baseline_revision']=='fixed_foot_mekf_rate_refit_04731_20260929'
assert metadata['previous_peak_residual_enabled'] is False
assert metadata['rate_baseline_previous_peak_used'] is False
assert '204_matched_normal_commands' in metadata['rate_baseline_fit_source']
assert 'not_closed_loop_validation' in metadata['rate_baseline_fit_source']
latency=metadata['control_latency']
assert latency['revision']=='control_latency_04724'
details=latency['overrun_detail']
assert details['total']==70 and details['stored']==64 and details['overflow']==6
assert details['budget_us']==2500 and details['rows_use_fields_above']
for i,row in enumerate(details['samples']):
    event=dict(zip(latency['fields'].split(','),row))
    assert len(row)==len(latency['fields'].split(','))
    assert event['sequence']==i+1 and event['done_age']==3000
    assert event['queue_submit_age_us']==25 and event['queue_receive_age_us']==180
    assert event['received_age']==200 and event['delivery_valid']==1
assert metadata['execution_placement']['wifi_event_configured_core']==0
solver = metadata['v46s_solver_audit']
assert solver['schema_version']==2 and solver['solver_revision']=='direct_q_branch_inverse_04718'
assert 'minimum_absolute_Q_error' in solver['inverse_policy']
assert 'ff_width_ms=65535_not_computed' in solver['field_semantics']
assert len(solver['events'])==128 and solver['events'][0]['ff_width_ms']==65535
assert metadata['terminal_state']['state']=='ESTOP' and metadata['terminal_state']['motor_cmd_mA']==0
assert metadata['terminal_state']['actual_current_mA']==-7 and metadata['terminal_state']['heartbeat_us']==123456
assert metadata['terminal_state']['last_error']=='imu_acquisition_overflow_backlog_or_stale'
assert metadata['control_work_profile']['pulse_on']['log_row']['mean_us']==777
assert len(metadata['energy_control_autonomous_peak_events']) == 256
assert len(metadata['energy_control_autonomous_zero_cross_events']) == 256
batch_rows=metadata['energy_control_autonomous_zero_cross_events']
assert batch_rows[0]['negative_probe_batch_index'] is None
assert batch_rows[0]['positive_current_batch_index'] is None
assert batch_rows[2]['positive_current_batch_index'] == 2
assert batch_rows[2]['positive_current_batch_planned_mA'] == 350
assert batch_rows[0]['negative_probe_batch_planned_width_ms'] is None
assert batch_rows[0]['negative_probe_batch_rate_skipped'] is None
assert batch_rows[1]['negative_probe_batch_index'] == 1
assert batch_rows[1]['negative_probe_batch_planned_width_ms'] == 20
assert batch_rows[1]['negative_probe_batch_rate_skipped'] is True
assert batch_rows[2]['negative_probe_batch_index'] is None
assert batch_rows[3]['negative_probe_batch_rate_skipped'] is False
assert 'target_energy_j' in metadata['energy_control_autonomous_peak_events'][0]
assert 'model_residual_j' in metadata['energy_control_autonomous_peak_events'][0]
assert 'kinetic_energy_j' in metadata['energy_control_autonomous_zero_cross_events'][0]
assert 'energy_model_extrapolated' in metadata['energy_control_autonomous_zero_cross_events'][0]
assert 'maximum_effective_arrival_energy_increment_j' in metadata['energy_control_autonomous_zero_cross_events'][0]
for key in ('prediction_bias_j','predicted_observed_energy_j',
            'predicted_observed_next_peak_deg','adaptation_mode',
            'wheel_speed_bin','adaptation_consistent_samples'):
    assert key in metadata['energy_control_autonomous_zero_cross_events'][0]
assert len(metadata['foot_frames']) == 768
assert metadata['foot_observation']['detector'] == 'sparse_rows_identity_v3'
assert metadata['foot_observation']['weak_candidate_confirmation']==dict(frames=3,min_x_step_px=20,min_y_step_px=16,
    contrast_ratio_below=.5,weight_ratio_below=.1,condition='recent_track_and_xy_steps_and_both_quality_drops')
assert metadata['foot_observation']['zero_reason'] == 'ready'
assert metadata['foot_observation']['zero_max_pitch_error_deg'] == 5
assert 'roll_offset_allowed_if_stationary' in metadata['foot_observation']['zero_pitch_reference']
assert metadata['foot_observation']['zero_max_nominal_offset_px'] == 35
assert metadata['foot_observation']['zero_max_spread_px'] == 4
assert metadata['foot_observation']['vertical_recovery_angle_accuracy_validated'] is False
assert metadata['foot_observation']['range'] == diagnostics['range']
assert metadata['foot_observation']['calibration']==calibration
assert metadata['foot_observation']['calibration_source_commit'] is None
for side in ('right','left'):
    assert metadata['foot_observation'][side+'_deg_per_px']==calibration[side+'_deg_per_px']
for side in ('right','left'):
    for actual,expected in zip(metadata['foot_observation'][side+'_support_x'],
                               diagnostics['range'][side+'_support_x']):
        assert abs(actual-expected)<.0001
assert metadata['foot_frames'][0]['right_valid']
assert not metadata['foot_frames'][0]['right_in_range']
assert metadata['foot_frames'][0]['right_deg'] == 5 # outside support is still serialized
assert metadata['foot_frames'][0]['right_scan_y'] == 42
assert metadata['foot_frames'][0]['left_templates'] == 17
assert metadata['foot_frames'][0]['left_candidates'] == 2
assert metadata['foot_frames'][0]['left_ambiguity'] == 0.4
assert metadata['foot_frames'][0]['zero_reason'] == 'ready'
assert metadata['foot_frames'][1]['right_reason'] == 'low_contrast'
assert metadata['foot_frames'][1]['right_deg'] is None
with tempfile.TemporaryDirectory() as tmp:
    output = Path(tmp)/'converted'
    # The converter remains usable as one downloaded Python file.
    standalone=Path(tmp)/'standalone_converter.py'
    shutil.copyfile('tools/convert_rwlog_to_csv.py', standalone)
    subprocess.run([sys.executable,str(standalone),str(source),'--out',str(Path(tmp)/'standalone')],check=True)
    converter.convert(source, output)
    with (output/'energy_control_autonomous_zero_cross_events.csv').open() as stream:
        converted_rows=list(csv.DictReader(stream))
    converted_zero=converted_rows[0]
    negative=converted_rows[1]
    assert negative['physical_next_peak_side']=='-1'
    assert negative['negative_recovery_baseline_ms']=='28' and negative['negative_recovery_candidate_ms']=='33'
    assert negative['negative_recovery_executed']=='True' and negative['negative_recovery_state']=='1'
    assert negative['long_foot_actual_ms']==''
    assert converted_zero['long_foot_actual_ms']=='150' and converted_zero['negative_recovery_candidate_ms']==''
    assert abs(float(converted_zero['maximum_effective_arrival_energy_increment_j'])-.00003841)<1e-8
    assert converted_zero['maximum_input_work_j']==''
    with (output/'timeseries.csv').open() as stream:
        samples=list(csv.DictReader(stream))
    for name in ('gyro_heading_deg','steering_actual_difference_deg',
                 'steering_desired_difference_deg','steering_cycle_yaw_rate_dps'):
        assert samples[0][name] == ''
    assert float(samples[0]['steering_delta_deg']) == 0
    assert samples[0]['gyro_heading_valid'] == '0'
    assert samples[0]['steering_cycles'] == '0'
    assert samples[0]['steering_reason'] == '8'
    for name in ('enabled','feedback_enabled','heading_enabled','response_check_enabled'):
        assert metadata['steering'][name] is False
    assert metadata['steering']['profile'] == 'disabled'
    assert metadata['steering']['reason_codes']['8'] == 'disabled'
    assert 'response_schedule_ms' not in metadata['steering']
    assert header['metadata_json_size'] < metadata['metadata_json_budget_bytes']*.8

    assert len((output/'foot_angles.csv').read_text().splitlines()) == 769
    with (output/'foot_angles.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    assert float(rows[0]['right_scan_y']) == 42 and rows[0]['left_templates'] == '17'
    assert rows[0]['left_candidates'] == '2' and float(rows[0]['left_ambiguity']) == 0.4
    assert rows[0]['zero_reason'] == 'ready'
    assert rows[1]['right_reason'] == 'low_contrast' and rows[1]['right_deg'] == ''
    # Older RWLOG files have no recovery diagnostics; conversion leaves blanks.
    converter.write_foot_frames({'foot_frames':[{'right_deg':5}]},output)
    with (output/'foot_angles.csv').open() as stream:
        legacy=list(csv.DictReader(stream))[0]
    assert legacy['right_deg'] == '5' and legacy['right_scan_y'] == '' and legacy['right_reason'] == ''
    assert legacy['left_candidates'] == '' and legacy['zero_reason'] == ''
    corrupt = bytearray(data); corrupt[-10] ^= 1
    bad = Path(tmp)/'bad.rwlog'; bad.write_bytes(corrupt)
    try:
        converter.convert(bad, Path(tmp)/'bad_csv')
        raise AssertionError('corruption accepted')
    except ValueError as error:
        assert 'CRC' in str(error)
print('maximum RWLOG JSON, complete event/foot counts, CSV and corrupt-file refusal PASS')

assert metadata["columns"]["timeseries"] == converter.CSV_COLUMNS_V56
assert metadata['mekf_roll_pitch_semantics'].startswith('v54_posterior_ZYX_roll_and_pitch_deg_same_sample')
assert header["format_version"] == 56 and header["log_sample_size"] == 172
assert samples[0]['roll_mekf_abs_deg'] == '12.34'
assert samples[0]['pitch_mekf_abs_deg'] == '5.67'
assert samples[0]['roller_speed_rpm'] == '123.45'
assert samples[0]['physical_roll_rate_dps'] == '60.0'
assert samples[0]['roller_speed_sequence'] == '1'
assert samples[0]['roller_applied_command_time_us'] == '999000'
assert samples[0]['roller_position_raw_cdeg'] == '-2147483648'
assert samples[0]['roller_position_deg'] == '-21474836.48'
assert samples[0]['roller_position_sample_time_us'] == '999900'
assert samples[0]['roller_position_sequence'] == '9'
assert samples[0]['roller_position_valid'] == '1'
assert metadata['wheel_work_velocity_source']=='position_local_quadratic_5_points'
assert metadata['wheel_work_inertia_kg_m2'] == .000028
assert metadata['wheel_speed_to_body_sign'] == -1
assert "magnetometer" not in metadata

# Validate complete mixed-mode RWLOGs, including CRC and null/value alignment.
# The startup/fallback and supported-work rows must share exactly one schema.
for work_first, suffix in ((False, '.fallback-first.rwlog'), (True, '.work-first.rwlog')):
    work_data = Path(str(source) + suffix).read_bytes()
    work_header = converter.parse_header(work_data)
    work_metadata = converter.expand_tables(json.loads(
        work_data[110:110 + work_header['metadata_json_size']],
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value))))
    assert work_metadata['energy_model'] == 'wheel_work_one_peak_control'
    assert work_metadata['peak_mpc']['active'] is False
    assert work_metadata['low_peak_energy_feedback']['active_for_output'] is False
    assert work_metadata['foot_phase_cap']['hardware_validated'] is False
    assert work_metadata['control_angle_origin_abs_deg'] == 0
    assert work_metadata['work_absolute_reference_enabled'] is True
    assert work_metadata['autonomous_duration_ms'] == 30000
    assert work_metadata['adaptive_lateral_cg']['enabled'] == work_first
    if work_first:
        assert work_metadata['lateral_cg']['valid'] is True
        assert work_metadata['lateral_cg']['physical_calibration_valid'] is False
        assert work_metadata['adaptive_lateral_cg']['initial_mm'] == .1
        assert work_metadata['adaptive_lateral_cg']['final_mm'] == .5
        assert work_metadata['adaptive_lateral_cg']['updates'] == 3
    assert work_metadata['energy_control_autonomous_negative_side_brake_width_max_ms'] == 70
    assert work_metadata['energy_control_autonomous_brake_width_max_ms'] == 70
    assert work_metadata['energy_control_autonomous_integral_Ki_plus_J_per_J'] == 0
    assert work_metadata['wheel_work_positive_current_candidates_mA'] == [300,350,400]
    assert work_metadata['tilt_priority']['branch'] == 15
    assert work_metadata['tilt_priority']['command_mA'] == 1200
    assert work_metadata['tilt_priority']['initial_target_deg'] == 4.5
    assert work_metadata['tilt_priority']['final_target_deg'] == 8
    assert work_metadata['tilt_priority']['target_step_deg'] == 0.5
    assert work_metadata['tilt_priority']['initial_width_ms'] == 60
    assert work_metadata['tilt_priority']['max_width_ms'] == 150
    assert work_metadata['foot_timing']['revision'] == 'zero_centre_inward_stall_rescue_v1'
    assert work_metadata['foot_timing']['event_kind'] == 'inward_stall_rescue'
    assert work_metadata['foot_timing']['hardware_validated'] is False
    assert work_metadata['foot_gap_trend']['revision'] == 'bounded_observation_gaps_v1'
    assert work_metadata['foot_gap_trend']['active_for_actuation'] is False
    foot_balance = work_metadata['foot_span_high_current']
    assert foot_balance['revision'] == 'three_adjacent_pairs_trial_loss_v5'
    assert foot_balance['proposal_age_ms'] == 1500
    assert foot_balance['command_mA'] == 1200
    assert foot_balance['minimum_stage_target_deg'] == 6
    assert foot_balance['valid_cycles'] == 6
    assert foot_balance['valid_pairs'] == 3
    assert 'disabled_in_firmware_1p0p92' in work_metadata['negative_cap_recovery']['policy']
    assert foot_balance['max_age_ms'] == 12000
    assert foot_balance['step_ms'] == 5
    assert foot_balance['max_offset_ms'] == 20
    assert foot_balance['trial_cap_ms'] == 100
    assert foot_balance['hardware_validated'] is False
    speed_guard = work_metadata['tilt_priority']['speed_guard']
    assert speed_guard['signal'] == 'zero_cross_abs_rate_dps'
    assert speed_guard['full_width_rate_per_target_deg'] == 4
    assert speed_guard['taper_full_rate_per_target_deg'] == 12
    assert speed_guard['taper_endpoint_ms'] == 20
    assert work_metadata['tilt_priority']['hardware_validated'] is False
    assert work_metadata['metadata_json_final_bytes'] == work_header['metadata_json_size']
    assert not work_metadata['metadata_event_detail_truncated']
    work_rows = work_metadata['energy_control_autonomous_zero_cross_events']
    assert len(work_rows) == 256
    assert work_metadata['energy_control_autonomous_zero_cross_event_rendered_count'] == 256
    for index, event in enumerate(work_rows):
        assert abs(event['cg_shift_mm']-(.35 if index%2 else -.15))<1e-6
        assert event['work_model_weight'] == (.75 if index%2 else 1)
        walking = index == 255
        experimental = index in (251,252,253,254)
        fallback = not walking and not experimental and ((index % 3 == 0) != work_first)
        cross_condition = walking or experimental or (not fallback and index % 4 == 1)
        if index % 2 == 0:
            assert event['work_bias_samples'] == 2
        assert event['event_index'] == index + 1
        assert event['energy_model_extrapolated'] == (fallback or cross_condition)
        assert event['work_prediction_cross_condition'] == cross_condition
        if experimental:
            expected = {251:'gradual_sway_unmodeled_trial_or_coast',
                        252:'weak_side_assist_v1_unmodeled',
                        253:'positive_asymmetry_unmodeled_trial',
                        254:'gradual_sway_unmodeled_trial_or_coast'}[index]
            assert event['work_control_branch'] == (15 if index == 251 else index-239)
            if index in (251,254):
                assert event['valid'] is True
                assert event['output_executed'] is (index == 251)
                assert event['command_current_mA'] == (1200 if index == 251 else 0)
                assert event['pulse_width_ms'] == (60 if index == 251 else 0)
                assert event['startup_target_deg'] == 4.5
                assert event['startup_positive_cap_ms'] == 60
            assert event['arrival_energy_model_revision'] is None
            assert event['free_model_revision'] == expected
        elif fallback:
            assert event['arrival_energy_model_revision'] is None
        elif walking:
            assert event['arrival_energy_model_revision'] == 'walking_coast_eight_runs_v6'
            assert event['free_model_revision'] == 'walking_coast_eight_runs_v6'
            assert event['predicted_loss_j'] == .00014
            assert abs(event['foot_balance_target_deg']-8.2)<.00001
            assert abs(event['foot_balance_left_mean']-2.5)<.00001
            assert abs(event['foot_balance_right_mean']-.2)<.00001
            assert abs(event['foot_energy_bias']-.08)<.00001
            assert event['foot_energy_left_px']==20 and event['foot_energy_right_px']==2
            assert event['foot_energy_valid'] is True and event['foot_energy_phase']==3 and event['foot_energy_reason']==4
            assert abs(event['low_peak_bias_mj']-.08)<.00001 and event['low_peak_active'] is True
            assert event['low_peak_selected'] is True and event['low_peak_base_width_ms']==80
            assert abs(event['low_peak_base_prediction_mj']-.281)<.00001
            assert abs(event['low_peak_applied_bias_mj']-.08)<.00001
            for key in ('foot_phase_left_deg','foot_phase_right_deg','foot_phase_start_ms','foot_phase_end_ms',
                        'foot_phase_baseline_ms','foot_phase_requested_ms','foot_phase_state','foot_phase_reason'):
                assert event[key] is None
            assert event['foot_phase_valid'] is False and event['foot_phase_selected'] is False
            for key,value in (('foot_span_left_deg',2.12345),('foot_span_right_deg',.23456),
                              ('foot_span_trimmed_left_deg',2.01234),('foot_span_trimmed_right_deg',.12345)):
                assert abs(event[key]-value)<.00001
            assert event['foot_span_start_ms']==1000 and event['foot_span_end_ms']==1800
            assert event['foot_span_baseline_ms']==80 and event['foot_span_requested_ms']==75
            assert event['foot_span_state']==4 and event['foot_span_accepted_cycles']==2
            assert abs(event['target_energy_j']-.00076)<.000001
            assert work_metadata['walking_energy_loss']['independent_validation'] is False
            assert work_metadata['walking_energy_loss']['low_rpm_coast_observations']==26
            assert work_metadata['walking_energy_loss']['low_rpm_branch']==11
            assert work_metadata['walking_energy_loss']['low_rpm_pulse_gain_validated'] is False
            assert event['work_control_branch']==11
        else:
            assert event['arrival_energy_model_revision'] == work_metadata['wheel_work_model_revision']
        if event['work_control_branch'] == 4:
            assert event['work_bias_samples'] == 3
            assert event['work_brake_cap_ms'] == (30 if cross_condition else 25)
    with tempfile.TemporaryDirectory() as tmp:
        converter.convert(Path(str(source) + suffix), Path(tmp) / 'csv')
        with (Path(tmp) / 'csv' / 'energy_control_autonomous_zero_cross_events.csv').open() as stream:
            exported = list(csv.DictReader(stream))
        assert len(exported) == 256
        flags=('valid','selected','startup','left_low','right_low','left_both','right_both','continuous',
               'left_outlier','right_outlier','left_edge_unverified','right_edge_unverified')
        for index,(row,event) in enumerate(zip(exported,work_rows)):
            for j,flag in enumerate(flags):
                key='foot_span_'+flag
                assert event[key] is bool(index & (1 << (j % 8)))
                assert row[key] == str(event[key])
            if index==255:
                for key in ('foot_span_left_deg','foot_span_right_deg','foot_span_trimmed_left_deg','foot_span_trimmed_right_deg'):
                    assert abs(float(row[key])-event[key])<.00001
                for key in ('foot_span_start_ms','foot_span_end_ms','foot_span_baseline_ms','foot_span_requested_ms','foot_span_state','foot_span_accepted_cycles'):
                    assert row[key]==str(event[key])
            if index in (251,254):
                for key in ('valid','output_executed','command_current_mA','pulse_width_ms'):
                    assert row[key] == str(event[key])
            for key in ('work_control_branch', 'work_brake_cap_ms', 'work_bias_samples', 'cg_shift_mm', 'work_model_weight'):
                assert row[key] == str(event[key])
            assert row['work_prediction_cross_condition'] == str(event['work_prediction_cross_condition'])
print('full mixed work/fallback RWLOGs: CRC, schema, model/null alignment and conversion PASS')
