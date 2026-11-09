#!/usr/bin/env bash
set -euo pipefail
python3 tools/embed_runtime_web.py --check
python3 tools/test_foot_log_version_mapping.py
python3 tools/test_positive_sway_log.py
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc test/test_positive_sway_probe.cpp -o /tmp/test_positive_sway_probe
/tmp/test_positive_sway_probe
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc test/test_tapered_amplitude_recovery.cpp -o /tmp/test_tapered_amplitude_recovery
/tmp/test_tapered_amplitude_recovery

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_weak_side_assist.cpp -o /tmp/test_weak_side_assist
/tmp/test_weak_side_assist

python3 tools/test_weak_assist_integration.py
python3 tools/test_side_target_integration.py
python3 tools/test_positive_boost_integration.py
python3 tools/test_walking_pulse_limits.py
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_tilt_priority.cpp -o /tmp/test_tilt_priority
/tmp/test_tilt_priority
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc test/test_negative_weak_probe.cpp -o /tmp/test_negative_weak_probe
/tmp/test_negative_weak_probe
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc test/test_long_foot_balance.cpp -o /tmp/test_long_foot_balance
/tmp/test_long_foot_balance
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc test/test_negative_cap_recovery.cpp -o /tmp/test_negative_cap_recovery
/tmp/test_negative_cap_recovery
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_wheel_cycle_guard.cpp -o /tmp/test_wheel_cycle_guard
/tmp/test_wheel_cycle_guard
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_timing.cpp -o /tmp/test_foot_timing
/tmp/test_foot_timing
python3 tools/test_foot_timing_wiring.py
python3 tools/test_tilt_priority_integration.py
python3 tools/test_early_runner_integration.py
python3 tools/test_angle_runner_integration.py
python3 tools/test_angle_build_wiring.py
python3 tools/test_early_reply_integration.py
python3 tools/test_early_frame_invalidation.py
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_early_motion_gate.cpp -o /tmp/test_early_motion_gate
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_early_clock.cpp -o /tmp/test_early_clock
/tmp/test_early_clock
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_early_diagnostics.cpp -o /tmp/test_early_diagnostics
/tmp/test_early_diagnostics
/tmp/test_early_motion_gate
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_positive_left_boost.cpp -o /tmp/test_positive_left_boost
/tmp/test_positive_left_boost

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_fullstride_observer.cpp -o /tmp/test_foot_fullstride_observer
/tmp/test_foot_fullstride_observer
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_span_candidate.cpp -o /tmp/test_foot_span_candidate
/tmp/test_foot_span_candidate
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_span_control.cpp -o /tmp/test_foot_span_control
/tmp/test_foot_span_control
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_span_queued.cpp -o /tmp/test_foot_span_queued
/tmp/test_foot_span_queued
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_span_retry.cpp -o /tmp/test_foot_span_retry
/tmp/test_foot_span_retry
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_trend.cpp -o /tmp/test_foot_trend
/tmp/test_foot_trend
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_gap_trend.cpp -o /tmp/test_foot_gap_trend
/tmp/test_foot_gap_trend
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_gap_integration.cpp -o /tmp/test_foot_gap_integration
/tmp/test_foot_gap_integration
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_delayed_invalid.cpp -o /tmp/test_foot_delayed_invalid
/tmp/test_foot_delayed_invalid
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_recent_gate.cpp -o /tmp/test_foot_recent_gate
/tmp/test_foot_recent_gate
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_stall_rescue.cpp -o /tmp/test_stall_rescue
/tmp/test_stall_rescue
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_trend_integration.cpp -o /tmp/test_foot_trend_integration
/tmp/test_foot_trend_integration
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_split.cpp -o /tmp/test_foot_split
/tmp/test_foot_split
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_once.cpp -o /tmp/test_foot_once
/tmp/test_foot_once
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_once_edges.cpp -o /tmp/test_foot_once_edges
/tmp/test_foot_once_edges
python3 tools/test_foot_span_context_wiring.py
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_feedback_stress.cpp -o /tmp/test_foot_feedback_stress
/tmp/test_foot_feedback_stress

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_stroke_observer.cpp -o /tmp/test_foot_stroke_observer
/tmp/test_foot_stroke_observer

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_phase_observer.cpp -o /tmp/test_foot_phase_observer
/tmp/test_foot_phase_observer
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_phase_cap.cpp -o /tmp/test_foot_phase_cap
/tmp/test_foot_phase_cap
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_phase_run_replay.cpp -o /tmp/test_foot_phase_run_replay
python3 tools/test_replay_phase_range.py /tmp/test_foot_phase_run_replay
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_phase_cap_replay.cpp -o /tmp/test_phase_cap_replay

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_peak_mpc.cpp -o /tmp/test_peak_mpc
/tmp/test_peak_mpc
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_gradual_start.cpp -o /tmp/test_gradual_start
/tmp/test_gradual_start

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_low_peak_energy.cpp -o /tmp/test_low_peak_energy
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_low_peak_log_replay.cpp -o /tmp/test_low_peak_log_replay
python3 tools/test_low_peak_evidence.py /tmp/test_low_peak_log_replay
/tmp/test_low_peak_energy

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_wheel_speed_balance.cpp -o /tmp/test_wheel_speed_balance
python3 tools/test_wheel_speed_evidence.py /tmp/test_wheel_speed_balance
/tmp/test_wheel_speed_balance

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_balance_adaptive.cpp -o /tmp/test_foot_balance_adaptive
/tmp/test_foot_balance_adaptive
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_energy_balance.cpp -o /tmp/test_foot_energy_balance
/tmp/test_foot_energy_balance

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_positive_fallback_cap.cpp -o /tmp/test_positive_fallback_cap
/tmp/test_positive_fallback_cap

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_unmodeled_walk_coast.cpp -o /tmp/test_unmodeled_walk_coast
/tmp/test_unmodeled_walk_coast

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_walking_energy_loss.cpp -o /tmp/test_walking_energy_loss
/tmp/test_walking_energy_loss
python3 tools/test_walking_loss_evidence.py /tmp/test_walking_energy_loss

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_target12_state_work.cpp -o /tmp/test_target12_state_work
/tmp/test_target12_state_work
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_negative_shared_energy.cpp -o /tmp/test_negative_shared_energy
/tmp/test_negative_shared_energy
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_target12_guard.cpp -o /tmp/test_target12_guard
/tmp/test_target12_guard
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_negative_assist.cpp -o /tmp/test_negative_assist
/tmp/test_negative_assist
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_work_targets.cpp -o /tmp/test_work_targets
/tmp/test_work_targets
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_side_energy_feedback.cpp -o /tmp/test_side_energy_feedback
/tmp/test_side_energy_feedback
python3 tools/test_work_start_wiring.py
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_absolute_reference.cpp -o /tmp/test_absolute_reference
/tmp/test_absolute_reference
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_boundary_energy.cpp -o /tmp/test_boundary_energy
/tmp/test_boundary_energy
python3 tools/fit_adaptive_lateral_cg.py
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_adaptive_lateral_cg.cpp -o /tmp/test_adaptive_lateral_cg
/tmp/test_adaptive_lateral_cg
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o -Isrc tools/test_lateral_cg.cpp src/lateral_cg_store.cpp -o /tmp/test_lateral_cg
/tmp/test_lateral_cg
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tools/test_negative_probe_batch.cpp -o /tmp/test_negative_probe_batch
/tmp/test_negative_probe_batch
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tools/test_positive_current_batch.cpp -o /tmp/test_positive_current_batch
/tmp/test_positive_current_batch
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_wheel_work_energy_control.cpp -o /tmp/test_wheel_work_energy_control
/tmp/test_wheel_work_energy_control

g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_balanced_energy.cpp -o /tmp/test_balanced_energy
/tmp/test_balanced_energy
g++ -std=c++11 -O2 -Wall -Wextra -Werror tools/test_one_peak_energy_inverse.cpp -o /tmp/test_one_peak_energy_inverse
/tmp/test_one_peak_energy_inverse
node --check web/runtime.js
node --check site/wheel-work.js
node tools/test_wheel_work.js
node --check site/one-peak-energy.js
node tools/test_one_peak_energy.js
node tools/test_position_work.js
python3 tools/test_position_observation.py
node --check web/pose_comparison.js
node tools/test_pose_comparison.js
node --check site/serial-monitor.js
node --check site/flasher-completion.js
node tools/test_flasher_completion.js
node tools/test_serial_monitor.js
node tools/test_runtime_web.js
node tools/test_offline_web.js
node tools/test_upright_reference_web.js
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o -Isrc tools/test_upright_reference.cpp src/upright_reference_store.cpp -o /tmp/test_upright_reference
/tmp/test_upright_reference
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o -Isrc tools/test_core_lifecycle.cpp -o /tmp/test_core_lifecycle
/tmp/test_core_lifecycle
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tools/test_offline_run.cpp -o /tmp/test_offline_run
/tmp/test_offline_run
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tools/test_tilt_stop.cpp src/mekf6.cpp -o /tmp/test_tilt_stop
/tmp/test_tilt_stop
g++ -std=c++17 -O2 -Wall -Wextra -Werror tools/test_stack_scan_policy.cpp -o /tmp/test_stack_scan_policy
/tmp/test_stack_scan_policy
python3 tools/test_realtime_code_link.py
python3 tools/test_motion_acquisition_link.py
python3 tools/fit_lateral_energy_seed.py > /tmp/lateral_energy_seed_fit.json
python3 tools/test_energy_validation.py
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tools/test_lateral_energy.cpp -o /tmp/test_lateral_energy
/tmp/test_lateral_energy
python3 tools/test_rwlog_v46_converter.py
for name in runtime_control export_protocol previous_peak_math diagnostic_journal; do
  g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o -Isrc tools/test_${name}.cpp src/control_work_profile.cpp src/control_latency.cpp -o /tmp/test_${name}
  /tmp/test_${name}
done
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o -Isrc tools/test_camera_task_patch.cpp src/camera_task_priority_patch.cpp -o /tmp/test_camera_task_patch
/tmp/test_camera_task_patch
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o tools/test_foot_tracking.cpp src/white_marker_tracker.cpp src/foot_angle_estimator.cpp -o /tmp/test_foot
/tmp/test_foot
g++ -std=c++11 -O2 tools/test_v46n_acquisition.cpp -o /tmp/test_acq
/tmp/test_acq
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o tools/test_v46o_startup.cpp src/control_latency.cpp -o /tmp/test_startup
/tmp/test_startup | tail -n 1
g++ -std=c++17 -O2 tools/test_mekf_host.cpp src/mekf6.cpp -o /tmp/test_mekf
/tmp/test_mekf
g++ -std=c++17 -O2 -Wall -Wextra -Werror tools/test_mekf_sparse_equivalence.cpp src/mekf6.cpp tools/fixtures/mekf6_dense_reference_0478.cpp -o /tmp/test_mekf_sparse
/tmp/test_mekf_sparse
g++ -std=c++17 -Os -Wall -Wextra -Werror tools/test_mekf_sparse_equivalence.cpp src/mekf6.cpp tools/fixtures/mekf6_dense_reference_0478.cpp -o /tmp/test_mekf_speed_policy
/tmp/test_mekf_speed_policy
python3 tools/verify_madgwick_dependency.py
python3 tools/test_madgwick_hot.py
g++ -std=c++17 -Os -Wall -Wextra -Werror tools/test_madgwick_pitch.cpp tools/fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_Madgwick.cpp -o /tmp/test_madgwick_pitch
/tmp/test_madgwick_pitch
g++ -std=c++17 -Os -Wall -Wextra -Werror tools/test_log_quantization.cpp src/log_quantization.cpp -o /tmp/test_log_quantization
/tmp/test_log_quantization
g++ -std=c++17 -Os -Wall -Wextra -Werror -Itools/host_v46o tools/test_log_sample_encoder.cpp src/log_sample_encoder.cpp src/log_quantization.cpp tools/fixtures/log_sample_reference_04712.cpp -o /tmp/test_log_sample_encoder
/tmp/test_log_sample_encoder
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o tools/test_mekf_diagnostics.cpp src/mekf6.cpp src/mekf_attitude_diagnostics.cpp -o /tmp/test_mekf_diagnostics
/tmp/test_mekf_diagnostics /tmp/mekf-diagnostics-fixture.json
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o tools/test_control_work_profile.cpp src/control_work_profile.cpp src/control_latency.cpp -o /tmp/test_control_work
/tmp/test_control_work /tmp/control-work-fixture.json
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Wno-format -ffunction-sections -fdata-sections -Itools/host_v46o tools/test_runtime_logger.cpp src/psram_logger.cpp src/foot_observer.cpp src/foot_angle_estimator.cpp src/white_marker_tracker.cpp src/immutable_export.cpp src/control_work_profile.cpp src/control_latency.cpp -Wl,--gc-sections -o /tmp/test_logger
/tmp/test_logger /tmp/runtime-fixture.rwlog
node tools/test_wheel_work.js /tmp/runtime-fixture.rwlog
node tools/test_position_work.js /tmp/runtime-fixture.rwlog
node tools/test_runtime_download.js
python3 tools/test_runtime_fixture.py

g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o -Isrc tools/test_control_latency.cpp src/control_latency.cpp -o /tmp/test_control_latency
/tmp/test_control_latency
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o -Isrc tools/test_imu_i2c_transport.cpp src/imu_i2c_transport.cpp src/control_latency.cpp -o /tmp/test_imu_i2c_transport
/tmp/test_imu_i2c_transport
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tools/test_v46u_reader.cpp -o /tmp/test_selective_reader
/tmp/test_selective_reader
python3 tools/test_deferred_comparison.py

python3 tools/test_bmm150_csv.py

g++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tools/test_gyro_steering.cpp -o /tmp/test_gyro_steering
/tmp/test_gyro_steering
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Isrc tools/test_gyro_heading_diagnostic.cpp src/gyro_heading_diagnostic.cpp -o /tmp/test_gyro_heading_diagnostic
/tmp/test_gyro_heading_diagnostic
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itools/host_v46o -Isrc tools/test_compact_json_table.cpp -o /tmp/test_compact_json_table
/tmp/test_compact_json_table

python3 tools/test_foot_observation_timestamp.py
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Wno-format -ffunction-sections -fdata-sections -Itools/host_v46o tools/test_observation_sampling.cpp src/foot_observer.cpp src/foot_angle_estimator.cpp src/white_marker_tracker.cpp -Wl,--gc-sections -o /tmp/test_observation_sampling
/tmp/test_observation_sampling | python3 tools/check_observation_sampling.py
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Isrc tools/test_foot_geometry_worker.cpp -o /tmp/test_geometry_default
/tmp/test_geometry_default
g++ -std=c++11 -O2 -Wall -Wextra -Werror -DFOOT_GEOMETRY_PROVISIONAL_EXPOSURE=1 -Isrc tools/test_foot_geometry_worker.cpp -o /tmp/test_geometry_trial
/tmp/test_geometry_trial
g++ -std=c++11 -O2 -Wall -Wextra -Werror -Itools/host_early -Isrc tools/test_async_init.cpp -o /tmp/test_geometry_init
/tmp/test_geometry_init
