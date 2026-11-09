"""Check actual source scope and the telemetry contract for timing experiment."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
runner=(root/'src/experiment_runner.cpp').read_text(encoding='utf-8')
policy=runner.split('void ExperimentRunner::updateEnergyControlAutonomousMotion(',1)[1].split('\nvoid ExperimentRunner::updateEnergyControlAutonomousPeakTracker(',1)[0]
assert 'threshold_deg=tilt_priority_.negativeEarlyThreshold' not in policy
rescue=runner.split('bool ExperimentRunner::observeInwardStallRescue(',1)[1].split('\nvoid ExperimentRunner::updateEnergyControlAutonomousMotion(',1)[0]
assert 'energy_control_autonomous_input_peak_percent_==0 &&' in rescue
assert '(peak_side==1 || peak_side==-1)' in rescue and 'SYMMETRIC_RESCUE_TRIAL' in rescue
assert 'peak_side==1 &&' in rescue
assert 'angle_deg*peak_side' in rescue and 'rate_dps*peak_side' in rescue
assert 'stall_rescue_detector_.observe' in rescue and 'observeInwardStallRescue(' in policy
assert 'APPROACH_RESCUE_TRIAL' in rescue and 'approach_stall_rescue_detector_.observe' in rescue
assert '25.f,2.5f,4.f' in rescue and 'stall_rescue_attempted_peak_ms_!=energy_control_autonomous_last_peak_ms_' in rescue
assert 'WAIT_ZERO_CROSS' in policy and '!energy_control_autonomous_zero_cross_consumed_for_peak_' in policy
assert '(crossing || confirmation_input || rescue)' in policy
assert 'stall_rescue_event_=rescue' in rescue and 'stall_rescue_event_=false' in rescue
assert '!physical_event_suppressed' in rescue and '!peak_just_confirmed' in rescue
assert 'energy_control_autonomous_target_peak_deg_==8.f' in rescue
assert 'before_deg > threshold_deg && detector_relative_angle_deg <= threshold_deg' in policy
assert 'rate_dps * peak_side < 0.0f' in policy
choice=runner.split('bool ExperimentRunner::tryTiltPriority(',1)[1].split('\nbool ExperimentRunner::tryWeakSideAssist(',1)[0]
# Rescue and early geometry trials both retain the 15 ms safety cap;
# an ordinary ZERO retains the existing hardware limit.
cap=' '.join(choice.split())
assert ('in.hardware_cap_ms=(stall_rescue_event_ || optional_early_event_) ? '
        'stall_rescue::kMaxPulseMs : Config::WALKING_TARGET8_MAX_PULSE_MS;') in cap
assert 'event.pre_input_wheel_speed_valid' in choice and 'imu_->acquisitionHealthy()' in choice
at_zero=runner.split('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross(',1)[1]
assert 'event.event_kind=stall_rescue_event_ ? 2 : (optional_early_event_ ? 3 : 0)' in at_zero
dispatch=runner.split('bool ExperimentRunner::dispatchTiltPriority(',1)[1].split('\n__attribute__((noinline)) void ExperimentRunner::finishSolverAudit(',1)[0]
assert 'if(stall_rescue_event_ || optional_early_event_)' in dispatch and 'finishFootEnergyDecision(event,t_test_ms)' in dispatch
cycle=runner.split('void ExperimentRunner::processFootSpanCycle()',1)[1].split('\nvoid ExperimentRunner::recordFootPhasePeak(',1)[0]
assert 'span.left.trimmed_span,span.right.trimmed_span,timing_quality' in cycle
assert '!span.left.single_outlier && !span.right.single_outlier' in cycle
assert '!span.left.edge_unverified && !span.right.edge_unverified' in cycle
logger=(root/'src/psram_logger.cpp').read_text(encoding='utf-8')
assert 'target8_zero_percent_uses_zero_centre_with_explicit_inward_stall_rescue' in logger
assert 'inward_stall_rescue' in logger
assert 'approach_inward_stall_rescue' in logger and 'event.event_kind=5' in at_zero
print('foot timing motion and telemetry wiring PASS')
