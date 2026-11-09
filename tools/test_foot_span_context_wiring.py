"""Check actual runner's camera context, 1200mA dispatch and logging."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'src/experiment_runner.cpp').read_text(encoding='utf8')
cycle=s.split('void ExperimentRunner::processFootSpanCycle()',1)[1].split('\nvoid ExperimentRunner::recordFootPhasePeak(',1)[0]
assert 'cycle.left=span.left.trimmed_span;cycle.right=span.right.trimmed_span' in cycle
assert 'positive_context=&span_positive_context_[i]' in cycle
for x in ('negative->trigger_deg','negative->rate_dps','negative->rpm','positive_context->rate_dps','positive_context->rpm','spanReliable(span.left.span,span.left.trimmed_span)','spanReliable(span.right.span,span.right.trimmed_span)','context_ok'):
    assert x in cycle
decision=s.split('void ExperimentRunner::finishFootEnergyDecision(',1)[1].split('\nbool ExperimentRunner::update',1)[0]
for x in ('d.trigger_deg=', 'd.rate_dps=event.zero_cross_abs_rate_dps',
          'context=span_positive_context_[span_positive_decision_next_]',
          'context.rate_dps=event.zero_cross_abs_rate_dps',
          'event.pre_input_wheel_speed_valid && event.pre_input_wheel_speed_age_us<=50000',
          'negative_preceding_positive_peak_ms_', 'span_positive_branch_'):
    assert x in decision
runner=s.split('bool ExperimentRunner::tryTiltPriority(',1)[1].split('\nbool ExperimentRunner::tryWeakSideAssist(',1)[0]
assert 'foot_span_control_.reset()' not in runner
assert 'long_foot_balance_.propose(' in runner and 'choice.width_ms<=100' in runner
assert 'tilt_priority_.target()>=6.f' in runner
assert 'tilt_priority_.timingCap(' in runner and 'Span_selected' in runner
assert 'long_foot_balance_.observeCycle(' in cycle and 'positive_branch==15' in cycle
assert 'plus_peak_ms>cycle.start_ms' in cycle and 'plus_peak_ms<negative->ms' in cycle
assert 'foot_trial_effective_[positive_index]' in cycle
assert 'foot_trial_generation_[positive_index]==long_foot_balance_.trialGeneration()' in cycle
assert 'span.start_ms>long_foot_balance_.lastChangeMs()' in cycle
assert 'negative_cap_recovery_.footCycle(' in cycle and 'neutral_foot_context' in cycle
assert 'negative_cap_recovery::normalFootReferenceAllowed(' in cycle and 'negative_recovery_last_applied_ms_' in cycle
assert 'foot_trial_pending_ms_==t_ms && event.output_executed' in decision
assert 'auto reference_guard=wheel_guard_before' in runner
assert 'long_foot_balance_.trialReferenceOffset()' in runner
assert 'negative_cap_recovery_.candidate(' in runner and 'long_foot_balance_.idleForNegativeTrial()' in runner
assert 'const bool negative_recovery_enabled=false' in runner
assert 'trial_in.negative_cap_extra_ms=5' in runner and 'auto candidate_guard=wheel_guard_before' in runner
assert 'delta>0 && delta<=5' in runner and 'negative_cap_recovery_.command(' in runner
assert 'foot_span_control_.cycle(' not in cycle and 'foot_span_control_.confirm(' not in decision
metadata=(root/'src/psram_logger.cpp').read_text(encoding='utf8')
assert 'three_adjacent_pairs_trial_loss_v5' in metadata
print('1200mA camera feedback, comparable context and log wiring PASS')
