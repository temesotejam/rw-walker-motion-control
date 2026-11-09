"""Exercise the shared gate in C++; guard both actual HTTP integration points here."""
from pathlib import Path
s=Path('src/web_ui.cpp').read_text(encoding='utf-8')
status=s.split('void WebUi::status()',1)[1].split('void WebUi::',1)[0]
ready=next(line for line in status.splitlines() if '\\"ready\\"' in line)
assert 'work_start_policy::ready(' in ready
assert 'upright_reference' not in ready
command=s.split('void WebUi::command(',1)[1].split('void WebUi::status()',1)[0]
assert 'work_start_policy::referenceAllowed(' in command
assert 'server_->arg("energy_model")=="work_control"' in command
assert command.count('"upright_reference_required"')==1
assert 'experiment_package_settings_invalid_reload_page' in command
assert 'work_target_policy::packageAllowed(target_deg,brake_max_ms,input_current_mA,peak_adaptation_enabled)' in command
runner=Path('src/experiment_runner.cpp').read_text(encoding='utf-8')
assert 'work_target_policy::packageAllowed(energy_control_autonomous_target_peak_deg_' in runner
assert 'state.target_deg=event.target_peak_deg;' in runner
assert 'd.command_sign=choice.command_sign;' in runner
assert 'chooseNegativeBidirectional(state,cap,' in runner
assert 'if(state.target_deg==12.f)' in runner
assert 'work_target12_controller_.choose(' in runner
assert 'work_target12_controller_.observe(' in runner
assert 'work_pending_calibration_weight_>=.9f' in runner
assert 'work_choice.branch==9' in runner
assert 'positiveBiasMj(run_cg_shift_mm_' in runner
assert 'positive_bias_mj,bridge_bias_mj,coast_only,' in runner
assert runner.count('work_target_policy::negativeFeedbackIndex(')==3
worker=Path('src/run_control_worker.h').read_text(encoding='utf-8')
assert 'work_target_policy::packageAllowed(target_deg, brake_max_ms, input_current_mA,' in worker
print('actual status readiness and HTTP start route have no unconditional reference gate PASS')
