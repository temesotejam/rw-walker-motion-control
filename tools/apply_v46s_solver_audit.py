#!/usr/bin/env python3
from pathlib import Path
import re, json, hashlib
R=Path(__file__).resolve().parents[1]
def rd(p): return (R/p).read_text()
def wr(p,s): (R/p).parent.mkdir(parents=True,exist_ok=True);(R/p).write_text(s)
def once(s,a,b):
    assert s.count(a)==1,(a[:90],s.count(a))
    return s.replace(a,b,1)
assert hashlib.sha256((R/'src/experiment_runner.cpp').read_bytes()).hexdigest()=='151bdc4ee2725320f3e6160f5a3bfd5d3e9fc40f483fddaa7b1686520a201ddc'
assert hashlib.sha256((R/'src/config.h').read_bytes()).hexdigest()=='acbfcc2ff4ab1d6995051ab2a2383ac66d342cabd7144df7f0e9e7caa49594b8'
u32='index t_test_ms start_us end_us decision_us free_model_us solver_start_us solver_end_us solver_us fast_solver_us ff_search_us selected_search_us gyro_sequence sample_time_us entry_sample_age_us exit_sample_age_us pulse_id_before pulse_id_after'.split()
u16='model_vbat_mV ff_width_ms fast_selected_width_ms selected_width_ms ff_eval_count selected_eval_count eval_count'.split()
u8='stage outcome_reason state_at_exit sample_time_valid solver_started solver_complete output_executed event_valid ff_valid selected_valid'.split()
i8='physical_side command_direction'.split()
f32='i0_mA free_peak_deg target_peak_deg target_energy_j passive_energy_j q_available_mA_s integral_mA_s signed_target_current_mA tau_s base_gain correction_c correction_gain correction_limit ff_q_mA_s selected_q_mA_s corrected_target_energy_j'.split()
fields={**{x:'uint32_t' for x in u32},**{x:'uint16_t' for x in u16},**{x:'uint8_t' for x in u8},**{x:'int8_t' for x in i8},**{x:'float' for x in f32}}
header='''#pragma once
#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// One writer (run-control), stopped-only reader (RWLOG export).
// No allocation, formatting or model evaluation in push()/clear().
namespace solver_audit {
static_assert(sizeof(float) == 4, "Replay requires float32");
struct Record {
'''
for k,t in fields.items():
    default='NAN' if t=='float' else ('65535' if k in ('ff_width_ms','fast_selected_width_ms','selected_width_ms') else '0')
    header+=f'  {t} {k} = {default};\n'
header+='''};
inline uint32_t floatBits(float value) { uint32_t bits; memcpy(&bits, &value, 4); return bits; }
template<class F> class ScopeExit {
 public:
  explicit ScopeExit(F& fn) : fn_(fn) {}
  ~ScopeExit() { fn_(); }
  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;
 private:
  F& fn_;
};
template<unsigned Capacity> class Buffer {
 public:
  static_assert(Capacity > 0, "Nonzero capacity required");
  void clear() { count_ = next_ = total_ = 0; }
  void push(const Record& input) {
    records_[next_] = input;
    records_[next_].index = ++total_;
    next_ = (next_ + 1U) % Capacity;
    if (count_ < Capacity) ++count_;
  }
  uint32_t count() const { return count_; }
  uint32_t overwritten() const { return total_ - count_; }
  const Record& at(unsigned index) const {
    return records_[((count_ == Capacity ? next_ : 0U) + index) % Capacity];
  }
  void appendJson(String& json) const {
    json += "{\\"schema_version\\":1,\\"solver_revision\\":\\"v46r_fast_solver_control_20260915\\",";
    json += "\\"policy\\":\\"diagnostic_only;elapsed_wall_time_includes_preemption;no_legacy_online;stopped_export\\",";
    json += "\\"capacity\\":" + String(Capacity) + ",\\"count\\":" + String(count_);
    json += ",\\"overwritten\\":" + String(overwritten());
    json += ",\\"stage_legend\\":\\"0=before_solver,1=ff_failed,2=corrected_target_failed,3=selected_failed,4=selected_complete\\",";
    json += "\\"timing_scope\\":\\"decision=start_to_return_before_audit_copy;solver=cached_setup_through_selection;ff_and_selected=search_only;sample_age=consumed_host_sample_not_sensor_timestamp\\",";
'''
header+='    json += "\\"float_fields\\":'+json.dumps(f32,separators=(',',':')).replace('"','\\"')+',\\"events\\":[";\n'
header+='''    for (unsigned i = 0; i < count_; ++i) {
      const Record& e = at(i);
      if (i) json += ",";
      json += "{";
'''
for i,k in enumerate(u32+u16+u8+i8):
    header+=f'      json += "{"," if i else ""}\\"{k}\\":" + String(e.{k});\n'
header+='      json += ",\\"f32_bits\\":[";\n'
for i,k in enumerate(f32): header+=f'      json += {chr(34)+","+chr(34)+" + " if i else ""}String(floatBits(e.{k}));\n'
header+='''      json += "]}";
    }
    json += "]}";
  }
 private:
  Record records_[Capacity];
  uint32_t count_ = 0, next_ = 0, total_ = 0;
};
}  // namespace solver_audit
'''
wr('src/solver_audit.h',header)
runner=rd('src/experiment_runner.cpp')
oldrunner=runner
start=runner.index('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross')
end=runner.index('void ExperimentRunner::runEnergyControlAutonomousSolverShadow',start)
control=runner[start:end]
def add_after(a,b):
    global control
    control=once(control,a,a+'\n  // V46s audit begin\n'+b+'\n  // V46s audit end')
add_after('  PsramLogger::EnergyControlAutonomousZeroCrossEvent event;', '''  solver_audit::Record audit;
  audit.t_test_ms = t_test_ms;
  audit.start_us = v46l_decision_t0_us;
  audit.pulse_id_before = status_.pulse_id;
  audit.model_vbat_mV = status_.beta_model_vbat_mV;
  audit.gyro_sequence = imu_->reading().gyro_sequence;
  audit.sample_time_us = imu_->reading().last_gyro_update_us;
  audit.sample_time_valid = audit.gyro_sequence != 0;
  audit.entry_sample_age_us = audit.sample_time_valid ?
      static_cast<uint32_t>(micros() - audit.sample_time_us) : 0;
  auto finish_audit = [&]() {
    audit.end_us = micros();
    audit.decision_us = static_cast<uint32_t>(audit.end_us - audit.start_us);
    audit.exit_sample_age_us = audit.sample_time_valid ?
        static_cast<uint32_t>(audit.end_us - audit.sample_time_us) : 0;
    audit.pulse_id_after = status_.pulse_id;
    audit.state_at_exit = static_cast<uint8_t>(status_.state);
    audit.outcome_reason = event.reason;
    audit.event_valid = event.valid;
    audit.output_executed = event.output_executed;
    audit.physical_side = event.physical_next_peak_side;
    audit.command_direction = event.q_command_direction;
    audit.i0_mA = event.i0_estimated_mA;
    audit.free_peak_deg = event.free_next_peak_amplitude_deg;
    audit.target_peak_deg = event.target_peak_deg;
    audit.target_energy_j = event.target_energy_j;
    audit.passive_energy_j = event.passive_energy_j;
    audit.q_available_mA_s = event.q_available_mA_s;
    audit.integral_mA_s = event.integral_side_mA_s;
    audit.base_gain = event.g_side_base_deg_per_mA_s;
    audit.correction_c = event.c_side_used_deg;
    audit.correction_gain = event.g_side_corrected_deg_per_mA_s;
    audit.correction_limit = Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG;
    if (audit.solver_started) {
      audit.solver_us = static_cast<uint32_t>(audit.solver_end_us - audit.solver_start_us);
    }
    logger_->addSolverAuditEvent(audit);
  };
  solver_audit::ScopeExit<decltype(finish_audit)> audit_exit(finish_audit);''')
add_after('  const uint32_t v46l_free_model_us = static_cast<uint32_t>(micros() - v46l_free_model_t0_us);','  audit.free_model_us = v46l_free_model_us;')
add_after('  event.delta_energy_required_j = event.target_energy_j - event.passive_energy_j;','  audit.solver_started = 1;\n  audit.solver_start_us = micros();')
add_after('  const float fast_tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA);','  audit.signed_target_current_mA = fast_signed_target_current_mA;\n  audit.tau_s = fast_tau_s;')
add_after('  const FastCandidate ff = fast_pick_width(event.target_energy_j);', '''  audit.ff_search_us = static_cast<uint32_t>(micros() - v46r_fast_solver_t0_us);
  audit.ff_eval_count = fast_eval_count;
  audit.eval_count = fast_eval_count;
  audit.ff_valid = ff.valid;
  audit.ff_width_ms = ff.valid ? ff.width_ms : 65535;
  audit.ff_q_mA_s = ff.q_mA_s;''')
add_after('  if (!ff.valid) {','    audit.stage = 1;\n    audit.solver_end_us = micros();')
add_after('  const float corrected_target_energy_j = energyControlPotentialJ(corrected_target_prediction_deg);','  audit.corrected_target_energy_j = corrected_target_energy_j;')
add_after('  if (!isfinite(corrected_target_energy_j)) {','    audit.stage = 2;\n    audit.solver_end_us = micros();')
control=once(control,'  const FastCandidate selected_fast = fast_pick_width(corrected_target_energy_j);','  // V46s audit begin\n  const uint32_t selected_search_t0_us = micros();\n  // V46s audit end\n  const FastCandidate selected_fast = fast_pick_width(corrected_target_energy_j);')
add_after('  const FastCandidate selected_fast = fast_pick_width(corrected_target_energy_j);', '''  audit.selected_search_us = static_cast<uint32_t>(micros() - selected_search_t0_us);
  audit.selected_eval_count = fast_eval_count - audit.ff_eval_count;
  audit.eval_count = fast_eval_count;
  audit.selected_valid = selected_fast.valid;
  audit.fast_selected_width_ms = selected_fast.valid ? selected_fast.width_ms : 65535;''')
add_after('  if (!selected_fast.valid) {','    audit.stage = 3;\n    audit.solver_end_us = micros();')
add_after('  (void)fast_eval_count;', '''  audit.fast_solver_us = v46r_fast_solver_us;
  audit.stage = 4;
  audit.solver_complete = 1;
  audit.solver_end_us = micros();
  audit.selected_width_ms = selected_width_ms;
  audit.selected_q_mA_s = selected_q_mA_s;''')
assert re.sub(r'\n  // V46s audit begin\n.*?\n  // V46s audit end','',control,flags=re.S)==oldrunner[start:end]
runner=runner[:start]+control+runner[end:]
wr('src/experiment_runner.cpp',runner)
h=rd('src/psram_logger.h')
h=once(h,'#include "log_types.h"','#include "log_types.h"\n#include "solver_audit.h"')
h=once(h,'  void addSolverShadowEvent(const SolverShadowEvent& event);','  void addSolverShadowEvent(const SolverShadowEvent& event);\n  void addSolverAuditEvent(const solver_audit::Record& event) { solver_audit_.push(event); }')
h=once(h,'  bool solver_shadow_event_overflow_ = false;','  bool solver_shadow_event_overflow_ = false;\n  solver_audit::Buffer<128> solver_audit_;')
wr('src/psram_logger.h',h)
s=rd('src/psram_logger.cpp')
assert s.count('  solver_shadow_event_overflow_ = false;')==2
s=s.replace('  solver_shadow_event_overflow_ = false;','  solver_shadow_event_overflow_ = false;\n  solver_audit_.clear();')
a='  json += "\\\"v46l_solver_shadow_revision\\\":\\\"v46l_discrete_ternary_shadow_20260914\\\",";'
s=once(s,a,'  json += "\\\"v46s_solver_audit\\\":";\n  solver_audit_.appendJson(json);\n  json += ",";\n'+a)
s=s.replace('legacy_exhaustive_solver_controls_motor;fast_solver_runs_only_after_normal_pulse_end;metadata_only','disabled_since_v46r;legacy_comparison_offline_only;use_v46s_solver_audit')
wr('src/psram_logger.cpp',s)
s=rd('src/config.h').replace('v46r_fast_solver_control_20260915','v46s_solver_audit_20260915');wr('src/config.h',s)
for p in ['site/index.html','site/manifest.json','tools/test_v46r_fast_solver_control.py','tools/test_v46g_highrate_source_guards.py','tools/test_v46i_task_split_source_guards.py']:
 s=rd(p).replace('v46r_fast_solver_control_20260915','v46s_solver_audit_20260915').replace('0.46.17','0.46.18').replace('V46r','V46s')
 wr(p,s)
s=rd('site/index.html')
anchor='    <section class="panel important">'
insert='''    <section class="panel important">
      <h2>V46s / 0.46.18：高速ソルバの計測とオフライン検証</h2>
      <p>高速ソルバの判断・出力制限・IMU停止条件はV46rと同じです。追加したのは記録です。
      処理時間、評価回数、入力サンプルの年齢、無入力・計算失敗を含む結果を
      <code>v46s_solver_audit</code> に保存します。旧ソルバは実機上で実行しません。</p>
      <p>再計算用の入力はfloat32のビット列で保存します。最後の128件を保持し、上書き件数を明示します。
      Run終了またはESTOP後のRWLOGを保存してください。新しい実機での遅延改善量は未確認です。</p>
      <p>解析方法：<a href="https://github.com/temesotejam/atoms3r-mekf-dynamic-validation/blob/main/docs/V46S_SOLVER_AUDIT.md">V46s検証手順</a></p>
    </section>

'''
s=s.replace(anchor,insert+anchor,1);wr('site/index.html',s)
protected=['src/main.cpp','src/imu_manager.cpp','src/imu_manager.h','src/roller485_manager.cpp','src/roller485_manager.h','src/run_control_worker.h','src/log_types.h','src/mekf6.cpp','src/mekf6.hpp']
manifest={'baseline':'09e6a0ea15cca6b25eba8209ed375bf392229beb','runner_sha256':hashlib.sha256(oldrunner.encode()).hexdigest(),'config_sha256':hashlib.sha256(rd('src/config.h').replace('v46s_solver_audit_20260915','v46r_fast_solver_control_20260915').encode()).hexdigest(),'protected':{p:hashlib.sha256((R/p).read_bytes()).hexdigest() for p in protected}}
wr('tools/v46s_audit_baseline.json',json.dumps(manifest,indent=2)+'\n')
print('Created V46s audit sources; control path unchanged outside marked diagnostics')
