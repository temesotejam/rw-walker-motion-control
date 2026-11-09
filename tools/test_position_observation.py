"""Guard position observation and the pre-probe controller math."""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
baseline='79de6323dbd08ec1ec8920d6df11446bbaf2d7d6'
allowed={
    'src/config.h':[
        'static constexpr uint32_t WHEEL_POSITION_READ_PERIOD_US = 5000UL;\n',
        'static constexpr uint16_t WHEEL_POSITION_READ_TIMEOUT_MS = 2;\n',
        # Authorized 1.0.27 work-control preset only; legacy duration stays frozen.
        'static constexpr uint32_t WHEEL_WORK_CONTROL_DURATION_MS = 30000UL;\n',
        # Diagnostic-only reasons added by the authorized work-controller
        # extension. Keep every pre-existing config constant frozen below.
        'static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_WORK_OUTSIDE_SUPPORT = 15;\n',
        'static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_WORK_PROTECT_OPPOSITE = 16;\n',
        # Explicit 8-degree walking trial constants; the legacy 100ms limit
        # and every other pre-existing config constant must still match.
        '// User-selected 8-degree walking experiment limits. Other autonomous modes\n'
        '// continue using their own previously configured limits.\n'
        'static constexpr uint16_t WALKING_TARGET8_MAX_PULSE_MS = 150;\n'
        'static constexpr int16_t WALKING_TARGET8_MAX_CURRENT_MA = 1200;\n'],
}
unchanged=['src/lateral_energy_control.h','src/peak_error_adaptation.h',
           'src/previous_peak_control_correction.h','src/control_math_cache.h',
           'src/mekf6.cpp','src/mekf6.hpp',
           'src/upright_reference_store.cpp','src/upright_reference.h']
def normalize_capped_lateral_energy(source):
    # Permit only the explicit optional positive-excitation upper bound. The
    # default stays kMaxMs and reverse/brake limits remain separately guarded.
    replacements=[
        ('uint16_t brake_max_ms=kBrakeMaxMs,float prediction_bias_j=0.f,\n'
         '      uint16_t excitation_max_ms=kMaxMs)',
         'uint16_t brake_max_ms=kBrakeMaxMs,float prediction_bias_j=0.f)'),
        ('brake_max_ms>kBrakeMaxMs || excitation_max_ms>kMaxMs)',
         'brake_max_ms>kBrakeMaxMs)'),
        ('const uint16_t max_ms=braking?brake_max_ms:excitation_max_ms;',
         'const uint16_t max_ms=braking?brake_max_ms:kMaxMs;'),
        ('float initial_current_mA,float goal_current_mA,float prediction_bias_j=0.f,\n'
         '      uint16_t excitation_max_ms=kMaxMs)',
         'float initial_current_mA,float goal_current_mA,float prediction_bias_j=0.f)'),
        ('!std::isfinite(goal_current_mA) || goal_current_mA<=0 || excitation_max_ms>kMaxMs) return d;',
         '!std::isfinite(goal_current_mA) || goal_current_mA<=0) return d;'),
        ('const float limit_q=charge(table,excitation_max_ms,initial_current_mA,goal_current_mA);',
         'const float limit_q=charge(table,kMaxMs,initial_current_mA,goal_current_mA);'),
        ('uint16_t lo=0,hi=excitation_max_ms;', 'uint16_t lo=0,hi=kMaxMs;'),
        ('for (uint16_t n : {lo,hi,excitation_max_ms}) {',
         'for (uint16_t n : {lo,hi,kMaxMs}) {'),
    ]
    for current,legacy in replacements:
        assert source.count(current)==1,('expected capped solver fragment',current)
        source=source.replace(current,legacy,1)
    return source

for name in list(allowed)+unchanged:
    before=subprocess.check_output(['git','show',baseline+':'+name],cwd=root).decode().replace('\r\n','\n')
    after=(root/name).read_text(encoding='utf-8')
    for line in allowed.get(name,[]):
        assert after.count(line)==1,(name,line)
        after=after.replace(line,'')
    if name=='src/lateral_energy_control.h':
        after=normalize_capped_lateral_energy(after)
    assert before==after,'unauthorized controller change: '+name
runner_before=subprocess.check_output(['git','show',baseline+':src/experiment_runner.cpp'],cwd=root).decode().replace('\r\n','\n')
runner_after=(root/'src/experiment_runner.cpp').read_text(encoding='utf-8')
def legacy_decision(source):
    return source.split('  auto decide_selected=[&]',1)[1].split('    return energy_bridge_enabled_',1)[1].split('  };',1)[0]
# The experiment chooses a table pointer only for randomized positive-current
# trials. All other modes keep the original 300 mA table and decision math.
assert 'const lateral_energy::CurrentTable* selected_current_table = &lateral_current_table_;' in runner_after
assert 'if (positiveCurrentBatchMode())\n    selectPositiveCurrentBatch(event, selected_current_mA, selected_current_table);' in runner_after
assert 'if (!positiveCurrentBatchMode() || event.physical_next_peak_side <= 0) return;' in runner_after
assert ('const uint16_t fallback_positive_cap=walking_positive_cap::fallback(\n'
        '      work_mode,work_choice.valid,event.physical_next_peak_side,event.target_peak_deg,\n'
        '      gradual_start_.positiveCapMs(),wheel_speed_balance_.capMs(),lateral_energy::kMaxMs);') in runner_after
legacy_after=legacy_decision(runner_after).replace('*selected_current_table','lateral_current_table_')
for capped,legacy in [
    ('event.integral_energy_j,aligned_initial_mA,goal_current_mA,bias,fallback_positive_cap)',
     'event.integral_energy_j,aligned_initial_mA,goal_current_mA,bias)'),
    ('bias,fallback_positive_cap);', 'bias);'),
]:
    assert legacy_after.count(capped)==1,('expected capped fallback argument',capped)
    legacy_after=legacy_after.replace(capped,legacy,1)
assert legacy_decision(runner_before)==legacy_after, \
    'legacy bridge/quadratic decision changed during inverse-mode addition'
assert '      observation.position_sequence != last_logged_position_sequence_ ||\n' in runner_after
assert '  last_logged_position_sequence_ = roller_telemetry.position_sequence;\n' in runner_after
assert '  uint32_t last_logged_position_sequence_ = 0;\n' in \
    (root/'src/experiment_runner.h').read_text(encoding='utf-8')
source=(root/'src/roller485_manager.cpp').read_text()
position=source.split('bool Roller485Manager::readPositionFresh() {',1)[1].split('bool Roller485Manager::readSpeedFresh()',1)[0]
assert 'readI32(REG_POSITION_READBACK, raw)' in position
assert 'recordIo(' not in position and 'setCurrentMa(' not in position and 'stop()' not in position
update=source.split('void Roller485Manager::update() {',1)[1].split('bool Roller485Manager::setCurrentMa',1)[0]
assert update.index('requested_current_mA_ != command_mA_')<update.index('readPositionFresh()')
assert 'if (requested_current_mA_ != command_mA_) return;\n  if (last_speed_due_us_' in update
print('Legacy decision math, MEKF and position observation guarded; inverse-mode addition allowed PASS')
