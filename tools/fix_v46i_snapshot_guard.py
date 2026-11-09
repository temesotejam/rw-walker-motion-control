from pathlib import Path

p = Path('src/experiment_runner.cpp')
text = p.read_text(encoding='utf-8')
old = '  if (roller_) status_.roller_actual_current_mA = roller_->telemetry().actual_current_mA;\n'
new = '  if (roller_) status_.roller_actual_current_mA = roller_->telemetrySnapshot().actual_current_mA;\n'
if text.count(old) != 1:
    raise RuntimeError(f'expected one stopMotor telemetry reference, got {text.count(old)}')
p.write_text(text.replace(old, new, 1), encoding='utf-8')
print('Remaining V46i Roller telemetry reference replaced with snapshot')
