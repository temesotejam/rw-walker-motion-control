from pathlib import Path

p = Path("tools/test_v46_motor_validation_source_guards.py")
text = p.read_text(encoding="utf-8")

replacements = {
    '    assert "R_y(pi)=diag(-1,+1,-1)" in runner\n': '',
    '    assert "mekf6::degToRad(-r.gx_dps), mekf6::degToRad(r.gy_dps), mekf6::degToRad(-r.gz_dps)" in runner\n':
        '    assert "r.gy_dps * Config::MEKF_GYRO_Y_SCALE" in runner\n',
    '    assert "mekf6::degToRad(-bx_dps), mekf6::degToRad(by_dps), mekf6::degToRad(-bz_dps)" in runner\n':
        '    assert "by_dps * Config::MEKF_GYRO_Y_SCALE" in runner\n',
}

for old, new in replacements.items():
    if text.count(old) != 1:
        raise RuntimeError(f"expected one stale guard, got {text.count(old)}: {old!r}")
    text = text.replace(old, new, 1)

p.write_text(text, encoding="utf-8")
print("V46f guard updated for Ry180 plus calibrated gyro-Y implementation")
