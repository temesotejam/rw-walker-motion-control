from pathlib import Path


def replace_one(path: str, old: str, new: str) -> None:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old!r}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


replace_one(
    "src/config.h",
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46c_mekf_rx180_upright_reinit_dynamic_beta_compare_20260913";\n',
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46d_mekf_video_calibrated_rx180_upright_reinit_20260913";\n'
    'static constexpr float MEKF_VIDEO_OUTPUT_SIGN = -1.0f;\n'
    'static constexpr float MEKF_VIDEO_OUTPUT_SCALE = 0.908911f;\n',
)

p = Path("src/experiment_runner.cpp")
text = p.read_text(encoding="utf-8")
old = "  status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_;\n"
new = (
    "  status_.pitch_mekf_abs_deg = Config::MEKF_VIDEO_OUTPUT_SIGN *\n"
    "      Config::MEKF_VIDEO_OUTPUT_SCALE * raw_mekf_pitch_abs_deg_;\n"
)
if text.count(old) != 2:
    raise RuntimeError(f"experiment_runner.cpp: expected two MEKF comparison assignments, got {text.count(old)}")
p.write_text(text.replace(old, new), encoding="utf-8")

replace_one(
    "src/experiment_runner.h",
    "  float pitch_mekf_abs_deg = 0.0f;          // continuous gravity-frame MEKF pitch coordinate\n",
    "  float pitch_mekf_abs_deg = 0.0f;          // video-calibrated MEKF comparison coordinate; control is unchanged\n",
)
replace_one(
    "src/main.cpp",
    "V46c MEKF motor-driven dynamic validation",
    "V46d MEKF motor-driven dynamic validation",
)
replace_one("src/main.cpp", "V46c identity:", "V46d identity:")
replace_one(
    "src/main.cpp",
    'displayLine("V46c MEKF", "V7 MOTOR VALIDATION");',
    'displayLine("V46d MEKF", "V7 MOTOR VALIDATION");',
)

p = Path("tools/test_v46_motor_validation_source_guards.py")
text = p.read_text(encoding="utf-8")
text = text.replace(
    '"v46c_mekf_rx180_upright_reinit_dynamic_beta_compare_20260913" in config',
    '"v46d_mekf_video_calibrated_rx180_upright_reinit_20260913" in config',
)
text = text.replace(
    '"V46c MEKF motor-driven dynamic validation" in main_cpp',
    '"V46d MEKF motor-driven dynamic validation" in main_cpp',
)
anchor = '    assert "pitch_madgwick_dynamic_abs_deg" in runner\n'
if anchor not in text:
    raise RuntimeError("source-guard anchor missing")
text = text.replace(
    anchor,
    anchor
    + '    assert "MEKF_VIDEO_OUTPUT_SCALE = 0.908911f" in config\n'
    + '    assert "MEKF_VIDEO_OUTPUT_SIGN = -1.0f" in config\n'
    + '    assert runner.count("Config::MEKF_VIDEO_OUTPUT_SCALE * raw_mekf_pitch_abs_deg_") == 2\n'
    + '    assert "status_.pitch_mekf_deg = raw_mekf_pitch_abs_deg_ - offset_mekf_pitch_deg_" in runner\n',
    1,
)
p.write_text(text, encoding="utf-8")

p = Path("site/index.html")
text = p.read_text(encoding="utf-8")
text = text.replace("V46c", "V46d").replace(
    "v46c_mekf_rx180_upright_reinit_dynamic_beta_compare_20260913",
    "v46d_mekf_video_calibrated_rx180_upright_reinit_20260913",
)
text = text.replace(
    "MEKF と dynamic-beta Madgwick を同時記録する V46d ファームウェアです。",
    "MEKF と dynamic-beta Madgwick を同時記録する V46d ファームウェアです。動画比較用MEKF出力には既存の固定ホライズン校正係数0.908911を適用し、制御用MEKF座標は変更しません。",
)
p.write_text(text, encoding="utf-8")

p = Path("site/manifest.json")
text = p.read_text(encoding="utf-8").replace("V46c", "V46d").replace("0.46.2", "0.46.3")
p.write_text(text, encoding="utf-8")

p = Path("docs/MEKF_DYNAMIC_COMPARE_V46.md")
text = p.read_text(encoding="utf-8").replace(
    "| `pitch_mekf_abs_deg` | Continuous MEKF gravity-frame angle | No; video comparison |",
    "| `pitch_mekf_abs_deg` | Video-calibrated MEKF comparison angle (`-0.908911 * internal MEKF pitch`) | No; video comparison |",
)
p.write_text(text, encoding="utf-8")

p = Path("docs/FIRST_V46_DYNAMIC_VALIDATION.md")
text = p.read_text(encoding="utf-8")
needle = "- `pitch_mekf_abs_deg`\n- `pitch_madgwick_dynamic_abs_deg`\n"
if needle in text:
    text = text.replace(
        needle,
        "- `pitch_mekf_abs_deg` (V46d: video-calibrated sign/amplitude; control MEKF is unchanged)\n- `pitch_madgwick_dynamic_abs_deg`\n",
        1,
    )
p.write_text(text, encoding="utf-8")

print("V46d video calibration patch prepared")
