from pathlib import Path


def replace_one(path: str, old: str, new: str) -> None:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old!r}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


# V46f: calibrate the MEKF *input model*, not its output.
# 0.908911 was previously identified from independent video peak calibration and
# was re-validated on the V46e synchronized 8-deg run (video rate ~= 0.9015*raw gy,
# corr ~= 0.9956). The estimator remains in the physical Ry180 body frame.
replace_one(
    "src/config.h",
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46e_mekf_ry180_physical_frame_upright_reinit_20260913";\n',
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46f_mekf_ry180_gyro_y_calibrated_upright_reinit_20260913";\n'
    'static constexpr float MEKF_GYRO_Y_SCALE = 0.908911f;\n',
)

p = Path("src/experiment_runner.cpp")
text = p.read_text(encoding="utf-8")
replace_pairs = [
    (
        "// V46e sensor-to-body adapter, identified from synchronized fixed-horizon\n"
        "// video plus the measured upright gravity vector. The installed AtomS3R has\n"
        "// raw upright gravity near -Z and physical/video pitch rate follows +raw_gy.\n"
        "// R_y(pi)=diag(-1,+1,-1) is the unique axis-aligned proper rotation that maps\n"
        "// upright -Z to filter +Z while preserving +raw_gy as positive body pitch rate.\n",
        "// V46f sensor-to-body adapter. Ry(pi) gives the physical/video body frame.\n"
        "// The Y gyro sensitivity is additionally calibrated before MEKF prediction;\n"
        "// this is an input-model correction, not a post-estimator angle scale.\n",
    ),
    (
        "  return {mekf6::degToRad(-r.gx_dps), mekf6::degToRad(r.gy_dps), mekf6::degToRad(-r.gz_dps)};\n",
        "  return {mekf6::degToRad(-r.gx_dps),\n"
        "          mekf6::degToRad(r.gy_dps * Config::MEKF_GYRO_Y_SCALE),\n"
        "          mekf6::degToRad(-r.gz_dps)};\n",
    ),
    (
        "  return {mekf6::degToRad(-bx_dps), mekf6::degToRad(by_dps), mekf6::degToRad(-bz_dps)};\n",
        "  return {mekf6::degToRad(-bx_dps),\n"
        "          mekf6::degToRad(by_dps * Config::MEKF_GYRO_Y_SCALE),\n"
        "          mekf6::degToRad(-bz_dps)};\n",
    ),
    (
        "    status_.mekf_bias_y_dps = mekf6::radToDeg(b.y);\n",
        "    status_.mekf_bias_y_dps = mekf6::radToDeg(b.y) / Config::MEKF_GYRO_Y_SCALE;\n",
    ),
]
for old, new in replace_pairs:
    if text.count(old) != 1:
        raise RuntimeError(f"experiment_runner.cpp expected one match, got {text.count(old)}")
    text = text.replace(old, new, 1)
p.write_text(text, encoding="utf-8")

# Correct stale MEKF metadata and record the calibrated sensor model explicitly.
p = Path("src/psram_logger.cpp")
text = p.read_text(encoding="utf-8")
old = '  json += "\\\"mekf_coordinate_mapping\\\":\\\"accel=raw_IMU_xyz;gyro_for_predict=-raw_gyro_xyz;reported_pitch=MEKF_pitch;preserves_V45_static_atan2_minus_ax_and_dynamic_minus_gy_detector_sign\\\",";\n'
new = (
    '  json += "\\\"mekf_coordinate_mapping\\\":\\\"Ry180_body_frame:accel=(-ax,+ay,-az);gyro=(-gx,+gy_scaled,-gz);reported_pitch=physical_video_sign\\\",";\n'
    '  json += "\\\"mekf_gyro_y_scale\\\":" + String(Config::MEKF_GYRO_Y_SCALE, 6) + ",";\n'
    '  json += "\\\"mekf_gyro_y_scale_role\\\":\\\"pre_prediction_sensor_calibration_not_output_angle_scaling\\\",";\n'
)
if text.count(old) != 1:
    raise RuntimeError("stale MEKF coordinate metadata line not found")
p.write_text(text.replace(old, new, 1), encoding="utf-8")

# Main identity.
for old, new in [
    ("V46e MEKF motor-driven dynamic validation", "V46f MEKF motor-driven dynamic validation"),
    ("V46e identity:", "V46f identity:"),
    ('displayLine("V46e MEKF", "V7 MOTOR VALIDATION");', 'displayLine("V46f MEKF", "V7 MOTOR VALIDATION");'),
]:
    replace_one("src/main.cpp", old, new)

# Native host test: the calibrated physical rate from raw +90 dps is +81.802 dps.
p = Path("tools/test_mekf_host.cpp")
text = p.read_text(encoding="utf-8")
text = text.replace("V46e proper sensor-to-body transform", "V46f calibrated physical sensor-to-body transform")
text = text.replace(
    "static mekf6::Vec3 gyroFilter(float gx, float gy, float gz) {\n"
    "  return {mekf6::degToRad(-gx), mekf6::degToRad(gy), mekf6::degToRad(-gz)};\n"
    "}\n",
    "static constexpr float kGyroYScale = 0.908911f;\n"
    "static mekf6::Vec3 gyroFilter(float gx, float gy, float gz) {\n"
    "  return {mekf6::degToRad(-gx), mekf6::degToRad(gy * kGyroYScale), mekf6::degToRad(-gz)};\n"
    "}\n",
)
text = text.replace(
    "// 2) Dynamic sign measured from synchronized video: +raw_gy => +pitch.\n",
    "// 2) Dynamic sign and calibrated scale measured from synchronized video.\n",
)
text = text.replace(
    "  if (!(p > 8.0f && p < 10.0f)) return 2;\n",
    "  if (!(p > 8.0f && p < 8.4f)) return 2;\n",
)
text = text.replace(
    "    if (!f.predict(gyroFilter(0, 90, 0), 0.005f)) return 3;\n",
    "    if (!f.predict(gyroFilter(0, 90.0f / kGyroYScale, 0), 0.005f)) return 3;\n",
)
text = text.replace(
    "V46e MEKF Ry180 physical-frame sign/dynamics/rejection test passed",
    "V46f MEKF Ry180 calibrated-gyro physical-frame test passed",
)
p.write_text(text, encoding="utf-8")

# Motor/source guard.
p = Path("tools/test_v46_motor_validation_source_guards.py")
text = p.read_text(encoding="utf-8")
text = text.replace(
    '"v46e_mekf_ry180_physical_frame_upright_reinit_20260913" in config',
    '"v46f_mekf_ry180_gyro_y_calibrated_upright_reinit_20260913" in config',
)
text = text.replace(
    '"V46e MEKF motor-driven dynamic validation" in main_cpp',
    '"V46f MEKF motor-driven dynamic validation" in main_cpp',
)
anchor = '    assert "status_.pitch_mekf_deg = raw_mekf_pitch_abs_deg_ - offset_mekf_pitch_deg_" in runner\n'
if anchor not in text:
    raise RuntimeError("source guard anchor missing")
text = text.replace(
    anchor,
    anchor
    + '    assert "MEKF_GYRO_Y_SCALE = 0.908911f" in config\n'
    + '    assert "r.gy_dps * Config::MEKF_GYRO_Y_SCALE" in runner\n'
    + '    assert "by_dps * Config::MEKF_GYRO_Y_SCALE" in runner\n'
    + '    assert "mekf_bias_y_dps = mekf6::radToDeg(b.y) / Config::MEKF_GYRO_Y_SCALE" in runner\n',
    1,
)
text = text.replace("V46e motor-driven validation source guards passed", "V46f motor-driven validation source guards passed")
p.write_text(text, encoding="utf-8")

# Web flasher identity. No video-output scaling language is introduced.
p = Path("site/index.html")
text = p.read_text(encoding="utf-8")
text = text.replace("V46e", "V46f").replace(
    "v46e_mekf_ry180_physical_frame_upright_reinit_20260913",
    "v46f_mekf_ry180_gyro_y_calibrated_upright_reinit_20260913",
)
p.write_text(text, encoding="utf-8")

p = Path("site/manifest.json")
text = p.read_text(encoding="utf-8").replace("V46e", "V46f").replace("0.46.4", "0.46.5")
p.write_text(text, encoding="utf-8")

# Documentation: the logged MEKF remains the direct estimator state, now with
# calibrated Y gyro input.
for doc in ["docs/MEKF_DYNAMIC_COMPARE_V46.md", "docs/FIRST_V46_DYNAMIC_VALIDATION.md"]:
    p = Path(doc)
    text = p.read_text(encoding="utf-8")
    text = text.replace("V46e", "V46f")
    p.write_text(text, encoding="utf-8")

print("V46f calibrated gyro-Y MEKF patch prepared")
