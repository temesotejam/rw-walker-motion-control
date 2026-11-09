from pathlib import Path


def replace_one(path: str, old: str, new: str) -> None:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old!r}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


# V46e removes the V46d post-estimator video scaling. The estimator itself is
# expressed in the physical/video body frame instead.
p = Path("src/config.h")
text = p.read_text(encoding="utf-8")
old = (
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46d_mekf_video_calibrated_rx180_upright_reinit_20260913";\n'
    'static constexpr float MEKF_VIDEO_OUTPUT_SIGN = -1.0f;\n'
    'static constexpr float MEKF_VIDEO_OUTPUT_SCALE = 0.908911f;\n'
)
new = 'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46e_mekf_ry180_physical_frame_upright_reinit_20260913";\n'
if text.count(old) != 1:
    raise RuntimeError("V46d config calibration block not found exactly once")
p.write_text(text.replace(old, new, 1), encoding="utf-8")

p = Path("src/experiment_runner.cpp")
text = p.read_text(encoding="utf-8")

old_adapter = '''// V46c sensor-to-body adapter: the installed AtomS3R has raw -Z upward at the
// measured upright pose. Rotate raw IMU vectors 180 deg about +X, R=diag(+1,-1,-1).
// This maps upright gravity to filter +Z, keeps a right-handed frame, preserves
// the historical static pitch coordinate, and gives d(pitch)/dt = -raw_gy.
mekf6::Vec3 mekfAccelFromRaw(const ImuReading& r) {
  return {r.ax_g, -r.ay_g, -r.az_g};
}

mekf6::Vec3 mekfGyroRadFromRaw(const ImuReading& r) {
  return {mekf6::degToRad(r.gx_dps), mekf6::degToRad(-r.gy_dps), mekf6::degToRad(-r.gz_dps)};
}

mekf6::Vec3 mekfStartupBiasFromRaw(float bx_dps, float by_dps, float bz_dps) {
  return {mekf6::degToRad(bx_dps), mekf6::degToRad(-by_dps), mekf6::degToRad(-bz_dps)};
}
'''
new_adapter = '''// V46e sensor-to-body adapter, identified from synchronized fixed-horizon
// video plus the measured upright gravity vector. The installed AtomS3R has
// raw upright gravity near -Z and physical/video pitch rate follows +raw_gy.
// R_y(pi)=diag(-1,+1,-1) is the unique axis-aligned proper rotation that maps
// upright -Z to filter +Z while preserving +raw_gy as positive body pitch rate.
mekf6::Vec3 mekfAccelFromRaw(const ImuReading& r) {
  return {-r.ax_g, r.ay_g, -r.az_g};
}

mekf6::Vec3 mekfGyroRadFromRaw(const ImuReading& r) {
  return {mekf6::degToRad(-r.gx_dps), mekf6::degToRad(r.gy_dps), mekf6::degToRad(-r.gz_dps)};
}

mekf6::Vec3 mekfStartupBiasFromRaw(float bx_dps, float by_dps, float bz_dps) {
  return {mekf6::degToRad(-bx_dps), mekf6::degToRad(by_dps), mekf6::degToRad(-bz_dps)};
}
'''
if text.count(old_adapter) != 1:
    raise RuntimeError("old Rx180 adapter block not found exactly once")
text = text.replace(old_adapter, new_adapter, 1)

# Remove V46d post-output scaling from both status update sites.
old_output = (
    "  status_.pitch_mekf_abs_deg = Config::MEKF_VIDEO_OUTPUT_SIGN *\n"
    "      Config::MEKF_VIDEO_OUTPUT_SCALE * raw_mekf_pitch_abs_deg_;\n"
)
if text.count(old_output) != 2:
    raise RuntimeError(f"expected two V46d scaled output assignments, got {text.count(old_output)}")
text = text.replace(old_output, "  status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_;\n")

# Convert MEKF-estimated bias back into raw IMU coordinates using the inverse
# of R_y(pi), which is the same matrix.
old_bias = '''    // Convert the MEKF kinematic-sign bias back to original raw IMU signs for logs.
    status_.mekf_bias_x_dps = -mekf6::radToDeg(b.x);
    status_.mekf_bias_y_dps = -mekf6::radToDeg(b.y);
    status_.mekf_bias_z_dps = -mekf6::radToDeg(b.z);
'''
new_bias = '''    // Convert the MEKF body-frame bias back to original raw IMU coordinates.
    status_.mekf_bias_x_dps = -mekf6::radToDeg(b.x);
    status_.mekf_bias_y_dps = mekf6::radToDeg(b.y);
    status_.mekf_bias_z_dps = -mekf6::radToDeg(b.z);
'''
if text.count(old_bias) != 1:
    raise RuntimeError("old MEKF bias-log conversion block not found")
text = text.replace(old_bias, new_bias, 1)

# V46c detector pitch was opposite +gy. V46e pitch is the physical/video sign,
# so return-to-centre rate is opposite the detector side, and the physical peak
# side equals detector side.
old_peak = '''  // The adopted detector is empirically opposite in sign to +gy. At a
  // detector-side extremum, return-to-centre +gy has detector-side sign.
  const bool rate_confirms_return = rate_sign == energy_control_autonomous_candidate_detector_side_;
'''
new_peak = '''  // V46e detector pitch uses the physical/video sign and therefore agrees with
  // +gy during outward motion. On return to centre, the rate sign is opposite
  // the detector/physical peak side.
  const bool rate_confirms_return = rate_sign == -energy_control_autonomous_candidate_detector_side_;
'''
if text.count(old_peak) != 1:
    raise RuntimeError("old peak return-sign logic not found")
text = text.replace(old_peak, new_peak, 1)

old_record = '''  const bool accepted = recordEnergyControlAutonomousPeak(
      energy_control_autonomous_candidate_peak_ms_,
      -energy_control_autonomous_candidate_detector_side_,
      energy_control_autonomous_candidate_peak_amplitude_deg_, detector_peak_angle_deg);
'''
new_record = '''  const bool accepted = recordEnergyControlAutonomousPeak(
      energy_control_autonomous_candidate_peak_ms_,
      energy_control_autonomous_candidate_detector_side_,
      energy_control_autonomous_candidate_peak_amplitude_deg_, detector_peak_angle_deg);
'''
if text.count(old_record) != 1:
    raise RuntimeError("old detector-to-physical peak-side mapping not found")
text = text.replace(old_record, new_record, 1)
p.write_text(text, encoding="utf-8")

replace_one(
    "src/experiment_runner.h",
    "  float pitch_mekf_abs_deg = 0.0f;          // video-calibrated MEKF comparison coordinate; control is unchanged\n",
    "  float pitch_mekf_abs_deg = 0.0f;          // continuous MEKF physical/video body-frame pitch coordinate\n",
)

for old, new in [
    ("V46d MEKF motor-driven dynamic validation", "V46e MEKF motor-driven dynamic validation"),
    ("V46d identity:", "V46e identity:"),
    ('displayLine("V46d MEKF", "V7 MOTOR VALIDATION");', 'displayLine("V46e MEKF", "V7 MOTOR VALIDATION");'),
]:
    replace_one("src/main.cpp", old, new)

# Rewrite native test for the physically identified R_y(pi) body frame.
p = Path("tools/test_mekf_host.cpp")
p.write_text(r'''#include <cassert>
#include <cmath>
#include <cstdio>
#include "../src/mekf6.hpp"

// V46e proper sensor-to-body transform identified from the real installation:
// R_y(pi)=diag(-1,+1,-1). Raw upright gravity is -Z and physical/video pitch
// rate follows +raw_gy.
static mekf6::Vec3 accelFilter(float ax, float ay, float az) {
  return {-ax, ay, -az};
}
static mekf6::Vec3 gyroFilter(float gx, float gy, float gz) {
  return {mekf6::degToRad(-gx), mekf6::degToRad(gy), mekf6::degToRad(-gz)};
}
static float reportedPitch(const mekf6::Mekf6& f) { return f.eulerDeg().pitch; }

int main() {
  // 1) Static physical/video sign. Positive fixed-horizon pitch has +raw ax.
  mekf6::Mekf6 f;
  const float a10 = mekf6::degToRad(10.0f);
  auto a = accelFilter(std::sin(a10), 0.0f, -std::cos(a10));
  assert(f.initializeFromAccel(a));
  if (std::fabs(reportedPitch(f) - 10.0f) > 0.05f) return 1;

  // 1b) Actual measured upright vector must initialize on the near-upright
  // Euler branch and accept gravity immediately.
  f.reset();
  const auto measured_upright = accelFilter(0.021626f, 0.033568f, -0.999202f);
  if (!f.initializeFromAccel(measured_upright)) return 9;
  if (!f.predict(gyroFilter(0, 0, 0), 0.005f)) return 10;
  const bool upright_used = f.updateAccel(measured_upright);
  const auto upright_diag = f.diagnostics();
  const auto upright_euler = f.eulerDeg();
  std::printf("measured_upright roll=%.3f pitch=%.3f used=%d conf=%.3f resid=%.3f\n",
              upright_euler.roll, upright_euler.pitch, upright_used ? 1 : 0,
              upright_diag.accel_confidence, upright_diag.accel_direction_residual_deg);
  if (!upright_used || !upright_diag.accel_used) return 11;
  if (upright_diag.accel_confidence < 0.99f ||
      upright_diag.accel_direction_residual_deg > 0.1f) return 12;
  if (std::fabs(upright_euler.roll) > 5.0f) return 13;
  if (std::fabs(upright_euler.pitch - 1.239f) > 0.1f) return 14;

  // 2) Dynamic sign measured from synchronized video: +raw_gy => +pitch.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, -1)));
  for (int i = 0; i < 20; ++i) f.predict(gyroFilter(0, 90, 0), 0.005f);
  const float p = reportedPitch(f);
  std::printf("reported_pitch_after_100ms_raw_gy_+90=%.3f\n", p);
  if (!(p > 8.0f && p < 10.0f)) return 2;

  // 3) Physically consistent +45-deg sweep: +raw gy and +raw ax.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, -1)));
  for (int i = 1; i <= 100; ++i) {
    const float theta = mekf6::degToRad(90.0f * i * 0.005f);
    if (!f.predict(gyroFilter(0, 90, 0), 0.005f)) return 3;
    if (!f.updateAccel(accelFilter(std::sin(theta), 0, -std::cos(theta)))) return 4;
  }
  const auto sweep_diag = f.diagnostics();
  std::printf("synthetic_sweep_pitch=%.3f conf=%.3f resid=%.3f\n",
              reportedPitch(f), sweep_diag.accel_confidence,
              sweep_diag.accel_direction_residual_deg);
  if (std::fabs(reportedPitch(f) - 45.0f) > 0.5f) return 5;
  if (!sweep_diag.accel_used || sweep_diag.accel_confidence < 0.99f) return 6;

  // 4) Strong translational acceleration must still be rejected.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, -1)));
  f.predict(gyroFilter(0, 0, 0), 0.005f);
  const bool used = f.updateAccel(accelFilter(0.80f, 0.0f, -1.0f));
  const auto reject_diag = f.diagnostics();
  std::printf("accel_reject used=%d conf=%.3f norm=%.3f resid=%.3f\n",
              used ? 1 : 0, reject_diag.accel_confidence,
              reject_diag.accel_norm_g, reject_diag.accel_direction_residual_deg);
  if (used || reject_diag.accel_used || reject_diag.accel_confidence >= 0.05f) return 7;

  const auto q = f.quaternion();
  const float qn = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  std::printf("q_norm=%.7f\n", qn);
  if (std::fabs(qn - 1.0f) > 1e-5f) return 8;

  std::puts("V46e MEKF Ry180 physical-frame sign/dynamics/rejection test passed");
  return 0;
}
''', encoding="utf-8")

# Source guard for the correct physical estimator coordinate and control-side
# consistency. V46d output-gain assertions are removed.
p = Path("tools/test_v46_motor_validation_source_guards.py")
text = p.read_text(encoding="utf-8")
text = text.replace(
    '"v46d_mekf_video_calibrated_rx180_upright_reinit_20260913" in config',
    '"v46e_mekf_ry180_physical_frame_upright_reinit_20260913" in config',
)
text = text.replace(
    '"V46d MEKF motor-driven dynamic validation" in main_cpp',
    '"V46e MEKF motor-driven dynamic validation" in main_cpp',
)
for line in [
    '    assert "MEKF_VIDEO_OUTPUT_SCALE = 0.908911f" in config\n',
    '    assert "MEKF_VIDEO_OUTPUT_SIGN = -1.0f" in config\n',
    '    assert runner.count("Config::MEKF_VIDEO_OUTPUT_SCALE * raw_mekf_pitch_abs_deg_") == 2\n',
]:
    text = text.replace(line, "")
anchor = '    assert "status_.pitch_mekf_deg = raw_mekf_pitch_abs_deg_ - offset_mekf_pitch_deg_" in runner\n'
if anchor not in text:
    raise RuntimeError("V46e source-guard anchor missing")
text = text.replace(
    anchor,
    anchor
    + '    assert "return {-r.ax_g, r.ay_g, -r.az_g};" in runner\n'
    + '    assert "mekf6::degToRad(r.gy_dps)" in runner\n'
    + '    assert "rate_sign == -energy_control_autonomous_candidate_detector_side_" in runner\n'
    + '    assert "energy_control_autonomous_candidate_peak_ms_,\\n      energy_control_autonomous_candidate_detector_side_," in runner\n',
    1,
)
p.write_text(text, encoding="utf-8")

# Web/docs identify the corrected estimator rather than a display calibration.
p = Path("site/index.html")
text = p.read_text(encoding="utf-8")
text = text.replace("V46d", "V46e").replace(
    "v46d_mekf_video_calibrated_rx180_upright_reinit_20260913",
    "v46e_mekf_ry180_physical_frame_upright_reinit_20260913",
)
text = text.replace(
    "動画比較用MEKF出力には既存の固定ホライズン校正係数0.908911を適用し、制御用MEKF座標は変更しません。",
    "MEKFのIMU→機体座標を実測重力方向と動画角速度に基づくRy(180°)へ修正し、推定状態そのものを固定ホライズン動画と同じ物理Pitch符号にしています。",
)
p.write_text(text, encoding="utf-8")

p = Path("site/manifest.json")
text = p.read_text(encoding="utf-8").replace("V46d", "V46e").replace("0.46.3", "0.46.4")
p.write_text(text, encoding="utf-8")

p = Path("docs/MEKF_DYNAMIC_COMPARE_V46.md")
text = p.read_text(encoding="utf-8").replace(
    "| `pitch_mekf_abs_deg` | Video-calibrated MEKF comparison angle (`-0.908911 * internal MEKF pitch`) | No; video comparison |",
    "| `pitch_mekf_abs_deg` | Continuous MEKF physical/video body-frame pitch (Ry180 sensor-to-body transform) | No; video comparison |",
)
p.write_text(text, encoding="utf-8")

p = Path("docs/FIRST_V46_DYNAMIC_VALIDATION.md")
text = p.read_text(encoding="utf-8").replace(
    "- `pitch_mekf_abs_deg` (V46d: video-calibrated sign/amplitude; control MEKF is unchanged)",
    "- `pitch_mekf_abs_deg` (V46e: physical/video body-frame MEKF pitch; no post-output scaling)",
)
p.write_text(text, encoding="utf-8")

print("V46e proper physical-frame MEKF patch prepared")
