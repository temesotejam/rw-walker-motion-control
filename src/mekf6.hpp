#pragma once

#include <cmath>
#include <cstdint>
#include "realtime_code.h"

namespace mekf6 {

constexpr float kPi = 3.14159265358979323846f;
constexpr float degToRad(float x) { return x * (kPi / 180.0f); }
constexpr float radToDeg(float x) { return x * (180.0f / kPi); }

struct Vec3 {
  float x;
  float y;
  float z;
  constexpr Vec3(float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f) : x(x_), y(y_), z(z_) {}
};

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
  constexpr Quaternion(float w_ = 1.0f, float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f)
      : w(w_), x(x_), y(y_), z(z_) {}
};

struct EulerDeg {
  float roll;
  float pitch;
  float yaw;
  constexpr EulerDeg(float roll_ = 0.0f, float pitch_ = 0.0f, float yaw_ = 0.0f)
      : roll(roll_), pitch(pitch_), yaw(yaw_) {}
};

struct Config {
  float gyro_noise_std_rad_s = 0.015f;
  float gyro_bias_rw_std_rad_s_sqrt_s = 0.0008f;
  float accel_direction_noise_std = 0.035f;
  float accel_mag_full_g = 0.08f;
  float accel_mag_reject_g = 0.30f;
  float accel_angle_full_deg = 6.0f;
  float accel_angle_reject_deg = 22.0f;
  float accel_min_confidence = 0.05f;
  float min_dt_s = 0.0005f;
  float max_dt_s = 0.0500f;
};

struct Diagnostics {
  float accel_norm_g = 0.0f;
  float accel_magnitude_error_g = 0.0f;
  float accel_direction_residual_deg = 0.0f;
  float accel_confidence = 0.0f;
  bool accel_used = false;
};

class Mekf6 {
 public:
  explicit Mekf6(const Config& cfg = Config{});
  void reset();
  void setConfig(const Config& cfg);
  const Config& config() const { return cfg_; }
  bool initializeFromAccel(const Vec3& accel_g);
  void setGyroBiasRadS(const Vec3& bias_rad_s);
  Vec3 gyroBiasRadS() const { return bias_; }
  bool RW_HOT_CODE predict(const Vec3& gyro_rad_s, float dt_s);
  bool RW_HOT_CODE updateAccel(const Vec3& accel_g);
  Quaternion quaternion() const { return q_; }
  EulerDeg eulerDeg() const;
  // Pure conversions also work on a copied posterior. The control owner only
  // needs pitch; HTTP can derive all display axes without reading this filter.
  static float RW_HOT_CODE pitchDegFromQuaternion(const Quaternion& quaternion);
  static EulerDeg eulerDegFromQuaternion(const Quaternion& quaternion);
  EulerDeg predictEulerDeg(const Vec3& gyro_rad_s, float dt_s) const;
  Diagnostics diagnostics() const { return diag_; }

 private:
  Config cfg_;
  Quaternion q_;
  Vec3 bias_;
  float P_[6][6]{};
  Diagnostics diag_;

  static float RW_HOT_CODE clampf(float v, float lo, float hi);
  static float RW_HOT_CODE norm(const Vec3& v);
  static Vec3 RW_HOT_CODE normalized(const Vec3& v);
  static float RW_HOT_CODE dot(const Vec3& a, const Vec3& b);
  static Quaternion RW_HOT_CODE quatMultiply(const Quaternion& a, const Quaternion& b);
  static Quaternion RW_HOT_CODE quatNormalized(const Quaternion& q);
  static Quaternion quatFromEuler(float roll, float pitch, float yaw);
  static Quaternion RW_HOT_CODE deltaQuat(const Vec3& dtheta);
  static Vec3 RW_HOT_CODE predictedSpecificForceUpBody(const Quaternion& q);
  static void RW_HOT_CODE skew(const Vec3& v, float S[3][3]);
  static bool RW_HOT_CODE inverse3x3(const float A[3][3], float invA[3][3]);
  static float RW_HOT_CODE smoothConfidence(float error, float full, float reject);
  void initializeCovariance();
  void RW_HOT_CODE symmetrizeCovariance();
  void RW_HOT_CODE injectErrorState(const float dx[6]);
  void RW_HOT_CODE applyResetJacobian(const Vec3& dtheta);
};

}  // namespace mekf6
