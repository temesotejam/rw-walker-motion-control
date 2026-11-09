#include "mekf6.hpp"

#include <algorithm>
#include <cstring>

// This small numerical kernel is deadline-sensitive. Favor execution speed
// over the platform's default -Os without enabling algebraic reassociation,
// approximate math, or finite-only assumptions. Keep the rest of the build's
// optimization policy unchanged; native differential tests also run with -Os.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "no-fast-math")
#endif

namespace mekf6 {

namespace {
// Fixed-size inner products avoid the innermost loop/index bookkeeping on
// the size-optimized ESP32 build. Keep the original accumulation order and
// initial zero; do not reassociate sums or enable fast-math.
inline float RW_HOT_CODE dotRows3(const float* a, const float* b) {
  float sum = 0.0f;
  sum += a[0] * b[0];
  sum += a[1] * b[1];
  sum += a[2] * b[2];
  return sum;
}
template <int Columns>
inline float RW_HOT_CODE dotColumn3(const float* a, const float (*b)[Columns], int column) {
  float sum = 0.0f;
  sum += a[0] * b[0][column];
  sum += a[1] * b[1][column];
  sum += a[2] * b[2][column];
  return sum;
}
}  // namespace

Mekf6::Mekf6(const Config& cfg) : cfg_(cfg) { reset(); }

void Mekf6::reset() {
  q_ = Quaternion{};
  bias_ = Vec3{};
  diag_ = Diagnostics{};
  initializeCovariance();
}

void Mekf6::setConfig(const Config& cfg) { cfg_ = cfg; }

float Mekf6::clampf(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }
float Mekf6::norm(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
Vec3 Mekf6::normalized(const Vec3& v) {
  const float n = norm(v);
  if (n < 1.0e-8f) return Vec3{};
  return {v.x / n, v.y / n, v.z / n};
}
float Mekf6::dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Quaternion Mekf6::quatMultiply(const Quaternion& a, const Quaternion& b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

Quaternion Mekf6::quatNormalized(const Quaternion& q) {
  const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1.0e-8f) return Quaternion{};
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}

Quaternion Mekf6::quatFromEuler(float roll, float pitch, float yaw) {
  const float cr = std::cos(0.5f * roll), sr = std::sin(0.5f * roll);
  const float cp = std::cos(0.5f * pitch), sp = std::sin(0.5f * pitch);
  const float cy = std::cos(0.5f * yaw), sy = std::sin(0.5f * yaw);
  return quatNormalized({
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
  });
}

Quaternion Mekf6::deltaQuat(const Vec3& dtheta) {
  const float angle = norm(dtheta);
  if (angle < 1.0e-7f) {
    return quatNormalized({1.0f, 0.5f * dtheta.x, 0.5f * dtheta.y, 0.5f * dtheta.z});
  }
  const float half = 0.5f * angle;
  const float scale = std::sin(half) / angle;
  return {std::cos(half), dtheta.x * scale, dtheta.y * scale, dtheta.z * scale};
}

Vec3 Mekf6::predictedSpecificForceUpBody(const Quaternion& q) {
  return {
      2.0f * (q.x * q.z - q.w * q.y),
      2.0f * (q.y * q.z + q.w * q.x),
      1.0f - 2.0f * (q.x * q.x + q.y * q.y),
  };
}

void Mekf6::skew(const Vec3& v, float S[3][3]) {
  S[0][0] = 0.0f;  S[0][1] = -v.z; S[0][2] = v.y;
  S[1][0] = v.z;   S[1][1] = 0.0f;  S[1][2] = -v.x;
  S[2][0] = -v.y;  S[2][1] = v.x;   S[2][2] = 0.0f;
}

bool Mekf6::inverse3x3(const float A[3][3], float invA[3][3]) {
  const float c00 = A[1][1] * A[2][2] - A[1][2] * A[2][1];
  const float c01 = A[1][2] * A[2][0] - A[1][0] * A[2][2];
  const float c02 = A[1][0] * A[2][1] - A[1][1] * A[2][0];
  const float det = A[0][0] * c00 + A[0][1] * c01 + A[0][2] * c02;
  if (std::fabs(det) < 1.0e-12f) return false;
  const float inv_det = 1.0f / det;
  invA[0][0] = c00 * inv_det;
  invA[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
  invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
  invA[1][0] = c01 * inv_det;
  invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
  invA[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
  invA[2][0] = c02 * inv_det;
  invA[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
  invA[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;
  return true;
}

float Mekf6::smoothConfidence(float error, float full, float reject) {
  if (error <= full) return 1.0f;
  if (error >= reject || reject <= full) return 0.0f;
  const float t = (error - full) / (reject - full);
  return 0.5f * (1.0f + std::cos(kPi * t));
}

void Mekf6::initializeCovariance() {
  std::memset(P_, 0, sizeof(P_));
  const float attitude_var = degToRad(5.0f) * degToRad(5.0f);
  const float bias_var = degToRad(0.5f) * degToRad(0.5f);
  for (int i = 0; i < 3; ++i) P_[i][i] = attitude_var;
  for (int i = 3; i < 6; ++i) P_[i][i] = bias_var;
}

bool Mekf6::initializeFromAccel(const Vec3& accel_g) {
  const float a_norm = norm(accel_g);
  if (!std::isfinite(a_norm) || a_norm < 0.2f) return false;
  const Vec3 a = normalized(accel_g);
  const float roll = std::atan2(a.y, a.z);
  const float pitch = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
  q_ = quatFromEuler(roll, pitch, 0.0f);
  initializeCovariance();
  return true;
}

void Mekf6::setGyroBiasRadS(const Vec3& bias_rad_s) { bias_ = bias_rad_s; }

bool Mekf6::predict(const Vec3& gyro_rad_s, float dt_s) {
  if (!std::isfinite(dt_s) || dt_s < cfg_.min_dt_s || dt_s > cfg_.max_dt_s) return false;
  const Vec3 omega{gyro_rad_s.x - bias_.x, gyro_rad_s.y - bias_.y, gyro_rad_s.z - bias_.z};
  q_ = quatNormalized(quatMultiply(q_, deltaQuat({omega.x * dt_s, omega.y * dt_s, omega.z * dt_s})));

  float W[3][3];
  skew(omega, W);
  float Phi[6][6]{};
  for (int i = 0; i < 6; ++i) Phi[i][i] = 1.0f;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) Phi[r][c] += -W[r][c] * dt_s;
    Phi[r][r + 3] = -dt_s;
  }

  float temp[6][6]{}, Pnew[6][6]{};
  // Phi = [ I-W*dt, -dt*I ; 0, I ]. Preserve the nonzero summation
  // order of Phi*P*Phi^T without multiplying its structural zero entries.
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 6; ++c) {
      temp[r][c] = dotColumn3(Phi[r], P_, c);
      temp[r][c] += Phi[r][r + 3] * P_[r + 3][c];
    }
  }
  for (int r = 3; r < 6; ++r)
    for (int c = 0; c < 6; ++c) temp[r][c] = P_[r][c];
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 3; ++c) {
      Pnew[r][c] = dotRows3(temp[r], Phi[c]);
      Pnew[r][c] += temp[r][c + 3] * Phi[c][c + 3];
    }
    for (int c = 3; c < 6; ++c) Pnew[r][c] = temp[r][c];
  }

  // Discrete process noise. The attitude term follows integrated gyro white noise;
  // the bias term follows the configured random-walk density.
  const float q_theta = cfg_.gyro_noise_std_rad_s * cfg_.gyro_noise_std_rad_s * dt_s * dt_s;
  const float q_bias = cfg_.gyro_bias_rw_std_rad_s_sqrt_s * cfg_.gyro_bias_rw_std_rad_s_sqrt_s * dt_s;
  for (int i = 0; i < 3; ++i) Pnew[i][i] += q_theta;
  for (int i = 3; i < 6; ++i) Pnew[i][i] += q_bias;
  std::memcpy(P_, Pnew, sizeof(P_));
  symmetrizeCovariance();
  return true;
}

bool Mekf6::updateAccel(const Vec3& accel_g) {
  diag_ = Diagnostics{};
  const float a_norm = norm(accel_g);
  diag_.accel_norm_g = a_norm;
  if (!std::isfinite(a_norm) || a_norm < 0.2f) return false;

  // Reuse the norm already checked above, retaining component-wise division.
  const Vec3 z{accel_g.x / a_norm, accel_g.y / a_norm, accel_g.z / a_norm};
  const Vec3 h = normalized(predictedSpecificForceUpBody(q_));
  const float mag_err = std::fabs(a_norm - 1.0f);
  const float cos_angle = clampf(dot(z, h), -1.0f, 1.0f);
  const float angle_deg = radToDeg(std::acos(cos_angle));
  const float c_mag = smoothConfidence(mag_err, cfg_.accel_mag_full_g, cfg_.accel_mag_reject_g);
  const float c_dir = smoothConfidence(angle_deg, cfg_.accel_angle_full_deg, cfg_.accel_angle_reject_deg);
  const float confidence = std::min(c_mag, c_dir);

  diag_.accel_magnitude_error_g = mag_err;
  diag_.accel_direction_residual_deg = angle_deg;
  diag_.accel_confidence = confidence;
  if (confidence < cfg_.accel_min_confidence) return false;

  float H[3][3];
  skew(h, H);
  const float y[3] = {z.x - h.x, z.y - h.y, z.z - h.z};

  float PHt[6][3]{};
  // H = [ skew(h), 0 ]; bias columns are identically zero.
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 3; ++c)
      PHt[r][c] = dotRows3(P_[r], H[c]);

  const float base_r = cfg_.accel_direction_noise_std * cfg_.accel_direction_noise_std;
  const float safe_conf = std::max(confidence, cfg_.accel_min_confidence);
  const float r_eff = base_r / (safe_conf * safe_conf);
  float S[3][3]{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      S[r][c] = dotColumn3(H[r], PHt, c);
      if (r == c) S[r][c] += r_eff;
    }
  }

  float Sinv[3][3];
  if (!inverse3x3(S, Sinv)) return false;
  float K[6][3]{};
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 3; ++c)
      K[r][c] = dotColumn3(PHt[r], Sinv, c);

  float dx[6]{};
  for (int r = 0; r < 6; ++r)
    dx[r] = dotRows3(K[r], y);

  // Joseph-form covariance update.
  float A[6][6]{};
  for (int i = 0; i < 6; ++i) A[i][i] = 1.0f;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 3; ++c) {
      A[r][c] -= K[r][0] * H[0][c];
      A[r][c] -= K[r][1] * H[1][c];
      A[r][c] -= K[r][2] * H[2][c];
    }
  }
  float AP[6][6]{}, Pj[6][6]{};
  // A = [ A00, 0 ; A10, I ]. Keep Joseph form, all cross covariance,
  // and the same addition order; only omit known zeros and identity products.
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      AP[r][c] = dotColumn3(A[r], P_, c);
      if (r >= 3) AP[r][c] += P_[r][c];
    }
  }
  // Each r_eff*K[r][k] is identical across the six output columns. Hoist
  // these 18 products, saving 90 multiplies without changing Joseph form.
  float KR[6][3];
  for (int r = 0; r < 6; ++r)
    for (int k = 0; k < 3; ++k) KR[r][k] = r_eff * K[r][k];
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      Pj[r][c] = dotRows3(AP[r], A[c]);
      if (c >= 3) Pj[r][c] += AP[r][c];
      Pj[r][c] += KR[r][0] * K[c][0];
      Pj[r][c] += KR[r][1] * K[c][1];
      Pj[r][c] += KR[r][2] * K[c][2];
    }
  }
  std::memcpy(P_, Pj, sizeof(P_));
  injectErrorState(dx);
  diag_.accel_used = true;
  symmetrizeCovariance();
  return true;
}

void Mekf6::injectErrorState(const float dx[6]) {
  const Vec3 dtheta{dx[0], dx[1], dx[2]};
  q_ = quatNormalized(quatMultiply(q_, deltaQuat(dtheta)));
  bias_.x += dx[3]; bias_.y += dx[4]; bias_.z += dx[5];
  applyResetJacobian(dtheta);
}

void Mekf6::applyResetJacobian(const Vec3& dtheta) {
  float S[3][3]; skew(dtheta, S);
  float G[6][6]{};
  for (int i = 0; i < 6; ++i) G[i][i] = 1.0f;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) G[r][c] -= 0.5f * S[r][c];
  float GP[6][6]{}, out[6][6]{};
  // G = [ I-0.5*skew(dtheta), 0 ; 0, I ].
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 6; ++c)
      GP[r][c] = dotColumn3(G[r], P_, c);
  for (int r = 3; r < 6; ++r)
    for (int c = 0; c < 6; ++c) GP[r][c] = P_[r][c];
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 3; ++c)
      out[r][c] = dotRows3(GP[r], G[c]);
    for (int c = 3; c < 6; ++c) out[r][c] = GP[r][c];
  }
  std::memcpy(P_, out, sizeof(P_));
}

void Mekf6::symmetrizeCovariance() {
  for (int r = 0; r < 6; ++r) {
    P_[r][r] = std::max(P_[r][r], 1.0e-12f);
    for (int c = r + 1; c < 6; ++c) {
      const float s = 0.5f * (P_[r][c] + P_[c][r]);
      P_[r][c] = s; P_[c][r] = s;
    }
  }
}

EulerDeg Mekf6::predictEulerDeg(const Vec3& gyro_rad_s, float dt_s) const {
  if (!std::isfinite(dt_s) || dt_s <= 0.0f) return eulerDeg();
  const Vec3 omega{gyro_rad_s.x - bias_.x, gyro_rad_s.y - bias_.y, gyro_rad_s.z - bias_.z};
  const Quaternion q = quatNormalized(quatMultiply(q_, deltaQuat({omega.x * dt_s, omega.y * dt_s, omega.z * dt_s})));
  const float sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
  const float cosr_cosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
  const float roll = std::atan2(sinr_cosp, cosr_cosp);
  const float sinp = clampf(2.0f * (q.w*q.y - q.z*q.x), -1.0f, 1.0f);
  const float pitch = std::asin(sinp);
  const float siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
  const float cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
  const float yaw = std::atan2(siny_cosp, cosy_cosp);
  return {radToDeg(roll), radToDeg(pitch), radToDeg(yaw)};
}

EulerDeg Mekf6::eulerDeg() const {
  return eulerDegFromQuaternion(q_);
}

float Mekf6::pitchDegFromQuaternion(const Quaternion& quaternion) {
  const Quaternion q = quatNormalized(quaternion);
  const float sinp = clampf(2.0f * (q.w * q.y - q.z * q.x), -1.0f, 1.0f);
  return radToDeg(std::asin(sinp));
}

EulerDeg Mekf6::eulerDegFromQuaternion(const Quaternion& quaternion) {
  const Quaternion q = quatNormalized(quaternion);
  const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  const float roll = std::atan2(sinr_cosp, cosr_cosp);
  const float sinp = clampf(2.0f * (q.w * q.y - q.z * q.x), -1.0f, 1.0f);
  const float pitch = std::asin(sinp);
  const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  const float yaw = std::atan2(siny_cosp, cosy_cosp);
  return {radToDeg(roll), radToDeg(pitch), radToDeg(yaw)};
}

}  // namespace mekf6

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
