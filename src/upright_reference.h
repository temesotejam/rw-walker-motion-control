#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

// Explicit operator calibration. No measurement-start or estimator reset can
// replace this record. It is a single, versioned NVS blob.
namespace upright_reference {
constexpr uint32_t kMagic = 0x55505231;
constexpr uint32_t kAverageUs = 2000000;
constexpr uint32_t kTimeoutUs = 3000000;
constexpr float kMaxSpreadDeg = .5f;
struct Record {
  uint32_t magic = 0, version = 1, revision = 0, samples = 0;
  float pitch_deg = 0, normalized_ax = 0, spread_deg = 0;
};
inline bool valid(const Record& r) {
  return r.magic == kMagic && r.version == 1 && r.revision > 0 && r.samples >= 100 &&
      std::isfinite(r.pitch_deg) && std::fabs(r.pitch_deg) < 85 &&
      std::isfinite(r.normalized_ax) && std::fabs(r.normalized_ax) < 1 &&
      std::isfinite(r.spread_deg) && r.spread_deg >= 0 && r.spread_deg <= kMaxSpreadDeg;
}
inline float relative(float absolute, const Record& r) {
  return valid(r) && std::isfinite(absolute) ? absolute - r.pitch_deg : NAN;
}
inline float pitchError(float ax, float norm, const Record& r) {
  if (!valid(r) || !std::isfinite(ax) || !std::isfinite(norm) || norm < .2f) return 180;
  return std::fabs(std::asin(std::max(-1.f, std::min(1.f, ax / norm))) -
      std::asin(r.normalized_ax)) * 57.295779513f;
}
class Capture {
 public:
  enum class State { Idle, Collecting, Ready, Failed };
  void begin(uint32_t now_us, uint32_t previous_revision) {
    *this = Capture{}; state_ = State::Collecting; begin_us_ = now_us;
    candidate_.revision = previous_revision + 1;
    if (!candidate_.revision) candidate_.revision = 1;
  }
  bool pending() const { return state_ == State::Collecting; }
  State state() const { return state_; }
  uint32_t samples() const { return candidate_.samples; }
  const Record& candidate() const { return candidate_; }
  const char* error() const { return error_; }
  void cancel(const char* reason = "upright_reference_cancelled") {
    if (pending()) { state_ = State::Failed; error_ = reason; }
  }
  void step(uint32_t now_us, uint32_t sample_us, uint32_t sequence,
            bool healthy, bool output_stopped, float pitch_deg,
            float ax, float az, float norm, float gyro_norm) {
    if (!pending()) return;
    if (!output_stopped) { cancel("upright_reference_requires_stopped"); return; }
    if (!healthy || !sample_us || static_cast<uint32_t>(now_us - sample_us) > 15000 ||
        !std::isfinite(pitch_deg)) { cancel("upright_reference_imu_not_fresh"); return; }
    if (!std::isfinite(norm) || std::fabs(norm - 1.f) > .03f ||
        !std::isfinite(gyro_norm) || gyro_norm > 1.5f || !std::isfinite(ax) ||
        !std::isfinite(az) || az >= 0 || std::fabs(pitch_deg) >= 85) {
      cancel("upright_reference_hold_still"); return;
    }
    if (static_cast<uint32_t>(now_us - begin_us_) > kTimeoutUs) {
      cancel("upright_reference_timeout"); return;
    }
    if (!sequence || sequence == last_sequence_) return;
    last_sequence_ = sequence;
    if (!candidate_.samples) { first_sample_us_ = sample_us; min_ = max_ = pitch_deg; }
    min_ = std::min(min_, pitch_deg); max_ = std::max(max_, pitch_deg);
    if (max_ - min_ > kMaxSpreadDeg) { cancel("upright_reference_pose_moved"); return; }
    sum_pitch_ += pitch_deg;
    sum_ax_ += std::max(-1.f, std::min(1.f, ax / norm));
    ++candidate_.samples;
    if (static_cast<uint32_t>(sample_us - first_sample_us_) < kAverageUs) return;
    candidate_.magic = kMagic;
    candidate_.pitch_deg = sum_pitch_ / candidate_.samples;
    candidate_.normalized_ax = sum_ax_ / candidate_.samples;
    candidate_.spread_deg = max_ - min_;
    if (!valid(candidate_)) { cancel("upright_reference_invalid"); return; }
    state_ = State::Ready;
  }
 private:
  State state_ = State::Idle;
  Record candidate_;
  uint32_t begin_us_ = 0, first_sample_us_ = 0, last_sequence_ = 0;
  double sum_pitch_ = 0, sum_ax_ = 0;
  float min_ = 0, max_ = 0;
  const char* error_ = "";
};
}  // namespace upright_reference
