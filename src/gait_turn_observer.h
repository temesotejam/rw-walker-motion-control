#pragma once

#include "periodic_foot_state.h"
#include <cmath>
#include <cstdint>

// Compare feet at the same lateral phase, once per complete + / - / + cycle.
// The signed gyro change describes body heading; floor travel is not measured here.
namespace gait_turn {

constexpr uint32_t minimum_half_ms = 150, maximum_half_ms = 850;
constexpr uint32_t maximum_gap_ms = 125;  // Five 40 Hz publication periods.
constexpr uint16_t minimum_half_samples = 5;
constexpr float minimum_reach_difference_deg = 2.f;
constexpr float minimum_swing_difference_deg = 1.5f;
constexpr float minimum_yaw_deg = 1.5f;
constexpr float minimum_pair_yaw_deg = 2 * minimum_yaw_deg;

enum class Reason : uint8_t {
  Waiting, Coverage, GyroUnavailable, Balanced, NoTurn, Pending, Confirmed, Phase
};

inline const char* reasonName(Reason reason) {
  switch (reason) {
    case Reason::Waiting: return "waiting";
    case Reason::Coverage: return "coverage";
    case Reason::GyroUnavailable: return "gyro_unavailable";
    case Reason::Balanced: return "below_asymmetry_gate";
    case Reason::NoTurn: return "below_yaw_gate";
    case Reason::Pending: return "pending";
    case Reason::Confirmed: return "confirmed";
    case Reason::Phase: return "phase";
  }
  return "unknown";
}

struct Result {
  uint32_t start_ms = 0, mid_ms = 0, end_ms = 0;
  uint16_t samples = 0;
  float left_deg = NAN, right_deg = NAN;  // Second-highest measured forward angles.
  float left_swing_deg = NAN, right_swing_deg = NAN;  // Second-highest minus second-lowest.
  float yaw_delta_deg = NAN, pair_yaw_delta_deg = NAN;
  float paired_swing_asymmetry = NAN;  // Used only when the two cycles jointly qualify.
  int8_t weak_side = 0;  // -1 left, +1 right.
  int8_t turn_sign = 0;  // Independent gyro sign; no assumed motor response.
  uint8_t streak = 0;
  Reason reason = Reason::Waiting;
};

struct Status {
  bool enabled = false;
  Result latest{};
  uint32_t confirmed_ms = 0;
  int8_t confirmed_side = 0;
};

namespace detail {
// Two distinct images must support either end of the excursion. Repeated angles
// on different images are allowed; repeated publications of one image are not.
struct Extrema {
  float high = -INFINITY, second_high = -INFINITY;
  float low = INFINITY, second_low = INFINITY;
  void add(float value) {
    if (value >= high) { second_high = high; high = value; }
    else if (value > second_high) second_high = value;
    if (value <= low) { second_low = low; low = value; }
    else if (value < second_low) second_low = value;
  }
};

struct Coverage {
  uint32_t first_ms = 0, last_ms = 0, longest_gap_ms = 0;
  uint16_t count = 0;
  void add(uint32_t t) {
    if (!count) first_ms = t;
    else if (t - last_ms > longest_gap_ms) longest_gap_ms = t - last_ms;
    last_ms = t;
    ++count;
  }
  bool valid(uint32_t begin, uint32_t end) const {
    return count >= minimum_half_samples && first_ms >= begin && last_ms <= end &&
        uint32_t(count) * (2 * periodic_foot::period_us / 1000) >= end - begin &&
        first_ms - begin <= maximum_gap_ms && end - last_ms <= maximum_gap_ms &&
        longest_gap_ms <= maximum_gap_ms;
  }
};

inline bool halfDurationValid(uint32_t begin, uint32_t end) {
  return end > begin && end - begin >= minimum_half_ms && end - begin <= maximum_half_ms;
}

inline int weakSide(float reach, float swing, float scale = 1.f) {
  if (reach >= scale*minimum_reach_difference_deg && swing >= scale*minimum_swing_difference_deg) return 1;
  if (reach <= -scale*minimum_reach_difference_deg && swing <= -scale*minimum_swing_difference_deg) return -1;
  return 0;
}

// At least one cycle independently clears both thresholds. The other must
// support the same foot and heading; a large turn followed by recovery is excluded.
inline bool pairedFootEvidence(const Result& previous, Result& current) {
  const float prev_reach = previous.left_deg - previous.right_deg;
  const float prev_swing = previous.left_swing_deg - previous.right_swing_deg;
  const float reach = current.left_deg - current.right_deg;
  const float swing = current.left_swing_deg - current.right_swing_deg;
  const int side = weakSide(reach, swing, .5f);
  if (!side || weakSide(prev_reach, prev_swing, .5f) != side ||
      (!current.weak_side && !weakSide(prev_reach, prev_swing)) ||
      weakSide(.5f*(prev_reach+reach), .5f*(prev_swing+swing)) != side ||
      !current.turn_sign || current.turn_sign * previous.yaw_delta_deg < minimum_yaw_deg) return false;
  const float total = current.left_swing_deg + current.right_swing_deg +
                      previous.left_swing_deg + previous.right_swing_deg;
  current.paired_swing_asymmetry = (prev_swing+swing)/total;
  current.weak_side = side;
  return true;
}
} // namespace detail

class Observer {
 public:
  void reset() { *this = Observer{}; }
  const Result& latest() const { return latest_; }

  void observe(const periodic_foot::Packet& p) {
    if (!p.epoch_us || !p.count) return;
    if (epoch_us_ && epoch_us_ != p.epoch_us) reset();
    epoch_us_ = p.epoch_us;
    const uint64_t image = p.left.image_us;
    if (!active_ || image <= last_image_us_ || image < epoch_us_ ||
        p.right.image_us != image || p.left.sequence != p.right.sequence ||
        image > p.published_us || p.published_us - image > periodic_foot::maximum_age_us ||
        !std::isfinite(p.left.measured) || !std::isfinite(p.right.measured)) return;
    last_image_us_ = image;
    const uint32_t t = static_cast<uint32_t>((image - epoch_us_) / 1000);
    if (t < start_ms_ || t - start_ms_ > 2 * maximum_half_ms) return;
    // A newly delivered image can date from just before the middle peak.
    auto& coverage = mid_ms_ && t >= mid_ms_ ? second_ : first_;
    coverage.add(t);
    left_.add(p.left.measured);
    right_.add(p.right.measured);
  }

  // known_ms is the time the peak was ACCEPTED since measurement start.
  // Yaw is the latest available unwrapped relative gyro heading (NaN when invalid).
  // A true return publishes exactly one completed or rejected cycle.
  bool peak(int side, uint32_t known_ms, float yaw) {
    if (side != 1 && side != -1) return false;
    if (!active_) {
      if (side == 1) begin(known_ms, yaw);
      return false;
    }
    const uint32_t last_peak = mid_ms_ ? mid_ms_ : start_ms_;
    if (known_ms <= last_peak) return false;
    if (side == -1 && !mid_ms_) {
      mid_ms_ = known_ms;
      mid_gyro_valid_ = std::isfinite(yaw);
      return false;
    }
    Result out;
    out.start_ms = start_ms_;
    out.mid_ms = mid_ms_;
    out.end_ms = known_ms;
    out.samples = first_.count + second_.count;
    const bool valid_phase = side == 1 && mid_ms_ &&
        detail::halfDurationValid(start_ms_, mid_ms_) && detail::halfDurationValid(mid_ms_, known_ms);
    if (!valid_phase) {
      out.reason = Reason::Phase;
    } else if (!first_.valid(start_ms_, mid_ms_) || !second_.valid(mid_ms_, known_ms)) {
      out.reason = Reason::Coverage;
    } else {
      classify(out, yaw);
    }
    latest_ = out;
    active_ = false;
    if (side == 1) begin(known_ms, yaw);
    return true;
  }

 private:
  void begin(uint32_t t, float yaw) {
    active_ = true;
    start_ms_ = t;
    mid_ms_ = 0;
    start_yaw_ = yaw;
    mid_gyro_valid_ = false;
    first_ = {}; second_ = {};
    left_ = {}; right_ = {};
  }

  void classify(Result& out, float yaw) const {
    out.left_deg = left_.second_high;
    out.right_deg = right_.second_high;
    out.left_swing_deg = out.left_deg - left_.second_low;
    out.right_swing_deg = out.right_deg - right_.second_low;
    const float reach_difference = out.left_deg - out.right_deg;
    const float swing_difference = out.left_swing_deg - out.right_swing_deg;
    out.weak_side = detail::weakSide(reach_difference, swing_difference);
    if (!std::isfinite(start_yaw_) || !mid_gyro_valid_ || !std::isfinite(yaw)) {
      out.reason = Reason::GyroUnavailable;
      return;
    }
    out.yaw_delta_deg = yaw - start_yaw_;
    if (out.yaw_delta_deg >= minimum_yaw_deg) out.turn_sign = 1;
    if (out.yaw_delta_deg <= -minimum_yaw_deg) out.turn_sign = -1;
    out.reason = !out.weak_side ? Reason::Balanced : (out.turn_sign ? Reason::Pending : Reason::NoTurn);
    out.streak = out.weak_side && out.turn_sign ? 1 : 0;
    if (latest_.end_ms != out.start_ms || !std::isfinite(latest_.yaw_delta_deg)) return;
    const int prev_side = detail::weakSide(latest_.left_deg-latest_.right_deg,
                                           latest_.left_swing_deg-latest_.right_swing_deg);
    if ((!out.weak_side || out.weak_side != prev_side) && !detail::pairedFootEvidence(latest_, out)) return;
    // A bend may accumulate mostly in one of the two cycles. A material
    // opposite turn in either cycle vetoes confirmation even if the sum is large.
    out.pair_yaw_delta_deg = latest_.yaw_delta_deg + out.yaw_delta_deg;
    if (std::fabs(out.pair_yaw_delta_deg) < minimum_pair_yaw_deg) return;
    const int sign = out.pair_yaw_delta_deg > 0 ? 1 : -1;
    if (sign * out.yaw_delta_deg <= -minimum_yaw_deg ||
        sign * latest_.yaw_delta_deg <= -minimum_yaw_deg) return;
    out.reason = Reason::Confirmed;
    out.turn_sign = sign;
    out.streak = 2;
    if (latest_.reason == Reason::Confirmed && latest_.turn_sign == sign)
      out.streak = latest_.streak < UINT8_MAX ? latest_.streak + 1 : UINT8_MAX;
  }

  uint64_t epoch_us_ = 0, last_image_us_ = 0;
  uint32_t start_ms_ = 0, mid_ms_ = 0;
  float start_yaw_ = NAN;
  bool active_ = false, mid_gyro_valid_ = false;
  detail::Coverage first_, second_;
  detail::Extrema left_, right_;
  Result latest_;
};
} // namespace gait_turn
