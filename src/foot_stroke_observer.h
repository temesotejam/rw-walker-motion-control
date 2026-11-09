#pragma once

#include <cmath>
#include <cstdint>

// Observation only. Input must be one continuously tracked marker coordinate
// in a consistent camera-relative unit (e.g. px or a calibrated relative
// angle). It is NOT a distance travelled on the ground. With sign=0, the
// observer reports directionless full-span candidates between consecutive
// confirmed opposite extrema; it does not call either end "forward". With
// sign=+/-1, the caller asserts which coordinate direction is forward.
// No motor, posture, contact, or actuator decision is made here.
namespace foot_stroke_observer {

enum class Kind : uint8_t { None, InitialToForward, BackToForward, DirectionlessSpan };
enum class Direction : uint8_t { Unknown, Increasing, Decreasing };
enum class Status : uint8_t { Tracking, InvalidConfiguration, InvalidSample,
                              NonMonotonicTime, GapReset };

struct Event {
  Kind kind = Kind::None;
  Direction direction = Direction::Unknown; // direction in input coordinate
  uint32_t from_ms = 0;       // sampled initial position or first extremum
  uint32_t to_ms = 0;         // sampled final extremum
  uint32_t confirmed_ms = 0;  // later frame which established the reversal
  float from = NAN;           // forward-positive if sign known; raw if unknown
  float to = NAN;
  float width = NAN;          // same coordinate unit, positive
};

class Observer {
 public:
  // reversal is a measured noise-informed reversal distance, not a range
  // limit. gap_ms bounds a missing-observation interval; zero disables it.
  Observer(int forward_sign, float reversal, uint32_t gap_ms)
      : sign_(forward_sign), reversal_(reversal), gap_ms_(gap_ms) {}

  void reset() { phase_ = Phase::Empty; have_back_ = false; center_eligible_ = false;
    have_unsigned_extreme_ = false;
    last_ms_ = 0; status_ = Status::Tracking; }
  Status status() const { return status_; }

  Event observe(uint32_t ms, float coordinate, bool valid) {
    Event result;
    if ((sign_ != 1 && sign_ != -1 && sign_ != 0) || !std::isfinite(reversal_) || reversal_ <= 0.f) {
      reset(); status_ = Status::InvalidConfiguration; return result;
    }
    if (!valid || !std::isfinite(coordinate)) {
      reset(); status_ = Status::InvalidSample; return result;
    }
    if (phase_ != Phase::Empty && ms <= last_ms_) {
      reset(); status_ = Status::NonMonotonicTime; return result;
    }
    const bool gap = phase_ != Phase::Empty && gap_ms_ && ms-last_ms_ > gap_ms_;
    if (gap) reset();
    const float x = static_cast<float>(sign_ == 0 ? 1 : sign_) * coordinate;
    if (phase_ == Phase::Empty) {
      center_ = extreme_ = x; center_ms_ = extreme_ms_ = last_ms_ = ms;
      center_eligible_ = true; phase_ = Phase::Undecided;
      status_ = gap ? Status::GapReset : Status::Tracking; return result;
    }
    last_ms_ = ms; status_ = Status::Tracking;
    if (phase_ == Phase::Undecided) {
      if (x-center_ >= reversal_) { phase_ = Phase::Rising; extreme_ = x; extreme_ms_ = ms; }
      else if (center_-x >= reversal_) {
        // The first observed movement is backwards. Do not invent a first
        // center-to-forward half step; a full stroke may start at the trough.
        phase_ = Phase::Falling; extreme_ = x; extreme_ms_ = ms; center_eligible_ = false;
      }
    } else if (phase_ == Phase::Rising) {
      if (x > extreme_) { extreme_ = x; extreme_ms_ = ms; }
      else if (extreme_-x >= reversal_) {
        if (sign_ == 0) {
          result = unsignedExtremum(extreme_, extreme_ms_, ms);
        } else if (have_back_) {
          result.kind = Kind::BackToForward; result.from = back_; result.from_ms = back_ms_;
        } else if (center_eligible_) {
          result.kind = Kind::InitialToForward; result.from = center_; result.from_ms = center_ms_;
        }
        if (sign_ != 0 && result.kind != Kind::None) {
          result.to = extreme_; result.to_ms = extreme_ms_; result.confirmed_ms = ms;
          result.width = result.to-result.from;
          result.direction = sign_ > 0 ? Direction::Increasing : Direction::Decreasing;
        }
        have_back_ = false; center_eligible_ = false;
        phase_ = Phase::Falling; extreme_ = x; extreme_ms_ = ms;
      }
    } else { // Falling
      if (x < extreme_) { extreme_ = x; extreme_ms_ = ms; }
      else if (x-extreme_ >= reversal_) {
        if (sign_ == 0) result = unsignedExtremum(extreme_, extreme_ms_, ms);
        back_ = extreme_; back_ms_ = extreme_ms_; have_back_ = true;
        phase_ = Phase::Rising; extreme_ = x; extreme_ms_ = ms;
      }
    }
    return result;
  }

 private:
  Event unsignedExtremum(float value, uint32_t time, uint32_t confirmed) {
    Event result;
    if (have_unsigned_extreme_) {
      result.kind = Kind::DirectionlessSpan;
      result.from = unsigned_extreme_; result.from_ms = unsigned_extreme_ms_;
      result.to = value; result.to_ms = time; result.confirmed_ms = confirmed;
      result.width = std::fabs(value-unsigned_extreme_);
      result.direction = value>unsigned_extreme_ ? Direction::Increasing : Direction::Decreasing;
    }
    unsigned_extreme_=value; unsigned_extreme_ms_=time; have_unsigned_extreme_=true;
    return result;
  }
  enum class Phase : uint8_t { Empty, Undecided, Rising, Falling };
  int sign_; float reversal_; uint32_t gap_ms_;
  Phase phase_ = Phase::Empty; Status status_ = Status::Tracking;
  bool have_back_ = false, center_eligible_ = false, have_unsigned_extreme_ = false;
  uint32_t last_ms_ = 0, center_ms_ = 0, extreme_ms_ = 0, back_ms_ = 0;
  uint32_t unsigned_extreme_ms_ = 0;
  float center_ = 0, extreme_ = 0, back_ = 0, unsigned_extreme_ = 0;
};

struct Comparison {
  bool valid = false;
  Event left, right;
  float difference = NAN; // left minus right, in the common input unit
};

class Pairer {
 public:
  // Clear the appropriate foot immediately on invalid data, tracking reset,
  // calibration/mapping change, or run change. A prior foot event must not be
  // silently paired with a new tracking epoch on the opposite foot.
  void invalidateLeft() { left_valid_ = false; }
  void invalidateRight() { right_valid_ = false; }
  void reset() { left_valid_ = right_valid_ = false; }
  void left(const Event& e) { if(full(e)) { left_=e; left_valid_=true; } }
  void right(const Event& e) { if(full(e)) { right_=e; right_valid_=true; } }

  // The consumer chooses tolerances based on observed gait timing. Each
  // comparison consumes both events exactly once; mismatched/old events are
  // not interpreted as a zero stroke or used to drive a motor.
  Comparison take(uint32_t now, uint32_t max_age_ms, uint32_t max_peak_skew_ms) {
    Comparison out;
    if(!left_valid_ || !right_valid_) return out;
    // Both sides must have the same semantics. A directionless half-cycle
    // cannot be silently paired with a direction-confirmed forward stroke.
    if(left_.kind != right_.kind) return out;
    if(now < left_.confirmed_ms || now < right_.confirmed_ms ||
       now < left_.to_ms || now < right_.to_ms) return out;
    // A slow reversal can confirm an old peak much later. Fresh confirmation
    // does not make the underlying foot movement recent.
    if(now-left_.to_ms > max_age_ms) { left_valid_ = false; return out; }
    if(now-right_.to_ms > max_age_ms) { right_valid_ = false; return out; }
    const uint32_t skew = left_.to_ms > right_.to_ms
        ? left_.to_ms-right_.to_ms : right_.to_ms-left_.to_ms;
    if(skew > max_peak_skew_ms) return out;
    out.valid = true; out.left=left_; out.right=right_;
    out.difference=left_.width-right_.width;
    left_valid_=right_valid_=false;
    return out;
  }
 private:
  static bool full(const Event& e) {
    return e.kind == Kind::BackToForward || e.kind == Kind::DirectionlessSpan;
  }
  Event left_, right_; bool left_valid_=false, right_valid_=false;
};
} // namespace foot_stroke_observer
