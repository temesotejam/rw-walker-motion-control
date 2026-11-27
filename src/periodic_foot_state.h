#pragma once
#include <cmath>
#include <cstdint>

namespace periodic_foot {
constexpr uint32_t period_us = 25000;
constexpr uint32_t maximum_age_us = 75000;  // provisional, three control periods

struct Side {
  float measured = NAN, estimated = NAN;
#if SUPPORT_TRANSFER_V39 || SUPPORT_TRANSFER_CARRY2
  float rate_dps = NAN;  // signed slope used by the existing bounded extrapolator
#endif
  uint64_t image_us = 0;
  uint32_t sequence = 0;
};
struct Packet {
  Side left, right;
  uint64_t epoch_us = 0, scheduled_us = 0, published_us = 0;
  uint32_t count = 0, missed = 0, both_valid = 0, max_late_us = 0;
};
#if SUPPORT_TRANSFER_CARRY2 || SUPPORT_TRANSFER_STALL_V3
inline bool matchesPair(const Packet& p,uint64_t now,uint32_t sequence,uint64_t image) {
  return p.count && p.epoch_us && image>=p.epoch_us && now>=p.published_us &&
      now-p.published_us<=period_us && p.published_us>=image && now-image<=maximum_age_us &&
      p.left.sequence==sequence && p.right.sequence==sequence &&
      p.left.image_us==image && p.right.image_us==image;
}
#endif

class Track {
 public:
  void observe(uint32_t sequence, uint64_t time, float angle) {
    const uint64_t dt = time - last_.image_us;
    const float change = angle - last_.measured;
    const bool adjacent = last_.image_us && dt >= 15000 && dt <= 50000;
    const float slope = adjacent ? change * 1000000.f / dt : 0.f;
    // Require two consistent intervals before extrapolating. Start again at a turn.
    rate_ = adjacent && slope * previous_slope_ > 0 && std::fabs(slope) <= 80.f
            && std::fabs(previous_slope_) <= 80.f
        ? std::copysign(std::fmin(std::fabs(slope), std::fabs(previous_slope_)), slope) : 0.f;
#if SUPPORT_TRANSFER_V39 || SUPPORT_TRANSFER_CARRY2
    const bool usable = adjacent && std::fabs(slope) <= 80.f;
    last_.rate_dps = usable && previous_usable_ ? rate_ : NAN;
    previous_usable_ = usable;
#endif
    previous_slope_ = slope;
    last_.sequence = sequence;
    last_.image_us = time;
    last_.measured = angle;
  }
  Side at(uint64_t now) const {
    Side value = last_;
    if (value.image_us && now >= value.image_us && now - value.image_us <= maximum_age_us) {
      const float seconds = std::fmin(float(now - value.image_us), 25000.f) * 1e-6f;
      value.estimated = value.measured + rate_ * seconds;
    }
    return value;
  }
 private:
  Side last_;
  float previous_slope_ = 0, rate_ = 0;
#if SUPPORT_TRANSFER_V39 || SUPPORT_TRANSFER_CARRY2
  bool previous_usable_ = false;
#endif
};

class Estimator {
 public:
  void start(uint64_t epoch) {
    *this = Estimator{};
    next_ = epoch + period_us;
    epoch_ = epoch;
    packet.epoch_us = epoch;
  }
  void observe(uint32_t sequence, uint64_t image, uint64_t received, bool clock_ok,
               bool left_seen, bool left_allowed, float l,
               bool right_seen, bool right_allowed, float r) {
    if (!clock_ok || image > received || image < epoch_) {
      left_ = Track{}; right_ = Track{};
      return;
    }
    if (image <= last_image_) return;
    last_image_ = image;
    update(left_, sequence, image, left_seen, left_allowed, l);
    update(right_, sequence, image, right_seen, right_allowed, r);
  }
  bool publish(uint64_t now) {
    if (!next_ || now < next_) return false;
    const uint32_t skipped = (now - next_) / period_us;
    packet.missed += skipped;
    packet.scheduled_us = next_ + uint64_t(skipped) * period_us;
    next_ = packet.scheduled_us + period_us;
    packet.published_us = now;
    const uint32_t late = now - packet.scheduled_us;
    if (late > packet.max_late_us) packet.max_late_us = late;
    packet.left = left_.at(now);
    packet.right = right_.at(now);
    ++packet.count;
    if (std::isfinite(packet.left.estimated) && std::isfinite(packet.right.estimated)) ++packet.both_valid;
    return true;
  }
#if SUPPORT_TRANSFER_CARRY2
  // Only the control core calls this, from an already selected return aid.
  // A read never advances or republishes the 40 Hz stream.
  bool rescuePair(uint64_t now,uint64_t epoch,uint32_t sequence,uint64_t image,
                  uint64_t first_received,Side& left,Side& right) const {
    if (!epoch || epoch!=epoch_ || !packet.count || now<packet.published_us ||
        image<=packet.left.image_us ||
        image<=packet.right.image_us || image<epoch || first_received<image ||
        first_received<=packet.published_us || now<first_received ||
        now-first_received>period_us || now-image>maximum_age_us) return false;
    const auto l=left_.at(now),r=right_.at(now);
    if (l.sequence!=sequence || r.sequence!=sequence || l.image_us!=image || r.image_us!=image ||
        !std::isfinite(l.estimated) || !std::isfinite(r.estimated) ||
        !std::isfinite(l.rate_dps) || !std::isfinite(r.rate_dps)) return false;
    left=l;right=r;
    return true;
  }
#endif
  Packet packet;
 private:
  static void update(Track& side,uint32_t seq,uint64_t image,bool seen,bool allowed,float angle) {
    if (!seen) return;  // recognition gap; the previous observation continues to age
    if (!allowed || !std::isfinite(angle)) side = Track{};
    else side.observe(seq,image,angle);
  }
  Track left_,right_;
  uint64_t epoch_ = 0, next_ = 0, last_image_ = 0;
};
}
