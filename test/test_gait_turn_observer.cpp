#include "gait_turn_observer.h"
#include <cassert>
#include <cstdio>

using gait_turn::Observer;
using gait_turn::Reason;
using periodic_foot::Packet;
constexpr uint64_t epoch = 21000000;

static Packet frame(unsigned ms, float left, float right) {
  Packet p;
  p.count = ms / 25 + 1;
  p.epoch_us = epoch;
  p.left.image_us = p.right.image_us = epoch + uint64_t(ms) * 1000;
  p.left.sequence = p.right.sequence = p.count;
  p.published_us = p.left.image_us + 20000;
  p.left.measured = left;
  p.right.measured = right;
  // Estimated values are intentionally different: only camera measurements count.
  p.left.estimated = p.right.estimated = 0.f;
  return p;
}

template<typename Edit>
static gait_turn::Result cycle(Observer& observer, unsigned start, float end_yaw,
                               Edit edit, float mid_yaw = 0) {
  for (unsigned t = 0; t < 800; t += 25) {
    if (t == 400) assert(!observer.peak(-1, start + t, mid_yaw));
    const float left = t >= 100 && t <= 200 ? 6.f : 0.f;
    const float right = t >= 500 && t <= 600 ? 1.f : 0.f;
    auto p = frame(start + t, left, right);
    edit(t, p);
    observer.observe(p);
  }
  assert(observer.peak(1, start + 800, end_yaw));
  return observer.latest();
}

static const auto unchanged = [](unsigned, Packet&) {};

static void persistenceAndSigns() {
  Observer observer;
  assert(!observer.peak(-1, 50, 0)); // Can't start in the middle.
  assert(!observer.peak(1, 100, 0));
  auto first = cycle(observer, 100, -4, unchanged);
  assert(first.reason == Reason::Pending && first.streak == 1 && first.samples == 32);
  auto second = cycle(observer, 900, -6, unchanged);
  assert(second.reason == Reason::Confirmed && second.weak_side == 1 && second.turn_sign == -1);
  assert(second.start_ms == 900 && second.mid_ms == 1300 && second.end_ms == 1700);
  assert(second.left_deg == 6 && second.right_deg == 1 && second.yaw_delta_deg == -2);
  assert(second.pair_yaw_delta_deg == -6);
  assert(second.left_swing_deg == 6 && second.right_swing_deg == 1);
  auto reversed_yaw = cycle(observer, 1700, 3, unchanged);
  assert(reversed_yaw.reason == Reason::Pending && reversed_yaw.streak == 1);
  assert(reversed_yaw.pair_yaw_delta_deg == 7); // Opposite 2° prevents a turn call.
  auto reversed_foot = cycle(observer, 2500, 7, [](unsigned, Packet& p) {
    const float left = p.left.measured;
    p.left.measured = p.right.measured;
    p.right.measured = left;
  });
  assert(reversed_foot.reason == Reason::Pending && reversed_foot.weak_side == -1);
  assert(reversed_foot.turn_sign == 1 && reversed_foot.streak == 1);
  assert(!std::isfinite(reversed_foot.pair_yaw_delta_deg));
  observer.reset();
  assert(observer.latest().reason == Reason::Waiting && !observer.latest().streak);
}

template<typename Edit>
static gait_turn::Result one(Edit edit, float end_yaw = 4) {
  Observer observer;
  assert(!observer.peak(1, 100, 0));
  return cycle(observer, 100, end_yaw, edit);
}

static void imageEvidence() {
  assert(one([](unsigned t, Packet& p) {
    p.left.measured = t == 125 ? 20.f : 0.f; // Isolated recognition spike.
  }).reason == Reason::Balanced);
  assert(one([](unsigned, Packet& p) {
    p.left.measured = p.right.measured + 5.f; // Offset without different excursion.
  }).reason == Reason::Balanced);
  assert(one(unchanged, .8f).reason == Reason::NoTurn);
  assert(one([](unsigned, Packet& p) { ++p.right.sequence; }).reason == Reason::Coverage);
  assert(one([](unsigned, Packet& p) { ++p.right.image_us; }).reason == Reason::Coverage);
  assert(one([](unsigned, Packet& p) { p.published_us = p.left.image_us - 1; }).reason == Reason::Coverage);
  assert(one([](unsigned, Packet& p) { p.published_us = p.left.image_us + 75001; }).reason == Reason::Coverage);
  assert(one([](unsigned t, Packet& p) {
    if (t >= 100 && t <= 225) p.right.measured = NAN;
  }).reason == Reason::Coverage); // Adequate count with a hole around a peak.
  assert(one([](unsigned t, Packet& p) {
    if (t >= 125 && t < 250) {
      const auto same = frame(200, 0, 0);
      p.left = same.left; p.right = same.right;
    }
  }).reason == Reason::Coverage); // Held or stale images cannot fill a hole.
  assert(one([](unsigned t, Packet& p) {
    if (t % 75 != 0) p.left.measured = NAN;
  }).reason == Reason::Coverage); // Sparse but evenly spaced images.
}

static void phaseAndGyro() {
  Observer observer;
  assert(!observer.peak(1, 100, 0));
  assert(cycle(observer, 100, 3, unchanged).reason == Reason::Pending);
  assert(cycle(observer, 900, 6, unchanged, NAN).reason == Reason::GyroUnavailable);
  assert(cycle(observer, 1700, 8, unchanged).reason == Reason::Pending);
  assert(observer.peak(1, 2600, 9)); // Positive peak with no intervening negative.
  assert(observer.latest().reason == Reason::Phase && !observer.latest().streak);
  assert(!observer.peak(-1, 3000, 9));
  assert(observer.peak(-1, 3100, 9)); // Bad order; next positive only rearms.
  assert(observer.latest().reason == Reason::Phase);
  assert(!observer.peak(1, 3200, 9));
  assert(!observer.peak(1, 3200, 9)); // Same notification twice.
  assert(!observer.peak(-1, 3250, 9));
  assert(observer.peak(1, 3600, 9) && observer.latest().reason == Reason::Phase);
  assert(cycle(observer, 3600, 12, unchanged).reason == Reason::Pending);
  auto next_run = frame(4400, 0, 0);
  ++next_run.epoch_us;
  observer.observe(next_run);
  assert(observer.latest().reason == Reason::Waiting);
  assert(!observer.peak(1, 4500, 180)); // A new run must start a fresh cycle.
  assert(one(unchanged, NAN).reason == Reason::GyroUnavailable);
}

static void unevenTurnAcrossTwoCycles() {
  Observer observer;
  observer.peak(1, 100, 0);
  assert(cycle(observer, 100, -1, unchanged).reason == Reason::NoTurn);
  const auto turning = cycle(observer, 900, 5, unchanged);
  assert(turning.reason == Reason::Confirmed && turning.turn_sign == 1 && turning.streak == 2);
  assert(turning.yaw_delta_deg == 6 && turning.pair_yaw_delta_deg == 5);
  const auto stationary = cycle(observer, 1700, 5, unchanged);
  assert(stationary.reason == Reason::Confirmed && stationary.yaw_delta_deg == 0);
  assert(stationary.pair_yaw_delta_deg == 6);
  const auto both_straight = cycle(observer, 2500, 5, unchanged);
  assert(both_straight.reason == Reason::NoTurn && !both_straight.streak && !both_straight.turn_sign);
  assert(both_straight.pair_yaw_delta_deg == 0);
  assert(cycle(observer, 3300, 10, unchanged).reason == Reason::Confirmed);
  // Opposite motion is significant at the boundary, despite the +3.5° sum.
  const auto reversal = cycle(observer, 4100, 8.5f, unchanged);
  assert(reversal.reason == Reason::Pending && reversal.turn_sign == -1);
}

static void twoCycleFootMotion() {
  // Strict cycle has left/right excursions of 6/1 degrees; borderline is 2.5/1.
  const auto borderline = [](unsigned, Packet& p) { p.left.measured *= 2.5f/6.f; };
  Observer observer;
  observer.peak(1, 100, 0);
  cycle(observer, 100, -3, unchanged);
  auto joined = cycle(observer, 900, -6, borderline);
  assert(joined.reason == Reason::Confirmed && joined.weak_side == 1 && joined.turn_sign == -1);
  assert(std::fabs(joined.paired_swing_asymmetry - 6.5f/10.5f) < .00001f);
  // The previous marginal cycle cannot qualify another marginal cycle.
  assert(cycle(observer, 1700, -9, borderline).reason == Reason::Balanced);
  joined = cycle(observer, 2500, -12, unchanged);
  assert(joined.reason == Reason::Confirmed && std::isfinite(joined.paired_swing_asymmetry));
  assert(!std::isfinite(cycle(observer, 3300, -15, unchanged).paired_swing_asymmetry));
  // A significant counter-turn, a missing observation, or a balanced foot blocks pooling.
  assert(cycle(observer, 4100, -12, borderline).reason != Reason::Confirmed);
  assert(cycle(observer, 4900, -9, [](unsigned, Packet& p) { p.left.measured = NAN; }).reason == Reason::Coverage);
  assert(cycle(observer, 5700, -6, borderline).reason == Reason::Balanced);
  cycle(observer, 6500, -3, unchanged);
  const auto balanced = cycle(observer, 7300, 0, [](unsigned, Packet& p) { p.left.measured /= 6.f; });
  assert(balanced.reason == Reason::Balanced && !std::isfinite(balanced.paired_swing_asymmetry));
}

int main() {
  persistenceAndSigns();
  imageEvidence();
  phaseAndGyro();
  unevenTurnAcrossTwoCycles();
  twoCycleFootMotion();
  std::puts("PASS paired gait cycles, independent gyro sign, fresh coverage and resets");
}
