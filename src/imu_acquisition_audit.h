#pragma once
#include <stdint.h>

// Plain-data accounting shared by the firmware and the native regression test.
// Caller provides synchronization. No allocation, I/O, or floating point here.
struct ImuAcquisitionAudit {
  static constexpr unsigned kBins = 41;  // 250 us bins, final bin >= 10000 us.
  static constexpr unsigned kSeconds = 32;
  static constexpr unsigned kGapCapacity = 128;
  struct Gap { uint32_t time_us = 0, dt_us = 0, sequence = 0; };
  bool active = false;
  bool finished = false;
  uint32_t epoch_us = 0, duration_us = 0;
  uint32_t samples = 0, delivered = 0, polls = 0, coalesced_wakes = 0;
  uint32_t dt_max_us = 0, poll_max_us = 0, age_max_us = 0;
  uint64_t dt_sum_us = 0, age_sum_us = 0;
  uint32_t over_4ms = 0, over_5ms = 0, over_10ms = 0;
  uint32_t queue_drops = 0, queue_high_water = 0, delivery_sequence_gaps = 0;
  uint32_t last_delivered_sequence = 0;
  uint32_t dt_bins[kBins] = {}, age_bins[kBins] = {};
  uint32_t samples_per_second[kSeconds] = {}, max_dt_per_second[kSeconds] = {};
  uint32_t max_age_per_second[kSeconds] = {};
  Gap gaps[kGapCapacity] = {};
  uint32_t gap_count = 0, gap_overflow = 0;

  static unsigned bin(uint32_t value) {
    const unsigned b = value / 250U;
    return b < kBins ? b : kBins - 1;
  }
  void start(uint32_t now_us) {
    *this = ImuAcquisitionAudit{};
    epoch_us = now_us;
    active = true;
  }
  void finish(uint32_t now_us) {
    if (!active) return;
    duration_us = now_us - epoch_us;
    active = false;
    finished = true;
  }
  void poll(uint32_t elapsed_us, uint32_t wakes) {
    if (!active) return;
    ++polls;
    if (wakes > 1) coalesced_wakes += wakes - 1;
    if (elapsed_us > poll_max_us) poll_max_us = elapsed_us;
  }
  void sample(uint32_t stamp_us, uint32_t dt_us, uint32_t sequence) {
    if (!active || static_cast<int32_t>(stamp_us - epoch_us) < 0) return;
    ++samples;
    dt_sum_us += dt_us;
    ++dt_bins[bin(dt_us)];
    if (dt_us > dt_max_us) dt_max_us = dt_us;
    if (dt_us > 4000) ++over_4ms;
    if (dt_us > 5000) ++over_5ms;
    if (dt_us > 10000) ++over_10ms;
    const uint32_t relative_us = stamp_us - epoch_us;
    const unsigned second = relative_us / 1000000U;
    if (second < kSeconds) {
      ++samples_per_second[second];
      if (dt_us > max_dt_per_second[second]) max_dt_per_second[second] = dt_us;
    }
    if (dt_us > 4000) {
      if (gap_count < kGapCapacity) {
        Gap& g = gaps[gap_count++];
        g.time_us = relative_us; g.dt_us = dt_us; g.sequence = sequence;
      }
      else ++gap_overflow;
    }
  }
  void delivery(uint32_t stamp_us, uint32_t sequence, uint32_t age_us, uint32_t depth) {
    if (!active || static_cast<int32_t>(stamp_us - epoch_us) < 0) return;
    ++delivered;
    age_sum_us += age_us;
    ++age_bins[bin(age_us)];
    if (age_us > age_max_us) age_max_us = age_us;
    if (depth > queue_high_water) queue_high_water = depth;
    if (last_delivered_sequence != 0 && sequence - last_delivered_sequence > 1U)
      delivery_sequence_gaps += sequence - last_delivered_sequence - 1U;
    last_delivered_sequence = sequence;
    const unsigned second = (stamp_us - epoch_us) / 1000000U;
    if (second < kSeconds && age_us > max_age_per_second[second])
      max_age_per_second[second] = age_us;
  }
  void drop() { if (active) ++queue_drops; }
};
