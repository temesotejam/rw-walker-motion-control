#pragma once
#include <stdint.h>
#include <string.h>
#include <type_traits>
#include "timing_deadline.h"
#include "realtime_code.h"

// V46q diagnostic only. One writer: the Core1 reader. No I/O, allocation or
// floating point. Export ONLY while measurement is inactive, on the idle owner.
// Durations are host wall times, not isolated CPU or pure I2C bus times.
struct ImuPollObservation {
  uint32_t start_us = 0, sample_us = 0, sequence = 0, dt_us = 0;
  uint32_t notify_age_us = 0, callback_gap_us = 0, callback_sequence = 0;
  uint32_t period_us = 0, update_us = 0, convert_us = 0, pack_us = 0;
  uint32_t publish_us = 0, total_us = 0, wakes = 0;
  uint32_t previous_total_us = 0, previous_yield_us = 0;
  uint32_t driver_status_us = 0, driver_data_us = 0;
  uint16_t driver_data_bytes = 0;
  uint8_t driver_failures = 0;
  bool driver_called = false;
  uint8_t mask = 0;
  bool has_notify = false, fresh_gyro = false, forced_yield = false;
};

struct ImuPollProfile {
  static constexpr unsigned kSeconds = 32, kBuckets = 320;
  static constexpr uint32_t kBucketUs = 100000;
  enum Stage { NOTIFY_AGE, CALLBACK_GAP, POLL_PERIOD, UPDATE, CONVERT,
               PACK, PUBLISH, TOTAL, YIELD, STAGES };
  struct Stats {
    uint32_t count = 0, max_us = 0;
    uint64_t sum_us = 0;
    void add(uint32_t v) { ++count; sum_us += v; if (v > max_us) max_us = v; }
  };
  struct Second {
    uint32_t polls = 0, gyro = 0, no_data = 0, forced_yields = 0;
    uint32_t notify_max = 0, update_max = 0, convert_max = 0;
    uint32_t total_max = 0, period_max = 0, long_gaps = 0;
  };
  struct Gap {
    uint32_t time_us = 0, dt_us = 0, sequence = 0, notify_age_us = 0;
    uint32_t callback_gap_us = 0, period_us = 0, update_us = 0, convert_us = 0;
    uint32_t pack_us = 0, publish_us = 0, total_us = 0;
    uint32_t previous_total_us = 0, previous_yield_us = 0, wakes = 0;
  };
  bool initialized = false;
  uint32_t epoch_us = 0, polls = 0, gyro = 0, no_data = 0;
  uint32_t coalesced_wakes = 0, forced_yields = 0, long_gaps = 0;
  uint32_t outside_buckets = 0, last_callback_sequence = 0;
  uint32_t record_overhead_max_us = 0;
  timing_deadline::Counter poll_work_deadline, gyro_interval_deadline;
  timing_deadline::Counter driver_status, driver_data;
  uint32_t driver_calls = 0, driver_failures = 0, driver_bytes = 0;
  Stats stage[STAGES];
  Second seconds[kSeconds];
  Gap worst_gap[kBuckets];

  void start(uint32_t epoch) {
    // Called by reader outside every spinlock. Never reset a large object in
    // an interrupt-disabled section or on each context refresh.
    memset(static_cast<void*>(this), 0, sizeof(*this));
    initialized = true; epoch_us = epoch;
  }
  static void maximum(uint32_t& a, uint32_t b) { if (b > a) a = b; }
  void RW_SPEED_CODE record(const ImuPollObservation& o) {
    if (!initialized || static_cast<int32_t>(o.start_us - epoch_us) < 0) return;
    ++polls; if (o.fresh_gyro) ++gyro; if (!o.mask) ++no_data;
    poll_work_deadline.add(o.total_us, 1000);
    if (o.fresh_gyro) gyro_interval_deadline.add(o.dt_us, 2500);
    if (o.driver_called) {
      ++driver_calls; driver_failures += o.driver_failures;
      driver_bytes += o.driver_data_bytes;
      driver_status.add(o.driver_status_us, 1000);
      driver_data.add(o.driver_data_us, 1000);
    }
    if (o.wakes > 1) coalesced_wakes += o.wakes - 1;
    if (o.forced_yield) ++forced_yields;
    if (o.has_notify) {
      stage[NOTIFY_AGE].add(o.notify_age_us);
      if (o.callback_sequence != last_callback_sequence) {
        if (o.callback_gap_us) stage[CALLBACK_GAP].add(o.callback_gap_us);
        last_callback_sequence = o.callback_sequence;
      }
    }
    if (o.period_us) stage[POLL_PERIOD].add(o.period_us);
    stage[UPDATE].add(o.update_us);
    if (o.mask) { stage[CONVERT].add(o.convert_us); stage[PACK].add(o.pack_us); }
    if (o.fresh_gyro) stage[PUBLISH].add(o.publish_us);
    stage[TOTAL].add(o.total_us);
    const bool gap = o.fresh_gyro && o.dt_us > 4000 &&
        static_cast<int32_t>(o.sample_us - epoch_us) >= 0;
    if (gap) ++long_gaps;
    const unsigned s = (o.start_us - epoch_us) / 1000000U;
    if (s < kSeconds) {
      Second& v = seconds[s]; ++v.polls;
      if (o.fresh_gyro) ++v.gyro;
      if (!o.mask) ++v.no_data;
      if (o.forced_yield) ++v.forced_yields;
      if (gap) ++v.long_gaps;
      maximum(v.notify_max, o.notify_age_us); maximum(v.update_max, o.update_us);
      maximum(v.convert_max, o.convert_us); maximum(v.total_max, o.total_us);
      maximum(v.period_max, o.period_us);
    }
    if (gap) {
      const uint32_t t = o.sample_us - epoch_us;
      const unsigned b = t / kBucketUs;
      if (b >= kBuckets) { ++outside_buckets; return; }
      Gap& g = worst_gap[b];
      // Representative worst gap in EACH 100ms window, not a first-N buffer.
      // All long gaps are still counted above; this is not a full event trace.
      if (o.dt_us > g.dt_us) {
        g.time_us=t; g.dt_us=o.dt_us; g.sequence=o.sequence;
        g.notify_age_us=o.notify_age_us; g.callback_gap_us=o.callback_gap_us;
        g.period_us=o.period_us; g.update_us=o.update_us; g.convert_us=o.convert_us;
        g.pack_us=o.pack_us; g.publish_us=o.publish_us; g.total_us=o.total_us;
        g.previous_total_us=o.previous_total_us; g.previous_yield_us=o.previous_yield_us;
        g.wakes=o.wakes;
      }
    }
  }
};
static_assert(sizeof(ImuPollProfile) < 20000, "Keep the acquisition probe bounded");

static_assert(std::is_trivially_copyable<ImuPollProfile>::value, "Profile must be plain copyable data");
