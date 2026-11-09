#include <assert.h>
#include <stdint.h>
#include <iostream>
#include "../src/imu_acquisition_audit.h"

int main() {
  ImuAcquisitionAudit a;
  const uint32_t epoch = 123456U;
  a.start(epoch);
  for (uint32_t i = 1; i <= 12000; ++i) {
    const uint32_t stamp = epoch + (i - 1) * 2500U;
    a.sample(stamp, 2500, i);
    a.delivery(stamp, i, 300, 1);
    a.poll(800, 1);
  }
  a.finish(epoch + 30000000U);
  assert(a.samples == 12000 && a.delivered == 12000);
  assert(a.dt_sum_us == 30000000ULL && a.age_sum_us == 3600000ULL);
  assert(a.dt_bins[10] == 12000 && a.age_bins[1] == 12000);
  assert(a.over_4ms == 0 && a.over_5ms == 0 && a.queue_drops == 0);
  assert(a.delivery_sequence_gaps == 0 && a.finished && !a.active);
  for (unsigned i = 0; i < 30; ++i) assert(a.samples_per_second[i] == 400);
  // Post-run sampling cannot mutate the retained run result.
  a.sample(epoch + 31000000U, 6000, 12001);
  assert(a.samples == 12000);

  a.start(0xFFFFF000U);
  a.sample(0xFFFFF800U, 2048, 200);
  a.sample(0x00000F70U, 6000, 201);
  a.delivery(0xFFFFF800U, 200, 2500, 2);
  a.delivery(0x00000F70U, 203, 11001, 4);
  a.drop();
  a.poll(1234, 4);
  assert(a.samples == 2 && a.over_5ms == 1);
  assert(a.gaps[0].time_us == 8048 && a.gaps[0].dt_us == 6000);
  assert(a.queue_drops == 1 && a.delivery_sequence_gaps == 2);
  assert(a.age_max_us == 11001 && a.queue_high_water == 4);
  assert(a.coalesced_wakes == 3 && a.poll_max_us == 1234);
  for (uint32_t i = 0; i < 200; ++i) a.sample(0x00010000U + i*6000U, 6000, i+300);
  assert(a.gap_count == 128 && a.gap_overflow == 73);
  a.finish(0x00100000U);
  a.start(9000000U);
  assert(a.samples == 0 && a.queue_drops == 0 && a.gap_count == 0);
  assert(a.active && !a.finished && a.last_delivered_sequence == 0);
  // A queued sample preceding the measurement must not enter the new run.
  a.sample(8999999U, 2500, 999);
  assert(a.samples == 0);
  std::cout << "V46n acquisition accounting: 12000 samples, gaps, boundaries and uint32 wrap PASS\n";
}
