#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "psram_string.h"

namespace control_latency {
// All spans are HOST WALL times, including interruptions. An I/O call span
// includes its setup/ISR/resume cost as well as the blocked time. Neither it
// nor (poll - I/O) is a pure CPU/preemption measurement.
struct Span {
  uint32_t sum = 0, start = 0, count = 0;
  bool active = false;
  void begin(uint32_t now) { start = now; active = true; ++count; }
  void end(uint32_t now) { if (active) sum += now - start; active = false; }
  uint32_t at(uint32_t now) const { return sum + (active ? now - start : 0); }
};
struct Activity {
  uint32_t poll_us, io_us, polls;
  Activity(uint32_t p = 0, uint32_t i = 0, uint32_t n = 0) : poll_us(p), io_us(i), polls(n) {}
};
struct Stat {
  uint32_t count = 0, max_us = 0; uint64_t sum_us = 0;
  void add(uint32_t v) { ++count; sum_us += v; if (v > max_us) max_us = v; }
};
enum Point : uint8_t { Received, Derived, Mekf, Ready, Decision, Done, Points };
struct Delivery {
  uint32_t submit_us, receive_us;
  bool valid;
  Delivery() : submit_us(0), receive_us(0), valid(false) {}
  Delivery(uint32_t submit, uint32_t receive) : submit_us(submit), receive_us(receive), valid(true) {}
};
struct Row {
  uint32_t sequence = 0, sample_us = 0, poll_to_sample_us = 0;
  uint32_t age[Points]{};
  uint32_t polls_to_ready = 0, poll_overlap_to_ready_us = 0, io_overlap_to_ready_us = 0;
  uint32_t poll_overlap_to_done_us = 0, io_overlap_to_done_us = 0;
  uint8_t valid = 0;
  uint32_t queue_submit_age_us = 0, queue_receive_age_us = 0;
  bool delivery_valid = false;
  bool pulse_active_at_entry = false, accel_fresh_at_entry = false;
};
struct Profile {
  static constexpr unsigned kBuckets = 32;
  static constexpr unsigned kOverrunCapacity = 64;
  static constexpr uint32_t kBudgetUs = 2500;
  Row current{}, worst[kBuckets]{};
  Row overruns[kOverrunCapacity]{};
  uint32_t overrun_total = 0, overrun_stored = 0, overrun_overflow = 0;
  Stat age[Points]{}, poll_to_sample{}, poll_to_ready{}, io_to_ready{};
  Stat queue_submit_age{}, queue_receive_age{}, submit_to_receive{}, receive_to_mark{};
  Activity received_activity{};
  uint32_t epoch = 0, outside_buckets = 0, io_calls = 0, io_failures = 0;
  uint32_t io_calls_with_control_progress = 0, io_max_us = 0;
  bool active = false;
  void reset() {
    current = Row{};
    for (auto& r : worst) r = Row{};
    for (auto& r : overruns) r = Row{};
    overrun_total = overrun_stored = overrun_overflow = 0;
    for (auto& s : age) s = Stat{};
    poll_to_sample = Stat{}; poll_to_ready = Stat{}; io_to_ready = Stat{};
    received_activity = Activity{};
    queue_submit_age = Stat{}; queue_receive_age = Stat{};
    submit_to_receive = Stat{}; receive_to_mark = Stat{};
    epoch = outside_buckets = io_calls = io_failures = 0;
    io_calls_with_control_progress = io_max_us = 0; active = false;
  }
  void receive(bool measurement, uint32_t seq, uint32_t sample, uint32_t poll,
               uint32_t now, Activity activity, Delivery delivery = Delivery{});
  void mark(Point point, uint32_t now, Activity activity);
  void finish(uint32_t now, Activity activity);
  void appendJson(PsramString& out) const;
};
extern Profile profile;
static_assert(sizeof(Profile) < 10000, "Keep latency detail in bounded internal RAM");
void pollBegin();
void pollEnd();
void ioBegin();
void ioEnd(bool ok);
Activity activity(uint32_t* timestamp = nullptr);
void setActive(bool active);
void mark(Point point);
void progress();
}
