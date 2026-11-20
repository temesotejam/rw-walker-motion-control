#pragma once
#if SUPPORT_TRANSFER_V39
#include <Arduino.h>
#include "psram_archive.h"
#include "psram_string.h"
#include "support_transfer_plan.h"

// Control publishes a copy of the causal decision state. Only Core 0 runs the
// candidate search; its results are log records and never feed a motor command.
class SupportTransferAudit {
 public:
  enum Gate : uint16_t {
    StaleImu = 1, StalePacket = 2, InvalidFoot = 4, MissingAcceleration = 8,
    ForeAft = 16, Current = 32, WheelSpeed = 64, Battery = 128, NoBudget = 256,
    PulseActive = 512
  };
  struct Sample {
    support_transfer_plan::Input input;
    uint32_t time_ms = 0, peak_ms = 0, packet = 0, captured_us = 0;
    uint16_t left_age_ms = UINT16_MAX, right_age_ms = UINT16_MAX, gate_mask = 0;
    uint8_t baseline_flags = 0;
    uint16_t raw_vbat_mv = 0, model_vbat_mv = 0;
    uint8_t model_vbat_status = 2;
  };
  void begin(float rise_tau_s);
  void startRun();
  void submit(const Sample& sample);
  void seal();
  void append(PsramString& json) const;

 private:
  struct Row {
    Sample sample;
    support_transfer_plan::Decision decision;
    uint32_t ready_after_us, solve_us;
  };
  static constexpr unsigned capacity = 1201;
  static void taskEntry(void* self);
  void work();
  struct Stats {
    uint32_t submitted = 0, replaced = 0, unavailable = 0, completed = 0, stored = 0;
    uint32_t max_ready_after_us = 0, max_solve_us = 0;
  };
  PsramArchive<Row, capacity> rows_;
  lateral_energy::CurrentTable current_;
  TaskHandle_t worker_ = nullptr;
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Sample pending_sample_;
  uint32_t generation_ = 0;
  bool active_ = false, pending_ = false;
  Stats stats_;
};
#endif
