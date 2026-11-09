#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "timing_deadline.h"

struct RollerTelemetry {
  timing_deadline::Counter pulse_current_read_work, pulse_current_intervals;
  bool roller_ok = false;
  int16_t actual_current_mA = 0;
  uint16_t battery_mV = 0;

  // Wheel-state telemetry, also sampled during output for work observation.
  // Existing pre-input prediction adaptation may use RPM; measured work is offline.
  float speed_rpm = NAN;
  int32_t speed_raw_x100 = INT32_MIN;
  uint32_t speed_sample_time_us = 0;
  uint32_t speed_read_start_us = 0;
  uint32_t speed_read_end_us = 0;
  uint32_t speed_sequence = 0;
  uint32_t speed_read_failure_count = 0;
  bool speed_valid = false;
  // Observation only: 0x90 cumulative mechanical position, centidegrees.
  int32_t position_raw_cdeg = 0;
  uint32_t position_sample_time_us = 0;
  uint32_t position_read_start_us = 0;
  uint32_t position_read_end_us = 0;
  uint32_t position_sequence = 0;
  uint32_t position_read_failure_count = 0;
  bool position_valid = false;
  uint32_t i2c_error_count = 0;
  uint8_t consecutive_errors = 0;
  uint8_t mode_raw = 0;
  uint8_t output_raw = 0;
  uint8_t status_raw = 0;
  uint8_t error_raw = 0;

  uint32_t current_sample_time_us = 0;
  uint32_t current_sequence = 0;
  uint32_t current_read_failure_count = 0;
  uint16_t current_audit_sample_count = 0;
  float q_meas_observed_mA_s = NAN;
  bool current_valid = false;
  bool q_meas_observed_valid = false;

  // V46j task-split diagnostics. These are status-only and do not alter RWLOG v46.
  bool io_task_running = false;
  bool io_task_ready = false;
  bool io_task_init_failed = false;  // last attempt failed; task is still retrying
  uint32_t io_init_attempt_count = 0;
  uint32_t io_recovery_count = 0;
  int16_t requested_current_mA = 0;
  int16_t applied_current_mA = 0;
  uint32_t last_command_latency_us = 0;
  uint32_t max_command_latency_us = 0;
  uint32_t command_sequence = 0;
  uint32_t applied_command_sequence = 0;
  uint32_t applied_command_time_us = 0;
};

class Roller485Manager {
public:
  bool begin();
  bool startIoTask(uint8_t core_id, uint8_t priority, uint32_t stack_bytes);
  bool ioReady() const { return io_task_ready_; }

  // Called only by the dedicated Roller I/O task after V46i starts.
  void update();

  // Control-core API: queue a desired current; no Roller I2C is performed here.
  bool setCurrentMa(int16_t current_mA);
  bool stop();

  RollerTelemetry telemetrySnapshot() const;
  uint32_t currentAgeUs(uint32_t now_us) const;
  bool ok() const;
  const char* lastError() const { return last_error_; }

private:
  struct RollerCommand {
    int16_t current_mA = 0;
    uint32_t requested_us = 0;
    uint32_t sequence = 0;
  };

  static void ioTaskEntry(void* arg);
  void ioTaskLoop();
  bool initializeIoOwner();
  bool applyCurrentMa(const RollerCommand& cmd);
  void publishTelemetry();

  bool writeU8(uint8_t reg, uint8_t value);
  bool writeI32(uint8_t reg, int32_t value);
  bool readBytes(uint8_t reg, uint8_t* buffer, size_t len);
  bool readI32(uint8_t reg, int32_t& value);
  bool readU8(uint8_t reg, uint8_t& value);
  bool readCurrentFresh(bool audit_sample);
  bool readSpeedFresh();
  bool readPositionFresh();
  void recordFreshCurrent(int32_t current_raw, uint32_t sample_time_us, bool audit_sample);
  void recordCurrentReadFailure(bool audit_sample);
  void recordSpeedReadFailure();
  void beginCurrentAuditPulse();
  void endCurrentAuditPulse();
  void recordIo(bool ok);

  RollerTelemetry telemetry_;
  RollerTelemetry telemetry_snapshot_;
  mutable portMUX_TYPE telemetry_mux_ = portMUX_INITIALIZER_UNLOCKED;

  QueueHandle_t command_queue_ = nullptr;
  TaskHandle_t io_task_handle_ = nullptr;
  volatile bool io_task_running_ = false;
  volatile bool io_task_ready_ = false;
  volatile bool io_task_init_failed_ = false;
  volatile int16_t requested_current_mA_ = 0;
  uint32_t io_init_attempt_count_ = 0;
  uint32_t io_recovery_count_ = 0;
  uint32_t command_sequence_ = 0;

  uint32_t last_read_due_us_ = 0;
  uint32_t last_fast_current_due_us_ = 0;
  uint32_t last_speed_due_us_ = 0;
  uint32_t last_position_due_us_ = 0;
  int16_t command_mA_ = 0;
  int16_t current_audit_previous_mA_ = 0;
  uint32_t current_audit_previous_sample_us_ = 0;
  bool current_audit_has_previous_sample_ = false;
  bool current_audit_active_ = false;
  bool current_audit_read_failed_ = false;
  const char* last_error_ = "not_initialized";
};
