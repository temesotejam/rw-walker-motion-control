from pathlib import Path


def read(path):
    return Path(path).read_text(encoding='utf-8')


def write(path, text):
    Path(path).write_text(text, encoding='utf-8')


def replace_once(path, old, new):
    text = read(path)
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f'{path}: expected exactly one match, got {n}: {old[:120]!r}')
    write(path, text.replace(old, new, 1))


roller_h = r'''#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

struct RollerTelemetry {
  bool roller_ok = false;
  int16_t actual_current_mA = 0;
  uint16_t battery_mV = 0;
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

  // V46i task-split diagnostics. These are status-only and do not alter RWLOG v46.
  bool io_task_running = false;
  int16_t requested_current_mA = 0;
  int16_t applied_current_mA = 0;
  uint32_t last_command_latency_us = 0;
  uint32_t max_command_latency_us = 0;
  uint32_t command_sequence = 0;
  uint32_t applied_command_sequence = 0;
};

class Roller485Manager {
public:
  bool begin();
  bool startIoTask(uint8_t core_id, uint8_t priority, uint32_t stack_bytes);

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
  bool applyCurrentMa(const RollerCommand& cmd);
  void publishTelemetry();

  bool writeU8(uint8_t reg, uint8_t value);
  bool writeI32(uint8_t reg, int32_t value);
  bool readBytes(uint8_t reg, uint8_t* buffer, size_t len);
  bool readI32(uint8_t reg, int32_t& value);
  bool readU8(uint8_t reg, uint8_t& value);
  bool readCurrentFresh(bool audit_sample);
  void recordFreshCurrent(int32_t current_raw, uint32_t sample_time_us, bool audit_sample);
  void recordCurrentReadFailure(bool audit_sample);
  void beginCurrentAuditPulse();
  void endCurrentAuditPulse();
  void recordIo(bool ok);

  RollerTelemetry telemetry_;
  RollerTelemetry telemetry_snapshot_;
  mutable portMUX_TYPE telemetry_mux_ = portMUX_INITIALIZER_UNLOCKED;

  QueueHandle_t command_queue_ = nullptr;
  TaskHandle_t io_task_handle_ = nullptr;
  volatile bool io_task_running_ = false;
  volatile int16_t requested_current_mA_ = 0;
  uint32_t command_sequence_ = 0;

  uint32_t last_read_due_us_ = 0;
  uint32_t last_fast_current_due_us_ = 0;
  int16_t command_mA_ = 0;
  int16_t current_audit_previous_mA_ = 0;
  uint32_t current_audit_previous_sample_us_ = 0;
  bool current_audit_has_previous_sample_ = false;
  bool current_audit_active_ = false;
  bool current_audit_read_failed_ = false;
  const char* last_error_ = "not_initialized";
};
'''
write('src/roller485_manager.h', roller_h)

roller_cpp = r'''#include "roller485_manager.h"

#include <Wire.h>

#include "config.h"

static constexpr uint8_t REG_OUTPUT = 0x00;
static constexpr uint8_t REG_MODE = 0x01;
static constexpr uint8_t REG_SYS_STATUS = 0x0C;
static constexpr uint8_t REG_ERROR_CODE = 0x0D;
static constexpr uint8_t REG_VIN = 0x34;
static constexpr uint8_t REG_CURRENT = 0xB0;
static constexpr uint8_t REG_CURRENT_READBACK = 0xC0;

bool Roller485Manager::begin() {
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
  Wire.setClock(Config::I2C_HZ);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);

  Wire.beginTransmission(Config::ROLLER_ADDR);
  const bool present = Wire.endTransmission() == 0;
  if (!present) {
    telemetry_.roller_ok = false;
    last_error_ = "roller_not_found";
    publishTelemetry();
    return false;
  }

  bool ok = true;
  ok &= writeU8(REG_MODE, Config::ROLLER_MODE_CURRENT);
  ok &= writeI32(REG_CURRENT, 0);
  ok &= writeU8(REG_OUTPUT, 0);
  telemetry_.roller_ok = ok;
  telemetry_.mode_raw = Config::ROLLER_MODE_CURRENT;
  telemetry_.output_raw = 0;
  command_mA_ = 0;
  requested_current_mA_ = 0;
  telemetry_.requested_current_mA = 0;
  telemetry_.applied_current_mA = 0;
  last_error_ = ok ? "" : "roller_zero_failed";
  publishTelemetry();
  return ok;
}

bool Roller485Manager::startIoTask(uint8_t core_id, uint8_t priority, uint32_t stack_bytes) {
  if (io_task_running_) return true;
  if (command_queue_ == nullptr) {
    command_queue_ = xQueueCreate(4, sizeof(RollerCommand));
    if (command_queue_ == nullptr) {
      last_error_ = "roller_command_queue_create_failed";
      return false;
    }
  }
  const BaseType_t rc = xTaskCreatePinnedToCore(
      &Roller485Manager::ioTaskEntry, "roller485-io", stack_bytes, this,
      priority, &io_task_handle_, core_id);
  if (rc != pdPASS) {
    vQueueDelete(command_queue_);
    command_queue_ = nullptr;
    io_task_handle_ = nullptr;
    last_error_ = "roller_io_task_create_failed";
    return false;
  }
  return true;
}

void Roller485Manager::ioTaskEntry(void* arg) {
  static_cast<Roller485Manager*>(arg)->ioTaskLoop();
}

void Roller485Manager::ioTaskLoop() {
  io_task_running_ = true;
  telemetry_.io_task_running = true;
  publishTelemetry();

  for (;;) {
    RollerCommand cmd;
    while (command_queue_ && xQueueReceive(command_queue_, &cmd, 0) == pdTRUE) {
      if (!applyCurrentMa(cmd)) {
        // Fail closed inside the I/O owner task. A failed nonzero write is never
        // allowed to leave a requested output latched without an immediate zero attempt.
        requested_current_mA_ = 0;
        RollerCommand zero;
        zero.current_mA = 0;
        zero.requested_us = micros();
        zero.sequence = ++command_sequence_;
        applyCurrentMa(zero);
      }
    }

    update();

    // Independent fail-closed reconciliation: if the desired state is zero but
    // either the applied command or observed OUTPUT is nonzero, force zero here.
    if (requested_current_mA_ == 0 && (command_mA_ != 0 || telemetry_.output_raw != 0)) {
      RollerCommand zero;
      zero.current_mA = 0;
      zero.requested_us = micros();
      zero.sequence = ++command_sequence_;
      applyCurrentMa(zero);
    }

    telemetry_.io_task_running = true;
    telemetry_.requested_current_mA = requested_current_mA_;
    telemetry_.applied_current_mA = command_mA_;
    publishTelemetry();

    // A new motor command wakes this task immediately; otherwise yield Core 0
    // for roughly 1 ms to Wi-Fi/system work before the next audit/telemetry pass.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
  }
}

void Roller485Manager::update() {
  const uint32_t now_us = micros();

  // V46i keeps the full 2 ms current audit, but it now runs only on Core 0.
  if (command_mA_ != 0 &&
      (last_fast_current_due_us_ == 0 ||
       static_cast<uint32_t>(now_us - last_fast_current_due_us_) >=
           Config::CURRENT_AUDIT_FAST_READ_PERIOD_US)) {
    last_fast_current_due_us_ = now_us;
    readCurrentFresh(true);
  }

  const uint32_t period_us = Config::ROLLER_READ_PERIOD_MS * 1000UL;
  if (last_read_due_us_ != 0 && static_cast<uint32_t>(now_us - last_read_due_us_) < period_us) return;
  last_read_due_us_ = now_us;

  int32_t vin_raw = 0;
  uint8_t mode = 0;
  uint8_t output = 0;
  uint8_t status = 0;
  uint8_t error = 0;

  bool ok = readCurrentFresh(command_mA_ != 0);
  ok &= readI32(REG_VIN, vin_raw);
  ok &= readU8(REG_MODE, mode);
  ok &= readU8(REG_OUTPUT, output);
  ok &= readU8(REG_SYS_STATUS, status);
  ok &= readU8(REG_ERROR_CODE, error);
  recordIo(ok);
  if (!ok) return;

  telemetry_.battery_mV = static_cast<uint16_t>(max<int32_t>(0, vin_raw * 10));
  telemetry_.mode_raw = mode;
  telemetry_.output_raw = output;
  telemetry_.status_raw = status;
  telemetry_.error_raw = error;
  telemetry_.roller_ok = error == 0;
  last_error_ = error == 0 ? "" : "roller_error_raw";
}

bool Roller485Manager::setCurrentMa(int16_t current_mA) {
  if (current_mA == 0) return stop();
  if (!io_task_running_ || command_queue_ == nullptr || io_task_handle_ == nullptr) {
    last_error_ = "roller_io_task_not_running";
    return false;
  }
  if (requested_current_mA_ == current_mA) return true;

  RollerCommand cmd;
  cmd.current_mA = current_mA;
  cmd.requested_us = micros();
  cmd.sequence = ++command_sequence_;
  if (xQueueSend(command_queue_, &cmd, 0) != pdTRUE) {
    last_error_ = "roller_command_queue_full";
    return false;
  }
  requested_current_mA_ = current_mA;
  xTaskNotifyGive(io_task_handle_);
  return true;
}

bool Roller485Manager::stop() {
  if (!io_task_running_ || command_queue_ == nullptr || io_task_handle_ == nullptr) {
    // Before the task starts, only a zero command is allowed and is applied
    // synchronously for fail-safe setup/teardown.
    RollerCommand zero;
    zero.current_mA = 0;
    zero.requested_us = micros();
    zero.sequence = ++command_sequence_;
    requested_current_mA_ = 0;
    return applyCurrentMa(zero);
  }
  if (requested_current_mA_ == 0) return true;

  // Stop has priority over any not-yet-applied nonzero request.
  xQueueReset(command_queue_);
  RollerCommand zero;
  zero.current_mA = 0;
  zero.requested_us = micros();
  zero.sequence = ++command_sequence_;
  if (xQueueSend(command_queue_, &zero, 0) != pdTRUE) {
    last_error_ = "roller_stop_queue_failed";
    return false;
  }
  requested_current_mA_ = 0;
  xTaskNotifyGive(io_task_handle_);
  return true;
}

bool Roller485Manager::applyCurrentMa(const RollerCommand& cmd) {
  const bool was_commanded = command_mA_ != 0;
  const int32_t raw = static_cast<int32_t>(cmd.current_mA) * Config::ROLLER_CURRENT_RAW_PER_MA;
  bool ok = true;
  ok &= writeU8(REG_MODE, Config::ROLLER_MODE_CURRENT);
  ok &= writeI32(REG_CURRENT, raw);
  ok &= writeU8(REG_OUTPUT, cmd.current_mA == 0 ? 0 : 1);
  recordIo(ok);

  const uint32_t applied_us = micros();
  const uint32_t latency_us = static_cast<uint32_t>(applied_us - cmd.requested_us);
  telemetry_.last_command_latency_us = latency_us;
  if (latency_us > telemetry_.max_command_latency_us) telemetry_.max_command_latency_us = latency_us;
  telemetry_.command_sequence = cmd.sequence;

  if (!ok) {
    last_error_ = "roller_current_write_failed";
    telemetry_.roller_ok = false;
    publishTelemetry();
    return false;
  }

  command_mA_ = cmd.current_mA;
  telemetry_.applied_command_sequence = cmd.sequence;
  telemetry_.mode_raw = Config::ROLLER_MODE_CURRENT;
  telemetry_.output_raw = cmd.current_mA == 0 ? 0 : 1;
  telemetry_.applied_current_mA = cmd.current_mA;
  if (!was_commanded && cmd.current_mA != 0) beginCurrentAuditPulse();
  else if (was_commanded && cmd.current_mA == 0) endCurrentAuditPulse();
  last_error_ = "";
  publishTelemetry();
  return true;
}

RollerTelemetry Roller485Manager::telemetrySnapshot() const {
  RollerTelemetry out;
  portENTER_CRITICAL(&telemetry_mux_);
  out = telemetry_snapshot_;
  portEXIT_CRITICAL(&telemetry_mux_);
  return out;
}

void Roller485Manager::publishTelemetry() {
  portENTER_CRITICAL(&telemetry_mux_);
  telemetry_snapshot_ = telemetry_;
  portEXIT_CRITICAL(&telemetry_mux_);
}

uint32_t Roller485Manager::currentAgeUs(uint32_t now_us) const {
  const RollerTelemetry t = telemetrySnapshot();
  if (t.current_sample_time_us == 0) return UINT32_MAX;
  return static_cast<uint32_t>(now_us - t.current_sample_time_us);
}

bool Roller485Manager::ok() const {
  const RollerTelemetry t = telemetrySnapshot();
  return t.roller_ok && t.consecutive_errors < 5 && t.error_raw == 0;
}

bool Roller485Manager::readCurrentFresh(bool audit_sample) {
  int32_t current_raw = 0;
  ++telemetry_.current_sequence;
  if (!readI32(REG_CURRENT_READBACK, current_raw)) {
    recordCurrentReadFailure(audit_sample);
    return false;
  }
  recordFreshCurrent(current_raw, micros(), audit_sample);
  return true;
}

void Roller485Manager::recordFreshCurrent(int32_t current_raw, uint32_t sample_time_us,
                                          bool audit_sample) {
  telemetry_.actual_current_mA = static_cast<int16_t>(
      current_raw / Config::ROLLER_CURRENT_RAW_PER_MA);
  telemetry_.current_sample_time_us = sample_time_us;
  telemetry_.current_valid = true;
  if (!audit_sample || !current_audit_active_) return;

  const int16_t current_mA = telemetry_.actual_current_mA;
  if (current_audit_has_previous_sample_) {
    const uint32_t dt_us = static_cast<uint32_t>(sample_time_us - current_audit_previous_sample_us_);
    if (dt_us > 0 && dt_us <= 50000UL && isfinite(telemetry_.q_meas_observed_mA_s)) {
      telemetry_.q_meas_observed_mA_s += 0.5f *
          (fabsf(static_cast<float>(current_audit_previous_mA_)) + fabsf(static_cast<float>(current_mA))) *
          static_cast<float>(dt_us) * 1.0e-6f;
    }
  } else {
    telemetry_.q_meas_observed_mA_s = 0.0f;
    current_audit_has_previous_sample_ = true;
  }
  current_audit_previous_mA_ = current_mA;
  current_audit_previous_sample_us_ = sample_time_us;
  if (telemetry_.current_audit_sample_count < UINT16_MAX) {
    ++telemetry_.current_audit_sample_count;
  }
  telemetry_.q_meas_observed_valid = telemetry_.current_audit_sample_count >= 2 &&
      !current_audit_read_failed_ && isfinite(telemetry_.q_meas_observed_mA_s);
}

void Roller485Manager::recordCurrentReadFailure(bool audit_sample) {
  telemetry_.current_valid = false;
  ++telemetry_.current_read_failure_count;
  if (audit_sample && current_audit_active_) {
    current_audit_read_failed_ = true;
    telemetry_.q_meas_observed_valid = false;
  }
}

void Roller485Manager::beginCurrentAuditPulse() {
  current_audit_active_ = true;
  current_audit_has_previous_sample_ = false;
  current_audit_read_failed_ = false;
  current_audit_previous_mA_ = 0;
  current_audit_previous_sample_us_ = 0;
  last_fast_current_due_us_ = 0;
  telemetry_.current_sample_time_us = 0;
  telemetry_.current_audit_sample_count = 0;
  telemetry_.q_meas_observed_mA_s = NAN;
  telemetry_.current_valid = false;
  telemetry_.q_meas_observed_valid = false;
}

void Roller485Manager::endCurrentAuditPulse() {
  current_audit_active_ = false;
}

bool Roller485Manager::writeU8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Roller485Manager::writeI32(uint8_t reg, int32_t value) {
  uint8_t* p = reinterpret_cast<uint8_t*>(&value);
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  Wire.write(p, 4);
  return Wire.endTransmission() == 0;
}

bool Roller485Manager::readBytes(uint8_t reg, uint8_t* buffer, size_t len) {
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    memset(buffer, 0, len);
    return false;
  }
  const uint8_t got = Wire.requestFrom(Config::ROLLER_ADDR, static_cast<uint8_t>(len));
  if (got != len) {
    memset(buffer, 0, len);
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; ++i) buffer[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool Roller485Manager::readI32(uint8_t reg, int32_t& value) {
  value = 0;
  return readBytes(reg, reinterpret_cast<uint8_t*>(&value), 4);
}

bool Roller485Manager::readU8(uint8_t reg, uint8_t& value) {
  value = 0;
  return readBytes(reg, &value, 1);
}

void Roller485Manager::recordIo(bool ok) {
  if (ok) {
    telemetry_.consecutive_errors = 0;
    telemetry_.roller_ok = true;
    return;
  }
  telemetry_.i2c_error_count++;
  if (telemetry_.consecutive_errors < 255) telemetry_.consecutive_errors++;
  telemetry_.roller_ok = false;
}
'''
write('src/roller485_manager.cpp', roller_cpp)

# Version / task allocation constants.
replace_once(
    'src/config.h',
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46h_mekf_400hz_web_quiet_predict_20260913";',
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46i_mekf_400hz_dual_core_roller_queue_20260913";')
replace_once(
    'src/config.h',
    'static constexpr uint32_t MEKF_CONTROL_PREDICTION_MAX_US = 10000UL;',
    'static constexpr uint32_t MEKF_CONTROL_PREDICTION_MAX_US = 10000UL;\n'
    'static constexpr uint8_t ROLLER_IO_TASK_CORE = 0;\n'
    'static constexpr uint8_t ROLLER_IO_TASK_PRIORITY = 4;\n'
    'static constexpr uint32_t ROLLER_IO_TASK_STACK_BYTES = 4096UL;')

# Main loop: all Roller I2C leaves Core 1. Keep runner/web single-threaded so no
# new ExperimentRunner/Web races are introduced; V46h already suppresses status
# polling while a measurement is active.
main = read('src/main.cpp')
main = main.replace('AtomS3R V46h MEKF motor-driven dynamic validation',
                    'AtomS3R V46i MEKF dual-core motor validation')
main = main.replace('V46h identity:', 'V46i identity:')
main = main.replace('displayLine("V46h MEKF", "V7 MOTOR VALIDATION");',
                    'displayLine("V46i MEKF", "DUAL-CORE V7");')
old = '''  const bool roller_ok = roller.begin();
  roller.stop();
  Serial.printf("Roller485: %s\\n", roller_ok ? "OK" : "FAILED");

  runner.begin(logger, imu, roller);
  web.begin(server, runner, imu, roller, logger);
'''
new = '''  const bool roller_ok = roller.begin();
  roller.stop();
  const bool roller_task_ok = roller_ok && roller.startIoTask(
      Config::ROLLER_IO_TASK_CORE, Config::ROLLER_IO_TASK_PRIORITY,
      Config::ROLLER_IO_TASK_STACK_BYTES);
  Serial.printf("Roller485: %s task=%s core=%u priority=%u\\n",
                roller_ok ? "OK" : "FAILED", roller_task_ok ? "OK" : "FAILED",
                Config::ROLLER_IO_TASK_CORE, Config::ROLLER_IO_TASK_PRIORITY);

  runner.begin(logger, imu, roller);
  web.begin(server, runner, imu, roller, logger);
'''
if old not in main:
    raise RuntimeError('main setup anchor missing')
main = main.replace(old, new, 1)
old_loop = '''void loop() {
  const uint32_t loop_start_us = micros();
  M5.update();

  runner.serviceFast();
  runner.updateImuDynamicBetaContext();
  imu.update();
  roller.update();
  runner.update();
  updateStartupPoseGuide();
  web.update();

  runner.setLoopDt(static_cast<uint32_t>(micros() - loop_start_us));
}
'''
new_loop = '''void loop() {
  const uint32_t loop_start_us = micros();

  // Core 1 timing path: BMI270 -> MEKF -> predicted control -> V7 state machine.
  // Roller485 I2C/current audit is owned by the dedicated Core 0 task.
  runner.serviceFast();
  runner.updateImuDynamicBetaContext();
  imu.update();
  runner.update();

  // Display/button servicing is unnecessary during the measurement itself.
  if (!runner.running()) {
    M5.update();
    updateStartupPoseGuide();
  }
  // V46h/V46i browser code does not poll status while a measurement is active;
  // keeping handleClient here preserves emergency-stop POST handling without
  // introducing a second thread that mutates ExperimentRunner.
  web.update();

  runner.setLoopDt(static_cast<uint32_t>(micros() - loop_start_us));
  taskYIELD();
}
'''
if old_loop not in main:
    raise RuntimeError('main loop anchor missing')
main = main.replace(old_loop, new_loop, 1)
write('src/main.cpp', main)

# Thread-safe telemetry snapshots on the Core-1 runner and Web status path.
runner = read('src/experiment_runner.cpp')
old = '''  status_.boot_elapsed_ms = now_ms - boot_start_ms_;
  status_.roller_actual_current_mA = roller_->telemetry().actual_current_mA;
  status_.roller_battery_mV = roller_->telemetry().battery_mV;
'''
new = '''  status_.boot_elapsed_ms = now_ms - boot_start_ms_;
  const RollerTelemetry roller_snapshot = roller_->telemetrySnapshot();
  status_.roller_actual_current_mA = roller_snapshot.actual_current_mA;
  status_.roller_battery_mV = roller_snapshot.battery_mV;
'''
if old not in runner:
    raise RuntimeError('runner top telemetry anchor missing')
runner = runner.replace(old, new, 1)
old = '  const RollerTelemetry& roller_telemetry = roller_->telemetry();\n'
if runner.count(old) != 1:
    raise RuntimeError(f'runner log telemetry anchor count={runner.count(old)}')
runner = runner.replace(old, '  const RollerTelemetry roller_telemetry = roller_->telemetrySnapshot();\n', 1)
write('src/experiment_runner.cpp', runner)

web = read('src/web_ui.cpp')
old = '  const auto& roller = roller_->telemetry();\n'
if web.count(old) != 1:
    raise RuntimeError(f'web telemetry anchor count={web.count(old)}')
web = web.replace(old, '  const RollerTelemetry roller = roller_->telemetrySnapshot();\n', 1)
# Expose task health/command latency after the run without touching the RWLOG binary layout.
anchor = '  json += ",\\\"roller_actual_current_mA\\\":" + String(roller.actual_current_mA);\n'
if anchor not in web:
    raise RuntimeError('web json roller anchor missing')
web = web.replace(anchor, anchor +
    '  json += ",\\\"roller_io_task_running\\\":" + String(roller.io_task_running ? "true" : "false");\n'
    '  json += ",\\\"roller_command_latency_us\\\":" + String(roller.last_command_latency_us);\n'
    '  json += ",\\\"roller_command_latency_max_us\\\":" + String(roller.max_command_latency_us);\n', 1)
write('src/web_ui.cpp', web)

# User-facing flasher identity.
for path in ('site/index.html',):
    text = read(path).replace('V46h', 'V46i').replace('v46h_mekf_400hz_web_quiet_predict_20260913',
                                                     'v46i_mekf_400hz_dual_core_roller_queue_20260913')
    write(path, text)
manifest = read('site/manifest.json').replace('V46h', 'V46i').replace('"version": "0.46.7"', '"version": "0.46.8"')
write('site/manifest.json', manifest)

print('V46i dual-core Roller task split patch applied')
