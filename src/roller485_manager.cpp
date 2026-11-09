#include "roller485_manager.h"
#include "runtime_diagnostics.h"

#include <Wire.h>

#include "config.h"

static constexpr uint8_t REG_OUTPUT = 0x00;
static constexpr uint8_t REG_MODE = 0x01;
static constexpr uint8_t REG_SYS_STATUS = 0x0C;
static constexpr uint8_t REG_ERROR_CODE = 0x0D;
static constexpr uint8_t REG_VIN = 0x34;
static constexpr uint8_t REG_CURRENT = 0xB0;
static constexpr uint8_t REG_CURRENT_READBACK = 0xC0;
// M5Stack Unit Roller485 I2C protocol: Speed Readback X100 Int at 0x60.
static constexpr uint8_t REG_SPEED_READBACK = 0x60;
static constexpr uint8_t REG_POSITION_READBACK = 0x90;

bool Roller485Manager::begin() {
  // V46j: do not touch Wire from the Arduino/control core. Core 0 will own the
  // Roller bus from initialization through all commands and telemetry reads.
  telemetry_ = RollerTelemetry{};
  telemetry_snapshot_ = RollerTelemetry{};
  requested_current_mA_ = 0;
  command_mA_ = 0;
  io_task_running_ = false;
  io_task_ready_ = false;
  io_task_init_failed_ = false;
  last_error_ = "roller_io_task_not_started";
  publishTelemetry();
  return true;
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
  io_task_init_failed_ = false;
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

  const uint32_t start_ms = millis();
  while (!io_task_running_ && static_cast<uint32_t>(millis() - start_ms) < 500UL) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return io_task_running_;
}

void Roller485Manager::ioTaskEntry(void* arg) {
  static_cast<Roller485Manager*>(arg)->ioTaskLoop();
}

bool Roller485Manager::initializeIoOwner() {
  // Recovery always starts from a clean bus controller state, on Core 0.
  Wire.end();
  vTaskDelay(pdMS_TO_TICKS(2));
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
  Wire.setClock(Config::I2C_HZ);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);

  Wire.beginTransmission(Config::ROLLER_ADDR);
  if (Wire.endTransmission() != 0) {
    telemetry_.roller_ok = false;
    last_error_ = "roller_not_found";
    return false;
  }

  bool ok = true;
  ok &= writeU8(REG_MODE, Config::ROLLER_MODE_CURRENT);
  ok &= writeI32(REG_CURRENT, 0);
  ok &= writeU8(REG_OUTPUT, 0);
  recordIo(ok);
  if (!ok) {
    last_error_ = "roller_zero_failed";
    return false;
  }

  command_mA_ = 0;
  requested_current_mA_ = 0;
  telemetry_.mode_raw = Config::ROLLER_MODE_CURRENT;
  telemetry_.output_raw = 0;
  telemetry_.requested_current_mA = 0;
  telemetry_.applied_current_mA = 0;
  telemetry_.applied_command_sequence = ++command_sequence_;
  telemetry_.applied_command_time_us = micros();
  // Break any integration interval spanning a reinitialization/recovery.
  ++telemetry_.speed_sequence;
  telemetry_.speed_valid = false;
  last_speed_due_us_ = 0;
  ++telemetry_.position_sequence;
  telemetry_.position_valid = false;
  last_position_due_us_ = 0;

  // Force one full status read before declaring READY. This makes battery=0 or
  // an inaccessible device a startup failure instead of a later motor surprise.
  last_read_due_us_ = 0;
  update();
  if (!telemetry_.roller_ok || telemetry_.consecutive_errors != 0) {
    if (last_error_[0] == '\0') last_error_ = "roller_initial_telemetry_failed";
    return false;
  }
  last_error_ = "";
  return true;
}

void Roller485Manager::ioTaskLoop() {
  io_task_running_ = true;
  io_task_ready_ = false;
  io_task_init_failed_ = false;
  telemetry_.io_task_running = true;
  telemetry_.io_task_ready = false;
  telemetry_.io_task_init_failed = false;
  publishTelemetry();

  for (;;) {
    RuntimeDiag::phase(RuntimeDiag::Lane::Roller, RuntimeDiag::Phase::RollerInit);
    if (!io_task_ready_) {
      requested_current_mA_ = 0;
      command_mA_ = 0;
      if (command_queue_) xQueueReset(command_queue_);
      ++io_init_attempt_count_;
      telemetry_.io_init_attempt_count = io_init_attempt_count_;
      telemetry_.io_recovery_count = io_recovery_count_;
      telemetry_.io_task_running = true;
      telemetry_.io_task_ready = false;

      if (initializeIoOwner()) {
        io_task_ready_ = true;
        io_task_init_failed_ = false;
        telemetry_.io_task_ready = true;
        telemetry_.io_task_init_failed = false;
        telemetry_.io_init_attempt_count = io_init_attempt_count_;
        telemetry_.io_recovery_count = io_recovery_count_;
        publishTelemetry();
      } else {
        io_task_init_failed_ = true;
        telemetry_.io_task_init_failed = true;
        telemetry_.io_task_ready = false;
        telemetry_.roller_ok = false;
        publishTelemetry();
        RuntimeDiag::beat(RuntimeDiag::Lane::Roller, 0);
        RuntimeDiag::phase(RuntimeDiag::Lane::Roller, RuntimeDiag::Phase::Wait);
        vTaskDelay(pdMS_TO_TICKS(Config::ROLLER_IO_RETRY_PERIOD_MS));
        continue;
      }
    }

    RuntimeDiag::phase(RuntimeDiag::Lane::Roller, RuntimeDiag::Phase::RollerIo);
    RollerCommand cmd;
    while (command_queue_ && xQueueReceive(command_queue_, &cmd, 0) == pdTRUE) {
      if (!applyCurrentMa(cmd)) {
        requested_current_mA_ = 0;
        if (command_queue_) xQueueReset(command_queue_);
        io_task_ready_ = false;
        io_task_init_failed_ = true;
        ++io_recovery_count_;
        telemetry_.io_recovery_count = io_recovery_count_;
        telemetry_.io_task_ready = false;
        telemetry_.io_task_init_failed = true;
        telemetry_.roller_ok = false;
        publishTelemetry();
        break;
      }
    }
    if (!io_task_ready_) continue;

    update();

    if (telemetry_.consecutive_errors >= Config::ROLLER_IO_RECOVERY_ERROR_LIMIT) {
      requested_current_mA_ = 0;
      if (command_queue_) xQueueReset(command_queue_);
      io_task_ready_ = false;
      io_task_init_failed_ = true;
      ++io_recovery_count_;
      telemetry_.io_recovery_count = io_recovery_count_;
      telemetry_.io_task_ready = false;
      telemetry_.io_task_init_failed = true;
      telemetry_.roller_ok = false;
      publishTelemetry();
      continue;
    }

    if (requested_current_mA_ == 0 && (command_mA_ != 0 || telemetry_.output_raw != 0)) {
      RollerCommand zero;
      zero.current_mA = 0;
      zero.requested_us = micros();
      zero.sequence = ++command_sequence_;
      if (!applyCurrentMa(zero)) {
        io_task_ready_ = false;
        io_task_init_failed_ = true;
        ++io_recovery_count_;
        telemetry_.io_recovery_count = io_recovery_count_;
        telemetry_.io_task_ready = false;
        telemetry_.io_task_init_failed = true;
        telemetry_.roller_ok = false;
        publishTelemetry();
        continue;
      }
    }

    telemetry_.io_task_running = true;
    telemetry_.io_task_ready = true;
    telemetry_.io_task_init_failed = false;
    telemetry_.io_init_attempt_count = io_init_attempt_count_;
    telemetry_.io_recovery_count = io_recovery_count_;
    telemetry_.requested_current_mA = requested_current_mA_;
    telemetry_.applied_current_mA = command_mA_;
    publishTelemetry();
    RuntimeDiag::beat(RuntimeDiag::Lane::Roller, io_task_ready_ ? 1 : 0);
    RuntimeDiag::phase(RuntimeDiag::Lane::Roller, RuntimeDiag::Phase::Wait);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
  }
}

void Roller485Manager::update() {
  const uint32_t now_us = micros();

  // V46v: try the observational current audit every 1 ms while a pulse is active.
  // This does not alter pulse timing or current command; it only reduces sample-age slack.
  bool current_already_fresh = false;
  if (command_mA_ != 0 &&
      (last_fast_current_due_us_ == 0 ||
       static_cast<uint32_t>(now_us - last_fast_current_due_us_) >=
           Config::CURRENT_AUDIT_FAST_READ_PERIOD_US)) {
    last_fast_current_due_us_ = now_us;
    current_already_fresh = readCurrentFresh(true);
  }

  // An already queued output transition has priority over optional telemetry.
  if (requested_current_mA_ != command_mA_) return;
  if (last_position_due_us_ == 0 ||
      static_cast<uint32_t>(now_us - last_position_due_us_) >= Config::WHEEL_POSITION_READ_PERIOD_US) {
    last_position_due_us_ = now_us;
    readPositionFresh();
  }
  if (requested_current_mA_ != command_mA_) return;
  if (last_speed_due_us_ == 0 ||
      static_cast<uint32_t>(now_us - last_speed_due_us_) >= Config::WHEEL_SPEED_READ_PERIOD_US) {
    last_speed_due_us_ = now_us;
    readSpeedFresh();
  }
  if (requested_current_mA_ != command_mA_) return;

  const uint32_t period_us = Config::ROLLER_READ_PERIOD_MS * 1000UL;
  if (last_read_due_us_ != 0 && static_cast<uint32_t>(now_us - last_read_due_us_) < period_us) return;
  last_read_due_us_ = now_us;

  int32_t vin_raw = 0;
  uint8_t mode = 0;
  uint8_t output = 0;
  uint8_t status = 0;
  uint8_t error = 0;

  // If the fast audit already obtained a valid current in this same loop,
  // reuse it instead of immediately reading CURRENT_READBACK a second time.
  bool ok = current_already_fresh || readCurrentFresh(command_mA_ != 0);

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
  if (!io_task_ready_ || !io_task_running_ || command_queue_ == nullptr || io_task_handle_ == nullptr) {
    last_error_ = "roller_io_task_not_ready";
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
  // No synchronous Wire fallback is allowed on the control core in V46j.
  // Before READY, zero is already the only permitted requested state.
  if (!io_task_ready_ || !io_task_running_ || command_queue_ == nullptr || io_task_handle_ == nullptr) {
    requested_current_mA_ = 0;
    return true;
  }
  if (requested_current_mA_ == 0) return true;

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
  telemetry_.applied_command_time_us = applied_us;
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
  const uint32_t timing_start = micros();
  int32_t current_raw = 0;
  ++telemetry_.current_sequence;
  if (!readI32(REG_CURRENT_READBACK, current_raw)) {
    recordCurrentReadFailure(audit_sample);
    if (audit_sample) telemetry_.pulse_current_read_work.add(
        static_cast<uint32_t>(micros() - timing_start), 2000);
    return false;
  }
  if (audit_sample) telemetry_.pulse_current_read_work.add(
      static_cast<uint32_t>(micros() - timing_start), 2000);
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
    telemetry_.pulse_current_intervals.add(dt_us, 2000);
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

bool Roller485Manager::readPositionFresh() {
  int32_t raw = 0;
  ++telemetry_.position_sequence;
  Wire.setTimeOut(Config::WHEEL_POSITION_READ_TIMEOUT_MS);
  telemetry_.position_read_start_us = micros();
  const bool ok = readI32(REG_POSITION_READBACK, raw);
  telemetry_.position_read_end_us = micros();
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);
  telemetry_.position_sample_time_us = telemetry_.position_read_start_us +
      static_cast<uint32_t>(telemetry_.position_read_end_us - telemetry_.position_read_start_us) / 2;
  telemetry_.position_valid = ok;
  if (ok) telemetry_.position_raw_cdeg = raw;
  else ++telemetry_.position_read_failure_count;
  // Position failure makes work unavailable, never changes the controller input.
  return ok;
}

bool Roller485Manager::readSpeedFresh() {
  int32_t speed_raw_x100 = 0;
  ++telemetry_.speed_sequence;
  // Bound the extra observation transaction, especially when STOP arrives.
  // The midpoint is a host-read timestamp, not a calibrated encoder timestamp.
  Wire.setTimeOut(Config::WHEEL_SPEED_READ_TIMEOUT_MS);
  telemetry_.speed_read_start_us = micros();
  const bool ok = readI32(REG_SPEED_READBACK, speed_raw_x100);
  telemetry_.speed_read_end_us = micros();
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);
  telemetry_.speed_sample_time_us = telemetry_.speed_read_start_us +
      static_cast<uint32_t>(telemetry_.speed_read_end_us - telemetry_.speed_read_start_us) / 2;
  if (!ok) {
    recordSpeedReadFailure();
    return false;
  }
  telemetry_.speed_rpm = static_cast<float>(speed_raw_x100) / 100.0f;
  telemetry_.speed_raw_x100 = speed_raw_x100;
  telemetry_.speed_valid = true;
  return true;
}

void Roller485Manager::recordSpeedReadFailure() {
  telemetry_.speed_valid = false;
  ++telemetry_.speed_read_failure_count;
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
