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


def replace_between(path, start, end, replacement):
    text = read(path)
    i = text.index(start)
    j = text.index(end, i)
    write(path, text[:i] + replacement + '\n\n' + text[j:])

replace_once(
    'src/config.h',
    'static constexpr uint32_t ROLLER_IO_TASK_STACK_BYTES = 6144UL;\n',
    'static constexpr uint32_t ROLLER_IO_TASK_STACK_BYTES = 6144UL;\n'
    'static constexpr uint32_t ROLLER_IO_RETRY_PERIOD_MS = 100UL;\n'
    'static constexpr uint8_t ROLLER_IO_RECOVERY_ERROR_LIMIT = 3;\n')

replace_once(
    'src/roller485_manager.h',
    '  bool io_task_init_failed = false;\n',
    '  bool io_task_init_failed = false;  // last attempt failed; task is still retrying\n'
    '  uint32_t io_init_attempt_count = 0;\n'
    '  uint32_t io_recovery_count = 0;\n')
replace_once(
    'src/roller485_manager.h',
    '  volatile int16_t requested_current_mA_ = 0;\n  uint32_t command_sequence_ = 0;\n',
    '  volatile int16_t requested_current_mA_ = 0;\n'
    '  uint32_t io_init_attempt_count_ = 0;\n'
    '  uint32_t io_recovery_count_ = 0;\n'
    '  uint32_t command_sequence_ = 0;\n')

replace_between(
    'src/roller485_manager.cpp',
    'bool Roller485Manager::startIoTask',
    'void Roller485Manager::ioTaskEntry',
    '''bool Roller485Manager::startIoTask(uint8_t core_id, uint8_t priority, uint32_t stack_bytes) {
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
}''')

replace_once(
    'src/roller485_manager.cpp',
    'bool Roller485Manager::initializeIoOwner() {\n  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);\n',
    'bool Roller485Manager::initializeIoOwner() {\n'
    '  // Recovery always starts from a clean bus controller state, on Core 0.\n'
    '  Wire.end();\n'
    '  vTaskDelay(pdMS_TO_TICKS(2));\n'
    '  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);\n')

replace_between(
    'src/roller485_manager.cpp',
    'void Roller485Manager::ioTaskLoop() {',
    'void Roller485Manager::update() {',
    '''void Roller485Manager::ioTaskLoop() {
  io_task_running_ = true;
  io_task_ready_ = false;
  io_task_init_failed_ = false;
  telemetry_.io_task_running = true;
  telemetry_.io_task_ready = false;
  telemetry_.io_task_init_failed = false;
  publishTelemetry();

  for (;;) {
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
        vTaskDelay(pdMS_TO_TICKS(Config::ROLLER_IO_RETRY_PERIOD_MS));
        continue;
      }
    }

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
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
  }
}''')

web = read('src/web_ui.cpp')
needle = '  json += ",\\"roller_io_task_init_failed\\":" + String(roller.io_task_init_failed ? "true" : "false");\n'
if needle in web and 'roller_io_init_attempt_count' not in web:
    web = web.replace(needle, needle +
        '  json += ",\\"roller_io_init_attempt_count\\":" + String(roller.io_init_attempt_count);\n'
        '  json += ",\\"roller_io_recovery_count\\":" + String(roller.io_recovery_count);\n', 1)
write('src/web_ui.cpp', web)

guard = read('tools/test_v46i_task_split_source_guards.py')
guard = guard.replace("'io_task_ready_', 'io_task_init_failed_', 'roller_io_task_start_timeout',\n", "'io_task_ready_', 'io_task_init_failed_',\n")
guard = guard.replace("# Creation is not readiness: startIoTask waits for Core 0 init completion.\n", "# Creation starts a persistent Core 0 owner; readiness may arrive after retries.\n")
guard = guard.replace("assert 'ROLLER_IO_TASK_STACK_BYTES = 6144UL' in config\n",
                      "assert 'ROLLER_IO_TASK_STACK_BYTES = 6144UL' in config\n"
                      "assert 'ROLLER_IO_RETRY_PERIOD_MS = 100UL' in config\n"
                      "assert 'ROLLER_IO_RECOVERY_ERROR_LIMIT = 3' in config\n")
guard = guard.replace("assert 'roller_io_task_init_failed' in web\n",
                      "assert 'roller_io_task_init_failed' in web\n"
                      "assert 'roller_io_init_attempt_count' in web\n"
                      "assert 'roller_io_recovery_count' in web\n")
guard += '''\n# Initialization/recovery must be self-healing, not one-shot.\nassert 'for (;;)' in roller_cpp\nassert 'initializeIoOwner()' in roller_cpp\nassert 'ROLLER_IO_RETRY_PERIOD_MS' in roller_cpp\nassert 'ROLLER_IO_RECOVERY_ERROR_LIMIT' in roller_cpp\nassert 'io_recovery_count_' in roller_cpp\nassert 'vTaskDelete(nullptr)' not in roller_cpp\nassert 'roller_io_task_start_timeout' not in roller_cpp\nassert 'return io_task_running_;' in roller_cpp\n'''
write('tools/test_v46i_task_split_source_guards.py', guard)

print('V46j automatic Roller initialization/recovery patch applied')
