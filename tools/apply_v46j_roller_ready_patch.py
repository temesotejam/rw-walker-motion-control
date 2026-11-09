from pathlib import Path


def read(path):
    return Path(path).read_text(encoding='utf-8')


def write(path, text):
    Path(path).write_text(text, encoding='utf-8')


def replace_once(path, old, new):
    text = read(path)
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f'{path}: expected exactly one match, got {n}: {old[:100]!r}')
    write(path, text.replace(old, new, 1))


def replace_between(path, start, end, replacement):
    text = read(path)
    i = text.index(start)
    j = text.index(end, i)
    write(path, text[:i] + replacement + '\n\n' + text[j:])

# -----------------------------------------------------------------------------
# Roller task: make Core 0 the sole owner of Wire from initialization onward.
# startIoTask() now waits until the task has initialized the bus/device and
# published READY; otherwise it fails closed before ExperimentRunner can start.
# -----------------------------------------------------------------------------
replace_once(
    'src/roller485_manager.h',
    '  // V46i task-split diagnostics. These are status-only and do not alter RWLOG v46.\n  bool io_task_running = false;\n',
    '  // V46j task-split diagnostics. These are status-only and do not alter RWLOG v46.\n  bool io_task_running = false;\n  bool io_task_ready = false;\n  bool io_task_init_failed = false;\n')
replace_once(
    'src/roller485_manager.h',
    '  bool startIoTask(uint8_t core_id, uint8_t priority, uint32_t stack_bytes);\n',
    '  bool startIoTask(uint8_t core_id, uint8_t priority, uint32_t stack_bytes);\n  bool ioReady() const { return io_task_ready_; }\n')
replace_once(
    'src/roller485_manager.h',
    '  static void ioTaskEntry(void* arg);\n  void ioTaskLoop();\n',
    '  static void ioTaskEntry(void* arg);\n  void ioTaskLoop();\n  bool initializeIoOwner();\n')
replace_once(
    'src/roller485_manager.h',
    '  volatile bool io_task_running_ = false;\n  volatile int16_t requested_current_mA_ = 0;\n',
    '  volatile bool io_task_running_ = false;\n  volatile bool io_task_ready_ = false;\n  volatile bool io_task_init_failed_ = false;\n  volatile int16_t requested_current_mA_ = 0;\n')

replace_between(
    'src/roller485_manager.cpp',
    'bool Roller485Manager::begin() {',
    'bool Roller485Manager::startIoTask',
    '''bool Roller485Manager::begin() {
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
}''')

replace_between(
    'src/roller485_manager.cpp',
    'bool Roller485Manager::startIoTask',
    'void Roller485Manager::ioTaskEntry',
    '''bool Roller485Manager::startIoTask(uint8_t core_id, uint8_t priority, uint32_t stack_bytes) {
  if (io_task_ready_) return true;
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

  // Do not report task success until Core 0 has initialized Wire, found the
  // Roller, forced zero output, and completed one telemetry snapshot.
  const uint32_t start_ms = millis();
  while (!io_task_ready_ && !io_task_init_failed_ &&
         static_cast<uint32_t>(millis() - start_ms) < 1000UL) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  if (!io_task_ready_) {
    if (!io_task_init_failed_) last_error_ = "roller_io_task_start_timeout";
    return false;
  }
  return true;
}''')

replace_between(
    'src/roller485_manager.cpp',
    'void Roller485Manager::ioTaskEntry(void* arg) {',
    'void Roller485Manager::update() {',
    '''void Roller485Manager::ioTaskEntry(void* arg) {
  static_cast<Roller485Manager*>(arg)->ioTaskLoop();
}

bool Roller485Manager::initializeIoOwner() {
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

  // Force one full status read before declaring READY. This makes battery=0 or
  // an inaccessible device a startup failure instead of a later motor surprise.
  last_read_due_us_ = 0;
  update();
  if (!telemetry_.roller_ok || telemetry_.consecutive_errors != 0) {
    if (last_error_[0] == '\\0') last_error_ = "roller_initial_telemetry_failed";
    return false;
  }
  last_error_ = "";
  return true;
}

void Roller485Manager::ioTaskLoop() {
  io_task_running_ = true;
  telemetry_.io_task_running = true;
  telemetry_.io_task_ready = false;
  telemetry_.io_task_init_failed = false;
  publishTelemetry();

  if (!initializeIoOwner()) {
    io_task_init_failed_ = true;
    io_task_ready_ = false;
    telemetry_.io_task_running = false;
    telemetry_.io_task_ready = false;
    telemetry_.io_task_init_failed = true;
    publishTelemetry();
    vTaskDelete(nullptr);
    return;
  }

  io_task_ready_ = true;
  telemetry_.io_task_ready = true;
  publishTelemetry();

  for (;;) {
    RollerCommand cmd;
    while (command_queue_ && xQueueReceive(command_queue_, &cmd, 0) == pdTRUE) {
      if (!applyCurrentMa(cmd)) {
        requested_current_mA_ = 0;
        RollerCommand zero;
        zero.current_mA = 0;
        zero.requested_us = micros();
        zero.sequence = ++command_sequence_;
        applyCurrentMa(zero);
      }
    }

    update();

    if (requested_current_mA_ == 0 && (command_mA_ != 0 || telemetry_.output_raw != 0)) {
      RollerCommand zero;
      zero.current_mA = 0;
      zero.requested_us = micros();
      zero.sequence = ++command_sequence_;
      applyCurrentMa(zero);
    }

    telemetry_.io_task_running = true;
    telemetry_.io_task_ready = true;
    telemetry_.io_task_init_failed = false;
    telemetry_.requested_current_mA = requested_current_mA_;
    telemetry_.applied_current_mA = command_mA_;
    publishTelemetry();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
  }
}''')

replace_between(
    'src/roller485_manager.cpp',
    'bool Roller485Manager::setCurrentMa(int16_t current_mA) {',
    'bool Roller485Manager::stop() {',
    '''bool Roller485Manager::setCurrentMa(int16_t current_mA) {
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
}''')

replace_between(
    'src/roller485_manager.cpp',
    'bool Roller485Manager::stop() {',
    'bool Roller485Manager::applyCurrentMa',
    '''bool Roller485Manager::stop() {
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
}''')

# -----------------------------------------------------------------------------
# Identity / startup / diagnostics.
# -----------------------------------------------------------------------------
replace_once('src/config.h',
             'static constexpr uint32_t ROLLER_IO_TASK_STACK_BYTES = 4096UL;',
             'static constexpr uint32_t ROLLER_IO_TASK_STACK_BYTES = 6144UL;')
replace_once('src/config.h',
             'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46i_mekf_400hz_dual_core_roller_queue_20260913";',
             'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46j_mekf_dual_core_roller_ready_20260913";')

main = read('src/main.cpp')
main = main.replace('AtomS3R V46i MEKF dual-core motor validation',
                    'AtomS3R V46j MEKF dual-core motor validation')
main = main.replace('V46i identity:', 'V46j identity:')
main = main.replace('displayLine("V46i MEKF", "DUAL-CORE V7");',
                    'displayLine("V46j MEKF", "DUAL-CORE V7");')
main = main.replace('  const bool roller_ok = roller.begin();\n  roller.stop();\n  const bool roller_task_ok = roller_ok && roller.startIoTask(',
                    '  const bool roller_ok = roller.begin();\n  const bool roller_task_ok = roller_ok && roller.startIoTask(')
main = main.replace('  Serial.printf("Roller485: %s task=%s core=%u priority=%u\\n",\n',
                    '  Serial.printf("Roller485: %s task_ready=%s core=%u priority=%u\\n",\n')
write('src/main.cpp', main)

# Web status exposes readiness separately from task existence.
web = read('src/web_ui.cpp')
old = '  json += ",\\"roller_io_task_running\\":" + String(roller.io_task_running ? "true" : "false");\n'
if old in web:
    web = web.replace(old, old +
        '  json += ",\\"roller_io_task_ready\\":" + String(roller.io_task_ready ? "true" : "false");\n'
        '  json += ",\\"roller_io_task_init_failed\\":" + String(roller.io_task_init_failed ? "true" : "false");\n', 1)
write('src/web_ui.cpp', web)

# Site identity.
for path in ('site/index.html',):
    s = read(path).replace('V46i', 'V46j')
    s = s.replace('v46i_mekf_400hz_dual_core_roller_queue_20260913',
                  'v46j_mekf_dual_core_roller_ready_20260913')
    write(path, s)
manifest = read('site/manifest.json').replace('V46i', 'V46j').replace('0.46.8', '0.46.9')
write('site/manifest.json', manifest)

# Update existing guards to the new identity and queue semantics.
for path in ('tools/test_v46g_highrate_source_guards.py', 'tools/test_v46_motor_validation_source_guards.py'):
    s = read(path).replace('v46i_mekf_400hz_dual_core_roller_queue_20260913',
                           'v46j_mekf_dual_core_roller_ready_20260913')
    s = s.replace('V46i', 'V46j')
    write(path, s)

motor = read('tools/test_v46_motor_validation_source_guards.py')
motor = motor.replace('assert "if (command_mA_ == 0 && telemetry_.output_raw == 0) return true;" in roller\n',
                      'assert "if (requested_current_mA_ == 0) return true;" in roller\n')
motor = motor.replace('assert "telemetry_.output_raw = current_mA == 0 ? 0 : 1;" in roller\n',
                      'assert "telemetry_.output_raw = cmd.current_mA == 0 ? 0 : 1;" in roller\n')
write('tools/test_v46_motor_validation_source_guards.py', motor)

task_guard = r'''from pathlib import Path

config = Path('src/config.h').read_text(encoding='utf-8')
main = Path('src/main.cpp').read_text(encoding='utf-8')
roller_h = Path('src/roller485_manager.h').read_text(encoding='utf-8')
roller_cpp = Path('src/roller485_manager.cpp').read_text(encoding='utf-8')
runner = Path('src/experiment_runner.cpp').read_text(encoding='utf-8')
web = Path('src/web_ui.cpp').read_text(encoding='utf-8')
manifest = Path('site/manifest.json').read_text(encoding='utf-8')

assert 'v46j_mekf_dual_core_roller_ready_20260913' in config
assert 'ROLLER_IO_TASK_CORE = 0' in config
assert 'ROLLER_IO_TASK_PRIORITY = 4' in config
assert 'ROLLER_IO_TASK_STACK_BYTES = 6144UL' in config
assert 'AtomS3R V46j MEKF dual-core motor validation' in main
assert 'roller.startIoTask(' in main
assert 'roller.stop();' not in main
assert 'roller.update();' not in main
assert 'if (!runner.running())' in main
assert 'taskYIELD();' in main

# Core 0 owns Wire from initialization onward; control core only queues commands.
assert 'bool Roller485Manager::initializeIoOwner()' in roller_cpp
assert 'Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);' in roller_cpp
assert roller_cpp.index('Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);') > roller_cpp.index('bool Roller485Manager::initializeIoOwner()')
begin_region = roller_cpp[roller_cpp.index('bool Roller485Manager::begin()'):roller_cpp.index('bool Roller485Manager::startIoTask')]
assert 'Wire.' not in begin_region
stop_region = roller_cpp[roller_cpp.index('bool Roller485Manager::stop()'):roller_cpp.index('bool Roller485Manager::applyCurrentMa')]
assert 'Wire.' not in stop_region
assert 'No synchronous Wire fallback' in stop_region

# Creation is not readiness: startIoTask waits for Core 0 init completion.
for token in (
    'io_task_ready_', 'io_task_init_failed_', 'roller_io_task_start_timeout',
    'roller_io_task_not_ready', 'initializeIoOwner()', 'vTaskDelay(pdMS_TO_TICKS(1))',
    'xTaskCreatePinnedToCore', 'xQueueCreate(4, sizeof(RollerCommand))',
    'xQueueSend(command_queue_', 'xQueueReset(command_queue_)',
    'xTaskNotifyGive(io_task_handle_)', 'ulTaskNotifyTake',
    'CURRENT_AUDIT_FAST_READ_PERIOD_US', 'telemetrySnapshot()',
    'portENTER_CRITICAL(&telemetry_mux_)'):
    assert token in roller_cpp or token in roller_h, token

assert 'Control-core API: queue a desired current; no Roller I2C is performed here.' in roller_h
assert 'roller_->telemetry()' not in runner
assert 'roller_->telemetry()' not in web
assert runner.count('telemetrySnapshot()') >= 2
assert 'roller_io_task_running' in web
assert 'roller_io_task_ready' in web
assert 'roller_io_task_init_failed' in web
assert 'roller_command_latency_max_us' in web
assert 'AtomS3R V46j MEKF Motor Validation' in manifest
assert '"version": "0.46.9"' in manifest
print('V46j dual-core Roller READY guards passed')
'''
write('tools/test_v46i_task_split_source_guards.py', task_guard)

print('V46j Roller task readiness patch applied')
