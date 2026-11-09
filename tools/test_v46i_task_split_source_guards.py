from pathlib import Path

config = Path('src/config.h').read_text(encoding='utf-8')
main = Path('src/main.cpp').read_text(encoding='utf-8')
roller_h = Path('src/roller485_manager.h').read_text(encoding='utf-8')
roller_cpp = Path('src/roller485_manager.cpp').read_text(encoding='utf-8')
runner = Path('src/experiment_runner.cpp').read_text(encoding='utf-8')
web = Path('src/web_ui.cpp').read_text(encoding='utf-8')
manifest = Path('site/manifest.json').read_text(encoding='utf-8')

assert ('v46l_fast_solver_shadow_20260914' in config or
        'v46aj_fixed_3ms_compensation_20260920' in config)
assert 'ROLLER_IO_TASK_CORE = 0' in config
assert 'ROLLER_IO_TASK_PRIORITY = 4' in config
assert 'ROLLER_IO_TASK_STACK_BYTES = 6144UL' in config
assert 'ROLLER_IO_RETRY_PERIOD_MS = 100UL' in config
assert 'ROLLER_IO_RECOVERY_ERROR_LIMIT = 3' in config
assert 'AtomS3R V46l MEKF dual-core motor validation' in main
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

# Creation starts a persistent Core 0 owner; readiness may arrive after retries.
for token in (
    'io_task_ready_', 'io_task_init_failed_',
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
# The simplified V46ak UI intentionally exposes only the Roller health/current/battery
# fields it renders; detailed Core-0 diagnostics remain in firmware/RWLOG, not status.json.
for token in ('roller_io_task_running', 'roller_io_task_ready', 'roller_io_task_init_failed',
              'roller_io_init_attempt_count', 'roller_io_recovery_count',
              'roller_command_latency_max_us'):
    assert token not in web, token
for token in ('roller_ok', 'roller_actual_current_mA', 'battery_mV'):
    assert token in web, token
assert ('AtomS3R V46q MEKF Motor Validation' in manifest or
        'AtomS3R V46aj Fast Solver Motor Validation' in manifest or
        'AtomS3R V46ak Pre-Input State Observation' in manifest or
        'AtomS3R V46al-R2 Previous-Peak Active Control' in manifest)
assert ('"version": "0.46.16"' in manifest or '"version": "0.46.35"' in manifest or
        '"version": "0.46.36"' in manifest or '"version": "0.46.42"' in manifest)
print('V46l/V46s dual-core Roller READY guards passed')

# Initialization/recovery must be self-healing, not one-shot.
assert 'for (;;)' in roller_cpp
assert 'initializeIoOwner()' in roller_cpp
assert 'ROLLER_IO_RETRY_PERIOD_MS' in roller_cpp
assert 'ROLLER_IO_RECOVERY_ERROR_LIMIT' in roller_cpp
assert 'io_recovery_count_' in roller_cpp
assert 'vTaskDelete(nullptr)' not in roller_cpp
assert 'roller_io_task_start_timeout' not in roller_cpp
assert 'return io_task_running_;' in roller_cpp
