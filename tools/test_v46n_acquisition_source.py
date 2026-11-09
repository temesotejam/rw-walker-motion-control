#!/usr/bin/env python3
from pathlib import Path
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = '50c9e049ab4c2dc73852aa6ccb2fd988ad18852c'
def text(path): return (ROOT / path).read_text(encoding='utf-8')
def baseline(path): return subprocess.check_output(['git', 'show', BASE + ':' + path], cwd=ROOT)

# Whole files, not just guard keywords: retain all motor authority and calibration.
for path in ('src/config.h', 'src/roller485_manager.cpp', 'src/roller485_manager.h',
             'src/mekf6.cpp', 'src/mekf6.hpp', 'src/upright_pose_guide.h',
             'src/log_types.h', 'platformio.ini'):
    assert (ROOT / path).read_bytes() == baseline(path), path + ' changed'

runner = text('src/experiment_runner.cpp')
old = baseline('src/experiment_runner.cpp').decode('utf-8').replace('\r\n', '\n')
anchor = 'r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start) {'
replacement = 'r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start &&\n        static_cast<int32_t>(r.last_gyro_update_us - timing_probe_event_.pulse_start_us) >= 0) {'
assert old.count(anchor) == 1
assert runner == old.replace(anchor, replacement, 1), 'controller changed beyond observation timestamp guard'

imu = text('src/imu_manager.cpp')
h = text('src/imu_manager.h')
main = text('src/main.cpp')
consumer = imu[imu.index('void ImuManager::update()'):imu.index('bool ImuManager::acquisitionHealthy()')]
callback = imu[imu.index('void ImuManager::timerCallback'):imu.index('void ImuManager::taskEntry')]
producer = imu[imu.index('void ImuManager::captureSensor()'):imu.index('void ImuManager::latchFault')]
assert imu.count('M5.Imu.update()') == 1
assert 'M5.Imu.update()' in producer
assert 'M5.Imu' not in consumer and 'M5.Imu' not in callback
assert 'xQueueReceive' in consumer and 'xTaskNotifyGive' in callback
assert 'portMAX_DELAY' not in consumer
assert 'kReaderCore = 1' in h and 'kReaderPriority = 6' in h
assert 'xTaskCreatePinnedToCore' in imu and 'xQueueCreateStatic' in imu
assert 'Config::IMU_POLL_PERIOD_US' in imu
assert 'internal_i2c_port_ != 1' in imu
assert 'kMaximumDeliveryAgeUs = 10000' in h
assert 'sequential && age_us > kMaximumDeliveryAgeUs' in consumer
assert 'imu_acquisition_queue_overflow' in imu
assert 'fault_ = false' not in imu, 'a latched runtime fault cannot auto-rearm'
assert 'imu_acquisition_overflow_backlog_or_stale' in main
assert main.index('checkAcquisitionHealth();') < main.index('runner.update();')
assert 'roller.startIoTask(' in main and 'roller.stop();' not in main
for forbidden in ('Serial.', 'String ', 'runner.', 'roller.', 'Wire.'):
    assert forbidden not in producer, forbidden
assert 'beta_context_' not in producer
assert 'capture_.' not in consumer
assert 'v46n_imu_acquisition' in text('src/psram_logger.cpp')
assert 'acquisitionDiagnosticsJson()' in text('src/web_ui.cpp')
manifest = json.loads(text('site/manifest.json'))
assert manifest['version'] == '0.46.16'
assert 'V46q' in manifest['name']
assert 'v46q_lightweight_acquisition_20260914' in text('site/index.html')

# Scheduling is explicit. Both threads block instead of continuously spinning.
assert 'kConsumerPriority = 2;' in main
assert 'vTaskPrioritySet(nullptr, kConsumerPriority);' in main
assert 'xQueueReceive(sample_queue_, &next, 1)' in consumer
assert 'static_assert(configTICK_RATE_HZ == 1000' in imu
reader_loop = imu[imu.index('void ImuManager::acquisitionLoop()'):imu.index('void ImuManager::captureSensor()')]
assert 'ulTaskNotifyTake(pdTRUE, portMAX_DELAY)' in reader_loop
assert 'if (elapsed >= Config::IMU_POLL_PERIOD_US) vTaskDelay(1);' in reader_loop

# The reader may publish a new millisecond timestamp after a caller captured now.
# Compare the sensor snapshot with a clock read after it, keeping the stale limit.
freshness = imu[imu.index('bool ImuManager::stale('):imu.index('void ImuManager::zeroPitch()')]
assert freshness.index('const uint32_t stamp =') < freshness.index('const uint32_t check_now_ms = millis();')
assert 'static_cast<uint32_t>(check_now_ms - stamp_ms) > Config::IMU_STALE_LIMIT_MS' in freshness
assert '!acquisitionHealthy() || stamp == 0 ||' in freshness
assert 'reading_.accel_fresh = reading_.accel_sequence != previous_accel_sequence;' in consumer

print('V46n exclusive ownership, bounded delivery and exact preserved control baseline PASS')
print('Reader priority 6 > consumer priority 2; bounded waits and coherent freshness PASS')

# V46p: preserve runtime stops; separate pre-start idle history instead.
assert 'boundary_.enter(now_us, latest_capture_sequence_)' in imu
assert 'sequential && !sequential_' in imu
assert 'boundary_.accepts(next.gyro_sequence)' in consumer
assert consumer.index('boundary_.accepts') < consumer.index('age_us > kMaximumDeliveryAgeUs')
assert 'xQueueReset' not in imu
assert 'cfg.internal_imu = false' in main
assert 'M5.Imu.begin(&M5.In_I2C, M5.getBoard())' in imu
assert 'attempt < 5' in imu and 'init_valid_accel_ >= 8' in imu
assert 'first_fault' in imu and 'start_sync_age_max_us' in imu
assert 'startupDiagnosticsJson()' in text('src/web_ui.cpp')
web = text('src/web_ui.cpp')
status = web[web.index('void WebUi::handleStatus()'):web.index('void WebUi::handleStartPassive()')]
assert status.index('if (run_control.active())') < status.index('statusJson()')
assert 'char body[192]' in status
assert 'run_control.snapshot()' in status
assert '41000' not in web and 'if(displayFrozen||refreshInFlight)' not in web
print('V46p startup boundary, cold-start validation, first-fault diagnostics and lean status PASS')
