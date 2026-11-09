#!/usr/bin/env python3
"""Exact V46p baseline plus source-ownership guards for V46q."""
from pathlib import Path
import hashlib
import json
import re
R = Path(__file__).resolve().parents[1]
PROTECTED = {'src/config.h': '9e3bdde27c2b5023e0a335d43651efe41219377143acd8e3faa10db4848b45aa', 'src/experiment_runner.cpp': '1cb21416176cfe9b7a73fff2ef64826bbc38cf997166532b5b3c91367b30ae9b', 'src/experiment_runner.h': '2e9d02f7c132b84d5cbce18045ee4507901142c1bd8a027948695151503ad8c1', 'src/roller485_manager.cpp': 'e1fcd50183f73f8ab748f4921ea5156f11c03a2bc8f3b25becde985f4ab49e9b', 'src/roller485_manager.h': '99f1b3a9316728a541a650be60bc629c6f8b0993c33811ef3757ee939e136810', 'src/mekf6.cpp': '150c05691e3d2b385c6a700fd0b227251da3cfdba42ad7fd1eaab22444cf9d07', 'src/mekf6.hpp': 'e6d9798ace7c42fa382704d4b7d6a3d0b0062c6dc2b706b5c6353b0ef6cd74e7', 'src/upright_pose_guide.h': '6e6a4247a4031687da6748eef9f203af5f94be9346aee33e10216787db5df872', 'src/log_types.h': '8f6279de5a311db286e840d9564b5fa91a7a3f517c5aa390026d322c4a1695f5', 'src/run_control_worker.h': '415e4f422d53d362ebdf54e339b0dc5d14a3794fb03faabcb6b4a7fa8f4abf5b', 'src/psram_logger.cpp': '2e16bc7fecb3ef975ab05a001408e0cbfa869d04b1c60fcf334d0ccbe6c2428e', 'src/imu_acquisition_audit.h': '68360c7af5a8886746359549594136d2f98acd407621feb08bee561e729917aa', 'src/imu_startup_boundary.h': '7808a8febdf1d90b8153fcf55999dd8bd9700f158b1cbd069dec03e828af5fcb', 'platformio.ini': 'e937cf69fa60276b3358c63a7b8dbe640c597ced28ab61f1fe7095b88c6c18fd'}
for path, expected in PROTECTED.items():
    assert hashlib.sha256((R/path).read_bytes()).hexdigest() == expected, path
imu=(R/'src/imu_manager.cpp').read_text()
producer=imu.split('void ImuManager::captureSensor()',1)[1].split('void ImuManager::latchFault',1)[0]
assert not re.search(r'\bnew\s+(?:[A-Za-z_]|\()', re.sub(r'//[^\n]*', '', producer))
consumer=imu.split('void ImuManager::update()',1)[1].split('bool ImuManager::acquisitionHealthy',1)[0]
callback=imu.split('void ImuManager::timerCallback',1)[1].split('void ImuManager::taskEntry',1)[0]
for x in ('sqrtf(', 'atan2f(', 'updateDerivedAccel(', 'String ', 'Serial.', 'delay('):
    assert x not in producer, x
for x in ('M5.Imu.', 'runner.', 'Serial.', 'String ', 'sqrtf(', 'atan2f('):
    assert x not in callback, x
assert 'updateDerivedAccel(next)' in consumer
assert consumer.index('age_us > kMaximumDeliveryAgeUs') < consumer.index('updateDerivedAccel(next)')
assert 'next.accel_sequence != previous_accel_sequence' in consumer
assert 'next.acc_norm_g = reading_.acc_norm_g' in consumer
assert 'if (capture_.accel_sequence != a_seq) updateDerivedAccel(capture_)' in imu
assert 'if (elapsed >= Config::IMU_POLL_PERIOD_US) vTaskDelay(1);' in imu
assert 'if (!active) return;' in imu
export=imu.split('String ImuManager::pollProfileJson()',1)[1]
assert export.index('if (active) return') < export.index('const auto& p = poll_profile_')
h=(R/'src/imu_poll_profile.h').read_text()
assert 'kBuckets = 320' in h and 'kBucketUs = 100000' in h
assert 'memset(static_cast<void*>(this)' in h
assert '*this = ImuPollProfile{}' not in h, 'Avoid large reader-stack temporaries'
assert 'v46q_poll_profile' in imu
assert 'kReaderPriority = 6' in (R/'src/imu_manager.h').read_text()
assert json.loads((R/'site/manifest.json').read_text())['version']=='0.46.16'
print('V46q exact V46p motor/control/MEKF baseline, lightweight ownership, bounded profiling PASS')
