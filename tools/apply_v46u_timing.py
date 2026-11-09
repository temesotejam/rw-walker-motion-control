#!/usr/bin/env python3
"""Apply reviewed timing diagnostics to the exact V46t baseline.
No estimator/controller arithmetic or motor output function is changed.
"""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[1]
DELTAS=[]

def change(path,old,new):
    p=ROOT/path;t=p.read_text()
    if t.count(old)!=1:raise ValueError(f'{path}: expected one match: {old[:65]}')
    DELTAS.append({'path':path,'old':old,'new':new})
    p.write_text(t.replace(old,new,1))

def main():
    if hashlib.sha256((ROOT/'src/experiment_runner.cpp').read_bytes()).hexdigest()!='2af5934f994c9c435ca53f4ef65ec98a5a95a4bd3355a740c0f1001e290f8c0e':
        raise ValueError('V46t controller changed; review before applying')
    change('src/config.h','v46t_current_observation_20260915','v46u_timing_reader_20260915')
    change('src/imu_manager.cpp','#include "upright_pose_guide.h"','#include "upright_pose_guide.h"\n#include "bmi270_timing_reader.h"')
    change('src/imu_manager.cpp','  const uint32_t update_done_us = micros();','''  const uint32_t update_done_us = micros();
  const auto driver = bmi270_timing::lastRead();
  poll_observation_.driver_called = driver.called;
  poll_observation_.driver_status_us = driver.status_us;
  poll_observation_.driver_data_us = driver.data_us;
  poll_observation_.driver_data_bytes = driver.data_bytes;
  poll_observation_.driver_failures = driver.failures;''')
    change('src/imu_poll_profile.h','#include <type_traits>','#include <type_traits>\n#include "timing_deadline.h"')
    change('src/imu_poll_profile.h','  uint8_t mask = 0;','''  uint32_t driver_status_us = 0, driver_data_us = 0;
  uint16_t driver_data_bytes = 0;
  uint8_t driver_failures = 0;
  bool driver_called = false;
  uint8_t mask = 0;''')
    change('src/imu_poll_profile.h','  Stats stage[STAGES];','''  timing_deadline::Counter poll_work_deadline, gyro_interval_deadline;
  timing_deadline::Counter driver_status, driver_data;
  uint32_t driver_calls = 0, driver_failures = 0, driver_bytes = 0;
  Stats stage[STAGES];''')
    change('src/imu_poll_profile.h','    ++polls; if (o.fresh_gyro) ++gyro; if (!o.mask) ++no_data;','''    ++polls; if (o.fresh_gyro) ++gyro; if (!o.mask) ++no_data;
    poll_work_deadline.add(o.total_us, 1000);
    if (o.fresh_gyro) gyro_interval_deadline.add(o.dt_us, 2500);
    if (o.driver_called) {
      ++driver_calls; driver_failures += o.driver_failures;
      driver_bytes += o.driver_data_bytes;
      driver_status.add(o.driver_status_us, 1000);
      driver_data.add(o.driver_data_us, 1000);
    }''')
    change('src/imu_manager.cpp','  static const char* names[ImuPollProfile::STAGES] = {','''  s += ",\\\"v46u_timing\\\":{\\\"driver_calls\\\":" + String(p.driver_calls);
  s += ",\\\"driver_failures\\\":" + String(p.driver_failures);
  s += ",\\\"data_bytes\\\":" + String(p.driver_bytes);
  s += ",\\\"status_read_max_us\\\":" + String(p.driver_status.maximum);
  s += ",\\\"data_read_max_us\\\":" + String(p.driver_data.maximum);
  s += ",\\\"poll_work_budget_us\\\":1000,\\\"poll_work_count\\\":" + String(p.poll_work_deadline.count);
  s += ",\\\"poll_work_over_budget\\\":" + String(p.poll_work_deadline.over);
  s += ",\\\"host_gyro_interval_budget_us\\\":2500,\\\"host_gyro_interval_count\\\":" + String(p.gyro_interval_deadline.count);
  s += ",\\\"host_gyro_interval_over_budget\\\":" + String(p.gyro_interval_deadline.over);
  s += ",\\\"host_gyro_interval_max_us\\\":" + String(p.gyro_interval_deadline.maximum);
  s += ",\\\"sensor_clock_verified\\\":false,\\\"policy\\\":\\\"strict_greater_than;host_polling_not_FIFO;no_tolerance_hidden\\\"}";
  static const char* names[ImuPollProfile::STAGES] = {''')
    change('src/run_control_worker.h','#include <string.h>','#include <string.h>\n#include "timing_deadline.h"')
    change('src/run_control_worker.h','    Timing recent[16] = {};','''    timing_deadline::Counter sample_completion, runner_work;
    Timing recent[16] = {};''')
    change('src/run_control_worker.h','  String diagnosticsJson() const {','''  void recordSampleCompletion(bool measurement, bool fresh, uint32_t sample_us,
                              uint32_t done_us, uint32_t runner_us) {
    if (!measurement || !fresh) return;
    portENTER_CRITICAL(&mux_);
    audit_.sample_completion.add(static_cast<uint32_t>(done_us - sample_us), 2500);
    audit_.runner_work.add(runner_us, 2500);
    portEXIT_CRITICAL(&mux_);
  }
  String diagnosticsJson() const {''')
    change('src/run_control_worker.h','    json += ",\\\"recent_steps\\\":[";','''    json += ",\\\"v46u_deadline\\\":{\\\"budget_us\\\":2500,\\\"count\\\":" + String(a.sample_completion.count);
    json += ",\\\"over_budget\\\":" + String(a.sample_completion.over);
    json += ",\\\"max_us\\\":" + String(a.sample_completion.maximum);
    json += ",\\\"mean_us\\\":" + String(a.sample_completion.count ?
        static_cast<double>(a.sample_completion.sum) / a.sample_completion.count : 0.0, 3);
    json += ",\\\"runner_over_budget\\\":" + String(a.runner_work.over);
    json += ",\\\"runner_max_us\\\":" + String(a.runner_work.maximum);
    json += ",\\\"all_observed_within_budget\\\":" + String(a.sample_completion.passed() ? "true" : "false");
    json += ",\\\"scope\\\":\\\"RUNNING_fresh_gyro_only;host_acquisition_to_runner_return;not_sensor_capture_to_motor_apply\\\"}";
    json += ",\\\"recent_steps\\\":[";''')
    change('src/main.cpp','  const uint32_t runner_t0_us = micros();','''  const bool timing_measurement = runner.status().state == ExperimentState::RUNNING_BATCH_SWEEP;
  const bool timing_fresh = imu.reading().gyro_fresh;
  const uint32_t timing_sample_us = imu.reading().last_gyro_update_us;
  const uint32_t runner_t0_us = micros();''')
    change('src/main.cpp','  const uint32_t path_us = static_cast<uint32_t>(micros() - loop_start_us);','''  const uint32_t timing_done_us = micros();
  const uint32_t path_us = static_cast<uint32_t>(timing_done_us - loop_start_us);
  run_control.recordSampleCompletion(timing_measurement, timing_fresh, timing_sample_us,
                                    timing_done_us, runner_update_us);''')
    change('src/roller485_manager.h','#include <freertos/task.h>','#include <freertos/task.h>\n#include "timing_deadline.h"')
    change('src/roller485_manager.h','  bool roller_ok = false;','''  timing_deadline::Counter pulse_current_read_work, pulse_current_intervals;
  bool roller_ok = false;''')
    change('src/roller485_manager.cpp','  int32_t current_raw = 0;\n  ++telemetry_.current_sequence;','''  const uint32_t timing_start = micros();
  int32_t current_raw = 0;
  ++telemetry_.current_sequence;''')
    change('src/roller485_manager.cpp','    recordCurrentReadFailure(audit_sample);\n    return false;','''    recordCurrentReadFailure(audit_sample);
    if (audit_sample) telemetry_.pulse_current_read_work.add(
        static_cast<uint32_t>(micros() - timing_start), 2000);
    return false;''')
    change('src/roller485_manager.cpp','  recordFreshCurrent(current_raw, micros(), audit_sample);','''  if (audit_sample) telemetry_.pulse_current_read_work.add(
      static_cast<uint32_t>(micros() - timing_start), 2000);
  recordFreshCurrent(current_raw, micros(), audit_sample);''')
    change('src/roller485_manager.cpp','    const uint32_t dt_us = static_cast<uint32_t>(sample_time_us - current_audit_previous_sample_us_);','''    const uint32_t dt_us = static_cast<uint32_t>(sample_time_us - current_audit_previous_sample_us_);
    telemetry_.pulse_current_intervals.add(dt_us, 2000);''')
    change('src/psram_logger.cpp','extern RunControlWorker run_control;','extern RunControlWorker run_control;\n#include "roller485_manager.h"\nextern Roller485Manager roller;')
    change('src/psram_logger.cpp','  json += "\\\"v46p_control_worker\\\":" + run_control.diagnosticsJson() + ",";','''  json += "\\\"v46p_control_worker\\\":" + run_control.diagnosticsJson() + ",";
  {
    const auto t = roller.telemetrySnapshot();
    json += "\\\"v46u_current_timing\\\":{\\\"scope\\\":\\\"since_boot_pulse_audit_only_not_per_run\\\",\\\"budget_us\\\":2000";
    json += ",\\\"read_count\\\":" + String(t.pulse_current_read_work.count);
    json += ",\\\"read_over_budget\\\":" + String(t.pulse_current_read_work.over);
    json += ",\\\"read_max_us\\\":" + String(t.pulse_current_read_work.maximum);
    json += ",\\\"interval_count\\\":" + String(t.pulse_current_intervals.count);
    json += ",\\\"interval_over_budget\\\":" + String(t.pulse_current_intervals.over);
    json += ",\\\"interval_max_us\\\":" + String(t.pulse_current_intervals.maximum) + "},";
  }''')
    change('platformio.ini','[env:atoms3cam]','[env:atoms3cam]\nextra_scripts = pre:tools/patch_bmi270_timing.py')
    change('.gitignore','.pio/','.pio/\nbmi270-build-patch.json')
    (ROOT/'tools/v46u_timing_delta.json').write_text(json.dumps(DELTAS,indent=2)+'\n')
    for f in ['tools/replay_v46s_solver_audit.py','tools/test_v46s_solver_audit.py']:
        p=ROOT/f;t=p.read_text().replace(".replace('v46t_current_observation_20260915',", ".replace('v46u_timing_reader_20260915', 'v46t_current_observation_20260915').replace('v46t_current_observation_20260915',")
        if f.endswith('test_v46s_solver_audit.py'):
            t=t.replace('from pathlib import Path','from pathlib import Path\nfrom v46u_timing_contract import original_timing_file')
            t=t.replace("hashlib.sha256((ROOT/path).read_bytes()).hexdigest()", "hashlib.sha256(original_timing_file(path)).hexdigest()")
        p.write_text(t)
    for f in ['tools/test_v46r_fast_solver_control.py','tools/test_v46g_highrate_source_guards.py','tools/test_v46i_task_split_source_guards.py']:
        p=ROOT/f;t=p.read_text().replace('v46t_current_observation_20260915','v46u_timing_reader_20260915').replace('0.46.19','0.46.20').replace('V46t','V46u');p.write_text(t)
    for f in ['site/manifest.json','site/index.html']:
        p=ROOT/f;t=p.read_text().replace('V46t','V46u').replace('0.46.19','0.46.20');p.write_text(t)
    p=ROOT/'site/index.html';t=p.read_text();key='    <section class="panel important">'
    panel='''    <section class="panel important">
      <h2>V46u / 0.46.20：時間内処理の検証版</h2>
      <p>BMI270の未読フラグ（STATUS）で更新を確認し、gyro単独6バイト／accelとgyro同時12バイトを取得します。
      ODR 400/200 Hz、1 msポーリング、400 kHz I2C、推定器・角度・制御ゲイン・10 ms配信遅延ESTOPは変更しません。</p>
      <p>RUNNING中の全fresh gyroについて、取得時刻からrunner終了までの2.5 ms超過を件数付きで記録。
      1 ms読出し処理、ホスト取得間隔、2 ms電流監査も別々に集計します。
      完走や平均値だけをもって時間内と判定しません。実機での達成は未確認です。</p>
      <p>この版はFIFOではありません。ホスト間隔の厳密な2.5 ms一定や、センサ内部の全サンプル無欠落は保証しません。
      <a href="https://github.com/temesotejam/atoms3r-mekf-dynamic-validation/blob/main/docs/V46U_TIMING.md">時間の定義・変更範囲・判定条件</a></p>
    </section>
'''
    t=t.replace(key,panel+key,1);p.write_text(t)
    print('Applied V46u transport/timing diagnostics; controller and estimators unchanged')
if __name__=='__main__':main()
