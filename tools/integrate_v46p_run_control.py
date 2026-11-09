#!/usr/bin/env python3
"""One-time V46o -> V46p integration on the review branch.
All controller, motor, estimator and physical safety files stay unchanged.
Normal main build/Pages never run this script or alter their source checkout.
"""
from pathlib import Path
import hashlib
ROOT = Path(__file__).resolve().parents[1]
def read(path): return (ROOT/path).read_text(encoding='utf-8')
def write(path, text): (ROOT/path).write_text(text, encoding='utf-8')
main = read('src/main.cpp')
if 'static bool runControlStep(void*) {' in main:
    raise SystemExit('V46p integration already applied; build committed source instead')
assert hashlib.sha256(main.encode()).hexdigest() == 'db51bfb3ab8aeb60fcf43f88ab15cd3855d601b214f1faffd4470bd4fad2c566'
protected = ('src/experiment_runner.cpp','src/experiment_runner.h','src/config.h','src/roller485_manager.cpp','src/roller485_manager.h','src/mekf6.cpp','src/mekf6.hpp','src/upright_pose_guide.h','src/log_types.h','platformio.ini')
before = {p:(ROOT/p).read_bytes() for p in protected}
main = main.replace('#include "web_ui.h"', '#include "web_ui.h"\n#include "run_control_worker.h"')
main = main.replace('WebUi web;', 'WebUi web;\nRunControlWorker run_control;\n\nstatic bool runControlStep(void*);\nstatic void captureRunState(void*, RunControlSnapshot&);')
main = main.replace('static_assert(kConsumerPriority < 6, "BMI270 reader must preempt the consumer");', 'static_assert(kConsumerPriority < RunControlWorker::kPriority, "Run control must preempt HTTP");\nstatic_assert(RunControlWorker::kPriority < 6, "BMI270 reader must preempt run control");')
main = main.replace('  runner.begin(logger, imu, roller);', '  runner.begin(logger, imu, roller);\n  const bool control_task_ok = run_control.begin(runControlStep, captureRunState, nullptr);\n  Serial.printf("Run control worker: %s core=1 priority=4; HTTP core=1 priority=2\\n",\n                control_task_ok ? "OK" : "FAILED");')
main = main[:main.index('void loop() {')] + r'''static void captureRunState(void*, RunControlSnapshot& out) {
  const auto& st = runner.status();
  out.running = runner.running();
  out.state_id = static_cast<uint8_t>(st.state);
  out.run_id = st.run_id;
  out.motor_cmd_mA = st.motor_cmd_mA;
  out.actual_current_mA = st.roller_actual_current_mA;
  out.remaining_ms = st.remaining_ms;
  snprintf(out.state_name, sizeof(out.state_name), "%s", runner.stateName());
  snprintf(out.last_error, sizeof(out.last_error), "%s", st.last_error ? st.last_error : "");
}

static bool runControlStep(void*) {
  const uint32_t loop_start_us = micros();
  updateAcquisitionContext();
  if (run_control.takeStopRequest()) {
    runner.requestEmergencyStop("web_estop");
    updateAcquisitionContext();
    run_control.recordStep(loop_start_us, 0, 0, static_cast<uint32_t>(micros() - loop_start_us));
    return false;
  }
  runner.serviceFast();
  runner.updateImuDynamicBetaContext();
  const bool v46k_timing_probe_active = runner.energyControlAutonomousMode() && runner.running();
  const uint32_t imu_t0_us = micros();
  imu.update();
  checkAcquisitionHealth();
  const uint32_t imu_update_us = static_cast<uint32_t>(micros() - imu_t0_us);
  const uint32_t runner_t0_us = micros();
  runner.update();
  const uint32_t runner_update_us = static_cast<uint32_t>(micros() - runner_t0_us);
  const uint32_t path_us = static_cast<uint32_t>(micros() - loop_start_us);
  if (v46k_timing_probe_active) runner.recordTimingProbeLoop(imu_update_us, runner_update_us, path_us);
  runner.setLoopDt(path_us);
  updateAcquisitionContext();
  run_control.recordStep(loop_start_us, imu_update_us, runner_update_us, path_us);
  return runner.running();
}

void loop() {
  // While a run is active, this lower-priority Arduino task owns only HTTP.
  // Never put a mutex around handleClient and the controller: that would
  // reintroduce network waits into the IMU-consumer deadline.
  if (run_control.active()) {
    web.update();
    delay(1);
    return;
  }

  // Idle ownership is exclusive again after the worker's final snapshot.
  const uint32_t loop_start_us = micros();
  updateAcquisitionContext();
  runner.serviceFast();
  runner.updateImuDynamicBetaContext();
  imu.update();
  checkAcquisitionHealth();
  runner.update();
  if (!runner.running()) {
    M5.update();
    updateStartupPoseGuide();
  }
  runner.setLoopDt(static_cast<uint32_t>(micros() - loop_start_us));
  web.update();

  // Establish the V46o boundary after the Start HTTP response. Then transfer
  // ownership exactly once; never touch the live controller after start().
  if (runner.running()) {
    updateAcquisitionContext();
    if (!run_control.start()) {
      runner.requestEmergencyStop("run_control_worker_not_ready");
      updateAcquisitionContext();
    }
  }
  taskYIELD();
}
'''
write('src/main.cpp', main)
web = read('src/web_ui.cpp').replace('#include "upright_pose_guide.h"', '#include "upright_pose_guide.h"\n#include "run_control_worker.h"\nextern RunControlWorker run_control;')
methods = ['handleStartPassive','handleStartEnergyControlV0','handleStartEnergyControlAutonomous','handleSetEnergyControlAutonomousTarget','handleStartQIdent','handleStart','handleStartZeroCross','handleStartIdentification','handleStartControl','handleZero','handleCurrentRollZero','handleSetCurrentRollTarget','handleSetQ1ShadowTargetPeakAbs','handleClear','handleSettings','handleRwLog','handleRoot']
for name in methods:
    marker = 'void WebUi::'+name+'() {'
    assert web.count(marker)==1, name
    web=web.replace(marker, marker+'\n  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }', 1)
web=web.replace('  server_->begin();','  server_->enableDelay(false);  // Empty HTTP polls must not add sleeps to idle acquisition.\n  server_->begin();',1)
web=web.replace('if (runner_->running()) { server_->send(409, "text/plain", "read_after_run"); return; }','if (run_control.active() || runner_->running()) { server_->send(409, "text/plain", "read_after_run"); return; }')
a=web.index('void WebUi::handleStatus() {');b=web.index('void WebUi::handleStartPassive() {')
web=web[:a]+r'''void WebUi::handleStatus() {
  if (run_control.active()) {
    // Copy only immutable POD status; do not read runner/logger/imu.reading
    // while the higher-priority worker owns them. No network I/O in a lock.
    const RunControlSnapshot st = run_control.snapshot();
    char body[192];
    snprintf(body, sizeof(body),
        "{\"running\":%s,\"state\":\"%s\",\"motor_cmd_mA\":%d,\"roller_actual_current_mA\":%d,\"remaining_ms\":%lu}",
        st.running ? "true" : "false", st.state_name,
        static_cast<int>(st.motor_cmd_mA), static_cast<int>(st.actual_current_mA),
        static_cast<unsigned long>(st.remaining_ms));
    server_->send(200, "application/json", body);
    return;
  }
  server_->send(200, "application/json", statusJson());
}

'''+web[b:]
web=web.replace('void WebUi::handleStop() {\n  runner_->requestEmergencyStop("web_estop");','void WebUi::handleStop() {\n  if (run_control.requestStop()) {\n    server_->send(202, "text/plain", "stop_requested");\n    return;\n  }\n  runner_->requestEmergencyStop("web_estop");')
web=web.replace('  // Refresh from the idle mailbox before the unchanged physical start gate.','  if (!run_control.ready()) { server_->send(503, "text/plain", "run_control_worker_not_ready"); return; }\n  // Refresh from the idle mailbox before the unchanged physical start gate.')
for name in ['handleStartPassive','handleStartEnergyControlV0']:
    marker='void WebUi::'+name+'() {\n'
    web=web.replace(marker,marker+'  if (!run_control.ready()) { server_->send(503, "text/plain", "run_control_worker_not_ready"); return; }\n',1)
# The reviewed archive had one additional final blank line; preserve its exact bytes.
web=web.rstrip('\n')+'\n\n'
write('src/web_ui.cpp',web)
logger=read('src/psram_logger.cpp').replace('extern ImuManager imu;','extern ImuManager imu;\n#include "run_control_worker.h"\nextern RunControlWorker run_control;')
marker='  json += "\\\"v46n_imu_acquisition\\\":" + imu.acquisitionDiagnosticsJson() + ",";'
assert logger.count(marker)==1
logger=logger.replace(marker,marker+'\n  json += "\\\"v46p_control_worker\\\":" + run_control.diagnosticsJson() + ",";')
write('src/psram_logger.cpp',logger)
for path in ['src/main.cpp','src/web_ui.cpp','src/imu_manager.cpp','site/index.html','site/manifest.json','tools/test_v46n_acquisition_source.py','tools/test_v46i_task_split_source_guards.py','tools/test_v46k_timing_probe_source_guards.py','tools/test_v46l_fast_solver_shadow.py']:
    write(path,read(path).replace('V46o','V46p').replace('0.46.14','0.46.15').replace('v46o_startup_boundary_20260914','v46p_run_control_worker_20260914'))
guard=read('tools/test_v46n_acquisition_source.py').replace("status.index('if (runner_->running())')","status.index('if (run_control.active())')")
guard=guard.replace("assert 'char body[192]' in status", "assert 'char body[192]' in status\nassert 'run_control.snapshot()' in status")
write('tools/test_v46n_acquisition_source.py',guard)
# Correct the pre-existing duplicated kana in the published V46o page.
page=read('site/index.html').replace('全取得サンプルルの','全取得サンプルの')
marker='    <section class="panel important">'
note='''    <section class="panel important">
      <h2>V46p：開始同期・制御もHTTPから独立化</h2>
      <p>V46o実機では待機データ4件の除外後、START_SYNC中に14.969 msの受け渡し遅延で停止しました。
      初期化と立位確認は成功しています。HTTPと制御の同一タスク実行をやめました。</p>
      <p>測定中：Core 1のIMU取得は優先度6、MEKF/V7/同期LED/ログは専用タスク優先度4、
      HTTPは優先度2です。RollerはCore 0の優先度4を維持します。
      状態表示はスナップショット、停止ボタンは専用の停止要求を通し、Webから制御状態を直接変更しません。</p>
      <p>開始前データの境界、10 ms遅延停止、32件キュー、初期化確認、立位判定と実モータ出力条件は維持します。
      RWLOGにはSTART_SYNCからの制御周期・処理時間・停止要求の診断を追加しました。
      この変更による実機での改善は次のRunで確認が必要です。</p>
    </section>

'''
page=page.replace(marker,note+marker,1).replace('Core 1 / 優先度2<br>取得タスクが制御計算へ割り込めます。キューが空なら最大1 tick待機。','Core 1 / 測定中は優先度4<br>HTTPは別タスク優先度2。取得タスクが制御計算へ割り込めます。').replace('<strong>MEKF・制御・Web</strong>','<strong>MEKF・制御／Web分離</strong>')
write('site/index.html',page)
for path, data in before.items(): assert (ROOT/path).read_bytes()==data, path
expected={'src/main.cpp':'2f1331932f871176449605a638e27e0f2b7c11857115d39c39f2b03f259ec022','src/web_ui.cpp':'9f46bc1293d2c99f1a2818593a6f546d41c54d906eef90de4ea8fe603877187b','src/psram_logger.cpp':'2e16bc7fecb3ef975ab05a001408e0cbfa869d04b1c60fcf334d0ccbe6c2428e','src/imu_manager.cpp':'37d38663c037c4e7d77e45fcd9777dfa1cf89faea29803283488956a8936e6ed','site/index.html':'0dfe017d98e8e52bddefe88198c6f6918b5dd0498c23aa436753c970bd68333f'}
for path, digest in expected.items(): assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest,path
print('V46p source matches the locally tested implementation; 10 physical/controller files unchanged')
