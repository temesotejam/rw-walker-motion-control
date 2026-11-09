from pathlib import Path
R=Path(__file__).resolve().parents[1]
main=(R/'src/main.cpp').read_text(); web=(R/'src/web_ui.cpp').read_text(); worker=(R/'src/run_control_worker.h').read_text()
step=main[main.index('static bool runControlStep(void*) {'):main.index('void loop() {')]
assert 'web.' not in step and 'server.' not in step and 'M5.update()' not in step
for token in ('run_control.takeStopRequest()', 'runner.requestEmergencyStop("web_estop")', 'imu.update()', 'checkAcquisitionHealth();','runner.update();','run_control.recordStep('): assert token in step,token
assert step.index('checkAcquisitionHealth();')<step.index('runner.update();')
loop=main[main.index('void loop() {'):]
active=loop[loop.index('if (run_control.active()) {'):loop.index('// Idle ownership')]
assert 'web.update();' in active and 'return;' in active
for token in ('runner.','imu.','logger.','M5.'): assert token not in active, token
assert loop.index('web.update();',loop.index('// Idle ownership'))<loop.index('if (runner.running())')<loop.index('run_control.start()')
for name in ('handleStartEnergyControlAutonomous','handleStartQIdent','handleStart','handleStartZeroCross','handleStartIdentification','handleStartControl','handleZero','handleClear','handleRwLog','handleRoot'):
    body=web.split('void WebUi::'+name+'() {',1)[1].split('\nvoid WebUi::',1)[0]
    assert 'run_control.active()' in body,name
    gate=body.index('run_control.active()')
    assert gate<min([i for word in ('runner_->','imu_->','logger_->') if (i:=body.find(word))>=0]+[len(body)]),name
status=web.split('void WebUi::handleStatus() {',1)[1].split('\nvoid WebUi::',1)[0]
active_status=status[:status.index('statusJson()')]
assert 'run_control.snapshot()' in active_status
for word in ('runner_->','imu_->','logger_->'): assert word not in active_status
stop=web.split('void WebUi::handleStop() {',1)[1].split('\nvoid WebUi::',1)[0]
assert stop.index('run_control.requestStop()')<stop.index('runner_->requestEmergencyStop')
assert 'return;' in stop[:stop.index('runner_->requestEmergencyStop')]
assert 'server_->enableDelay(false)' in web
assert 'kCore = 1' in worker and 'kPriority = 4' in worker
assert 'ulTaskNotifyTake(pdTRUE, portMAX_DELAY)' in worker
assert 'if (!still_running) active_ = false;' in worker
assert 'capture_(context_, next);' in worker
assert 'v46p_control_worker' in (R/'src/psram_logger.cpp').read_text()
print('V46p exclusive run ownership, snapshot-only HTTP, bounded stop mailbox and unchanged startup boundary PASS')
