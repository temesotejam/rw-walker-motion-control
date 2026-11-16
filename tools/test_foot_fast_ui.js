'use strict';
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
// Keep the established v19 fixture's DOM, status validation and rendering.
const prior=fs.readFileSync('tools/test_start_diagnostics_ui_v19.js','utf8').replace(/\r\n/g,'\n');
const marker='const checks=String.raw`';
assert(prior.includes(marker),'v19 UI fixture marker changed');
const prefix=prior.slice(0,prior.indexOf(marker));
const checks=String.raw`
state.diagnostic_build_id='walking-foot-fast-v20';
Object.assign(state.foot,{target_fps:25,fps:22.4,pipeline_revision:'sparse_cached_rows_double_buffer_v20',
  frame_age_ms:48.1,ready_age_ms:35.8,driver_delivery_ms:8.8,frame_to_ready_ms:12.3,
  recognition_us:3500,recognition_max_us:5700,row_copy_us:180,row_cache_used:true,row_cache_ready:true,control_poll_ms:2});
render();
assert.match(element('work-title').textContent,/v20.*足角度低遅延版/);
assert.match(element('work-description').textContent,/制御方式はv19を継承/);
assert.match(element('work-description').textContent,/実ZEROクロスで主入力を再計算/);
assert.match(element('work-description').textContent,/合わせて半周期80ms/);
assert.match(element('work-description').textContent,/モデル支持.*ZEROで再評価/);
assert.match(element('work-description').textContent,/前後の持続逸脱/);
assert.match(element('work-description').textContent,/最初はモータを開始せず/);
assert.match(element('work-description').textContent,/目標25fps/);
assert.match(element('work-description').textContent,/driver基準.*露光時刻.*未校正/);
assert.strictEqual(element('fps').textContent,'22.4 / 25 fps');
assert.match(element('foot-status').textContent,/更新 22\.4 fps（目標 25）/);
assert.match(element('foot-status').textContent,/認識 3\.5 ms（最大 5\.7）/);
assert.match(element('foot-status').textContent,/driverフレーム→ready 12\.3 ms/);
assert.match(element('foot-status').textContent,/許容範囲内.*3\.0°の外挿/);
assert.match(element('guide').textContent,/最初はモータを開始せず.*足角度と画像の更新/);
assert.match(element('work-start-status').textContent,/最初はモータを開始せず/);
assert.strictEqual(element('work-start').disabled,false);
assert.strictEqual(state.motor_mA,0);assert.strictEqual(state.command.submitted,0);
for(const fn of ['walkingEnergyTrial','walkingTransferTrial','walkingAdmissionTrial','walkingFootFastTrial'])
  assert.strictEqual(vm.runInContext(fn+'(state)',context),true,fn);
assert.strictEqual(vm.runInContext('walkingFootFastTrial({diagnostic_build_id:"walking-foot-fast-v200"})',context),false);
assert.strictEqual(vm.runInContext('walkingFootFastTrial({diagnostic_build_id:"walking-foot-fast-v20-test"})',context),true);
assert.strictEqual(vm.runInContext('walkingFootFastTrial(null)',context),false);
const diagnostic=JSON.parse(element('diagnostic-view').textContent).foot;
assert.strictEqual(diagnostic.driver_delivery_ms,8.8);assert.strictEqual(diagnostic.frame_age_ms,48.1);
assert.strictEqual(diagnostic.ready_age_ms,35.8);assert.strictEqual(diagnostic.row_copy_us,180);
assert.strictEqual(diagnostic.control_poll_ms,2);assert.strictEqual(diagnostic.row_cache_ready,true);
vm.runInContext('lastSeen=Date.now()',context);
assert.strictEqual(vm.runInContext('statusPollDelayMs()',context),100);
for(const busy of ['offlineMode','transferRunning','previewRunning','poseRunning','commandInFlight']){
  vm.runInContext(busy+'=true',context);
  assert.strictEqual(vm.runInContext('statusPollDelayMs()',context),800,busy);
  vm.runInContext(busy+'=false',context);
}
state.running=true;assert.strictEqual(vm.runInContext('statusPollDelayMs()',context),800);state.running=false;
state.state='ESTOP';assert.strictEqual(vm.runInContext('statusPollDelayMs()',context),800);state.state='READY_TO_MEASURE';
vm.runInContext('lastSeen=0',context);assert.strictEqual(vm.runInContext('statusPollDelayMs()',context),800);
vm.runInContext('lastSeen=Date.now()',context);
state.diagnostic_build_id='walking-energy-admission-v19';
assert.strictEqual(vm.runInContext('statusPollDelayMs()',context),800);
state.diagnostic_build_id='walking-foot-fast-v20';

state.foot_control_ready=false;state.foot_control_reason='left_out_of_range';
state.foot.left_control_range_ok=false;state.foot.left_extrapolation_deg=8;render();
assert.strictEqual(element('work-start').disabled,true);
assert.match(element('work-start-status').textContent,/許容5°を超え/);
state.foot_control_reason='left_not_detected';state.foot.left_valid=false;state.foot.left_reason='ambiguous';render();
assert.strictEqual(element('left').textContent,'—');
assert.match(element('foot-status').textContent,/候補を区別できません/);
assert.strictEqual(element('work-start').disabled,true);
Object.assign(state.foot,{left_valid:true,left_reason:'detected',left_control_range_ok:true,left_extrapolation_deg:3,
  recognition_us:null,recognition_max_us:undefined,frame_to_ready_ms:-1});render();
assert.match(element('foot-status').textContent,/認識 — ms（最大 —）/);
assert.match(element('foot-status').textContent,/driverフレーム→ready — ms/);
assert.doesNotMatch(element('foot-status').textContent,/NaN|undefined|null|-1\.0/);
state.foot.age_ms=600;state.foot_control_reason='frame_stale';render();
assert.strictEqual(element('right').textContent,'—');assert.strictEqual(element('left').textContent,'—');
assert.strictEqual(element('fps').textContent,'0.0 / 25 fps');
assert.match(element('foot-status').textContent,/画像更新なし/);
assert.match(element('work-start-status').textContent,/200ms/);

state.state='FINISHED';state.running=false;render();
assert.strictEqual(element('fps').textContent,'停止中');
assert.strictEqual(element('left').textContent,'-0.50°');
assert.match(element('foot-status').textContent,/最終フレームを表示/);
assert.match(element('foot-status').textContent,/最終更新 22\.4 fps/);
state.state='RUNNING_BATCH_SWEEP';state.running=true;state.foot.age_ms=70;
state.foot_control_ready=true;state.foot_control_reason='ready';render();
assert.match(element('run-model').textContent,/前倒し候補のモデル支持/);
assert.match(element('run-brake').textContent,/合わせて半周期80ms/);
state.state='ESTOP';state.running=false;state.ready=false;state.last_error='fore_aft_outward_divergence';render();
assert.match(element('guide').textContent,/外向きに持続して増え.*入力を停止/);

state.state='READY_TO_MEASURE';state.ready=true;state.last_error='';state.foot.target_fps=null;render();
assert.strictEqual(element('fps').textContent,'22.4 / 15 fps');
for(const key of ['target_fps','pipeline_revision','frame_age_ms','ready_age_ms','driver_delivery_ms','frame_to_ready_ms',
                 'recognition_us','recognition_max_us','row_copy_us','row_cache_used','row_cache_ready','control_poll_ms'])delete state.foot[key];
for(const [id,title] of [['walking-energy-admission-v19',/v19/],['walking-energy-transfer-v18',/v18/],
                       ['walking-return-energy-v17',/v17/]]){
  state.diagnostic_build_id=id;render();assert.match(element('work-title').textContent,title);
  assert.doesNotMatch(element('work-description').textContent,/低遅延版|目標25fps|最初はモータを開始せず/);
  assert.doesNotMatch(element('foot-status').textContent,/認識 .* ms|driverフレーム→ready/);
  assert.strictEqual(element('fps').textContent,'22.4 / 15 fps');
  assert.match(element('work-start-status').textContent,/開始できます/);
}
assert.throws(()=>vm.runInContext('adoptStatus(null)',context),/不完全/);
vm.runInContext('latest=null;lastSeen=0;controls()',context);
assert.strictEqual(element('work-start').disabled,true);
assert.match(element('work-start-status').textContent,/通信/);
console.log('v20 UI: identity/v19 inheritance/target FPS/driver timing/preview first/5deg/stale/terminal/null/old versions PASS');
`;
const runner=prefix+'\nvm.runInNewContext(fixture+'+JSON.stringify(checks)+', {require,console,Buffer,Date,Map,DataView,Uint8Array,TextDecoder,setTimeout,clearTimeout,AbortController});';
vm.runInNewContext(runner,{require,console,Buffer,Date,Map,DataView,Uint8Array,TextDecoder,
  setTimeout,clearTimeout,AbortController},{filename:'test_foot_fast_ui.js'});
