'use strict';
// Reuse the v18 DOM/status fixture; the v19 checks run in a fresh VM.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const prior=fs.readFileSync('tools/test_start_diagnostics_ui.js','utf8');
const marker="state.diagnostic_build_id='walking-return-energy-v17-rangefix1'";
assert(prior.includes(marker),'v18 status fixture marker changed');
const fixture=prior.slice(0,prior.indexOf(marker));
const checks=String.raw`
state.diagnostic_build_id='walking-energy-admission-v19';render();
assert.match(element('work-title').textContent,/v19/);
assert.match(element('work-description').textContent,/実ZEROクロスで主入力を再計算/);
assert.match(element('work-description').textContent,/合わせて半周期80ms/);
assert.match(element('work-description').textContent,/前倒し候補のモデル支持を確認し、範囲外はZEROで再評価/);
assert.match(element('work-description').textContent,/前後の持続逸脱では入力を停止/);
assert.match(element('work-description').textContent,/校正端から5°以内/);
assert.doesNotMatch(element('work-description').textContent,/同定済|転倒を防|数度以上.*拒否/);
assert.strictEqual(element('work-start').disabled,false);
assert.match(element('foot-status').textContent,/許容範囲内.*3\.0°の外挿/);
// The page continues to use the device's readiness decision, without adding
// a blanket browser-side rejection at a few fore-aft degrees.
state.mekf={valid:true,fresh:true,roll_deg:4,pitch_deg:.2,yaw_deg:0};render();
assert.strictEqual(element('work-start').disabled,false);
state.foot_control_ready=false;state.foot_control_reason='left_out_of_range';
state.foot.left_control_range_ok=false;state.foot.left_extrapolation_deg=8;render();
assert.strictEqual(element('work-start').disabled,true);
assert.match(element('work-start-status').textContent,/許容5°を超え/);
state.foot_control_reason='left_not_detected';state.foot.left_valid=false;render();
assert.strictEqual(element('work-start').disabled,true);
assert.match(element('work-start-status').textContent,/左足の有効な検出/);
state.foot_control_ready=true;state.foot_control_reason='ready';state.foot.left_valid=true;
state.foot.left_control_range_ok=true;state.foot.left_extrapolation_deg=3;
state.state='RUNNING_BATCH_SWEEP';state.running=true;render();
assert.match(element('run-percent').textContent,/実ZEROクロスで主入力を再計算/);
assert.match(element('run-brake').textContent,/救済と主入力を合わせて半周期80ms/);
assert.match(element('run-model').textContent,/前倒し候補のモデル支持/);
assert.match(element('run-model').textContent,/範囲外はZEROで再評価/);
assert.match(element('guide').textContent,/前後の持続逸脱では入力を停止/);
assert.match(element('run-adaptation').textContent,/救済のない周期だけ/);
vm.runInContext('requestedTargetDeg=8;requestedInputPercent=0;requestedBrakeMaxMs=40;requestedEnergyModel="work_control";renderOffline()',context);
assert.match(element('run-brake').textContent,/合わせて半周期80ms/);
assert.match(element('run-model').textContent,/モデル支持.*ZEROで再評価/);
assert.match(element('guide').textContent,/前後の持続逸脱でも入力を停止/);
assert.strictEqual(vm.runInContext('walkingAdmissionTrial({diagnostic_build_id:"walking-energy-admission-v190"})',context),false);
assert.strictEqual(vm.runInContext('walkingAdmissionTrial({diagnostic_build_id:"walking-energy-admission-v19-test"})',context),true);
assert.strictEqual(vm.runInContext('walkingAdmissionTrial({diagnostic_build_id:"walking-energy-transfer-v18"})',context),false);
state.running=false;state.ready=false;state.state='ESTOP';state.downloadable=true;
for(const [reason,phrase] of [['fore_aft_outward_divergence',/外向きに持続して増え/],
                            ['fore_aft_invalid_sample',/IMUデータが無効/],
                            ['fore_aft_stale_sample',/IMUの更新が古く/]]){
  state.last_error=reason;render();assert.match(element('guide').textContent,phrase);
  assert.match(element('guide').textContent,/入力を停止しました/);
}
state.running=true;state.ready=true;state.state='RUNNING_BATCH_SWEEP';state.last_error='';
state.diagnostic_build_id='walking-energy-transfer-v18';render();
assert.match(element('work-title').textContent,/v18/);
assert.doesNotMatch(element('work-description').textContent,/前倒し候補のモデル支持/);
assert.match(element('run-model').textContent,/救済は戻り通過の不足分/);
const W=require('../web/wheel_work.js');
const v18=W.analyze([],{walking_energy:{revision:'walking_energy_transfer_v18'}});
const v19=W.analyze([],{walking_energy:{revision:'walking_energy_admission_v19'}});
assert.strictEqual(v19.walking_control_revision,'walking_energy_admission_v19');
assert.deepStrictEqual(v19.measurement,v18.measurement);
assert.match(W.describe(v19),/v19の救済完了予測は、戻り通過の要求に対する充足/);
assert.match(W.describe(v19),/次ピークの到達エネルギーを表す値ではありません/);
assert.match(W.describe(v19),/前倒し候補のモデル支持.*ZEROで再評価/);
assert.match(W.describe(v19),/前後の持続逸脱では入力を停止/);
assert.doesNotMatch(W.describe(v18),/前倒し候補のモデル支持/);
console.log('v19 UI: identity/5deg tolerance/start authority/model support/ZERO deferral/shared80ms/fore-aft stop/v18 retained PASS');
`;
vm.runInNewContext(fixture+checks,{require,console,Buffer,Date,Map,DataView,Uint8Array,TextDecoder,
  setTimeout,clearTimeout,AbortController},{filename:'test_start_diagnostics_ui_v19.js'});
