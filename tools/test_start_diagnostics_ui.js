'use strict';
// Source UI tests intentionally do not depend on the generated embedded page.
// The parent integration check verifies that page after regenerating it.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const elements=new Map();
const canvas=()=>new Proxy({}, {get:()=>()=>{}});
function element(id){
  if(!elements.has(id))elements.set(id,{textContent:'',style:{},width:640,height:480,getContext:canvas});
  return elements.get(id);
}
const context=vm.createContext({document:{getElementById:element,createElement:()=>({getContext:canvas})},
  Date,Map,DataView,Uint8Array,TextDecoder,setTimeout,clearTimeout,AbortController,console,
  btoa:s=>Buffer.from(s,'binary').toString('base64'),atob:s=>Buffer.from(s,'base64').toString('binary')});
const source=fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+
  fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/,'');
vm.runInContext(source,context);
const state={input_peak_percent:0,input_current_mA:1200,brake_max_ms:40,peak_adaptation_enabled:false,
  state:'READY_TO_MEASURE',export_phase:'empty',running:false,ready:true,downloadable:false,
  controller_fresh:true,imu_ok:true,motor_mA:0,pulse_active:false,target_deg:8,energy_model:'work_control',
  upright:{stable:true},
  upright_reference:{valid:true,capturing:false,abs_pitch_deg:0,relative_pitch_deg:0,revision:1},
  command:{pending:false,completed:0,submitted:0},foot_control_required:true,foot_control_ready:true,foot_control_reason:'ready',
  foot:{available:true,zero_ready:true,age_ms:70,frame_valid:true,frame_timestamp_valid:true,left_valid:true,right_valid:true,
    left_in_range:false,right_in_range:true,left_control_range_ok:true,right_control_range_ok:true,
    left_extrapolation_deg:3,control_extension_deg:5,left_deg:-.5,right_deg:0,left_reason:'detected',right_reason:'detected'}};
context.state=state;
function render(){vm.runInContext('adoptStatus(state);render(state)',context);}

state.diagnostic_build_id='walking-return-energy-v17-rangefix1';render();
assert.match(element('work-title').textContent,/v17/);
assert.match(element('work-description').textContent,/主入力・救済は最大80ms/);
assert.strictEqual(element('work-start').disabled,false);

state.diagnostic_build_id='walking-energy-transfer-v18';render();
assert.match(element('work-title').textContent,/v18/);
assert.match(element('work-description').textContent,/実ZEROクロスで主入力を再計算/);
assert.match(element('work-description').textContent,/合わせて半周期80ms/);
assert.match(element('work-description').textContent,/新しい画像と機体の減速条件/);
assert.match(element('work-description').textContent,/校正端から5°以内/);
assert.strictEqual(element('work-start').disabled,false);
assert.match(element('foot-status').textContent,/許容範囲内.*3\.0°の外挿/);

state.foot_control_ready=false;state.foot_control_reason='left_not_detected';
state.foot.left_valid=false;state.foot.left_reason='ambiguous';render();
assert.strictEqual(element('work-start').disabled,true); // Dropout aid never changes the start gate.
assert.match(element('work-start-status').textContent,/左足の有効な検出/);
state.foot_control_reason='left_out_of_range';state.foot.left_valid=true;
state.foot.left_control_range_ok=false;state.foot.left_extrapolation_deg=8;render();
assert.strictEqual(element('work-start').disabled,true);
assert.match(element('work-start-status').textContent,/許容5°を超え/);

state.foot_control_ready=true;state.foot_control_reason='ready';
state.foot.left_control_range_ok=true;state.foot.left_extrapolation_deg=3;
state.state='RUNNING_BATCH_SWEEP';state.running=true;render();
assert.match(element('run-percent').textContent,/実ZEROクロスで主入力を再計算/);
assert.match(element('run-brake').textContent,/救済と主入力を合わせて半周期80ms/);
assert.doesNotMatch(element('run-brake').textContent,/とも最大80ms/);
assert.match(element('run-model').textContent,/救済は戻り通過の不足分/);
assert.match(element('run-adaptation').textContent,/救済のない周期だけ/);
assert.match(element('guide').textContent,/新しい画像と機体の減速条件/);

vm.runInContext('requestedTargetDeg=8;requestedInputPercent=0;requestedBrakeMaxMs=40;requestedEnergyModel="work_control";renderOffline()',context);
assert.match(element('run-brake').textContent,/合わせて半周期80ms/);
assert.match(element('run-percent').textContent,/実ZEROクロスで主入力を再計算/);
assert.match(element('run-model').textContent,/救済は戻りの不足分/);
assert.strictEqual(vm.runInContext('walkingTransferTrial({diagnostic_build_id:"walking-energy-transfer-v180"})',context),false);
assert.strictEqual(vm.runInContext('walkingTransferTrial({diagnostic_build_id:"walking-energy-transfer-v18-test"})',context),true);

state.diagnostic_build_id='walking-return-energy-v17';render();
assert.match(element('work-title').textContent,/v17/);
assert.match(element('run-brake').textContent,/とも最大80ms/);

const W=require('../web/wheel_work.js');
const oldReport=W.analyze([],{energy_model:'walking_return_energy_v17'});
const newReport=W.analyze([],{energy_model:'walking_energy_transfer_v18'});
assert.deepStrictEqual(newReport.measurement,oldReport.measurement);
assert.strictEqual(oldReport.walking_control_revision,null);
assert.strictEqual(newReport.walking_control_revision,'walking_energy_transfer_v18');
assert.match(W.describe(newReport),/救済完了予測は、戻り通過の要求に対する充足/);
assert.match(W.describe(newReport),/次ピークの到達エネルギーを表す値ではありません/);
assert.doesNotMatch(W.describe(oldReport),/v18の救済/);
console.log('v18 UI: identity/start gates/5deg tolerance/shared80ms/actual ZERO/aid prediction labels/v17 retained PASS');
