'use strict';
const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const elements=new Map();
const canvas=new Proxy({}, {get:()=>()=>{}});
const element=id=>{if(!elements.has(id))elements.set(id,{textContent:'',style:{},getContext:()=>canvas});return elements.get(id);};
const status={state:'READY_TO_MEASURE',ready:true,running:false,downloadable:false,controller_fresh:true,
  imu_ok:true,motor_mA:0,pulse_active:false,export_phase:'empty',input_peak_percent:0,input_current_mA:300,
  brake_max_ms:50,peak_adaptation_enabled:false,target_deg:8,boot_id:1,
  command:{pending:false,submitted:0,completed:0,result:''},foot:{},upright:{},
  upright_reference:{valid:false,capturing:false,abs_pitch_deg:null,relative_pitch_deg:null,revision:0}};
let polls=0,fail=false,cgCommand=false;
const context=vm.createContext({document:{getElementById:element},Date,setTimeout,clearTimeout,AbortController,
  Map,DataView,Uint8Array,TextDecoder,fetch:async(path)=>{
    if(path.startsWith('/lateral-cg/')) {
      assert.equal(status.running,false);cgCommand=true;
      ++status.command.submitted;status.command.pending=true;polls=0;
      return {ok:true,text:async()=>'command_queued'};
    }
    if(path==='/upright-reference/capture') {
      cgCommand=false;
      assert.equal(element('start').disabled,true);
      ++status.command.submitted; status.command.pending=true; status.upright_reference.capturing=true;
      polls=0; return {ok:true,text:async()=>'command_queued'};
    }
    assert.equal(path,'/status.json');
    if(status.command.pending && ++polls===2) {
      status.command.pending=false; status.command.completed=status.command.submitted;
      status.command.ok=!fail; status.command.result=cgCommand ? 'lateral_cg_saved' : fail?'upright_reference_pose_moved':'upright_reference_saved';
      if(cgCommand)status.lateral_cg={valid:true,shift_mm:-.5,positive_target_mj:.597,negative_target_mj:.870};
      status.upright_reference.capturing=false;
      if(!fail)Object.assign(status.upright_reference,{valid:true,abs_pitch_deg:2.2,relative_pitch_deg:0,revision:1});
    }
    return {ok:true,json:async()=>JSON.parse(JSON.stringify(status))};
  }});
vm.runInContext(fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+
  fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/,''),context);
(async()=>{
  await vm.runInContext('refresh()',context);
  assert.equal(element('start').disabled,true);
  assert.equal(element('work-start').disabled,false);
  assert.equal(element('upright-reference-capture').disabled,false);
  await element('upright-reference-capture').onclick();
  assert.match(element('message').textContent,/本体に保存しました/);
  assert.equal(element('start').disabled,false);
  status.lateral_cg={automatic:true,valid:false,shift_mm:-.206,pairs:0,updates:0,
    positive_target_mj:.677,negative_target_mj:.790};
  await vm.runInContext('refresh()',context);
  assert.equal(element('work-start').disabled,false);
  assert.match(element('cg-status').textContent,/自動補正/);
  assert.equal(element('cg-save').disabled,false);
  element('cg-shift').value='-0.5';await element('cg-save').onclick();
  assert.equal(status.lateral_cg.shift_mm,-.5);
  assert.match(element('cg-status').textContent,/-0.500/);
  element('cg-shift').value='3';const submitted=status.command.submitted;
  await element('cg-save').onclick();assert.equal(status.command.submitted,submitted);
  fail=true;await element('upright-reference-capture').onclick();
  assert.match(element('message').textContent,/元の基準を保っています/);
  assert.equal(status.upright_reference.abs_pitch_deg,2.2);
  status.running=true;await vm.runInContext('refresh()',context);
  assert.equal(element('upright-reference-capture').disabled,true);
  assert.equal(element('cg-save').disabled,true);assert.equal(element('cg-capture').disabled,true);
  status.running=false;status.command.pending=true;polls=0;await vm.runInContext('refresh()',context);
  assert.equal(element('upright-reference-capture').disabled,true);
  status.command.pending=false;status.imu_ok=false;await vm.runInContext('refresh()',context);
  assert.equal(element('upright-reference-capture').disabled,true);
  status.imu_ok=true;status.state='FINISHED';status.downloadable=true;await vm.runInContext('refresh()',context);
  assert.equal(element('upright-reference-capture').disabled,false);
  console.log('Web reference capture, missing-reference start gate, rejected-motion feedback and busy/running gates PASS');
})().catch(e=>{console.error(e);process.exitCode=1;});
