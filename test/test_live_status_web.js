// Exercise real browser refresh, fresh-state replacement and detail saving.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const elements=new Map();
const canvas=()=>new Proxy({}, {get:()=>()=>{}});
function element(id){if(!elements.has(id))elements.set(id,{textContent:'',style:{},width:640,height:480,getContext:canvas});return elements.get(id);}
const context=vm.createContext({document:{getElementById:element,createElement:()=>({getContext:canvas})},
  setTimeout,clearTimeout,AbortController,Date,DataView,Uint8Array,TextDecoder,Map,console,
  btoa:s=>Buffer.from(s,'binary').toString('base64'),atob:s=>Buffer.from(s,'base64').toString('binary')});
vm.runInContext(fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/,''),context);
const state={boot_id:1,run_id:0,input_peak_percent:0,input_current_mA:300,brake_max_ms:40,peak_adaptation_enabled:true,
  state:'READY_TO_MEASURE',export_phase:'empty',running:false,ready:true,downloadable:false,controller_fresh:true,
  imu_ok:true,motor_mA:0,foot_control_required:true,foot_control_ready:true,
  upright_reference:{valid:false,capturing:false},command:{pending:false,completed:0,submitted:0},upright:{stable:true},
  foot:{available:true,zero_ready:true,age_ms:20,right_valid:true,left_valid:true,right_deg:2,left_deg:3,
    right_in_range:true,left_in_range:true,right_reason:'detected',left_reason:'detected',
    range:{right_support_x:[67,203],left_support_x:[63,202]}}};
(async()=>{
  let paths=[];let response=structuredClone(state);
  context.fetch=async path=>{paths.push(path);return {ok:true,json:async()=>structuredClone(response)};};
  await vm.runInContext('refresh()',context);assert.equal(paths.pop(),'/status.json');
  response={...structuredClone(state),live_status_supported:true,status_view:'full',camera_receive:{short_frames:3}};
  await vm.runInContext('refresh()',context);assert.equal(paths.pop(),'/status.json');
  response={...structuredClone(state),live_status_supported:true,status_view:'live'};
  response.foot.right_deg=7;response.foot_control_ready=false;response.foot_control_reason='frame_stale';
  await vm.runInContext('refresh()',context);assert.equal(paths.pop(),'/status.json?view=live');
  assert.equal(element('right').textContent,'7.00°');assert.equal(element('work-start').disabled,true);
  assert.equal(vm.runInContext('latest.camera_receive',context),undefined,'Never merge stale diagnostics into live state');
  await vm.runInContext('refresh(true)',context);assert.equal(paths.pop(),'/status.json');
  response={...structuredClone(state),boot_id:2,run_id:3,status_view:'full',camera_receive:{short_frames:9}};
  const saved=await vm.runInContext('diagnosticSnapshotWithDetails()',context);
  assert.equal(saved.boot_id,1);assert.equal(saved.device_detail.boot_id,2);
  assert.equal(saved.detail_fetch.same_boot,false);assert.equal(saved.detail_fetch.same_run,false);
  assert.equal(vm.runInContext('latest.boot_id',context),1);
  assert.equal(vm.runInContext('diagnosticInFlight',context),false);
  context.fetch=async()=>{throw Error('unreachable');};
  const failed=await vm.runInContext('diagnosticSnapshotWithDetails()',context);
  assert.equal(failed.detail_fetch.ok,false);assert.equal(failed.foot.right_deg,7);
  assert.equal(vm.runInContext('diagnosticInFlight',context),false);
  vm.runInContext('offlineMode=true',context);
  const offline=await vm.runInContext('diagnosticSnapshotWithDetails()',context);
  assert.equal(offline.detail_fetch,undefined,'Do not poll during offline walking');
  vm.runInContext('offlineMode=false; transferRunning=true',context);
  assert.equal((await vm.runInContext('diagnosticSnapshotWithDetails()',context)).detail_fetch,undefined);
  vm.runInContext('transferRunning=false',context);
  context.fetch=async path=>{paths.push(path);return {ok:true,json:async()=>structuredClone(state)};};
  await vm.runInContext('refresh()',context); // old firmware replaces the capability too
  await vm.runInContext('refresh()',context);assert.equal(paths.pop(),'/status.json');
  if(process.argv[2]) {
    const actual=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
    for(const view of ['full','live']) {
      context.fetch=async()=>({ok:true,json:async()=>structuredClone(actual[view])});
      await vm.runInContext('refresh()',context);
      assert.equal(element('connection').textContent,'接続中');
      assert.equal(vm.runInContext('latest.boot_id',context),actual.boot_id);
      assert.equal(element('right').textContent,actual[view].foot.right_deg.toFixed(2)+'°');
      assert.equal(element('left').textContent,actual[view].foot.left_deg.toFixed(2)+'°');
    }
    console.log('PASS actual device full/live JSON rendered with fresh angles and original calibration ranges');
  }
  console.log('PASS live capability negotiation, fresh fields/start gates, old firmware, full diagnostics, reboot separation, cached failure and offline/transfer silence');
})().catch(e=>{console.error(e);process.exit(1);});
