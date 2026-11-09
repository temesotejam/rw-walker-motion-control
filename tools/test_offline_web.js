'use strict';
const fs=require('fs'), vm=require('vm'), assert=require('node:assert/strict');
let now=100000, failStart=false, loseReply=false, calls=[];
const storage=new Map(), elements=new Map();
const canvas={clearRect(){},fillRect(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},fillText(){},arc(){},fill(){}};
const element=id=>{if(!elements.has(id))elements.set(id,{textContent:'',style:{},getContext:()=>canvas});return elements.get(id);};
const status={state:'READY_TO_MEASURE',ready:true,running:false,downloadable:false,controller_fresh:true,
  target_deg:8,input_peak_percent:0,input_current_mA:1200,brake_max_ms:50,peak_adaptation_enabled:true,energy_model:'quadratic',boot_id:1,
  export_phase:'empty',command:{pending:false,submitted:0,completed:0,result:''},foot:{},upright:{},
  upright_reference:{valid:true,capturing:false,abs_pitch_deg:2.2,relative_pitch_deg:0,revision:1}};
const sessionStorage={getItem:k=>storage.get(k),setItem:(k,v)=>storage.set(k,v),removeItem:k=>storage.delete(k)};
const context=vm.createContext({document:{getElementById:element},Date:{now:()=>now},sessionStorage,
  setTimeout:()=>1,clearTimeout(){},AbortController,URLSearchParams,Map,DataView,Uint8Array,TextDecoder,
  fetch:async(path)=>{
    calls.push(path);
    if(path.startsWith('/start-energy-control-autonomous?target_deg=')) {
      const params=new URLSearchParams(path.split('?')[1]);
      const target=Number(params.get('target_deg'));assert.ok([8,10,12].includes(target));
      const angle=Number(params.get('input_peak_percent'));assert.ok(angle>=0&&angle<=100);
      if(!failStart)status.input_peak_percent=angle;
      const current=Number(params.get('input_current_mA'));assert.ok(current>=100&&current<=1200&&current%10===0);
      if(!failStart)status.input_current_mA=current;
      const brake=Number(params.get('brake_max_ms'));assert.ok([0,15,20,25,30,40,50].includes(brake));
      if(!failStart)status.brake_max_ms=brake;
      const adaptation=params.get('peak_adaptation');assert.ok(['0','1'].includes(adaptation));
      if(!failStart)status.peak_adaptation_enabled=adaptation==='1';
      const model=params.get('energy_model');assert.ok(['bridge','bridge_inverse','work_control','bridge_probe','bridge_probe_batch','bridge_probe_extended','bridge_probe_positive_current','quadratic'].includes(model));
      if(!failStart)status.energy_model=model;
      if(!failStart)status.target_deg=target;
      if(loseReply)throw Error('reply lost after acceptance');
      return failStart ? {ok:false,status:409,text:async()=>'busy'} : {ok:true,text:async()=>'offline_run_queued'};
    }
    assert.equal(path,'/status.json');return {ok:true,json:async()=>status};
  }});
const source=fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/,'');
vm.runInContext(source,context);
(async()=>{
  await vm.runInContext('refresh()',context);assert.equal(element('start').disabled,false);
  assert.equal(element('target').value,'8');assert.equal(element('target').disabled,false);
  element('target').value='9';calls=[];await vm.runInContext('startOfflineRun()',context);
  assert.equal(calls.length,0);assert.equal(vm.runInContext('offlineMode',context),false);
  for(const bad of ['', '-0.1', '100.1', 'NaN']) {
    element('target').value='10';element('input-percent').value=bad;calls=[];
    await vm.runInContext('startOfflineRun()',context);assert.equal(calls.length,0);
  }
  element('input-percent').value='0';element('input-current').value='500';
  for(const bad of ['', '99', '1201', '555']) { element('input-current').value=bad;calls=[];
    await vm.runInContext('startOfflineRun()',context);assert.equal(calls.length,0); }
  element('input-current').value='500';
  element('brake-max').value='10';calls=[];await vm.runInContext('startOfflineRun()',context);assert.equal(calls.length,0);
  element('brake-max').value='20';element('peak-adaptation').value='0';
  element('target').value='10';await vm.runInContext('refresh()',context);
  assert.equal(element('input-percent').value,'0');assert.equal(element('input-current').value,'500');
  assert.equal(element('target').value,'10'); // polling must not overwrite a user's selection
  calls=[];await vm.runInContext('startOfflineRun()',context);
  assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=10&input_peak_percent=0&input_current_mA=500&brake_max_ms=20&peak_adaptation=0&energy_model=quadratic']);
  assert.equal(element('target').disabled,true);assert.match(element('run-target').textContent,/目標角度：10°/);
  await vm.runInContext('poll()',context);now+=20000;await vm.runInContext('poll()',context);
  assert.equal(calls.length,1);assert.equal(element('start').disabled,true);
  assert.equal(element('download').disabled,true);assert.equal(element('pitch').textContent,'—');
  assert.ok(storage.size===1);
  // Transport failure after expected finish must keep the UI recoverable.
  now+=26000;
  const fetch=context.fetch;context.fetch=async()=>{throw Error('HTTP not listening');};
  await vm.runInContext('poll()',context);assert.match(element('connection').textContent,/復帰待ち/);
  context.fetch=fetch;
  Object.assign(status,{state:'ESTOP',ready:false,downloadable:true,last_error:'fore_aft_tilt_90deg'});
  await vm.runInContext('poll()',context);
  assert.equal(vm.runInContext('offlineMode',context),false);assert.equal(storage.size,0);
  assert.equal(element('download').disabled,false);assert.match(element('message').textContent,/fore_aft_tilt_90deg/);
  assert.match(element('run-target').textContent,/今回の目標角度：10°/);
  assert.match(element('run-brake').textContent,/20 ms/);
  assert.equal(element('input-percent').disabled,true);assert.equal(element('input-current').disabled,true);
  assert.match(element('run-percent').textContent,/500 mA/);
  assert.match(element('run-percent').textContent,/ZEROクロス/);
  assert.equal(element('target').disabled,true); // sealed run remains associated with its target
  assert.equal(status.peak_adaptation_enabled,false);
  assert.equal(element('peak-adaptation').disabled,true);
  assert.match(element('run-adaptation').textContent,/使用しない/);
  Object.assign(status,{state:'READY_TO_MEASURE',ready:true,downloadable:false,last_error:''});
  await vm.runInContext('refresh()',context);element('target').value='12';
  failStart=true;await vm.runInContext('startOfflineRun()',context);
  assert.equal(vm.runInContext('offlineMode',context),false);
  failStart=false;loseReply=true;calls=[];await vm.runInContext('startOfflineRun()',context);
  await vm.runInContext('poll()',context);assert.equal(calls.length,1);
  assert.equal(vm.runInContext('offlineMode',context),true); // Never retry a possibly accepted START.
  // A reload of an already loaded page preserves the quiet countdown.
  const restoredElements=new Map();
  const restoredElement=id=>{if(!restoredElements.has(id))restoredElements.set(id,{...element(id)});return restoredElements.get(id);};
  const restored=vm.createContext({...context,sessionStorage,document:{getElementById:restoredElement}});
  vm.runInContext(source,restored);assert.equal(vm.runInContext('offlineMode',restored),true);
  // A status request already in flight at START must not clear the pause when
  // its old READY response arrives. Otherwise polls would burden the live AP.
  vm.runInContext('clearOffline()',context);loseReply=false;
  let deliver;
  context.fetch=async(path)=>path==='/status.json'
    ? new Promise(resolve=>{deliver=resolve;}) : fetch(path);
  const staleRefresh=vm.runInContext('refresh()',context);
  calls=[];await vm.runInContext('startOfflineRun()',context);
  deliver({ok:true,json:async()=>status});await staleRefresh;
  assert.equal(vm.runInContext('offlineMode',context),true);
  assert.match(element('connection').textContent,/Wi-Fi接続を維持/);
  await vm.runInContext('poll()',context);assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=12&input_peak_percent=0&input_current_mA=500&brake_max_ms=20&peak_adaptation=0&energy_model=quadratic']);
  // Default expiry resumes GETs automatically, including normal completion.
  context.fetch=fetch;now+=31000;
  Object.assign(status,{state:'FINISHED',ready:false,downloadable:true,last_error:''});
  await vm.runInContext('poll()',context);
  assert.equal(vm.runInContext('offlineMode',context),false);
  assert.equal(element('download').disabled,false);
  assert.match(element('message').textContent,/Web表示が復帰/);
  assert.match(element('run-target').textContent,/今回の目標角度：12°/);
  // The next run may select 8 again. A device reboot also restores its default.
  Object.assign(status,{state:'READY_TO_MEASURE',ready:true,downloadable:false});
  await vm.runInContext('refresh()',context);assert.equal(element('target').disabled,false);
  element('target').value='8';calls=[];await vm.runInContext('startOfflineRun()',context);
  assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=8&input_peak_percent=0&input_current_mA=500&brake_max_ms=20&peak_adaptation=0&energy_model=quadratic']);
  vm.runInContext('clearOffline()',context);status.boot_id=2;status.input_peak_percent=0;status.input_current_mA=1200;status.brake_max_ms=50;
  element('target').value='12';await vm.runInContext('refresh()',context);
  assert.equal(element('target').value,'8');
  assert.equal(element('input-percent').value,'0');assert.equal(element('input-current').value,'1200');
  element('energy-model').value='bridge';vm.runInContext('controls()',context);
  assert.equal(element('brake-max').disabled,true);
  calls=[];await vm.runInContext('startOfflineRun()',context);
  assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=8&input_peak_percent=0&input_current_mA=1200&brake_max_ms=0&peak_adaptation=0&energy_model=bridge']);
  assert.match(element('run-model').textContent,/旧到達角予測/);
  vm.runInContext('clearOffline()',context);
  element('target').value='12';element('input-current').value='1200';
  element('peak-adaptation').value='1';
  calls=[];await vm.runInContext('startOfflineRun(true)',context);
  assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=8&input_peak_percent=0&input_current_mA=300&brake_max_ms=25&peak_adaptation=0&energy_model=bridge_probe_batch']);
  assert.match(element('run-brake').textContent,/負側5条件を順序を変えて測定/);
  assert.match(element('guide').textContent,/測定30秒/);
  vm.runInContext('clearOffline()',context);
  calls=[];await vm.runInContext("startOfflineRun(false, 'bridge_probe_positive_current')",context);
  assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=8&input_peak_percent=0&input_current_mA=300&brake_max_ms=50&peak_adaptation=0&energy_model=bridge_probe_positive_current']);
  assert.match(element('run-brake').textContent,/負側40 ms固定/);
  assert.match(element('guide').textContent,/測定30秒/);
  vm.runInContext('clearOffline()',context);
  calls=[];await vm.runInContext("startOfflineRun(false, 'bridge_probe_extended')",context);
  assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=8&input_peak_percent=0&input_current_mA=300&brake_max_ms=40&peak_adaptation=0&energy_model=bridge_probe_extended']);
  assert.match(element('run-brake').textContent,/負側30・35・40 ms/);
  assert.match(element('guide').textContent,/測定30秒/);
  vm.runInContext('clearOffline()',context);
  Object.assign(status,{state:'READY_TO_MEASURE',ready:true,downloadable:false});
  await vm.runInContext('refresh()',context);
  assert.equal(element('inverse-start').disabled,false);
  assert.equal(vm.runInContext('offlineMode',context),false);
  assert.equal(vm.runInContext('commandInFlight',context),false);
  element('energy-model').value='bridge';
  element('target').value='12';element('input-current').value='1200';
  element('brake-max').value='50';element('peak-adaptation').value='1';
  calls=[];await vm.runInContext("startOfflineRun(false, 'bridge_inverse')",context);
  assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=8&input_peak_percent=0&input_current_mA=300&brake_max_ms=25&peak_adaptation=0&energy_model=bridge_inverse'],element('message').textContent);
  assert.equal(vm.runInContext('requestedEnergyModel',context),'bridge_inverse');
  vm.runInContext('clearOffline()',context);
  Object.assign(status,{state:'READY_TO_MEASURE',ready:true,downloadable:false});
  status.upright_reference={valid:false,capturing:false,revision:0};
  await vm.runInContext('refresh()',context);
  assert.equal(element('work-start').disabled,false);
  assert.equal(element('start').disabled,true); // Legacy preset still needs its saved origin.
  assert.equal(vm.runInContext('offlineMode',context),false);
  assert.equal(vm.runInContext('commandInFlight',context),false);
  assert.equal(typeof element('work-start').onclick,'function');
  calls=[];await vm.runInContext("startOfflineRun(false, 'work_control')",context);
  assert.deepEqual(calls,['/start-energy-control-autonomous?target_deg=8&input_peak_percent=0&input_current_mA=300&brake_max_ms=40&peak_adaptation=0&energy_model=work_control'],element('message').textContent);
  assert.equal(vm.runInContext('requestedEnergyModel',context),'work_control');
  for (const target of [10,12]) {
    vm.runInContext('clearOffline()',context);
    Object.assign(status,{state:'READY_TO_MEASURE',ready:true,downloadable:false});
    await vm.runInContext('refresh()',context);
    assert.equal(element('work-start-'+target).disabled,false);
    calls=[];await element('work-start-'+target).onclick();
    assert.deepEqual(calls,[`/start-energy-control-autonomous?target_deg=${target}&input_peak_percent=0&input_current_mA=300&brake_max_ms=40&peak_adaptation=0&energy_model=work_control`],element('message').textContent+JSON.stringify(vm.runInContext('({offlineMode,commandInFlight,requestedTargetDeg})',context)));
    assert.equal(vm.runInContext('requestedTargetDeg',context),target);
  }
  vm.runInContext('clearOffline()',context);
  calls=[];await vm.runInContext("startOfflineRun(false,'work_control',9)",context);
  assert.equal(calls.length,0);
  console.log('HTTP pause browser: no run polling, stale pre-START response, lost/rejected START, HTTP retry, FINISHED/ESTOP download and restored countdown PASS');
  console.log('Target UI: 8/10/12 request values, invalid selection, polling preservation, run lock, applied result and reboot PASS');
})().catch(e=>{console.error(e);process.exitCode=1;});
