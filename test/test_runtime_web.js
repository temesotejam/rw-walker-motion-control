'use strict';
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const elements = new Map();
function canvasContext() {return new Proxy({}, {get:(_target,name)=>name==='createImageData'
  ? (w,h)=>({data:new Uint8Array(w*h*4)}) : ()=>{}});}
function element(id) {if(!elements.has(id)) elements.set(id, {textContent:'',style:{},width:640,height:480,getContext:canvasContext}); return elements.get(id);}
const code=fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/, '');
const header=fs.readFileSync('src/runtime_web.h','utf8');
const embedded=header.split('R"FREEFOOT(')[1].split(')FREEFOOT";')[0];
const expected=fs.readFileSync('web/index.html','utf8').replace('/*POSE_SCRIPT*/',fs.readFileSync('web/pose_comparison.js','utf8'))
  .replace('/*WHEEL_WORK_SCRIPT*/',fs.readFileSync('web/wheel_work.js','utf8')).replace('/*RUNTIME_SCRIPT*/',fs.readFileSync('web/runtime.js','utf8'));
assert.strictEqual(embedded.replace(/\r\n/g,'\n'),expected.replace(/\r\n/g,'\n'),'Embedded runtime UI must match the tested browser sources');
const compressed=Buffer.from(Array.from(header.split('RUNTIME_HTML_GZIP[] PROGMEM =')[1].matchAll(/\\x([0-9a-f]{2})/g),
  match=>parseInt(match[1],16)));
assert.deepStrictEqual(require('zlib').gunzipSync(compressed),Buffer.from(expected.replace(/\r\n/g,'\n')));
const context=vm.createContext({document:{getElementById:element,createElement:()=>({getContext:canvasContext})},setTimeout,clearTimeout,AbortController,
  Date,DataView,Uint8Array,TextDecoder,Map,console,btoa:s=>Buffer.from(s,'binary').toString('base64'),
  atob:s=>Buffer.from(s,'base64').toString('binary'),fetch:()=>{throw Error('unset');}});
vm.runInContext(code,context);
(async()=>{
  // Headers succeed immediately, response body never completes until aborted.
  context.fetch=async(_path,opts)=>({ok:true,json:()=>new Promise((_,reject)=>opts.signal.addEventListener('abort',()=>reject(Error('body_aborted'))))});
  await assert.rejects(vm.runInContext("request('/status.json',{timeout:25})",context),
    error => error.name === 'RequestTimeoutError' && error.code === 'request_timeout' && error.path === '/status.json');
  assert.strictEqual(vm.runInContext("crc32(new Uint8Array([49,50,51,52,53,54,55,56,57]))",context),0xcbf43926);
  await vm.runInContext('refresh()',context); // real production refresh cleanup
  assert.strictEqual(vm.runInContext('refreshInFlight',context),false);
  context.fetch=async()=>({ok:true,json:async()=>({})});
  assert.deepStrictEqual(JSON.parse(JSON.stringify(await vm.runInContext("request('/status.json')",context))),{});
  await vm.runInContext('refresh()',context);
  assert.strictEqual(vm.runInContext('latest',context),null);
  assert.strictEqual(element('start').disabled,true);
  assert.match(element('connection').textContent,/状態データ/);
  const valid={input_peak_percent:0,input_current_mA:1200,brake_max_ms:50,peak_adaptation_enabled:true,state:'READY_TO_MEASURE',export_phase:'empty',running:false,ready:true,downloadable:false,
    controller_fresh:true,imu_ok:true,motor_mA:0,pulse_active:false,
    upright_reference:{valid:true,capturing:false,abs_pitch_deg:2.2,relative_pitch_deg:0,revision:1},
    command:{pending:false,completed:0,submitted:0},
    foot:{available:true,zero_ready:true,age_ms:80,frame_valid:true,frame_timestamp_valid:true,
      right_valid:true,left_valid:true,right_in_range:true,left_in_range:true,
      right_deg:8,left_deg:-6,right_reason:'detected',left_reason:'detected'},upright:{stable:false}};
  valid.revision='1.0.99-alternate-foot-trial'; valid.diagnostic_build_id='foot-control-observation-v1';
  context.fetch=async()=>({ok:true,json:async()=>valid});
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('connection').textContent,'接続中');
  assert.strictEqual(element('start').disabled,false);
  assert.match(element('firmware-revision').textContent,/1\.0\.99-alternate-foot-trial/);
  assert.match(element('firmware-revision').textContent,/foot-control-observation-v1/);
  assert.strictEqual(element('probe-batch').disabled,false);
  assert.strictEqual(element('inverse-start').disabled,false);
  // The main pitch display can use the saved upright zero. The pre-start
  // comparison must use the unchanged absolute posterior actually in RWLOG.
  const startPose={...valid,diagnostic_build_id:'walking-support-transfer-v39-walk1',
    pitch_deg:-0.13,mekf:{valid:true,fresh:true,pitch_deg:2.07}};
  context.fetch=async()=>({ok:true,json:async()=>startPose});
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('walking-start-pose').hidden,false);
  assert.strictEqual(element('walking-start-pitch').textContent,'+2.07°');
  assert.strictEqual(element('work-start').disabled,false);
  startPose.mekf.pitch_deg=-0.23;
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('walking-start-pitch').textContent,'-0.23°');
  startPose.mekf.fresh=false;
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('walking-start-pitch').textContent,'更新待ち');
  startPose.state='START_SYNC';startPose.running=true;
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('walking-start-pose').hidden,true);
  context.fetch=async()=>({ok:true,json:async()=>valid});
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('walking-start-pose').hidden,true);
  const footPacket = {count:401,missed:1,both_valid:397,published_hz:39.9,packet_age_ms:10,
    left:{estimated_deg:4,age_us:31000},right:{estimated_deg:null,age_us:80000}};
  context.fetch=async()=>({ok:true,json:async()=>({periodic_run:true,foot_periodic:footPacket,walking_profile:'v14_small_step',
    state:'RUNNING_BATCH_SWEEP',remaining_ms:15000,pitch_deg:1,motor_mA:0,actual_mA:0})});
  await vm.runInContext('refresh()',context);
  assert.match(element('foot-status').textContent,/39.9 Hz/);
  assert.match(element('run-percent').textContent,/v14式 小歩幅/);
  assert.match(element('foot-status').textContent,/周期抜け1回/);
  assert.match(element('right').textContent,/推定無効/);
  assert.strictEqual(vm.runInContext('statusPollDelayMs()',context),2000);
  assert.strictEqual(element('work-start').disabled,true);
  context.fetch=async()=>({ok:true,json:async()=>valid});
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('start').disabled,false);
  assert(!code.includes('steering-status'));
  const html=fs.readFileSync('web/index.html','utf8');
  assert.match(html,/id="probe-batch"/);
  assert.match(html,/id="inverse-start"/);
  assert.doesNotMatch(html,/id="probe-(0|15|30)"/);
  const walkingArea = html.split('<div id="work-energy-control">')[1].split('</div>')[0];
  assert.deepStrictEqual([...walkingArea.matchAll(/<button id="([^"]+)"/g)].map(match => match[1]), ['work-start']);
  assert.match(walkingArea,/歩行を開始（±8°・30秒）/);
  assert.doesNotMatch(html,/id="(?:small-step|return-history|return-prediction|left-transfer-carry)-start"|<option value="work_control"/);
  assert.match(html,/<details id="fixed-leg-diagnostics"><summary>固定脚の測定・校正<\/summary>[\s\S]+id="work-start-10"[\s\S]+id="work-start-12"/);
  assert.doesNotMatch(html,/±0.2|応答確認|ジャイロ方位/);
  assert.strictEqual(element('right').textContent,'8.00°'); // Tilting does not invalidate a locked zero.
  assert.strictEqual(element('left').textContent,'-6.00°');
  valid.foot.right_in_range=false;
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('right').textContent,'8.00°');
  assert.match(element('foot-status').textContent,/右：設定範囲外/);
  valid.ready=false;valid.foot.right_valid=false;valid.foot.right_reason='low_contrast';
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('connection').textContent,'接続中');
  assert.strictEqual(element('right').textContent,'—');
  assert.strictEqual(element('left').textContent,'-6.00°');
  assert.match(element('foot-status').textContent,/右：未検出（明暗差不足）/);
  assert.match(element('guide').textContent,/足マーカーは開始条件ではありません/);
  assert.strictEqual(element('start').disabled,true);
  valid.foot.right_reason='low_weight';
  await vm.runInContext('refresh()',context);
  assert.match(element('foot-status').textContent,/右：未検出（白領域不足）/);
  valid.foot.left_valid=false;valid.foot.right_reason=valid.foot.left_reason='no_frame';valid.foot.frame_valid=false;
  await vm.runInContext('refresh()',context);
  assert.match(element('foot-status').textContent,/画像取得失敗/);
  assert.doesNotMatch(element('foot-status').textContent,/明暗差不足|白領域不足/);
  valid.foot.frame_valid=true;valid.foot.frame_timestamp_valid=false;
  valid.foot.right_reason=valid.foot.left_reason='detected';
  await vm.runInContext('refresh()',context);
  assert.match(element('foot-status').textContent,/画像時刻が無効/);
  assert.strictEqual(element('right').textContent,'—');
  Object.assign(valid.foot,{right_valid:true,left_valid:true,right_in_range:true,frame_timestamp_valid:true,age_ms:600});
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('right').textContent,'—');assert.match(element('foot-status').textContent,/画像更新なし/);
  valid.state='FINISHED';await vm.runInContext('refresh()',context);
  assert.strictEqual(element('right').textContent,'8.00°');assert.match(element('foot-status').textContent,/最終フレーム/);
  valid.state='READY_TO_MEASURE';valid.ready=true;valid.foot.age_ms=80;
  context.fetch=async()=>({ok:true,json:async()=>({})});
  await vm.runInContext('refresh()',context);
  assert.strictEqual(vm.runInContext('latest.state',context),'READY_TO_MEASURE');
  assert.strictEqual(element('start').disabled,true); // Last good state is evidence, not fresh authority.
  // Even a rendering exception outside refresh's normal recovery must schedule
  // the next poll. The next valid response restores the display without reload.
  let scheduled=0;
  context.setTimeout=(fn,ms)=>{if(ms===800){++scheduled;return 0;} return setTimeout(fn,ms);};
  vm.runInContext('const realControls=controls; controls=()=>{throw Error("render failure")}',context);
  await vm.runInContext('poll()',context); assert.strictEqual(scheduled,1);
  vm.runInContext('controls=realControls',context);
  context.fetch=async()=>({ok:true,json:async()=>valid});
  await vm.runInContext('poll()',context); assert.strictEqual(scheduled,2);
  assert.strictEqual(element('connection').textContent,'接続中');
  const packet=new Uint8Array(19),view=new DataView(packet.buffer);packet.set([1,2,3],16);
  context.packet=packet;view.setUint32(0,0x31484346,true);view.setUint32(4,4096,true);view.setUint32(8,3,true);
  view.setUint32(12,vm.runInContext('crc32(packet.subarray(16))',context),true);
  assert.strictEqual(vm.runInContext('validateChunk(packet,4096,3).length',context),3);
  assert.throws(()=>vm.runInContext('validateChunk(packet,0,3)',context),/不一致/);
  packet[18]^=1;assert.throws(()=>vm.runInContext('validateChunk(packet,4096,3)',context),/CRC/);
  valid.foot.zero_ready=false;valid.foot.zero_reason='position_mismatch';valid.ready=true;
  await vm.runInContext('refresh()',context);
  assert.match(element('guide').textContent,/MEKF絶対角0°/);
  assert.strictEqual(element('start').disabled,false);
  valid.foot.available=false;valid.foot.frame_valid=false;valid.foot.age_ms=null;
  await vm.runInContext('refresh()',context);
  assert.strictEqual(element('start').disabled,false);
  assert.match(element('foot-status').textContent,/足角度（任意）/);
  valid.foot.available=true;valid.foot.frame_valid=true;valid.foot.age_ms=80;valid.ready=false;
  valid.foot.zero_reason='body_moving';valid.upright={stable:false,error_deg:5.9,pitch_error_deg:0.2,gyro_dps:0.4};
  await vm.runInContext('refresh()',context);
  assert.match(element('guide').textContent,/pitch基準差 0.2°（5°以内）/);
  assert.doesNotMatch(element('guide').textContent,/姿勢誤差 5.9°/);
  valid.foot.zero_ready=true;valid.foot.zero_reason='ready';valid.foot.preview_available=true;valid.ready=true;
  valid.mekf={valid:true,fresh:true,roll_deg:20,pitch_deg:0.2,yaw_deg:3};
  valid.foot.range={right_support_x:[40,173],left_support_x:[39,177.5]};
  await vm.runInContext('refresh()',context);
  assert.strictEqual(JSON.parse(element('diagnostic-view').textContent).mekf.roll_deg,20);
  assert.strictEqual(JSON.parse(element('diagnostic-view').textContent).mekf.yaw_deg,3);
  const pixels=Uint8Array.from({length:19200},(_,i)=>i%256);context.pixels=pixels;
  const manifest={format:'gray8',width:160,height:120,source_width:320,source_height:240,bytes:19200,
    token:7,sequence:17,crc32:vm.runInContext('crc32(pixels)',context),age_ms:30,zero_reason:'ready',
    mekf:{valid:true,roll_deg:18,pitch_deg:0.1,yaw_deg:2,sample_us:1700000},
    right:{valid:true,candidates:1,x:172,scan_y:66,width:19,reason:'detected'},
    left:{valid:true,candidates:1,x:175,scan_y:184,width:19,reason:'detected'}};
  context.manifest=manifest;
  assert.throws(()=>vm.runInContext('validatePreviewManifest({...manifest,bytes:76800})',context),/形式/);
  let chunks=0,badWholeCrc=false,wrongToken=false;
  context.fetch=async(path,opts)=>{
    if(path==='/vision/capture'){
      assert.strictEqual(opts.method,'POST');assert.strictEqual(element('start').disabled,true);
      return {ok:true,json:async()=>({...manifest,crc32:badWholeCrc ? (manifest.crc32^1)>>>0 : manifest.crc32})};
    }
    const url=new URL(path,'http://device');assert.strictEqual(url.pathname,'/vision/chunk');
    assert.strictEqual(url.searchParams.get('token'),'7');
    if(wrongToken)return {ok:false,status:409,text:async()=>'preview_token_or_range_mismatch'};
    ++chunks;
    const offset=Number(url.searchParams.get('offset')),length=Number(url.searchParams.get('length'));
    assert(length<=4096 && offset+length<=pixels.length);
    const body=new Uint8Array(length+16),header=new DataView(body.buffer);body.set(pixels.subarray(offset,offset+length),16);
    context.payload=body.subarray(16);
    header.setUint32(0,0x31484346,true);header.setUint32(4,offset,true);header.setUint32(8,length,true);
    header.setUint32(12,vm.runInContext('crc32(payload)',context),true);
    // Independently polled values must never replace frozen image metadata.
    vm.runInContext('latest.foot.left_x=35',context);
    return {ok:true,arrayBuffer:async()=>body.buffer};
  };
  await vm.runInContext('capturePreview()',context);
  assert.strictEqual(chunks,5);assert.strictEqual(vm.runInContext('previewRunning',context),false);
  assert.strictEqual(vm.runInContext('previewEvidence.left.x',context),175);
  assert.strictEqual(vm.runInContext('previewEvidence.mekf.roll_deg',context),18);
  assert.strictEqual(vm.runInContext('previewEvidence.mekf.sample_us',context),1700000);
  assert.deepStrictEqual(Buffer.from(vm.runInContext('previewEvidence.pixels_gray8_base64',context),'base64'),Buffer.from(pixels));
  assert.strictEqual(element('preview-save').disabled,false);assert.strictEqual(element('start').disabled,false);
  badWholeCrc=true;await vm.runInContext('capturePreview()',context);
  assert.match(element('preview-status').textContent,/画像全体のCRC/);
  assert.match(element('preview-status').textContent,/前回の画像/);
  assert.strictEqual(vm.runInContext('previewEvidence.sequence',context),17);
  badWholeCrc=false;wrongToken=true;await vm.runInContext('capturePreview()',context);
  assert.match(element('preview-status').textContent,/409/);
  assert.strictEqual(vm.runInContext('previewRunning',context),false);
  assert.strictEqual(element('start').disabled,false);
  const gate={...valid,state:'READY_TO_MEASURE',ready:true,running:false,export_phase:'empty',
    diagnostic_build_id:'walking-return-energy-v17-startfix1',foot_control_required:true,foot_control_ready:false,
    foot_control_reason:'left_out_of_range',foot:{...valid.foot,available:true,zero_ready:true,frame_valid:true,
      frame_timestamp_valid:true,left_valid:true,right_valid:true,left_in_range:false,right_in_range:true,age_ms:70}};
  context.gate=gate;
  vm.runInContext('adoptStatus(gate);render(gate)',context);
  assert.strictEqual(element('work-start').disabled,true);
  assert.match(element('work-start-status').textContent,/左足は検出されています.*校正範囲外/);
  assert.match(element('work-title').textContent,/v17/);
  assert.doesNotMatch(element('work-description').textContent,/300・350・400/);
  gate.foot_control_reason='zero_not_ready';gate.foot.zero_ready=false;
  vm.runInContext('adoptStatus(gate);render(gate)',context);
  assert.match(element('work-start-status').textContent,/検出されていても.*ゼロ点/);
  gate.foot_control_reason='frame_stale';
  vm.runInContext('adoptStatus(gate);render(gate)',context);
  assert.match(element('work-start-status').textContent,/200ms/);
  gate.foot_control_reason='ready';gate.foot_control_ready=true;
  Object.assign(gate.foot,{zero_ready:true,left_in_range:true});
  vm.runInContext('adoptStatus(gate);render(gate)',context);
  assert.strictEqual(element('work-start').disabled,false);
  assert.match(element('work-start-status').textContent,/開始できます/);
  for (const suffix of ['', '-webfix1', '-carry1']) {
    context.trialId = 'walking-support-transfer-v39' + suffix;
    vm.runInContext("render({...gate,diagnostic_build_id:trialId})",context);
    assert.match(element('work-title').textContent,/v39.*荷重移行予測/);
    assert.match(element('work-description').textContent,/実際の電流指令はv19/);
  }
  vm.runInContext("render({...gate,diagnostic_build_id:'walking-support-transfer-v39-carry2'})",context);
  assert.match(element('work-title').textContent,/carry2.*足裏円弧/);
  vm.runInContext("render({...gate,diagnostic_build_id:'walking-support-transfer-v39-turn1', gait_turn:{end_ms:6901,reason:'confirmed',left_deg:6.6,right_deg:.3,pair_yaw_delta_deg:-5.7,weak_side:1,confirmed_side:1,confirmed_ms:6901}})",context);
  assert.match(element('work-title').textContent,/turn1/);
  assert.match(element('gait-turn').textContent,/6.90秒.*右足.*旋回を確認.*2周期.*-5.7°/);
  vm.runInContext("render({...gate,diagnostic_build_id:'walking-support-transfer-v39-turn2'})",context);
  assert.match(element('work-title').textContent,/turn2/);
  vm.runInContext("render({...gate,diagnostic_build_id:'walking-support-transfer-v39-turn3'})",context);
  assert.match(element('work-title').textContent,/turn3/);
  vm.runInContext("renderGaitTurn({end_ms:9550,reason:'coverage',confirmed_side:1,confirmed_ms:6901})",context);
  assert.match(element('gait-turn').textContent,/観測が不足.*前回.*6.90秒/);
  vm.runInContext("renderOffline()",context);
  assert.strictEqual(element('gait-turn').hidden,true);
  vm.runInContext("render({...gate,diagnostic_build_id:'walking-support-transfer-v39-carry2'})",context);
  assert.match(element('work-description').textContent,/主入力用の時間/);
  assert.strictEqual(element('work-start').disabled,false);
  gate.diagnostic_build_id='walking-return-energy-v17-rangefix1';
  Object.assign(gate.foot,{left_in_range:false,left_control_range_ok:true,left_extrapolation_deg:3,control_extension_deg:5});
  vm.runInContext('adoptStatus(gate);render(gate)',context);
  assert.strictEqual(element('work-start').disabled,false);
  assert.match(element('foot-status').textContent,/許容範囲内.*3\.0°の外挿/);
  assert.doesNotMatch(element('foot-status').textContent,/設定範囲外/);
  gate.foot.left_control_range_ok=false;gate.foot.left_extrapolation_deg=8;
  gate.foot_control_ready=false;gate.foot_control_reason='left_out_of_range';
  vm.runInContext('adoptStatus(gate);render(gate)',context);
  assert.strictEqual(element('work-start').disabled,true);
  assert.match(element('work-start-status').textContent,/許容5°を超え/);
  vm.runInContext('latest=null;lastSeen=0;controls()',context);
  assert.strictEqual(element('work-start').disabled,true);
  assert.match(element('work-start-status').textContent,/通信/);
  gate.diagnostic_build_id='walking-support-transfer-v39-walk4';
  gate.foot_periodic=footPacket;
  vm.runInContext('adoptStatus(gate);render(gate)',context);
  assert.match(element('work-description').textContent,/左足に支持が移った後/);
  assert.doesNotMatch(element('work-description').textContent,/選んで|比較|v14/);
  assert.strictEqual(element('work-start').disabled,true);
  const startRequests=[];
  context.fetch=async(path,opts)=>{
    assert.strictEqual(opts.method,'POST');
    startRequests.push(new URL(path,'http://device'));
    return {ok:true,text:async()=>'offline_run_queued'};
  };
  await element('work-start').onclick();
  assert.strictEqual(startRequests.length,0);
  gate.foot_control_ready=true;
  element('input-current').value='300';
  vm.runInContext('adoptStatus(gate);render(gate)',context);
  assert.strictEqual(element('work-start').disabled,false);
  assert.match(element('work-start-status').textContent,/歩行を開始できます/);
  await element('work-start').onclick();
  await element('work-start').onclick();
  assert.strictEqual(startRequests.length,1);
  assert.strictEqual(startRequests[0].pathname,'/start-energy-control-autonomous');
  assert.deepStrictEqual(Object.fromEntries(startRequests[0].searchParams),
    {target_deg:'8',input_peak_percent:'0',input_current_mA:'300',brake_max_ms:'40',peak_adaptation:'0',energy_model:'work_control'});
  assert.match(element('run-model').textContent,/開始要求.*左支持に移った後/);
  context.fetch=async()=>({ok:true,json:async()=>({periodic_run:true,foot_periodic:footPacket,walking_profile:'left_transfer_carry_v4',
    state:'RUNNING_BATCH_SWEEP',remaining_ms:24000,pitch_deg:1,motor_mA:0,actual_mA:0})});
  await vm.runInContext('refresh()',context);
  assert.match(element('run-model').textContent,/中心通過後の主入力/);
  assert.match(element('run-percent').textContent,/必要な通電時間/);
  assert.match(element('work-start-status').textContent,/歩行測定中/);
  for (const [profile,text] of [['left_transfer_carry_v3',/左補助の後/],['left_transfer_carry_v2',/中心を越える前後/],['left_transfer_carry_v1',/左支持に移った後/],['left_return_history_v1',/左への戻り/],['left_return_prediction_v1',/支持移行を予測/],['right_arrival_v1',/到達エネルギー/]]) {
    vm.runInContext(`renderWalkingProfile('${profile}')`,context);
    assert.match(element('run-model').textContent,text);
  }
  console.log('single walking start, camera gate, queued and reported support transfer, preserved historical run display PASS');
  console.log('v17 Start: detected but uncalibrated/zero pending/stale explanations, actual gate, null/offline and release text PASS');
  console.log('neutral-zero gate, frozen preview metadata/pixels, five bounded chunks, whole CRC and token failure recovery PASS');
  console.log('foot failure reasons, tilted/range/terminal display, body timeout, status recovery, stale authority and corrupt chunks PASS');
})().catch(e=>{console.error(e);process.exitCode=1;});
