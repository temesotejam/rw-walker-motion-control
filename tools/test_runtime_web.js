'use strict';
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const elements = new Map();
function canvasContext() {return new Proxy({}, {get:(_target,name)=>name==='createImageData'
  ? (w,h)=>({data:new Uint8Array(w*h*4)}) : ()=>{}});}
function element(id) {if(!elements.has(id)) elements.set(id, {textContent:'',style:{},width:640,height:480,getContext:canvasContext}); return elements.get(id);}
const code=fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/, '');
const context=vm.createContext({document:{getElementById:element,createElement:()=>({getContext:canvasContext})},setTimeout,clearTimeout,AbortController,
  Date,DataView,Uint8Array,TextDecoder,Map,console,btoa:s=>Buffer.from(s,'binary').toString('base64'),
  atob:s=>Buffer.from(s,'base64').toString('binary'),fetch:()=>{throw Error('unset');}});
vm.runInContext(code,context);
(async()=>{
  // Headers succeed immediately, response body never completes until aborted.
  context.fetch=async(_path,opts)=>({ok:true,json:()=>new Promise((_,reject)=>opts.signal.addEventListener('abort',()=>reject(Error('body_aborted'))))});
  await assert.rejects(vm.runInContext("request('/status.json',{timeout:25})",context),/body_aborted/);
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
  assert(!code.includes('steering-status'));
  const html=fs.readFileSync('web/index.html','utf8');
  assert.match(html,/id="probe-batch"/);
  assert.match(html,/id="inverse-start"/);
  assert.doesNotMatch(html,/id="probe-(0|15|30)"/);
  assert.match(html,/ZEROクロスで必要な入力時間を計算/);
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
  console.log('neutral-zero gate, frozen preview metadata/pixels, five bounded chunks, whole CRC and token failure recovery PASS');
  console.log('foot failure reasons, tilted/range/terminal display, body timeout, status recovery, stale authority and corrupt chunks PASS');
})().catch(e=>{console.error(e);process.exitCode=1;});
