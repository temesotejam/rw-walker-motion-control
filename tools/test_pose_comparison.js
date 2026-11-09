'use strict';
const assert = require('assert'), fs=require('fs'), vm=require('vm');
const P=require('../web/pose_comparison.js');
const clone=o=>JSON.parse(JSON.stringify(o));
function qFromEuler(roll,pitch=0,yaw=0) {
  const [r,p,y]=[roll,pitch,yaw].map(v=>v*Math.PI/360), [cr,sr,cp,sp,cy,sy]=[Math.cos(r),Math.sin(r),Math.cos(p),Math.sin(p),Math.cos(y),Math.sin(y)];
  return {w:cr*cp*cy+sr*sp*sy,x:sr*cp*cy-cr*sp*sy,y:cr*sp*cy+sr*cp*sy,z:cr*cp*sy-sr*sp*cy};
}
function sample(i,roll=0,foot=0,start=10000000) {
  const t=start+i*800000;
  return {revision:'0.47.6-pose-comparison',boot_id:123,run_id:0,state_id:2,
    format:'gray8',width:160,height:120,source_width:320,source_height:240,bytes:19200,
    token:i+1,sequence:i+1,frame_us:t-80000,delivered_us:t,timestamp_valid:true,age_ms:30,crc32:0,
    zero_ready:true,right_valid:true,left_valid:true,right_in_range:true,left_in_range:true,
    right_zero_x:171,left_zero_x:171,right_deg:foot,left_deg:foot,
    right:{valid:true,x:171-foot/.167779119,candidates:1,reason:'detected'},
    left:{valid:true,x:171-foot/.162645305,candidates:1,reason:'detected'},
    mekf:{valid:true,fresh:true,frame:'mekf',euler_order:'ZYX',age_us:30000,sample_us:t,
      quaternion:qFromEuler(roll),inputs:{valid:true,frame:'mekf',axis_order:'xyz',accel_age_us:32000,
        accel_g:[0,Math.sin(roll*Math.PI/180),Math.cos(roll*Math.PI/180)],gyro_dps:[.1,-.2,.3],gyro_bias_dps:[.1,-.2,.3]}}};
}
const series=(roll=0,foot=0,start=10000000)=>Array.from({length:5},(_,i)=>sample(i,roll,foot,start));
const base=P.summarize(series(),true), pose=P.summarize(series(-20,20,20000000));
const c=P.compare(base,pose);assert(Math.abs(c.right_residual_deg)<1e-10);assert(c.planar_check);
assert(Math.abs(P.compare(base,P.summarize(series(0,0,30000000))).delta.roll_deg)<1e-10);
for (const patch of [m=>m.mekf.fresh=false,m=>m.mekf.inputs.accel_g=[null,0,1],m=>m.mekf.inputs.accel_age_us=600000,
  m=>m.mekf.quaternion.w=0,m=>m.left_valid=false,m=>m.state_id=3,m=>m.timestamp_valid=false,
  m=>m.mekf.inputs.gyro_dps=[8,0,0],m=>m.age_ms=500]) {
  const s=series();patch(s[2]);assert.throws(()=>P.summarize(s));
}
for (const patch of [s=>s[2].boot_id++,s=>s[2].run_id++,s=>s[2].right_zero_x++,s=>s[2].frame_us=s[1].frame_us,
  s=>s[2].sequence=s[1].sequence,s=>s[2].left.x+=4,s=>s[2].mekf.quaternion=qFromEuler(3)]) {
  const s=series();patch(s);assert.throws(()=>P.summarize(s));
}
assert.throws(()=>P.summarize(series(-20,20),true),/直立/);
assert.throws(()=>P.compare(base,{...pose,boot_id:456}),/条件/);
assert.throws(()=>P.compare(base,base),/条件/);
const range=series(-20,20,20000000);range[1].right_in_range=false;
assert(P.compare(base,P.summarize(range)).flags.includes('outside_range'));
assert(P.compare(base,P.summarize(range)).planar_check); // range is diagnostic, not rejection
const rotated=series(-20,20,20000000);rotated.forEach(m=>m.mekf.quaternion=qFromEuler(-20,3,8));
assert.deepStrictEqual(P.compare(base,P.summarize(rotated)).flags,['sideways_changed']);
// Yaw wrapping and the two equivalent quaternion signs cannot become a 360-degree motion.
const wrap=series();wrap.forEach((m,i)=>m.mekf.quaternion=qFromEuler(0,0,i%2?179.8:-179.8));
assert(!('yaw_deg' in P.summarize(wrap)));
assert(Math.abs(P.summarize(wrap).roll_deg)<1e-9);
const sign=series();sign[2].mekf.quaternion={w:-1,x:0,y:0,z:0};assert.strictEqual(P.summarize(sign).roll_deg,0);
const drift=series();drift.forEach((m,i)=>m.mekf.quaternion=qFromEuler(0,0,i*.7));
const driftBase=P.summarize(drift,true);assert(!('yaw_deg' in driftBase));
assert(P.compare(driftBase,pose).planar_check); // Heading does not enter tilt validation.
// Actual 0.47.5 endpoint values; no claim these supplied files were multi-frame holds.
const b={...base,...P.orientation({w:.999103,x:.016590,y:.004115,z:-.038735}),right_deg:-.0884,left_deg:-.0062};
const p={...pose,...P.orientation({w:.986237,x:-.161376,y:.002497,z:.035901}),right_deg:21.5289,left_deg:21.0602};
const observed=P.compare(b,p);
assert(Math.abs(observed.right_residual_deg-1.183247)<.001);
assert(Math.abs(observed.left_residual_deg-.632347)<.001);
assert(!('yaw_deg' in observed.delta));
console.log('Static-pose geometry, observed endpoints, movement/stale/reboot/zero/duplicate rejection with heading excluded PASS');

// Exercise the real browser capture: failed samples/CRC/cancel never replace a
// saved baseline; all five metadata samples and the final image stay matched.
function el(){return {textContent:'',disabled:false,style:{},replaceChildren(){},appendChild(){}};}
const elements=new Map();const get=id=>{if(!elements.has(id))elements.set(id,el());return elements.get(id);};
const context=vm.createContext({document:{getElementById:get,createElement:el},Date,Uint8Array,DataView,AbortController,
  btoa:s=>Buffer.from(s,'binary').toString('base64'),atob:s=>Buffer.from(s,'base64').toString('binary'),
  setTimeout:(fn,ms)=>ms<1000?setTimeout(fn,0):setTimeout(fn,ms),clearTimeout,console});
vm.runInContext(fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/,''),context);
vm.runInContext('showEvidence = e => {previewEvidence=e}',context);
context.good={input_peak_percent:0,input_current_mA:1200,brake_max_ms:50,peak_adaptation_enabled:true,state:'READY_TO_MEASURE',revision:sample(0).revision,boot_id:123,run_id:0,export_phase:'empty',running:false,
  ready:true,downloadable:false,controller_fresh:true,command:{pending:false,completed:0,submitted:0},
  upright_reference:{valid:true,capturing:false,abs_pitch_deg:2.2,relative_pitch_deg:0,revision:1},
  foot:{zero_ready:true,preview_available:true},upright:{},mekf:sample(0).mekf};
vm.runInContext('adoptStatus(good)',context);
let index=0,kind='base',badCrc=false,cancel=false,requests=0;
const bytes=new Uint8Array(19200);context.bytes=bytes;const checksum=vm.runInContext('crc32(bytes)',context);
context.fetch=async path=>{
  ++requests;
  if(path==='/vision/capture'){
    assert(get('start').disabled && get('preview').disabled);assert(!get('stop').disabled);
    if(cancel)vm.runInContext('poseCancelled=true',context);
    const m=sample(index++,kind==='base'?0:-20,kind==='base'?0:20,kind==='base'?10000000:20000000);
    m.crc32=badCrc?(checksum^1)>>>0:checksum;
    return {ok:true,json:async()=>m};
  }
  const url=new URL(path,'http://device'),offset=+url.searchParams.get('offset'),length=+url.searchParams.get('length');
  assert.strictEqual(+url.searchParams.get('token'),5);
  const packet=new Uint8Array(16+length),view=new DataView(packet.buffer);context.part=packet.subarray(16);
  view.setUint32(0,0x31484346,true);view.setUint32(4,offset,true);view.setUint32(8,length,true);
  view.setUint32(12,vm.runInContext('crc32(part)',context),true);
  return {ok:true,arrayBuffer:async()=>packet.buffer};
};
(async()=>{
  await vm.runInContext('capturePose(true)',context);
  assert.strictEqual(requests,10);assert.strictEqual(vm.runInContext('poseSession.baseline.frames.length',context),5);
  assert.strictEqual(vm.runInContext('poseSession.baseline.image.sequence',context),5);
  assert(get('pose-base').disabled);assert(!get('pose-add').disabled);assert(get('pose-reset').disabled);
  kind='pose';index=0;await vm.runInContext('capturePose(false)',context);
  assert.strictEqual(vm.runInContext('poseSession.poses.length',context),1);
  assert(Math.abs(vm.runInContext('poseSession.poses[0].comparison.right_residual_deg',context))<1e-9);
  index=0;badCrc=true;await vm.runInContext('capturePose(false)',context);
  assert.match(get('pose-status').textContent,/CRC/);assert.strictEqual(vm.runInContext('poseSession.poses.length',context),1);
  badCrc=false;cancel=true;index=0;await vm.runInContext('capturePose(false)',context);
  assert.match(get('pose-status').textContent,/中止/);assert.strictEqual(vm.runInContext('poseRunning',context),false);
  assert(!get('start').disabled && !get('pose-save').disabled);
  vm.runInContext('lastSeen=0;controls()',context);assert(get('pose-add').disabled);assert(!get('pose-save').disabled);
  vm.runInContext('poseSaved=true;controls()',context);assert(!get('pose-reset').disabled);
  console.log('Multi-pose capture, coherent evidence, CRC/cancel retention, command exclusion and offline save PASS');
})().catch(e=>{console.error(e);process.exitCode=1;});
