'use strict';
// Actual browser transfer code + actual C++ export fixture. Network failures are
// injected here; this does not simulate the ESP32 radio or desktop Wi-Fi stack.
const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const file=fs.readFileSync('/tmp/runtime-fixture.rwlog'),elements=new Map();
const element=id=>{if(!elements.has(id))elements.set(id,{textContent:'',style:{}});return elements.get(id);};
const manifest={phase:'ready',token:'0123456789abcdef',bytes:file.length,chunk_bytes:4096,
  crc32:file.readUInt32LE(file.length-4),filename:'review.rwlog'};
let disconnected=true,saved=0,offsets=[];
const context=vm.createContext({document:{getElementById:element,createElement:()=>({click:()=>++saved})},
  setTimeout:(fn,ms)=>{const t=setTimeout(fn,ms<3000?0:ms);if(ms>=3000)t.unref();return t;},clearTimeout,
  AbortController,Date,DataView,Uint8Array,TextDecoder,Map,Blob,URL,
  fetch:async path=>{
    if(path==='/export/prepare')return {ok:true,text:async()=>'preparing'};
    if(path==='/export/manifest')return {ok:true,json:async()=>({...manifest})};
    const u=new URL(path,'http://192.168.4.1');assert.equal(u.pathname,'/export/chunk');
    assert.equal(u.searchParams.get('token'),manifest.token);
    const offset=Number(u.searchParams.get('offset')),length=Number(u.searchParams.get('length'));
    offsets.push(offset);if(disconnected&&offset>=8192)throw Error('simulated disconnect');
    const payload=new Uint8Array(file.subarray(offset,offset+length));context.payload=payload;
    const packet=new Uint8Array(length+16),header=new DataView(packet.buffer);
    header.setUint32(0,0x31484346,true);header.setUint32(4,offset,true);header.setUint32(8,length,true);
    header.setUint32(12,vm.runInContext('crc32(payload)',context),true);packet.set(payload,16);
    return {ok:true,arrayBuffer:async()=>packet.buffer};
  }});
vm.runInContext(fs.readFileSync('site/wheel-work.js','utf8')+'\n'+fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/,''),context);
element('work-inertia').value='0.000028';element('work-sign').value='-1';
vm.runInContext("latest={state:'ESTOP',export_phase:'ready',running:false,ready:false,downloadable:true,controller_fresh:true,command:{pending:false},foot:{},upright:{}};lastSeen=Date.now();",context);
(async()=>{
  await vm.runInContext('download()',context);
  assert.equal(saved,0);assert.equal(vm.runInContext('completedFile',context),null);
  assert.equal(vm.runInContext('memoryCache.size',context),2);
  assert.equal(vm.runInContext('transferRunning',context),false);
  assert.equal(element('download').disabled,false);
  disconnected=false;offsets=[];await vm.runInContext('download()',context);
  assert.equal(offsets[0],8192);assert(!offsets.includes(0)&&!offsets.includes(4096));
  assert(Buffer.from(vm.runInContext('completedFile',context)).equals(file));assert.equal(saved,1);
  assert.equal(vm.runInContext('completedFoot.length',context),768);assert.equal(element('csv').disabled,false);
  assert.match(element('transfer').textContent,/CRC検証完了/);
  assert.equal(vm.runInContext('wheelWorkReport.speed_attempts',context),1);
  assert.equal(vm.runInContext('wheelWorkReport.measurement.observed_net_j',context),null);
  assert.match(element('work-result').textContent,/review.rwlog/);
  assert.equal(vm.runInContext('footCsv(completedFoot)',context).split('\r\n').filter(Boolean).length,769);
  console.log(`RWLOG ${file.length} bytes: 8192-byte interruption/resume, exact content/CRC, 768 foot CSV rows PASS`);
})().catch(e=>{console.error(e);process.exitCode=1;});
