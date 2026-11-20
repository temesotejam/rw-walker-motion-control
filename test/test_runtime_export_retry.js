'use strict';
// Execute production browser code with transport/body aborts and bad packets.
// Timers are accelerated; this is not a measurement of Wi-Fi or ESP32 latency.
const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const fixture=fs.readFileSync(process.argv[2]);
const sources=fs.readFileSync('web/wheel_work.js','utf8')+'\n'+fs.readFileSync('web/pose_comparison.js','utf8')+'\n'+
  fs.readFileSync('web/runtime.js','utf8').replace(/poll\(\);\s*$/,'');
const manifest={phase:'ready',token:'0123456789abcdef',bytes:fixture.length,chunk_bytes:4096,
  crc32:fixture.readUInt32LE(fixture.length-4),filename:'retry.rwlog'};

function harness(scenario={}) {
  const elements=new Map(),requests=[],saves=[];
  const element=id=>{if(!elements.has(id))elements.set(id,{textContent:'',style:{}});return elements.get(id);};
  let context,clock=0,prepareCount=0,manifestCount=0;
  class Clock extends Date {static now(){return clock;}}
  const abort=signal=>new Promise((_,reject)=>{
    if(signal.aborted)reject(new DOMException('signal is aborted without reason','AbortError'));
    else signal.addEventListener('abort',()=>reject(new DOMException('signal is aborted without reason','AbortError')),{once:true});
  });
  const packet=(offset,length)=>{
    const bytes=new Uint8Array(length+16),v=new DataView(bytes.buffer);
    v.setUint32(0,0x31484346,true);v.setUint32(4,offset,true);v.setUint32(8,length,true);
    const payload=new Uint8Array(fixture.subarray(offset,offset+length));context.testPayload=payload;
    v.setUint32(12,vm.runInContext('crc32(testPayload)',context),true);bytes.set(payload,16);return bytes;
  };
  context=vm.createContext({document:{getElementById:element,createElement:()=>({click(){saves.push({name:this.download,url:this.href});}})},
    setTimeout:(fn,ms)=>{const timer=setTimeout(()=>{if(ms<3000)clock+=ms;fn();},ms>=3000?15:0);if(ms===60000)timer.unref();return timer;},
    clearTimeout,AbortController,Date:Clock,DataView,Uint8Array,TextDecoder,Map,Blob,
    URL:{createObjectURL:blob=>blob,revokeObjectURL:()=>{}},
    fetch:async(path,opts)=>{
      requests.push(path);
      if(path==='/export/prepare'){
        ++prepareCount;
        if (prepareCount <= (scenario.disconnectedPrepares || 0)) throw new TypeError('Failed to fetch');
        if(scenario.transientPrepare && prepareCount===1)return {ok:false,status:409,text:async()=>'run_preparing'};
        if(scenario.prepareAbort && prepareCount===1)return abort(opts.signal);
        if(scenario.prepareStatus)return {ok:false,status:scenario.prepareStatus,text:async()=>'blocked'};
        return {ok:true,text:async()=>'preparing'};
      }
      if(path==='/export/manifest'){
        ++manifestCount;
        if(scenario.pauseManifest){element('cancel').onclick();return abort(opts.signal);}
        if(scenario.manifestAbort==='always' || scenario.manifestAbort===manifestCount)
          return {ok:true,json:()=>abort(opts.signal)}; // received headers; body aborts
        if(scenario.exportError)return {ok:true,json:async()=>({phase:'error',error:'metadata_build_failed'})};
        if(scenario.buildForever)return {ok:true,json:async()=>({phase:'building',hashed_bytes:0,bytes:0})};
        if(scenario.building && manifestCount===2)return {ok:true,json:async()=>({phase:'building',hashed_bytes:4096,bytes:fixture.length})};
        return {ok:true,json:async()=>({...manifest,...scenario.manifest})};
      }
      const url=new URL(path,'http://192.168.4.1');
      if (url.pathname === '/export/complete') {
        assert.equal(saves.length,1);
        assert.equal(opts.method,'POST');
        assert.equal(url.searchParams.get('token'),manifest.token);
        assert.equal(Number(url.searchParams.get('crc32')),manifest.crc32);
        assert.equal(url.searchParams.get('run_id'),'1');
        if (scenario.ackLost) throw new TypeError('Failed to fetch');
        return {ok:true,text:async()=>'confirmed'};
      }
      assert.equal(url.pathname,'/export/chunk');
      assert.equal(url.searchParams.get('token'),(scenario.manifest??manifest).token ?? manifest.token);
      const offset=Number(url.searchParams.get('offset')),length=Number(url.searchParams.get('length'));
      if(scenario.chunkDisconnect && offset>=8192)throw new TypeError('Failed to fetch');
      if(scenario.chunkStatus)return {ok:false,status:scenario.chunkStatus,text:async()=>'export_token_or_range_mismatch'};
      const bytes=packet(offset,length);if(scenario.badCrc)bytes[16]^=1;
      return {ok:true,arrayBuffer:async()=>bytes.buffer};
    }});
  vm.runInContext(sources,context);
  element('work-inertia').value='0.000028';element('work-sign').value='-1';
  vm.runInContext("latest={state:'FINISHED',boot_id:123,run_id:1,export_phase:'building',running:false,ready:false,downloadable:true,controller_fresh:true,command:{pending:false},foot:{},upright:{}};lastSeen=Date.now();",context);
  const run=()=>vm.runInContext('download()',context);
  const diagnostic=()=>JSON.parse(vm.runInContext('JSON.stringify(diagnosticSnapshot())',context));
  return {context,element,requests,saves,run,diagnostic,scenario,packet,
    get prepareCount(){return prepareCount;},get manifestCount(){return manifestCount;}};
}

(async()=>{
  // The recorded failure: after reboot the manifest becomes empty. Do not wait
  // 120 seconds or promise resumption of data that no longer exists on device.
  for (const changed of [{phase:'empty'}, {boot_id:999}, {boot_id:999,phase:'ready'}, {boot_id:123, run_id:2}]) {
    const lost=harness({manifest:changed});await lost.run();
    assert.equal(lost.manifestCount,1);assert.equal(lost.saves.length,0);
    assert.equal(lost.diagnostic().browser_transfer.source_lost,true);
    assert.match(lost.element('transfer').textContent,/再起動/);
    assert.match(lost.element('transfer').textContent,/続きは取得できません/);
    assert(!lost.requests.some(p=>p.startsWith('/export/chunk')));
  }
  const same=harness({manifest:{boot_id:123,metadata_bytes:12345}});await same.run();
  assert.equal(same.diagnostic().browser_transfer.phase,'complete');
  assert(!same.requests.some(path=>path.startsWith('/export/complete')));
  const reconnect=harness({disconnectedPrepares:6});
  vm.runInContext("latest.diagnostic_build_id='walking-support-transfer-v39-webfix2'",reconnect.context);
  await reconnect.run();
  assert.equal(reconnect.prepareCount,7);
  assert.equal(reconnect.diagnostic().browser_transfer.phase,'complete');
  for (const ackLost of [false,true]) {
    const trial=harness({manifest:{boot_id:123,run_id:1,completion_ack:true},ackLost});
    await trial.run();
    assert.equal(trial.diagnostic().browser_transfer.completion_ack,!ackLost);
    assert.equal(trial.diagnostic().browser_transfer.phase,'complete');
    assert.equal(trial.requests.filter(path=>path.startsWith('/export/complete')).length,1);
    assert(Buffer.from(vm.runInContext('completedFile',trial.context)).equals(fixture));
  }
  // Both formerly-unretried phases fail once. A timed-out POST may already have
  // queued export; retry the idempotent endpoint, then wait through building.
  let h=harness({prepareAbort:true,manifestAbort:1,building:true});await h.run();
  assert.equal(h.prepareCount,2);assert.equal(h.manifestCount,3);
  assert(Buffer.from(vm.runInContext('completedFile',h.context)).equals(fixture));
  let d=h.diagnostic();assert.equal(d.browser_transfer.phase,'complete');
  assert.equal(d.browser_transfer.retries,2);assert.equal(d.browser_transfer.confirmed_bytes,fixture.length);
  assert.equal(d.browser_transfer.last_failure.path,'/export/manifest');
  assert.equal(d.browser_transfer.last_failure.code,'request_timeout');
  assert.equal(d.browser_transfer.manifest.phase,'ready');assert.equal(d.state,'FINISHED');
  assert.equal(vm.runInContext('transferRunning',h.context),false);
  assert.equal(h.saves.length,1);assert.match(h.element('transfer').textContent,/CRC検証完了/);
  await h.element('diagnostics').onclick();
  const savedDiagnostic=JSON.parse(await h.saves[1].url.text());
  assert.equal(savedDiagnostic.browser_transfer.phase,'complete');

  // Exhausted manifest retries must retain both the verified cache and any
  // previously completed file; retries and stage are now in the saved evidence.
  h=harness({chunkDisconnect:true});await h.run();
  assert.equal(vm.runInContext('memoryCache.size',h.context),2);
  assert.equal(h.diagnostic().browser_transfer.confirmed_bytes,8192);
  h.scenario.chunkDisconnect=false;h.scenario.manifestAbort='always';
  vm.runInContext("completedFile=new Uint8Array([1,2,3]);completedName='previous.rwlog'",h.context);
  const before=h.manifestCount;await h.run();
  assert.equal(h.manifestCount-before,5);assert.equal(vm.runInContext('memoryCache.size',h.context),2);
  assert.deepEqual(Array.from(vm.runInContext('completedFile',h.context)),[1,2,3]);
  d=h.diagnostic().browser_transfer;assert.equal(d.phase,'error');assert.equal(d.retries,4);
  assert.equal(d.failed_phase,'manifest');assert.match(h.element('transfer').textContent,/ログ確定状況の確認/);
  assert.equal(d.last_failure.path,'/export/manifest');assert.match(d.error,/8秒/);
  assert(!h.element('transfer').textContent.includes('signal is aborted'));
  h.scenario.manifestAbort=false;const resumeStart=h.requests.length;await h.run();
  const chunks=h.requests.slice(resumeStart).filter(p=>p.startsWith('/export/chunk'));
  assert.equal(new URL(chunks[0],'http://host').searchParams.get('offset'),'8192');
  assert.equal(h.diagnostic().browser_transfer.cached_bytes,8192);
  assert(Buffer.from(vm.runInContext('completedFile',h.context)).equals(fixture));

  for(const scenario of [{prepareStatus:409},{prepareStatus:400}]){
    h=harness(scenario);await h.run();assert.equal(h.prepareCount,1);
    assert.equal(h.diagnostic().browser_transfer.phase,'error');assert.equal(h.saves.length,0);
    assert.equal(h.diagnostic().browser_transfer.failed_phase,'prepare');
    assert.match(h.element('transfer').textContent,/本体へのログ確定要求/);
  }
  h=harness({transientPrepare:true});await h.run();assert.equal(h.prepareCount,2);
  assert.equal(h.diagnostic().browser_transfer.phase,'complete');

  h=harness({transientPrepare:true});
  const shortTimer=h.context.setTimeout;
  let waitingForRetry;
  const retryStarted=new Promise(resolve=>{waitingForRetry=resolve;});
  h.context.setTimeout=(fn,ms)=>{
    if(ms!==1000)return shortTimer(fn,ms);
    waitingForRetry();
    return setTimeout(fn,20000);
  };
  const cancellable=h.run();
  await retryStarted;h.element('cancel').onclick();await cancellable;
  assert.equal(h.prepareCount,1);assert.equal(h.manifestCount,0);
  assert.equal(h.diagnostic().browser_transfer.phase,'paused');

  // An in-flight browser poll owns the only client until its body settles.
  // Even a forced refresh while downloading must not start a second request.
  h=harness();
  const productionFetch=h.context.fetch;
  let releaseStatus, releasePrepare, enterPrepare, statusCount=0;
  const prepareEntered=new Promise(resolve=>{enterPrepare=resolve;});
  h.context.fetch=async (path,options)=>{
    if(path==='/status.json'){
      ++statusCount;
      if(statusCount===1)await new Promise(resolve=>{releaseStatus=resolve;});
      throw new TypeError('Failed to fetch');
    }
    if(path==='/export/prepare'){
      enterPrepare();
      await new Promise(resolve=>{releasePrepare=resolve;});
    }
    return productionFetch(path,options);
  };
  const pendingPoll=vm.runInContext('refresh()',h.context);
  const downloading=h.run();
  assert.equal(h.prepareCount,0);assert.equal(releasePrepare,undefined);
  await vm.runInContext('refresh(true)',h.context);assert.equal(statusCount,1);
  releaseStatus();await pendingPoll;await prepareEntered;
  await vm.runInContext('refresh(true)',h.context);assert.equal(statusCount,1);
  releasePrepare();await downloading;assert.equal(h.saves.length,1);
  await vm.runInContext('refresh()',h.context);assert.equal(statusCount,2);
  h=harness({exportError:true});await h.run();assert.equal(h.manifestCount,1);
  assert.match(h.element('transfer').textContent,/metadata_build_failed/);
  h=harness({buildForever:true});await h.run();
  assert.match(h.element('transfer').textContent,/120秒/);assert(h.manifestCount<250);
  assert.equal(h.diagnostic().browser_transfer.manifest.phase,'building');

  // Cancellation interrupts an active response body and does not retry it.
  h=harness({pauseManifest:true});await h.run();assert.equal(h.manifestCount,1);
  assert.equal(h.diagnostic().browser_transfer.phase,'paused');assert.equal(h.saves.length,0);
  assert.equal(vm.runInContext('transferRunning',h.context),false);

  // CRC corruption is never cached or published; token rejection is permanent.
  h=harness({badCrc:true});await h.run();
  assert.equal(h.requests.filter(p=>p.startsWith('/export/chunk')).length,5);
  assert.equal(vm.runInContext('memoryCache.size',h.context),0);assert.equal(h.saves.length,0);
  h=harness({chunkStatus:409});await h.run();
  assert.equal(h.requests.filter(p=>p.startsWith('/export/chunk')).length,1);

  // A new export token must not reuse bytes cached for a prior sealed export.
  h=harness({manifest:{token:'fedcba9876543210'}});
  vm.runInContext("memoryCache.set('0123456789abcdef:0',new Uint8Array([0]))",h.context);
  await h.run();assert(Buffer.from(vm.runInContext('completedFile',h.context)).equals(fixture));
  assert.equal(h.diagnostic().browser_transfer.cached_bytes,0);
  assert(!vm.runInContext("memoryCache.has('0123456789abcdef:0')",h.context));

  // The v20 raw receipt timestamps survive sparse supplemental expansion.
  const expanded=vm.runInContext(`expandFootTable({fields:['delivered_us','control_observed_us'],rows:[[100,120],[200,230],[300,0]]},
    {version:1,rows:[[0,20]]})`,h.context);
  assert.deepEqual(Array.from(expanded,x=>x.control_observed_us),[120,230,null]);
  assert(h.requests.every(p=>p.startsWith('/export/')));
  console.log('Export retry PASS: prepare/header + manifest/body timeout, building, bounded failure, cache resume, cancellation, CRC/token rejection, saved diagnostics, v20 receipts');
})().catch(error=>{console.error(error);process.exitCode=1;});
