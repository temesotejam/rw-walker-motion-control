// Execute the firmware's actual embedded script with delayed network responses.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const page=fs.readFileSync(path.join(__dirname,'../src/web_ui.cpp'),'utf8');
const source=page.match(/<script>([\s\S]*?)<\/script>/)[1];
const html=page.split('<script>')[0],elements=new Map(),requests=[];
for(const match of html.matchAll(/<(\w+)\b([^>]*\bid="([^"]+)"[^>]*)>/g)){
 elements.set(match[3],{tagName:match[1].toUpperCase(),disabled:/\bdisabled\b/.test(match[2]),value:'',options:[],classList:{toggle(){}}});
}
const get=id=>{assert(elements.has(id),'Missing DOM element: '+id);return elements.get(id);};
assert(!elements.has('timingCompensation'));assert(html.includes('遅延補償：3 ms固定'));
assert(!source.includes('setTimingCompensation'));assert(!source.includes('timing_ms'));
const context=vm.createContext({document:{getElementById:get,activeElement:null},fetch:(url,options)=>new Promise(resolve=>requests.push({url,options,resolve})),setTimeout:()=>1,clearTimeout(){},setInterval(){},AbortController,alert(){},confirm:()=>true,console});
const run=code=>vm.runInContext(code,context);
const tick=async()=>{for(let i=0;i<10;i++)await Promise.resolve();};
const ready={state:'FINISHED',running:false,autonomous_timing_compensation_ms:3,autonomous_timing_compensation_selectable:false};
async function reply(req,data,ok=true){assert(req);req.resolve({ok,json:async()=>data,text:async()=>String(data)});await tick();}
(async()=>{
 vm.runInContext(source,context);
 assert(get('energy').disabled);
 await reply(requests.shift(),ready);assert(!get('energy').disabled);
 // An older status reply cannot unlock Start while a start request is pending.
 run('refresh()');const stale=requests.shift();
 run('startEnergy()');const start=requests.shift();
 assert.strictEqual(start.url,'/start-energy-control-autonomous');assert.strictEqual(start.options.method,'POST');
 assert(get('energy').disabled);run('startEnergy()');assert.strictEqual(requests.length,0);
 await reply(stale,ready);assert(get('energy').disabled);
 await reply(start,'ok');assert(get('energy').disabled);assert(!get('stop').disabled);
 await reply(requests.shift(),{running:true,state:'START_SYNC'});assert(get('energy').disabled);assert(!get('stop').disabled);
 // Emergency stop remains usable; completion restores normal controls.
 run('postStop()');const stop=requests.shift();assert.strictEqual(stop.url,'/stop');
 await reply(stop,'ok');await reply(requests.shift(),ready);assert(!get('energy').disabled);
 // Rejected starts and failed status requests require a fresh status to unlock.
 run('startEnergy()');await reply(requests.shift(),'not_upright',false);
 assert(get('energy').disabled);await reply(requests.shift(),ready);assert(!get('energy').disabled);
 run('refresh()');await reply(requests.shift(),'unavailable',false);assert(get('energy').disabled);
 run('startEnergy()');assert.strictEqual(requests.length,0);
 run('refresh()');await reply(requests.shift(),{...ready,downloading:true});assert(get('energy').disabled);
 run('refresh()');await reply(requests.shift(),ready);assert(!get('energy').disabled);
 console.log('V46aj UI PASS: fixed label, no selector, start without timing argument, stale status/double-start guards, stop, failed requests');
})().catch(error=>{console.error(error);process.exitCode=1;});
