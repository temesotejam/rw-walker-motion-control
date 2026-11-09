'use strict';
const assert=require('node:assert/strict'),fs=require('fs'),W=require('../site/wheel-work.js');
const J=2.8e-5,D=Math.PI/180,R=Math.PI/30;
const close=(a,b,tol=1e-10)=>assert(Math.abs(a-b)<tol,`${a} != ${b}`);
function rows(rate=()=>60,angle=t=>-6000*(t/1e6)**2,initial=0) {
  return Array.from({length:51},(_,i)=>{
    const t=i*5000;
    return {t,state:3,pulse_id:1,body_t:t,body_stamp_valid:true,body_rate_dps:rate(t),
      speed_t:t,speed_start:t-100,speed_end:t+100,speed_seq:i+1,speed_valid:true,
      speed_rpm:0, // Deliberately unrelated: no RPM fallback or control dependency.
      command_seq:t>=225000?3:t>=25000?2:1,command_t:t>=225000?225000:t>=25000?25000:0,
      applied_mA:t>=25000 && t<225000 ? 300:0,
      position_raw_cdeg:(Math.round(angle(t)*100)+initial)|0,position_t:t,
      position_start:t-100,position_end:t+100,position_seq:i+1,position_valid:true};
  });
}
const run=(r,opts={})=>W.analyze(r,{}, {sign:1,inertia:J,...opts});
let r=run(rows());
assert.equal(r.velocity_source,'position_local_quadratic');
assert.equal(r.position_attempts,51);assert.equal(r.position_velocity_samples,47);
close(r.measurement.observed_net_j,J*60*D*460*R);
close(r.measurement.coverage,.92); // 10 ms at each edge needs centered support.
close(r.pulses[0].observed_net_j,J*60*D*400*R);close(r.pulses[0].coverage,1);
assert(r.pulses[0].complete);
close(run(rows(()=>0)).measurement.observed_net_j,0);
close(run(rows(),{sign:-1}).measurement.observed_net_j,-r.measurement.observed_net_j);
// Stationary relative wheel, changing body speed: absolute rotor spin correction.
let a=rows(t=>60*t/250000,()=>0);
const angular=t=>60*t/250000*D;
close(run(a).measurement.observed_net_j,-.5*J*(angular(240000)**2-angular(10000)**2));
// Duplicate snapshots don't count as new observations.
close(run(rows().flatMap(x=>[x,{...x}])).measurement.observed_net_j,r.measurement.observed_net_j);
// Signed position rollover, valid even at INT32_MIN.
a=rows(()=>60,t=>6000*(t/1e6)**2,2147483628);
close(run(a).measurement.observed_net_j,-r.measurement.observed_net_j);
// No bridging failed reads, missing sequence, long transaction or position reset.
a=rows();a[25].position_valid=false;
let bad=run(a);assert(bad.position_fit_rejected.position_read_invalid>0);
assert(bad.pulses[0].coverage<1);assert(!bad.pulses[0].complete);
a=rows();a.splice(25,1);bad=run(a);assert(bad.position_fit_rejected.position_sequence_gap>0);
a=rows();a[25].position_start-=5000;bad=run(a);assert(bad.position_fit_rejected.position_read_too_long>0);
a=rows();a[25].position_raw_cdeg+=100000;bad=run(a);assert(bad.position_fit_rejected.position_discontinuity>0);
a=rows();a[25].position_t=a[24].position_t;bad=run(a);assert(bad.position_fit_rejected.position_time_gap>0);
a=rows();for(const x of a)x.position_valid=false;
bad=run(a);assert.equal(bad.measurement.observed_net_j,null);
assert.equal(bad.measurement.coverage,0);assert.equal(bad.velocity_source,'position_local_quadratic');
// No extrapolation of body rate or of wheel velocity near endpoints.
a=rows();for(const x of a)x.body_stamp_valid=false;
assert.equal(run(a).measurement.observed_net_j,null);
assert.equal(run(rows().slice(0,4)).measurement.observed_net_j,null);
// Nonuniform acquisition times: use the actual clock, not nominal 5 ms.
a=rows();a.forEach((x,i)=>{
  const t=i*5000+(i%2)*500;
  Object.assign(x,{t,body_t:t,position_t:t,position_start:t-100,position_end:t+100,
    position_raw_cdeg:Math.round(-6000*(t/1e6)**2*100)});
});
close(run(a).measurement.observed_net_j,r.measurement.observed_net_j,1e-7);
// Independent wider-window comparison is available for later sensitivity checks.
assert.equal(run(rows(),{positionFitPoints:9}).position_velocity_samples,43);
assert.match(W.describe(r),/位置の5点近似/);assert.match(W.csv(r.kinematics,r),/fit_span_us/);

// Binary v56 decoding at exact C++ packed offsets and a micros() rollover.
function fixture() {
  const data=rows(),meta=new TextEncoder().encode('{}'),offset=110+meta.length,end=offset+data.length*172;
  const bytes=new Uint8Array(end+4),v=new DataView(bytes.buffer),origin=0xfffff000;
  bytes.set(new TextEncoder().encode('RWLOG01'));v.setUint16(8,56,true);v.setUint16(10,110,true);
  v.setUint32(16,origin,true);v.setUint32(24,meta.length,true);v.setUint32(28,data.length,true);
  v.setUint16(40,172,true);v.setUint32(58,1,true);v.setUint32(62,offset,true);
  for(const p of [66,70,74])v.setUint32(p,end,true);bytes.set(meta,110);
  data.forEach((x,i)=>{
    const p=offset+i*172,u=(k,n)=>v.setUint32(p+k,n>>>0,true);
    u(0,x.t);v.setUint8(p+8,3);u(9,1);u(130,x.speed_seq);v.setUint8(p+134,1);
    u(118,origin+x.speed_start);u(122,origin+x.speed_t);u(126,origin+x.speed_end);
    v.setInt16(p+135,6000,true);u(137,origin+x.body_t);
    v.setInt16(p+141,x.applied_mA,true);u(143,x.command_seq);u(147,origin+x.command_t);
    v.setInt32(p+151,x.position_raw_cdeg,true);u(155,origin+x.position_start);
    u(159,origin+x.position_t);u(163,origin+x.position_end);u(167,x.position_seq);v.setUint8(p+171,1);
  });
  let crc=0xffffffff;for(const b of bytes.subarray(0,end)){crc^=b;for(let i=0;i<8;++i)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}
  v.setUint32(end,(crc^0xffffffff)>>>0,true);return bytes;
}
const bytes=fixture();r=W.fromBytes(bytes,{sign:1});
close(r.measurement.observed_net_j,J*60*D*460*R);
const corrupt=bytes.slice();corrupt[200]^=1;assert.throws(()=>W.fromBytes(corrupt),/CRC/);
if(process.argv[2]) {
  const parsed=W.parse(new Uint8Array(fs.readFileSync(process.argv[2])));
  assert.equal(parsed.rows[0].position_raw_cdeg,-2147483648);
  assert.equal(parsed.rows[0].position_seq,9);assert.equal(parsed.rows[0].position_valid,true);
}
console.log('Position work: analytic energy, fixed body, absolute spin, position/time rollover, gaps, CRC, no RPM fallback PASS');
