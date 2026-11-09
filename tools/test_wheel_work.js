'use strict';
const assert=require('node:assert/strict'), fs=require('fs');
const W=require('../site/wheel-work.js'), J=2.8e-5, D=Math.PI/180, R=2*Math.PI/60;
const close=(a,b,tol=1e-12)=>assert(Math.abs(a-b)<tol,`${a} != ${b}`);
function rows(rate=t=>60,rpm=t=>-100*t/100000) {
  return Array.from({length:11},(_,i)=>{
    const t=i*10000;return {t,state:3,pulse_id:1,body_t:t,body_stamp_valid:true,
      body_rate_dps:rate(t),speed_t:t,speed_start:t-100,speed_end:t+100,
      speed_seq:i+1,speed_valid:true,speed_rpm:rpm(t),
      command_seq:i===10?2:1,command_t:i===10?100000:0,applied_mA:i===10?0:100};
  });
}
const run=r=>W.analyze(r,{}, {sign:1,inertia:J});
let r=run(rows());
close(r.measurement.observed_net_j,J*60*D*100*R);
close(r.pulses[0].coverage,1);assert(r.pulses[0].complete);
assert.equal(r.intervals.length,10);assert.equal(r.measurement.missing_us,0);
// Repeated rows are not additional encoder samples.
const duplicated=rows().flatMap(x=>[x,{...x}]);
close(run(duplicated).measurement.observed_net_j,r.measurement.observed_net_j);
// Fixed body: wheel can store energy without doing rotational work on body.
close(run(rows(()=>0)).measurement.observed_net_j,0);
// Reverse the wheel acceleration -> remove body energy.
close(run(rows(()=>60,t=>100*t/100000)).measurement.observed_net_j,-r.measurement.observed_net_j);
close(W.analyze(rows(),{}, {sign:-1,inertia:J}).measurement.observed_net_j,-r.measurement.observed_net_j);
// Relative encoder unchanged, body accelerates: retain absolute-spin correction.
close(run(rows(t=>60*t/100000,()=>100)).measurement.observed_net_j,-0.5*J*(60*D)**2);
// Changing inertia scales the result, no hidden torque/current model.
close(W.analyze(rows(),{}, {sign:1,inertia:J*2}).measurement.observed_net_j,2*r.measurement.observed_net_j);
// Failed observations and missing sequences never bridge a gap.
let a=rows();a[5].speed_valid=false;
let bad=run(a);assert.equal(bad.rejected.speed_read_invalid,2);
close(bad.pulses[0].coverage,.8);assert(!bad.pulses[0].complete);
assert(bad.intervals.filter(i=>!i.valid).every(i=>i.work_j===null));
a=rows();a.splice(5,1);bad=run(a);assert.equal(bad.rejected.speed_sequence_gap,1);
a=rows();a[4].speed_start-=5000;bad=run(a);assert.equal(bad.rejected.speed_read_too_long,2);
a=rows();for(const v of a)v.body_stamp_valid=false;
bad=run(a);assert.equal(bad.measurement.observed_net_j,null);assert.equal(bad.measurement.coverage,0);
// Command boundary between speed reads is apportioned explicitly, not assigned
// wholly to the adjacent pulse. The final command is observed.
a=rows();for(let i=0;i<10;++i){a[i].command_seq=1;a[i].command_t=15000;}
r=run(a);close(r.pulses[0].coverage,1);
close(r.pulses[0].observed_net_j,J*60*D*85*R);
assert(r.pulses[0].boundary_interpolated);
// No observed end command is never a complete pulse.
a=rows();a[10].command_seq=1;a[10].applied_mA=100;a[10].command_t=0;
assert(!run(a).pulses[0].complete);
assert.throws(()=>W.analyze(rows(),{}, {sign:0}),/符号/);
assert.throws(()=>W.analyze(rows(),{}, {inertia:0}),/慣性/);
// Decode a binary fixture at real C++ offsets, including micros() rollover.
function fixture(origin=0xfffff000) {
  const metadata=new TextEncoder().encode(JSON.stringify({wheel_speed_to_body_sign:1,wheel_work_inertia_kg_m2:J}));
  const samples=rows(), offset=110+metadata.length, n=offset+samples.length*151;
  const bytes=new Uint8Array(n+4), v=new DataView(bytes.buffer);
  bytes.set(new TextEncoder().encode('RWLOG01'));v.setUint16(8,55,true);v.setUint16(10,110,true);
  v.setUint32(16,origin,true);v.setUint32(24,metadata.length,true);v.setUint32(28,samples.length,true);
  v.setUint16(40,151,true);v.setUint32(58,1,true);v.setUint32(62,offset,true);
  v.setUint32(66,n,true);v.setUint32(70,n,true);v.setUint32(74,n,true);bytes.set(metadata,110);
  samples.forEach((r,i)=>{
    const p=offset+i*151;
    const u=(j,x)=>v.setUint32(p+j,x>>>0,true);
    u(0,r.t);v.setUint8(p+8,3);u(9,1);
    v.setInt32(p+114,Math.round(r.speed_rpm*100),true);
    u(118,origin+r.speed_start);u(122,origin+r.speed_t);u(126,origin+r.speed_end);
    u(130,r.speed_seq);v.setUint8(p+134,1);v.setInt16(p+135,6000,true);
    u(137,origin+r.body_t);v.setInt16(p+141,r.applied_mA,true);u(143,r.command_seq);u(147,origin+r.command_t);
  });
  function crc(data){let c=0xffffffff;for(const b of data){c^=b;for(let i=0;i<8;++i)c=(c>>>1)^((c&1)?0xedb88320:0);}return(c^0xffffffff)>>>0;}
  v.setUint32(n,crc(bytes.subarray(0,n)),true);return bytes;
}
const bytes=fixture();r=W.fromBytes(bytes);
close(r.measurement.observed_net_j,J*60*D*100*R);
assert.equal(r.pulses[0].end_us,100000);
assert.match(W.describe(r),/読み値の遅れは未補正/);
assert.match(W.csv(r.pulses,r),/inertia_kg_m2/);
const corrupt=bytes.slice();corrupt[120]^=1;assert.throws(()=>W.fromBytes(corrupt),/CRC/);
const old=bytes.slice();new DataView(old.buffer).setUint16(8,54,true);assert.throws(()=>W.fromBytes(old),/v55/);
if(process.argv[2]) {
  const parsed=W.parse(new Uint8Array(fs.readFileSync(process.argv[2])));
  assert.equal(parsed.rows.length,1); // actual C++ serializer fixture
  assert.equal(parsed.rows[0].speed_rpm,123.45);
  assert.equal(parsed.rows[0].body_rate_dps,60);
}
console.log('Wheel work: physical signs, absolute spin, gap rejection, pulse edges, CRC and clock rollover PASS');
