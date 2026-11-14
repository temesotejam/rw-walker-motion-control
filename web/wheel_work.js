/* Offline reaction-couple work. The same implementation runs on the device's
 * browser page, the public file reader, and in Node tests/CLI. No motor control. */
(function(root) {
  'use strict';
  const PI = Math.PI, DEG = PI / 180, RPM = 2 * PI / 60;
  function crc32(bytes) {
    let c = 0xffffffff;
    for (const b of bytes) {
      c ^= b;
      for (let j = 0; j < 8; ++j) c = (c >>> 1) ^ ((c & 1) ? 0xedb88320 : 0);
    }
    return (c ^ 0xffffffff) >>> 0;
  }
  function parse(bytes) {
    if (!(bytes instanceof Uint8Array) || bytes.length < 114) throw Error('RWLOGが短すぎます');
    const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    if (new TextDecoder().decode(bytes.subarray(0, 7)) !== 'RWLOG01' || v.getUint16(10, true) !== 110)
      throw Error('RWLOGヘッダーが一致しません');
    const version=v.getUint16(8,true), rowSize=v.getUint16(40,true);
    if (!((version===55 && rowSize===151) || (version===56 && rowSize===172)))
      throw Error('RWLOG v55または位置を記録したv56が必要です。旧ログからは計算できません。');
    const count = v.getUint32(28, true), offset = v.getUint32(62, true);
    const crcOffset = v.getUint32(74, true), metaLength = v.getUint32(24, true);
    if (crcOffset + 4 !== bytes.length || offset !== 110 + metaLength ||
        offset + count * rowSize > crcOffset || offset + count * rowSize !== v.getUint32(66, true))
      throw Error('RWLOGの長さ・オフセットが不正です');
    if (!(v.getUint32(58, true) & 1) || crc32(bytes.subarray(0, crcOffset)) !== v.getUint32(crcOffset, true))
      throw Error('RWLOGのCRCが一致しません');
    const metadata = JSON.parse(new TextDecoder().decode(bytes.subarray(110, offset)));
    const origin = v.getUint32(16, true), rows = [];
    const time = x => (x - origin) | 0; // signed modular difference; runs are <35 min
    let previousTime = 0;
    for (let i = 0; i < count; ++i) {
      const p = offset + i * rowSize, u32 = j => v.getUint32(p + j, true);
      const rate = v.getInt16(p + 135, true), speed = v.getInt32(p + 114, true);
      const bodyTime = time(u32(137)), speedTime = time(u32(122));
      // The legacy measurement-boundary marker has log_time_us=0. Its sensor
      // clocks remain continuous; do not mistake that marker for a clock reset.
      const positionTime=version===56 ? time(u32(159)) : 0;
      const rowTime = Math.max(previousTime, u32(0), bodyTime, speedTime, positionTime);
      previousTime = rowTime;
      rows.push({t:rowTime, state:v.getUint8(p + 8), pulse_id:u32(9),
        body_t:bodyTime, body_stamp_valid:u32(137) !== 0,
        body_rate_dps:rate === -32768 ? NaN : rate / 100,
        speed_t:speedTime, speed_start:time(u32(118)), speed_end:time(u32(126)),
        speed_seq:u32(130), speed_valid:v.getUint8(p + 134) === 1,
        speed_rpm:speed === -2147483648 ? NaN : speed / 100,
        command_seq:u32(143), command_t:time(u32(147)),
        applied_mA:v.getInt16(p + 141, true),
        ...(version===56 ? {position_raw_cdeg:v.getInt32(p+151,true),
          position_start:time(u32(155)),position_t:positionTime,position_end:time(u32(163)),
          position_seq:u32(167),position_valid:v.getUint8(p+171)===1} : {})});
    }
    return {rows, metadata};
  }
  // Centered offline fit. Time is scaled to 10 ms and position is centered to
  // preserve numerical accuracy for long cumulative-angle records.
  function solve3(a) {
    for(let k=0;k<3;++k) {
      let pivot=k;
      for(let i=k+1;i<3;++i) if(Math.abs(a[i][k])>Math.abs(a[pivot][k])) pivot=i;
      if(Math.abs(a[pivot][k])<1e-12) return null;
      [a[k],a[pivot]]=[a[pivot],a[k]];
      const divisor=a[k][k];for(let j=k;j<4;++j)a[k][j]/=divisor;
      for(let i=0;i<3;++i) if(i!==k) {
        const f=a[i][k];for(let j=k;j<4;++j)a[i][j]-=f*a[k][j];
      }
    }
    return a.map(r=>r[3]);
  }
  function positionVelocity(positions, fitPoints=5) {
    if(![5,9].includes(fitPoints)) throw Error('位置近似の点数は5または9にしてください');
    const half=(fitPoints-1)/2, nodes=[], rejected={};
    let angle=0;
    const pp=positions.map((p,i)=>{
      if(i) angle+=((p.raw-positions[i-1].raw)|0)/100;
      return {...p,angle};
    });
    for(let k=half;k<pp.length-half;++k) {
      const p=pp[k], window=pp.slice(k-half,k+half+1);
      let reason='';
      for(let j=0;j<window.length;++j) {
        const w=window[j];
        if(!w.valid || !Number.isFinite(w.raw)) {reason='position_read_invalid';break;}
        if(w.duration<0 || w.duration>3000) {reason='position_read_too_long';break;}
        if(j) {
          const prev=window[j-1],dt=w.t-prev.t;
          if(((w.seq-prev.seq)>>>0)!==1) {reason='position_sequence_gap';break;}
          if(dt<=0 || dt>15000) {reason='position_time_gap';break;}
          // Refuse a per-revolution reset or implausible jump; signed-int32
          // counter rollover itself is handled above, never a 360-deg unwrap.
          if(Math.abs(w.angle-prev.angle)>6000*6*dt/1e6+1) {reason='position_discontinuity';break;}
        }
      }
      const sums=Array(5).fill(0),rhs=Array(3).fill(0);
      if(!reason) for(const w of window) {
        const x=(w.t-p.t)/10000,y=w.angle-p.angle;
        for(let i=0;i<5;++i)sums[i]+=x**i;
        for(let i=0;i<3;++i)rhs[i]+=y*x**i;
      }
      const c=reason ? null : solve3(Array.from({length:3},(_,i)=>[sums[i],sums[i+1],sums[i+2],rhs[i]]));
      if(!reason && !c) reason='position_fit_singular';
      const residual=c ? Math.sqrt(window.reduce((sum,w)=>{
        const x=(w.t-p.t)/10000;
        return sum+(w.angle-p.angle-c[0]-c[1]*x-c[2]*x*x)**2;
      },0)/window.length) : null;
      if(!reason && residual>.5) reason='position_fit_residual';
      if(reason) rejected[reason]=(rejected[reason]||0)+1;
      nodes.push({t:p.t,seq:p.seq,valid:!reason,reason,duration:p.duration,
        rpm:reason ? NaN : c[1]*100/6,position_deg:p.angle,
        fit_span_us:window[window.length-1].t-window[0].t,fit_rmse_deg:residual});
    }
    return {nodes,rejected};
  }
  function analyze(rows, metadata = {}, options = {}) {
    const inertia = options.inertia ?? metadata.wheel_work_inertia_kg_m2 ?? 2.8e-5;
    const sign = options.sign ?? metadata.wheel_speed_to_body_sign ?? -1;
    if (!Number.isFinite(inertia) || inertia <= 0 || (sign !== 1 && sign !== -1))
      throw Error('慣性モーメントは正の数、軸の符号は+1または−1にしてください');
    const maxSpeedGap = 30000, maxBodyGap = 25000, maxReadTime = 3000;
    const usePosition=metadata.wheel_work_velocity_source==='position_local_quadratic_5_points' ||
      rows.some(r=>Object.hasOwn(r,'position_seq'));
    const positionSign=metadata.wheel_position_to_speed_sign ?? 1;
    if(positionSign!==1 && positionSign!==-1) throw Error('位置と回転速度の符号が不正です');
    const fitPoints=options.positionFitPoints ?? 5;
    const gyros = new Map(), positions=[], speeds=[], commands = [], states = [];
    let lastSpeed, lastPosition, lastCommand, lastState;
    for (const r of rows) {
      if (r.body_stamp_valid && Number.isFinite(r.body_rate_dps) &&
          r.body_t <= r.t && r.t - r.body_t <= 10000)
        gyros.set(r.body_t, r.body_rate_dps * DEG);
      if (r.speed_seq !== lastSpeed && r.speed_seq !== 0) {
        speeds.push({t:r.speed_valid ? r.speed_t : (r.speed_start + r.speed_end)/2,
          rpm:r.speed_rpm, seq:r.speed_seq, valid:r.speed_valid,
          duration:r.speed_end-r.speed_start});
        lastSpeed = r.speed_seq;
      }
      if(usePosition && r.position_seq!==lastPosition && r.position_seq!==0 && r.position_seq!==undefined) {
        positions.push({t:r.position_valid ? r.position_t : (r.position_start+r.position_end)/2,
          raw:r.position_raw_cdeg,seq:r.position_seq,valid:r.position_valid,
          duration:r.position_end-r.position_start});
        lastPosition=r.position_seq;
      }
      if (r.command_seq !== lastCommand && r.command_seq !== 0) {
        commands.push({t:r.command_t, seq:r.command_seq, current:r.applied_mA, pulse_id:r.pulse_id});
        lastCommand = r.command_seq;
      }
      if (r.state !== lastState) { states.push({t:r.t, state:r.state}); lastState = r.state; }
    }
    const positionResult=usePosition ? positionVelocity(positions,fitPoints) : {nodes:[],rejected:{}};
    const wheels=usePosition ? positionResult.nodes.map(n=>({...n,rpm:n.rpm*positionSign})) : speeds;
    const body = [...gyros].sort((a,b) => a[0]-b[0]);
    commands.sort((a,b) => a.t-b.t);
    function bodyAt(t) {
      let lo=0, hi=body.length;
      while (lo<hi) {const m=(lo+hi)>>>1; if (body[m][0]<t) lo=m+1; else hi=m;}
      if (lo<body.length && body[lo][0]===t) return body[lo][1];
      if (!lo || lo===body.length) return null; // no extrapolation
      const a=body[lo-1], b=body[lo];
      if (b[0]-a[0]>maxBodyGap) return null;
      return a[1]+(b[1]-a[1])*(t-a[0])/(b[0]-a[0]);
    }
    const endTime = rows.length ? rows[rows.length-1].t : 0;
    const pulses=[];
    for (let i=0; i<commands.length; ++i) {
      const c=commands[i];
      if (!c.current) continue;
      const next=commands[i+1];
      pulses.push({index:pulses.length+1, pulse_id:c.pulse_id, command_sequence:c.seq,
        start_us:c.t, end_us:next ? next.t : endTime, current_mA:c.current,
        end_observed:!!next, observed_net_j:0, added_j:0, removed_j:0, covered_us:0,
        boundary_interpolated:false});
    }
    const intervals=[], rejected={};
    const total={observed_net_j:0, added_j:0, removed_j:0, covered_us:0, missing_us:0};
    const coast={observed_net_j:0, added_j:0, removed_j:0, covered_us:0};
    const active={observed_net_j:0, added_j:0, removed_j:0, covered_us:0};
    function stateAt(t) {
      let state=0;
      for (const s of states) {if (s.t>t) break; state=s.state;}
      return state;
    }
    function add(target,w,dt) {
      target.observed_net_j+=w; target.added_j+=Math.max(0,w);
      target.removed_j+=Math.max(0,-w); target.covered_us+=dt;
    }
    for (let i=1; i<wheels.length; ++i) {
      const a=wheels[i-1], b=wheels[i], dt=b.t-a.t;
      const oa=bodyAt(a.t), ob=bodyAt(b.t);
      let reason='';
      if (dt<=0) reason='nonmonotonic_speed_time';
      else if (!a.valid || !b.valid || !Number.isFinite(a.rpm) || !Number.isFinite(b.rpm))
        reason=usePosition ? 'position_velocity_unavailable' : 'speed_read_invalid';
      else if (((b.seq-a.seq)>>>0)!==1) reason='speed_sequence_gap';
      else if (dt>maxSpeedGap) reason='speed_time_gap';
      else if (a.duration<0 || b.duration<0 || a.duration>maxReadTime || b.duration>maxReadTime) reason='speed_read_too_long';
      else if (oa===null || ob===null) reason='body_rate_not_bracketed';
      const cuts=[a.t,b.t];
      for (const c of commands) if(c.t>a.t && c.t<b.t) cuts.push(c.t);
      for (const s of states) if(s.t>a.t && s.t<b.t) cuts.push(s.t);
      if (!reason && oa*ob<0) cuts.push(a.t+dt*(-oa)/(ob-oa));
      cuts.sort((x,y)=>x-y);
      const wa=sign*RPM*a.rpm+oa, wb=sign*RPM*b.rpm+ob;
      if (reason) rejected[reason]=(rejected[reason]||0)+1;
      for (let j=1; j<cuts.length; ++j) {
        const l=cuts[j-1], r=cuts[j];
        if (r<=l) continue;
        const midpoint=(l+r)/2, measuring=stateAt(midpoint)===3;
        const pulse=pulses.find(p=>midpoint>=p.start_us && midpoint<p.end_us);
        const fraction=(r-l)/dt;
        const omegaMean = reason ? null : oa+(ob-oa)*((midpoint-a.t)/dt);
        const delta = reason ? null : (wb-wa)*fraction;
        const w = reason ? null : -inertia*omegaMean*delta;
        intervals.push({start_us:l,end_us:r,measurement:measuring,
          pulse_index:pulse?.index ?? 0,command_current_mA:pulse?.current_mA ?? 0,
          valid:!reason,reason,body_mean_rad_s:omegaMean,wheel_abs_delta_rad_s:delta,
          body_torque_nm:reason ? null : -inertia*delta/((r-l)*1e-6),
          work_j:w,boundary_interpolated:l!==a.t || r!==b.t});
        if (measuring) {
          if (reason) total.missing_us+=r-l;
          else { add(total,w,r-l); add(pulse ? active : coast,w,r-l); }
        }
        if (pulse && !reason) {
          add(pulse,w,r-l);
          pulse.boundary_interpolated ||= l!==a.t || r!==b.t;
        }
      }
    }
    for (const p of pulses) {
      const duration=p.end_us-p.start_us;
      p.coverage=duration>0 ? Math.min(1,p.covered_us/duration) : 0;
      p.complete=p.end_observed && p.coverage>=0.999999;
      if (!p.covered_us) p.observed_net_j=p.added_j=p.removed_j=null;
    }
    // Include uncovered measurement edges, not only holes between wheel reads.
    let measurementUs=0;
    for (let i=0;i<states.length;++i)
      if(states[i].state===3) measurementUs+=Math.max(0,(states[i+1]?.t ?? endTime)-states[i].t);
    total.missing_us=Math.max(total.missing_us,measurementUs-total.covered_us,0);
    total.coverage=measurementUs>0 ? Math.min(1,total.covered_us/measurementUs) : 0;
    for (const result of [total,active,coast])
      if (!result.covered_us) result.observed_net_j=result.added_j=result.removed_j=null;
    return {revision:usePosition ? 'reaction_couple_position_work_v2_20261016' : 'reaction_couple_work_v1_20261016',inertia_kg_m2:inertia,
      wheel_speed_to_body_sign:sign,provisional:true,readback_delay_compensated:false,
      velocity_source:usePosition ? 'position_local_quadratic' : 'legacy_filtered_rpm',
      legacy_filtered_rpm_warning:!usePosition,position_to_speed_sign:positionSign,
      position_fit_points:usePosition ? fitPoints : null,
      position_fit_rejected:positionResult.rejected,
      position_attempts:positions.length,position_velocity_samples:positionResult.nodes.filter(n=>n.valid).length,
      kinematics:usePosition ? wheels.map(n=>({time_us:n.t,sequence:n.seq,valid:n.valid,reason:n.reason,
        relative_position_deg:n.position_deg,relative_speed_rpm:n.valid ? n.rpm : null,
        fit_span_us:n.fit_span_us,fit_rmse_deg:n.fit_rmse_deg})) : [],
      body_energy_definition:'reaction_couple_work_excluding_rotor_absolute_axial_spin',
      measurement:total,active,coast,pulses,intervals,rejected,
      speed_attempts:speeds.length,body_samples:body.length,
      limits:{max_speed_gap_us:maxSpeedGap,max_body_bracket_gap_us:maxBodyGap,max_read_us:maxReadTime,
        max_position_fit_adjacent_gap_us:15000,position_fit_max_rmse_deg:.5}};
  }
  function fromBytes(bytes,options={}) {
    const {rows,metadata}=parse(bytes); return analyze(rows,metadata,options);
  }
  function csv(rows, report) {
    if (!rows.length) return '';
    if (report) rows=rows.map(r=>({inertia_kg_m2:report.inertia_kg_m2,
      wheel_speed_to_body_sign:report.wheel_speed_to_body_sign,velocity_source:report.velocity_source,
      position_fit_points:report.position_fit_points,provisional:true,...r}));
    const keys=Object.keys(rows[0]);
    const cell=v=>v==null?'': '"'+String(v).replace(/"/g,'""')+'"';
    return keys.map(cell).join(',')+'\r\n'+rows.map(r=>keys.map(k=>cell(r[k])).join(',')).join('\r\n');
  }
  function describe(r) {
    const mj=x=>x==null?'算出不可':(x*1000).toFixed(4)+' mJ';
    const lines=[`暫定の仕事：J=${r.inertia_kg_m2} kg·m²、軸の符号=${r.wheel_speed_to_body_sign}`,
      `測定中の観測できた正味の仕事：${mj(r.measurement.observed_net_j)}`,
      `加えた分 ${mj(r.measurement.added_j)} / 取り除いた分 ${mj(r.measurement.removed_j)}`,
      `指令適用中 ${mj(r.active.observed_net_j)} / 入力を切った区間 ${mj(r.coast.observed_net_j)}`,
      `測定区間の計算対象 ${(r.measurement.coverage*100).toFixed(1)}% / 欠測 ${(r.measurement.missing_us/1000).toFixed(1)} ms`,
      '入力後の区間も軸受などの反作用を含む仕事です。次のピークに残るエネルギーとは異なります。',
      r.velocity_source==='position_local_quadratic'
        ? `位置の${r.position_fit_points}点近似から速度を算出しています。位置が欠測した区間は速度読み値で補いません。`
        : '旧ログの速度読み値による参考値です。読み値の遅れは未補正です。入力中と入力後の配分は結論に使えません。',
      '指令の境界は線形補間しています。取得時刻は内部センサ時刻ではなく、慣性・軸の符号・平面近似にも依存します。',
      '', '各入力（指令を書き込めた時刻から、次の指令を書き込めた時刻まで）：'];
    for(const p of r.pulses) lines.push(`${p.index}. ${p.current_mA} mA / ${((p.end_us-p.start_us)/1000).toFixed(1)} ms：${mj(p.observed_net_j)} / 計算対象 ${(p.coverage*100).toFixed(1)}%${p.complete?'':'（区間の一部が未取得）'}`);
    return lines.join('\n');
  }
  root.WheelWorkAnalyzer={parse,analyze,fromBytes,csv,describe,positionVelocity};
  if (typeof module!=='undefined') module.exports=root.WheelWorkAnalyzer;
})(typeof globalThis!=='undefined' ? globalThis : this);
