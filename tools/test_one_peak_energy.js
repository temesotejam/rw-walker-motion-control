'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs'),vm=require('node:vm');
const P=require('../site/one-peak-energy.js');
const close=(a,b,tol=1e-9)=>assert(Math.abs(a-b)<tol,`${a} != ${b}`);
const table=rows=>({fields:Object.keys(rows[0]),rows:rows.map(row=>Object.values(row))});
const event={event_index:31,negative_probe_batch_index:13,zero_cross_time_ms:10924,
  negative_probe_batch_planned_width_ms:25,negative_probe_batch_rate_skipped:false,
  physical_next_peak_side:-1,valid:true,pulse_width_ms:25,output_executed:true,
  zero_cross_rate_dps:-69.2992,pre_input_wheel_speed_rpm:234.77,
  pre_input_wheel_speed_valid:true,previous_peak_amplitude_deg:7.58116,vbat_mV:7250};
const peak={physical_peak_side:-1,peak_time_ms:11300,peak_amplitude_deg:10.5};
const metadata={negative_probe_batch:true,energy_control_autonomous_normal_current_mA:300,
  energy_control_autonomous_target_peak_deg:8,firmware_revision:'fixture',
  upright_reference_abs_pitch_deg:1.932539,
  energy_control_autonomous_zero_cross_events:table([event]),
  energy_control_autonomous_peak_events:table([peak])};
const result=P.analyze(metadata),row=result.rows[0];
assert.equal(result.summary.scored,1);
assert.equal(P.model.training_events,304);
close(row.predicted_peak_deg,10.282807170383641);
close(row.predicted_energy_mj,1.1042532603239432);
close(P.potentialMj(8),.7334225990577391,1e-9);
assert.equal(row.target_reachable_with_tested_widths,false);
assert.equal(row.predicted_best_width_ms,25);
assert(row.predicted_best_energy_error_mj>0);
const withinEnvelope={...metadata,energy_control_autonomous_target_peak_deg:10.5};
const selected=P.analyze(withinEnvelope).rows[0];
assert.equal(selected.target_reachable_with_tested_widths,true);
assert.equal(selected.predicted_best_width_ms,20);
assert(Math.abs(selected.predicted_best_energy_error_mj)<.001);
const changedPeak={...metadata,energy_control_autonomous_peak_events:table([{...peak,peak_amplitude_deg:12}])};
close(P.analyze(changedPeak).rows[0].predicted_peak_deg,row.predicted_peak_deg);
assert.notEqual(P.analyze(changedPeak).rows[0].error_deg,row.error_deg);
const lowVoltage={...metadata,energy_control_autonomous_zero_cross_events:table([{...event,vbat_mV:6100}])};
assert.notEqual(P.analyze(lowVoltage).rows[0].predicted_peak_deg,row.predicted_peak_deg);
assert.equal(P.analyze(lowVoltage).summary.scored,0);
assert.equal(P.analyze(lowVoltage).summary.outside_support.n,1);
assert(Number.isFinite(P.analyze(lowVoltage).summary.outside_support.rmse_deg));
assert.match(P.analyze(lowVoltage).rows[0].exclusions,/battery_mV/);
const oldReference={...metadata,upright_reference_abs_pitch_deg:1.772013};
assert.equal(P.analyze(oldReference).summary.scored,0);
assert.match(P.analyze(oldReference).rows[0].exclusions,/直立基準/);
assert.throws(()=>P.analyze({...metadata,energy_control_autonomous_normal_current_mA:400}),/300 mA/);
assert.throws(()=>P.analyze({...metadata,metadata_event_detail_truncated:true}),/途中/);
const html=fs.readFileSync(require('node:path').join(__dirname,'../site/wheel-work.html'),'utf8');
const scriptStart=html.lastIndexOf('<script>'),scriptEnd=html.indexOf('</script>',scriptStart);
assert(scriptStart>=0 && scriptEnd>scriptStart);
new vm.Script(html.slice(scriptStart+8,scriptEnd));
console.log('One-peak 14-run prediction, future-peak isolation, reference and voltage support PASS');
