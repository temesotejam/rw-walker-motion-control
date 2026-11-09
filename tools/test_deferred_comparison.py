#!/usr/bin/env python3
"""Compile real estimator/display methods against the frozen 0.47.18 methods.
Exercise pulse changes BETWEEN preparation and deferred execution. A host
comparison verifies arithmetic/state, not ESP32 scheduling or hardware time.
"""
from pathlib import Path
import json, re, subprocess, tempfile
root = Path(__file__).resolve().parents[1]
source = (root/'src/experiment_runner.cpp').read_text()
baseline = json.loads((root/'tools/fixtures/filter_order_04718.json').read_text())
def method(name):
    m = re.search(r'^(?:void|float|uint16_t) ExperimentRunner::'+name+r'\(', source, re.M)
    start = m.start(); end = source.index('{', start)+1; depth = 1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}'); end += 1
    return source[start:end]
cpp = '''#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <Arduino.h>
#include <Adafruit_AHRS.h>
#include "config.h"
#include "psram_logger.h"
#include "control_work_profile.h"
#include "control_latency.h"
#include "madgwick_pitch.h"
#define private public
#include "experiment_runner.h"
'''+baseline['class'].replace('ExperimentRunner','Baseline')+'\n#undef private\n'
cpp += baseline['helpers']
cpp += '\n'.join(v.replace('ExperimentRunner::','Baseline::') for v in baseline['methods'].values())
cpp += '\n'.join(method(n) for n in [*baseline['methods'], 'finishDeferredComparison', 'updateComparisonDisplayAngles'])
cpp += r'''
static void equal(float a,float b) { assert((std::isnan(a)&&std::isnan(b)) || a==b); }
int main() {
  unsigned deferred=0;
  for (bool autonomous : {false,true}) {
    Baseline old; ExperimentRunner now;
    old.energy_control_autonomous_mode_=now.energy_control_autonomous_mode_=autonomous;
    old.beginFilters(); now.beginFilters();
    old.bias_ready_=now.bias_ready_=true;
    old.status_.gyro_bias_x_dps=now.status_.gyro_bias_x_dps=0.2f;
    old.status_.gyro_bias_y_dps=now.status_.gyro_bias_y_dps=-0.3f;
    old.status_.gyro_bias_z_dps=now.status_.gyro_bias_z_dps=0.1f;
    old.status_.mekf_measurement_zero_abs_deg=now.status_.mekf_measurement_zero_abs_deg=0.2f;
    old.status_.mekf_start_sync_zero_abs_deg=now.status_.mekf_start_sync_zero_abs_deg=-0.4f;
    ImuReading r{};
    host_us=4000000;
    for(unsigned n=1;n<=16000;++n) {
      host_us+=2500;
      const bool measuring=n>500;
      old.status_.state=now.status_.state=measuring ? ExperimentState::RUNNING_BATCH_SWEEP : ExperimentState::START_SYNC;
      old.status_.pulse_active=now.status_.pulse_active=(n%173)<33;
      old.last_pulse_end_ms_=now.last_pulse_end_ms_=millis()-((n%300)*3);
      old.status_.predicted_beta_min=now.status_.predicted_beta_min=0.01f;
      r.gyro_sequence=n; r.gyro_update_dt_us=2500; r.last_gyro_update_us=host_us;
      r.gx_dps=2*sinf(n*0.018f); r.gy_dps=75*cosf(n*0.038f); r.gz_dps=3*cosf(n*0.021f);
      if (n%2) {
        ++r.accel_sequence; r.last_accel_update_us=host_us;
        r.ax_g=0.15f*sinf(n*0.038f); r.ay_g=0.02f; r.az_g=-cosf(r.ax_g);
        r.acc_norm_g=sqrtf(r.ax_g*r.ax_g+r.ay_g*r.ay_g+r.az_g*r.az_g);
        r.pitch_accel_only_deg=Config::PITCH_SIGN*atan2f(-r.ax_g,sqrtf(r.ay_g*r.ay_g+r.az_g*r.az_g))*57.2957795f;
      }
      old.updateFilterSeries(r); old.updateDisplayedAngles(r);
      now.updateFilterSeries(r); now.updateDisplayedAngles(r);
      if (autonomous) equal(now.status_.pitch_mekf_deg,now.status_.pitch_mekf_detector_relative_deg);
      else equal(old.status_.pitch_mekf_deg,now.status_.pitch_mekf_deg);
      if (autonomous) equal(now.status_.pitch_mekf_detector_relative_deg,now.status_.pitch_mekf_measurement_relative_deg);
      else equal(old.status_.pitch_mekf_detector_relative_deg,now.status_.pitch_mekf_detector_relative_deg);
      equal(old.status_.pitch_mekf_measurement_relative_deg,now.status_.pitch_mekf_measurement_relative_deg);
      const auto q1=old.mekf_.quaternion(), q2=now.mekf_.quaternion();
      equal(q1.w,q2.w);equal(q1.x,q2.x);equal(q1.y,q2.y);equal(q1.z,q2.z);
      if(now.deferred_comparison_.pending) {
        ++deferred;
        // Simulate output starting/stopping and beta-context changes during
        // the decision. The already captured comparator input must win.
        now.status_.pulse_active=!now.status_.pulse_active;
        now.last_pulse_end_ms_=millis(); host_us+=700;
      }
      now.finishDeferredComparison();
      equal(old.status_.pitch_madgwick_dynamic_abs_deg,now.status_.pitch_madgwick_dynamic_abs_deg);
      equal(old.status_.pitch_madgwick_dynamic_bias_deg,now.status_.pitch_madgwick_dynamic_bias_deg);
      equal(old.status_.pitch_accel_only_deg,now.status_.pitch_accel_only_deg);
      equal(old.status_.beta_smooth,now.status_.beta_smooth);
      for(unsigned i=0;i<Config::DYNAMIC_BETA_COUNT;++i)
        equal(old.status_.pitch_dynamic_beta_deg[i],now.status_.pitch_dynamic_beta_deg[i]);
      assert(!now.deferred_comparison_.pending);
    }
  }
  assert(deferred==7750);
  std::cout << "32,000 production/frozen filter states: posterior angles and same-sample comparison exact; unprojected angle threshold coordinate verified; 7,750 deferred pulse transitions PASS\n";
}
'''
with tempfile.TemporaryDirectory() as directory:
    p=Path(directory); (p/'test.cpp').write_text(cpp)
    subprocess.run(['g++','-std=c++17','-Os','-ffp-contract=off','-Itools/host_v46o','-Isrc',str(p/'test.cpp'),
                    'src/mekf6.cpp','src/mekf_attitude_diagnostics.cpp','src/control_work_profile.cpp','src/control_latency.cpp',
                    'tools/fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_Madgwick.cpp','-o',str(p/'test')],cwd=root,check=True)
    subprocess.run([str(p/'test')],check=True)
