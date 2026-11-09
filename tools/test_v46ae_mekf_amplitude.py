#!/usr/bin/env python3
"""Execute production MEKF peak/motion/error-feedback methods with host sensors."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / 'src/experiment_runner.cpp').read_text()
header = (ROOT / 'src/experiment_runner.h').read_text()
logger = (ROOT / 'src/psram_logger.cpp').read_text()
logger_h = (ROOT / 'src/psram_logger.h').read_text()
config = (ROOT / 'src/config.h').read_text()

def block(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

names = ['void resetEnergyControlAutonomousPeakTracker', 'void updateEnergyControlAutonomousMotion',
         'void updateEnergyControlAutonomousPeakTracker', 'bool recordEnergyControlAutonomousPeak',
         'float energyControlPotentialJ',
         'float energyControlAutonomousGainForSide', 'void energyControlAutonomousCorrectionParameters',
         'float energyControlAutonomousCorrectedPrediction']
methods = '\n'.join(block(runner, name.replace(' ', ' ExperimentRunner::', 1)).replace('ExperimentRunner::', '')
                    for name in names)
fields = header[header.index('  bool energy_control_autonomous_mode_ ='):
                header.index('  PsramLogger::SolverShadowEvent solver_shadow_event_')]
enums = '\n'.join(block(header, 'enum class ' + name) + ';' for name in
                  ['EnergyControlAutonomousPhase', 'EnergyControlAutonomousHalfCycleState'])
event = block(logger_h, 'struct EnergyControlAutonomousPeakEvent') + ';'
assert 'energy_control_autonomous_gyro_relative_deg_' not in runner + header
assert 'ENERGY_CONTROL_AUTONOMOUS_GYRO_TO_VIDEO_PEAK_SCALE' not in runner + config
assert 'ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US = 3000UL' in config
assert 'ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_ENABLED = false' in config
assert 'energy_nonfinite_mekf_state' in methods
for token in ['no_delay_projection;no_output_scaling', 'live_MEKF_bias',
              'legacy_gyro_fit_disabled', 'RWLOG_FORMAT_VERSION = 51']:
    assert token in logger, token
# Trace the tested peak amplitude through the real downstream feedforward path.
zero = block(runner, 'void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross')
assert 'event.previous_peak_amplitude_deg = energy_control_autonomous_last_peak_amplitude_deg_' in zero
assert 'energyControlAutonomousFreeNextPeakAmplitude' not in runner + header
assert 'rate_baseline::evaluate(\n      event.zero_cross_abs_rate_dps, event.physical_next_peak_side)' in zero
assert 'Config::Q1_SHADOW_RATE_SUPPORT_MIN_DPS * Config::MEKF_GYRO_Y_SCALE' in zero
source = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>
#include "config.h"
#include "rate_baseline_correction.h"
using std::isfinite;
struct ImuReading { float gy_dps=0; uint32_t last_gyro_update_us=0; };
struct Imu { ImuReading sample; const ImuReading& reading()const{return sample;} };
struct PsramLogger {
EVENT
  std::vector<EnergyControlAutonomousPeakEvent> peaks;
  bool energyControlAutonomousEventCapacityReached()const{return false;}
  void markEnergyControlAutonomousEventOverflow(){}
  void addEnergyControlAutonomousPeakEvent(const EnergyControlAutonomousPeakEvent& e){peaks.push_back(e);}
};
struct ExperimentRunner {
ENUMS
FIELDS
  struct Status { bool pulse_active=false; float mekf_bias_y_dps=0;
    float pitch_mekf_measurement_relative_deg=0, pitch_mekf_detector_relative_deg=0; } status_;
  Imu* imu_; PsramLogger* logger_; uint32_t run_start_ms_=0;
  int stops=0, crosses=0; float last_cross_rate=0;
  void requestEmergencyStop(const char*){++stops;status_.pulse_active=false;}
  void updateEnergyControlAutonomousAtZeroCross(uint32_t,float rate,float,float,float,float){++crosses;last_cross_rate=rate;}
METHODS
};
void close(float a,float b){assert(std::abs(a-b)<1e-5f);}
using Phase=ExperimentRunner::EnergyControlAutonomousPhase;
using Half=ExperimentRunner::EnergyControlAutonomousHalfCycleState;
struct Fixture {
  Imu imu;PsramLogger log;ExperimentRunner r;
  uint32_t clock=0;
  Fixture(){r.imu_=&imu;r.logger_=&log;r.energy_control_autonomous_mode_=true;
    r.energy_control_autonomous_phase_=Phase::WAIT_FIRST_PEAK;r.resetEnergyControlAutonomousPeakTracker(true);}
  void sample(float angle,float rate,float delay_ms=3,float bias=2){
    clock+=2500;imu.sample.last_gyro_update_us=clock;
    imu.sample.gy_dps=rate/Config::MEKF_GYRO_Y_SCALE+bias;r.status_.mekf_bias_y_dps=bias;
    r.status_.pitch_mekf_measurement_relative_deg=angle;
    r.status_.pitch_mekf_detector_relative_deg=angle+rate*delay_ms/1000;
    r.updateEnergyControlAutonomousMotion(clock/1000);
  }
};
int main(){
  // The real tracker must use the posterior extremum, even if a delayed
  // projection has a larger extremum earlier in the cycle.
  for(int side:{-1,1}) {
    constexpr float delay=Config::ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US/1000.f;
    Fixture f;
    f.sample(side*7.0f,side*100.f,delay);
    f.sample(side*8.0f,side*1.f,delay);
    f.sample(side*7.9f,-side*1.f,delay);
    assert(f.log.peaks.empty());
    // Re-reading the same sensor sample cannot satisfy 3-sample confirmation.
    for(int i=0;i<10;++i)f.r.updateEnergyControlAutonomousMotion(f.clock/1000);
    assert(f.log.peaks.empty());
    f.sample(side*7.8f,-side*2.f,delay);
    f.sample(side*7.7f,-side*3.f,delay);
    assert(f.log.peaks.size()==1);
    const auto& e=f.log.peaks[0];close(e.peak_amplitude_deg,8);close(e.detector_peak_angle_deg,side*8);
    close(e.peak_error_deg,0);assert(e.physical_peak_side==side);
    close(f.r.energy_control_autonomous_last_peak_amplitude_deg_,8);
    assert(f.r.energy_control_autonomous_half_cycle_state_==Half::WAIT_ZERO_CROSS);
  }
  // V46ai: after a zero-width decision, the projected angle has crossed but
  // the posterior can remain on the previous side for several samples.
  // It must never seed the next extremum on that previous side.
  for(int side:{-1,1}) {
    constexpr float delay=Config::ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US/1000.f;
    Fixture f;f.clock=1000000;
    f.r.energy_control_autonomous_phase_=Phase::ENERGY_CONTROL;
    f.r.energy_control_autonomous_last_accepted_zero_cross_valid_=true;
    f.r.energy_control_autonomous_last_accepted_zero_cross_ms_=1000;
    f.r.energy_control_autonomous_pending_peak_=true;
    f.r.energy_control_autonomous_pending_next_side_=side;
    f.r.energy_control_autonomous_pending_q_command_mA_s_=0;
    if(delay>0) {
      f.sample(-side*.10f,side*60.f,delay);
      assert(!f.r.energy_control_autonomous_peak_tracker_started_);
      assert(f.log.peaks.empty());
    }
    f.sample(side*.05f,side*60.f,delay);
    assert(f.r.energy_control_autonomous_candidate_detector_side_==side);
    f.clock=1200000;
    f.sample(side*7.9f,side*2.f,delay);f.sample(side*8.f,side*.5f,delay);
    f.sample(side*7.99f,-side*1.f,delay);
    f.sample(side*7.98f,-side*2.f,delay);
    f.sample(side*7.97f,-side*3.f,delay);
    assert(f.log.peaks.size()==1);
    const auto& e=f.log.peaks[0];assert(e.physical_peak_side==side);
    assert(e.pending_command_matched);close(e.pending_q_command_mA_s,0);
    close(e.peak_amplitude_deg,8);
    assert(f.r.energy_control_autonomous_half_cycle_state_==Half::WAIT_ZERO_CROSS);
  }
  // Real peak-error feedback uses MEKF amplitude; startup bias magnitude and
  // elapsed run time must not introduce an accumulated-angle offset.
  for(float bias:{-20.f,0.f,20.f})for(int side:{-1,1}){
    Fixture f;f.clock=28000000;
    f.sample(side*8.5f,side*2.f,3,bias);f.sample(side*9.f,side*.5f,3,bias);
    for(float a:{8.9f,8.8f,8.7f})f.sample(side*a,-side*1.f,3,bias);
    assert(f.log.peaks.size()==1);close(f.log.peaks[0].peak_amplitude_deg,9);
    close(f.log.peaks[0].peak_error_deg,-1);
    close(side>0?f.r.energy_control_autonomous_integral_plus_mA_s_:f.r.energy_control_autonomous_integral_minus_mA_s_,-.1f);
  }
  // At zero cross, the current MEKF bias and input scale also define rate.
  for(int side:{-1,1}){
    Fixture f;f.r.energy_control_autonomous_phase_=Phase::ENERGY_CONTROL;
    f.r.energy_control_autonomous_half_cycle_state_=Half::WAIT_ZERO_CROSS;
    f.r.energy_control_autonomous_last_peak_valid_=true;
    f.sample(-side*.08f,side*10.f,3,50);f.sample(-side*.02f,side*10.f,3,50);
    assert(f.r.crosses==1);close(f.r.last_cross_rate,side*10.f);
    assert(f.r.status_.pitch_mekf_measurement_relative_deg*side<0); // still before posterior zero
  }
  {Fixture f;f.r.status_.pulse_active=true;for(float a:{7.f,8.f,7.9f,7.8f,7.7f})f.sample(a,-2);
    assert(f.log.peaks.empty()&&f.r.crosses==0);}
  for(int field=0;field<3;++field){Fixture f;f.sample(1,1);
    f.imu.sample.last_gyro_update_us+=2500;
    if(field==0)f.imu.sample.gy_dps=NAN;
    if(field==1)f.r.status_.pitch_mekf_measurement_relative_deg=NAN;
    if(field==2)f.r.status_.pitch_mekf_detector_relative_deg=NAN;
    f.r.updateEnergyControlAutonomousMotion(5);assert(f.r.stops==1&&f.r.energy_control_autonomous_phase_==Phase::STOP);}
  // Actual model functions: no legacy residual remains on either side.
  {Fixture f;for(int side:{-1,1})for(float rate:{0.f,20.f,65.f,85.f})for(float q:{0.f,1.f,8.f}){
    float c=NAN,g=NAN,residual=NAN;f.r.energyControlAutonomousCorrectionParameters(side,&c,&g);
    close(c,0);close(g,f.r.energyControlAutonomousGainForSide(side));
    float free=rate_baseline::evaluate(rate,side).adjusted_deg;assert(free>=0);
    close(f.r.energyControlAutonomousCorrectedPrediction(free,side,q,&residual),free+g*q);close(residual,0);
  }}
  std::cout<<"V46ai native MEKF peak/rate/error-feedback: both sides, fixed 3 ms, bias/time independence, duplicate samples, pulse suppression, zero-output expected-side rearm, nonfinite ESTOP, base-model path PASS\n";
}
'''
for key,value in {'EVENT':event,'ENUMS':enums,'FIELDS':fields,'METHODS':methods}.items():
    source=source.replace(key,value)
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(source)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                    '-I'+str(ROOT/'src'),'-I'+str(ROOT/'tools/host_v46o'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
