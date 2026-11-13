"""Reproduce v15's stale-caller-clock abort and verify the v16 lifecycle fix.

The actual pulse begin AND update methods run in their real call order. A
transport double advances the clocks during a successful/failed write. This
tests software command duration, not when physical current starts or stops.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
runner=(ROOT/'src/experiment_runner.cpp').read_text(encoding='utf8')
config=(ROOT/'src/config.h').read_text(encoding='utf8')


def method(name, result):
    start=runner.index(result+' ExperimentRunner::'+name+'(')
    return runner[start:runner.index('\n}\n',start)+3]


methods=method('beginEnergyControlAutonomousPulse','bool')+'\n'+method('updateEnergyControlAutonomousPulse','void')
constants=[]
for name in sorted(set(re.findall(r'Config::(\w+)',methods))):
    match=re.search(r'static constexpr\s+\w+\s+'+re.escape(name)+r'\s*=[^;]+;',config)
    assert match, name
    constants.append(match.group())

fixture=r'''
#include "autonomous_input_current.h"
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <iostream>
#define CHECK(x) do {if(!(x)){std::cerr<<"check failed: "<<#x<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
static uint32_t clock_ms=0,clock_us=0;
inline uint32_t millis(){return clock_ms;}
inline uint32_t micros(){return clock_us;}
namespace Config {CONSTANTS}
enum class ExperimentState {RUNNING_BATCH_SWEEP,OTHER};
enum class EnergyControlAutonomousPhase {STRONG_START_KICK,WAIT_FIRST_PEAK,ENERGY_CONTROL,HOLD,STOP};
enum class EnergyControlAutonomousHalfCycleState {WAIT_ZERO_CROSS,PULSE_ACTIVE,WAIT_PEAK};
struct Status {
 bool emergency_stop=false,pulse_active=false;
 ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;
 int16_t current_mA_setting=0,motor_cmd_mA=0;int8_t pulse_direction=0;
 uint16_t pulse_width_ms_setting=0;uint32_t pulse_id=0;
 float beta_model_vbat_mV=7500;
};
struct Roller {
 bool ready=true,write_ok=true;uint32_t delay_ms=0,writes=0;
 bool* active=nullptr;bool active_during_write=false;int16_t last_command=0;
 bool ok()const{return ready;}
 bool setCurrentMa(int16_t command){
  ++writes;last_command=command;active_during_write=active && *active;
  clock_ms+=delay_ms;clock_us+=delay_ms*1000U;return write_ok;
 }
};
class ExperimentRunner {public:
 Status status_;Roller driver_;Roller* roller_=&driver_;
 bool work=true,energy_control_autonomous_mode_=true;
 float energy_control_autonomous_target_peak_deg_=8.f;
 bool energy_control_autonomous_pulse_authorized_=false;
 bool optional_early_event_=false,optional_early_write_uncertain_=false;
 EnergyControlAutonomousPhase energy_control_autonomous_phase_=EnergyControlAutonomousPhase::ENERGY_CONTROL;
 EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
 uint32_t run_start_ms_=0,active_pulse_start_ms_=111,active_pulse_start_test_ms_=222;
 uint32_t predicted_current_end_ms_=0;float predicted_signed_current_end_mA_=0;
#if PULSE_LIFECYCLE_FIX
 float walking_pulse_start_i0_ma_=NAN,walking_pulse_goal_ma_=NAN,walking_pulse_tau_s_=NAN;
#endif
 unsigned stops=0,motor_stops=0,resets=0,model_updates=0,probes=0;
 uint32_t stopped_ms=0,probe_test_ms=0,probe_write_us=0;uint16_t probe_width=0;
 ExperimentRunner(){driver_.active=&status_.pulse_active;}
 bool wheelWorkMode()const{return work;}
 // Current prediction is a deterministic double; the tested contract is
 // its time origin and full selected width, not current-model calibration.
 float predictCurrentGoalMa(int,float)const{return 300.f;}
 float predictRiseTauS(int)const{return .0316f;}
 void updatePulseModelPrediction(){++model_updates;}
 void stopMotor(){++motor_stops;status_.pulse_active=false;status_.motor_cmd_mA=0;}
 void stopActivePulse(uint32_t now){++stops;stopped_ms=now;status_.pulse_active=false;status_.motor_cmd_mA=0;}
 void resetEnergyControlAutonomousPeakTracker(bool){++resets;}
 void startTimingProbe(int,uint32_t test_ms,int16_t,uint16_t width,uint32_t,
                       uint32_t write_us,uint32_t,uint32_t,uint32_t,uint32_t){
   ++probes;probe_test_ms=test_ms;probe_width=width;probe_write_us=write_us;
 }
 bool beginEnergyControlAutonomousPulse(uint32_t,uint32_t,int8_t,uint16_t,int16_t);
 void updateEnergyControlAutonomousPulse(uint32_t);
};
METHODS
static bool close(float a,float b){return std::fabs(a-b)<.0001f;}
int main(){
 unsigned reproduced=0,full_duration=0;
 for(uint32_t captured:{100000U,UINT32_MAX-15U,UINT32_MAX-3U})
 for(uint32_t delay:{0U,1U,7U})for(uint32_t decision_age:{0U,3U})
 for(unsigned mode=0;mode<4;++mode){
  ExperimentRunner r;r.run_start_ms_=captured-1000U;r.driver_.delay_ms=delay;
  r.work=mode!=3;r.energy_control_autonomous_target_peak_deg_=mode==1?10.f:mode==2?12.f:8.f;
  r.predicted_current_end_ms_=captured-30U;r.predicted_signed_current_end_mA_=100.f;
  clock_ms=captured+decision_age;clock_us=123456U;
  const uint16_t width=40;
  CHECK(r.beginEnergyControlAutonomousPulse(captured,1000,1,width,1200));
  const bool rebased=EARLY_ENERGY_MAIN && mode==0;
  const uint32_t origin=rebased?captured+decision_age+delay:captured;
  CHECK(r.active_pulse_start_ms_==origin);
  CHECK(r.active_pulse_start_test_ms_==static_cast<uint32_t>(origin-r.run_start_ms_));
  CHECK(r.predicted_current_end_ms_==static_cast<uint32_t>(origin+width));
  CHECK(r.status_.pulse_width_ms_setting==width && r.status_.pulse_id==1);
  CHECK(r.status_.pulse_active && !r.driver_.active_during_write);
  CHECK(r.driver_.writes==1 && r.model_updates==1 && r.probes==1);
  CHECK(r.probe_test_ms==r.active_pulse_start_test_ms_ && r.probe_width==width);
  CHECK(r.probe_write_us==delay*1000U);
  const float i0=100.f*std::exp(-static_cast<float>(static_cast<uint32_t>(origin-(captured-30U)))/70.f);
  CHECK(close(r.predicted_signed_current_end_mA_,300.f+(i0-300.f)*std::exp(-.040f/.0316f)));
#if PULSE_LIFECYCLE_FIX
  CHECK(close(r.walking_pulse_start_i0_ma_,i0));
  CHECK(r.walking_pulse_goal_ma_==300.f && close(r.walking_pulse_tau_s_,.0316f));
#endif
  // The real outer iteration captured `captured` BEFORE the successful
  // driver write. It passes that same stale value to the update immediately.
  r.updateEnergyControlAutonomousPulse(captured);
  const bool old_abort=EARLY_ENERGY_MAIN && !PULSE_LIFECYCLE_FIX && mode==0 && (delay+decision_age)>0;
  if(old_abort){
   CHECK(!r.status_.pulse_active && r.stops==1);++reproduced;
   // The pre-fix failure occurs with zero positive elapsed software time:
   // unsigned(captured-origin) wrapped into an apparently enormous duration.
   CHECK(static_cast<uint32_t>(captured-origin)>=width);
   CHECK(r.stopped_ms==captured);continue;
  }
  CHECK(r.status_.pulse_active && r.stops==0);
  clock_ms=origin+width-1U;r.updateEnergyControlAutonomousPulse(clock_ms);
  CHECK(r.status_.pulse_active && r.stops==0);
  clock_ms=origin+width;r.updateEnergyControlAutonomousPulse(clock_ms);
  CHECK(!r.status_.pulse_active && r.stops==1 && r.stopped_ms==clock_ms);
  CHECK(r.status_.current_mA_setting==0 && r.status_.pulse_width_ms_setting==0);
  CHECK(!r.energy_control_autonomous_pulse_authorized_ && r.resets==1);
  CHECK(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_PEAK);
  clock_ms+=100;r.updateEnergyControlAutonomousPulse(clock_ms);CHECK(r.stops==1);
  ++full_duration;
 }
 CHECK(full_duration>0);
 if(EARLY_ENERGY_MAIN && !PULSE_LIFECYCLE_FIX)CHECK(reproduced>0);
 else CHECK(reproduced==0);
 // Transport failure remains unarmed; uncertainty is explicit only for an
 // optional early attempt. A failed write does not prove physical no-output.
 for(bool optional:{false,true})for(uint32_t delay:{0U,1U,7U}){
  ExperimentRunner r;r.run_start_ms_=99000;r.optional_early_event_=optional;
  r.driver_.delay_ms=delay;r.driver_.write_ok=false;
  r.predicted_current_end_ms_=99970;r.predicted_signed_current_end_mA_=100;
  clock_ms=100000;clock_us=123456;
  CHECK(!r.beginEnergyControlAutonomousPulse(100000,1000,1,40,1200));
  CHECK(!r.status_.pulse_active && r.status_.pulse_id==0 && r.driver_.writes==1);
  CHECK(r.active_pulse_start_ms_==111 && r.active_pulse_start_test_ms_==222);
  CHECK(r.predicted_current_end_ms_==99970 && r.predicted_signed_current_end_mA_==100);
  CHECK(!r.energy_control_autonomous_pulse_authorized_ && r.motor_stops==1);
  CHECK(r.optional_early_write_uncertain_==optional && r.model_updates==0 && r.probes==0);
#if PULSE_LIFECYCLE_FIX
  CHECK(std::isnan(r.walking_pulse_start_i0_ma_) && std::isnan(r.walking_pulse_goal_ma_) && std::isnan(r.walking_pulse_tau_s_));
#endif
  r.updateEnergyControlAutonomousPulse(100000);CHECK(r.stops==0);
 }
 std::cout<<"pulse lifecycle: early="<<EARLY_ENERGY_MAIN<<" fix="<<PULSE_LIFECYCLE_FIX
          <<" reproduced_stale_abort="<<reproduced<<" full_duration="<<full_duration<<" PASS\n";
}
'''.replace('CONSTANTS','\n'.join(constants)).replace('METHODS',methods)

with tempfile.TemporaryDirectory(prefix='pulse_lifecycle_v16_') as temporary:
    tmp=Path(temporary)
    (tmp/'test.cpp').write_text(fixture,encoding='utf8')
    for name,early,fix in [('v14_reference',0,0),('v15_reproduced',1,0),('v16_fixed',1,1)]:
        exe=tmp/(name+'.exe')
        subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
                        f'-DEARLY_ENERGY_MAIN={early}',f'-DPULSE_LIFECYCLE_FIX={fix}',
                        '-I'+str(ROOT/'src'),str(tmp/'test.cpp'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)

print('actual begin/update lifecycle: stale-caller reproduction, write 0/1/7ms, rollover, full width and failed-write PASS')
