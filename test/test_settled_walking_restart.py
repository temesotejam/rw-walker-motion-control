"""Exercise the settled restart runner, real signed dispatch and pulse-stop paths."""
from pathlib import Path
import csv
import json
import subprocess
import tempfile
import _project
from convert_rwlog_to_csv import write_walking_restart

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf-8")


def method(start, following):
    return runner[runner.index(start):runner.index(following, runner.index(start) + len(start))]


PREFIX = r'''
#define WALKING_RECOVERY_V6 1
#define SUPPORT_TRANSFER_V39 1
#define PULSE_LIFECYCLE_FIX 1
#define EARLY_ENERGY_MAIN 1
#define WALKING_ADMISSION_V19 1
#include "walking/wheel_input.h"
#include "periodic_foot_state.h"
#include "control_setup/run_settings.h"
#include "models/peak_error_adaptation.h"
#include <cstdio>
#include <cstdlib>
#define CHECK(x) do{if(!(x)){std::fprintf(stderr,"check failed at line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
static uint32_t clock_ms=0;
uint32_t millis(){return clock_ms;} uint32_t micros(){return clock_ms*1000;}
uint64_t esp_timer_get_time(){return micros();}
enum class ExperimentState {RUNNING_BATCH_SWEEP,FINISHED,ESTOP,READY,START_SYNC,END_SYNC};
enum class EnergyControlAutonomousPhase {ENERGY_CONTROL,HOLD,STRONG_START_KICK,WAIT_FIRST_PEAK,STOP};
enum class EnergyControlAutonomousHalfCycleState {WAIT_ZERO_CROSS,PULSE_ACTIVE,WAIT_PEAK};
namespace Config {
constexpr int WALKING_TARGET8_MAX_PULSE_MS=150,ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS=100;
constexpr int WALKING_TARGET8_MAX_CURRENT_MA=1200, ENERGY_CONTROL_AUTONOMOUS_NORMAL_OUTPUT_SIGN=-1,MOTOR_DRIVER_POLARITY=-1;
constexpr float MODEL_VBAT_REFERENCE_V=7.5f;
}
struct Telemetry {
 bool current_valid=true,speed_valid=true;uint32_t current_sample_time_us=0,speed_sample_time_us=0;
 float speed_rpm=0;int consecutive_errors=0,error_raw=0,requested_current_mA=0,applied_current_mA=0,battery_mV=7540,actual_current_mA=0;
};
struct Bus {
 Telemetry tel;int writes=0,stops=0,command=0;bool ack=true;
 Telemetry telemetrySnapshot()const{return tel;}bool ok()const{return true;}
 bool setCurrentMa(int v){++writes;command=v;return ack;}void stop(){++stops;command=0;}
};
struct Imu{bool healthy=true;bool acquisitionHealthy()const{return healthy;}bool stale(uint32_t)const{return !healthy;}};
class ExperimentRunner {
 public:
 struct Status {
   ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;bool emergency_stop=false,pulse_active=false;
   uint16_t pulse_width_ms_setting=0,input_interval_ms=0;int pulse_direction=0,pulse_id=0,motor_cmd_mA=0;
   int current_mA_setting=0,beta_model_vbat_mV=7540,roller_actual_current_mA=0;
 }status_;
 Bus bus;Imu sensor;Bus* roller_=&bus;Imu* imu_=&sensor;
 EnergyControlAutonomousPhase energy_control_autonomous_phase_=EnergyControlAutonomousPhase::ENERGY_CONTROL;
 EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
 bool energy_control_autonomous_mode_=true,energy_control_autonomous_last_peak_valid_=true;
 float energy_control_autonomous_target_peak_deg_=8;
 bool optional_early_event_=false,optional_early_write_uncertain_=false;
 bool energy_control_autonomous_pulse_authorized_=false,energy_control_autonomous_pending_peak_=false;
 bool walking_return_pending_=false,work_positive_pending_=false,work_negative_pending_=false,work_walking_pending_=false,work_walking_low_pending_=false;
 float energy_control_autonomous_pending_model_energy_j=NAN,energy_control_autonomous_pending_model_energy_j_=NAN;
 bool q_ident_pulse_authorized_=false,energy_control_v0_pulse_authorized_=false,zero_cross_mode_=true;
 int walking_energy_stop_reason_=2;float walking_pulse_start_i0_ma_=0,walking_pulse_goal_ma_=0,walking_pulse_tau_s_=0;
 float predicted_signed_current_end_mA_=0;uint32_t predicted_current_end_ms_=0,active_pulse_start_ms_=0,active_pulse_start_test_ms_=0;
 uint32_t run_start_ms_=0,last_pulse_end_ms_=0,next_pulse_start_test_ms_=0;
 settled_walking_restart::Detector walking_restart_;wheel_cycle_guard::Controller wheel_cycle_guard_;
 periodic_foot::Packet periodic_foot_packet_;uint32_t walking_restart_checked_packet_=0;
 struct {float angle_deg=1,rate_dps=0;} walking_fore_aft_last_;
 struct {uint16_t timingCap(int,uint32_t,uint16_t n){return n;}} tilt_priority_;
 lateral_energy::CurrentTable walking_energy_current_;bool allow=true,foot_fresh=true,old_epoch=false;
 ExperimentRunner(){walking_energy_current_.prepare(.035f);}
 bool wheelWorkMode()const{return true;}bool walkingForeAftAllowsOutput()const{return allow;}
 uint32_t measurementTotalDurationMs()const{return 30000;}
 float predictCurrentGoalMa(int,float)const{return 325;}float predictRiseTauS(int)const{return .035f;}
 void updatePulseModelPrediction(){} template<class... T>void startTimingProbe(T...){}
 void resetEnergyControlAutonomousPeakTracker(bool){}void finishWalkingEnergyPulse(uint32_t,int){}
 void stopMotor();void stopActivePulse(uint32_t);void updateEnergyControlAutonomousPulse(uint32_t);
 bool beginEnergyControlAutonomousPulse(uint32_t,uint32_t,int8_t,uint16_t,int16_t);
 bool serviceWalkingRestart(uint32_t,uint32_t,float,float,float);
 bool tick(uint32_t t,float b,float a,float rate){
   clock_ms=t;bus.tel.current_sample_time_us=bus.tel.speed_sample_time_us=micros();
   auto& p=periodic_foot_packet_;p.count=t;p.epoch_us=old_epoch?500000:100;p.published_us=micros();
   p.left.sequence=p.right.sequence=1;p.left.image_us=p.right.image_us=foot_fresh?micros()-20000:100;
   p.left.measured=p.right.measured=p.left.estimated=p.right.estimated=2;
   return serviceWalkingRestart(t,micros(),b,a,rate);
 }
 void seed(int side){tick(995,-side,-side,side*22.f);tick(1000,-.1f*side,.1f*side,side*22.f);tick(1005,.1f*side,side,side*22.f);}
 void settle(int side,uint32_t a,uint32_t b){
   for(uint32_t t=a;t<=b;t+=5){tick(t,-side*4.8f,-side*4.8f,0);updateEnergyControlAutonomousPulse(t);}
 }
};
'''

TEST = r'''
int main(){
 lateral_energy::CurrentTable table;table.prepare(.035f);
 CHECK(!std::isfinite(settled_walking_restart::excitationAtWidth(table,1,-2,-40,325,2))); // shortened below current reversal
 CHECK(!std::isfinite(settled_walking_restart::excitationAtWidth(table,-1,0,0,500,80))); // above fitted charge
 CHECK(std::isfinite(settled_walking_restart::excitationAtWidth(table,-1,0,0,325,30)));
 ExperimentRunner history;history.seed(-1);
 history.tick(1010,-1,1,20);history.tick(1015,1,-1,-24);history.tick(1020,-1,1,20);
 history.tick(1025,1,-.1f,-9);history.tick(1030,-.1f,.1f,8);history.tick(1035,.1f,-.1f,-7);
 CHECK(history.walking_restart_.referenceSpeed(-1)==23.f); // zero chatter never reached either new excursion
 history.tick(1040,-.1f,-1,-7);CHECK(history.walking_restart_.referenceSpeed(-1)==22.f);
 for(int dir:{-1,1}){
   ExperimentRunner r;r.seed(dir);r.settle(dir,1005,2495);CHECK(r.bus.writes==0);
   r.settle(dir,2500,2500);CHECK(r.bus.writes==1&&r.bus.command==dir*1200&&r.walking_restart_.pending()==dir);
   CHECK(r.walking_restart_.records[0].queued && r.walking_restart_.records[0].predicted_j>0);
   r.tick(2510,-.1f*dir,.1f*dir,dir*8);CHECK(!r.status_.pulse_active&&r.bus.command==0&&r.walking_restart_.records[0].elapsed==10);
   CHECK(r.walking_restart_.records[0].cross==0);
   CHECK(!r.tick(2520,.1f*dir,.7f*dir,dir*8));CHECK(r.walking_restart_.pending()==0&&r.walking_restart_.records[0].cross==2520);
   r.walking_restart_.peak(2650,dir);CHECK(r.walking_restart_.records[0].opposite_peak==2650);
   CHECK(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_PEAK);
 }
 ExperimentRunner stuck;stuck.seed(-1);stuck.settle(-1,1005,7000);
 CHECK(stuck.bus.writes==2 && stuck.walking_restart_.records[0].timeout && stuck.walking_restart_.records[1].timeout);
 CHECK(stuck.walking_restart_.records[0].elapsed<=stuck.walking_restart_.records[0].width+4);
 CHECK(stuck.walking_restart_.remaining()<80 && stuck.bus.command==0 && !stuck.walking_restart_.pending());
 CHECK(!stuck.tick(7100,0.1f,-.1f,-9));CHECK(!stuck.tick(7110,-.1f,-.6f,-9));
 CHECK(!stuck.walking_restart_.records[1].cross && stuck.walking_restart_.remaining()==80);
 ExperimentRunner failed;failed.bus.ack=false;failed.seed(1);failed.settle(1,1005,7000);
 CHECK(failed.bus.writes==1 && !failed.walking_restart_.records[0].queued && !failed.walking_restart_.pending());
 ExperimentRunner operated;operated.seed(1);operated.settle(1,1005,2500);CHECK(operated.status_.pulse_active);
 clock_ms=2510;operated.stopMotor();operated.status_.state=ExperimentState::ESTOP;operated.status_.emergency_stop=true;
 operated.settle(1,2515,7000);CHECK(operated.bus.writes==1&&!operated.bus.command && operated.walking_restart_.records[0].elapsed==10);
 for(auto state:{ExperimentState::FINISHED,ExperimentState::ESTOP,ExperimentState::READY,ExperimentState::START_SYNC,ExperimentState::END_SYNC}){
   ExperimentRunner r;r.seed(-1);r.status_.state=state;r.settle(-1,1005,5000);CHECK(!r.bus.writes);
 }
 for(int invalid=0;invalid<8;++invalid){
   ExperimentRunner r;r.seed(-1);
   if(!invalid)r.foot_fresh=false;if(invalid==1)r.sensor.healthy=false;if(invalid==2)r.bus.tel.current_valid=false;
   if(invalid==3)r.walking_fore_aft_last_.angle_deg=12;if(invalid==4)r.bus.tel.battery_mV=0;
   if(invalid==5)r.allow=false;
   if(invalid==6)r.bus.tel.speed_rpm=1001;
   if(invalid==7)r.old_epoch=true;
   r.settle(-1,1005,5000);CHECK(!r.bus.writes);
 }
 ExperimentRunner healthy;healthy.seed(-1);
 for(uint32_t t=1005;t<6000;t+=5){float angle=4+2*std::sin(t*.012f);healthy.tick(t,angle,angle,24*std::cos(t*.012f));}
 CHECK(healthy.bus.writes==0);
 ExperimentRunner end;end.seed(1);end.settle(1,29500,31000);CHECK(end.bus.writes==0);
 std::puts("settled walk: signed command, real crossing + peak, timer/manual stop, bounded failed attempts and vetoes PASS");
}
'''

source = PREFIX + method("bool ExperimentRunner::beginEnergyControlAutonomousPulse(",
                         "bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse")
source += method("void ExperimentRunner::updateEnergyControlAutonomousPulse(", "#if defined(FOOT_SUPPORT_ASSIST)")
source += method("void ExperimentRunner::stopActivePulse(", "float ExperimentRunner::predictedChargeMaS")
source += method("void ExperimentRunner::stopMotor()", "void ExperimentRunner::setSyncLed")
source += method("bool ExperimentRunner::serviceWalkingRestart(", "void ExperimentRunner::appendWalkingRestartMetadata(") + TEST
with tempfile.TemporaryDirectory() as tmp:
    test = Path(tmp) / "restart.cpp"
    test.write_text(source, encoding="utf-8")
    binary = test.with_suffix(".exe")
    subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(ROOT / "src"), str(test), "-o", str(binary)],check=True)
    subprocess.run([str(binary)],check=True,stdin=subprocess.DEVNULL)

    # Compile the production writer so a renamed or reordered column cannot
    # silently put the wrong physical event in the exported CSV.
    start = runner.index("void ExperimentRunner::appendWalkingRestartMetadata(")
    writer = runner[start:runner.index("\n}\n", start) + 3]
    test.write_text('''
#include "psram_string.h"
#include "walking/wheel_input.h"
#include <cstdio>
class ExperimentRunner {
public:
  settled_walking_restart::Detector walking_restart_;
  void appendWalkingRestartMetadata(PsramString&) const;
};
''' + writer + '''
int main() {
  ExperimentRunner runner;
  auto& row = runner.walking_restart_.attempt(23228, -1);
  row.angle=4.8f; row.reference_speed=8.635f; row.demand_j=.00002033f; row.predicted_j=.00002145f;
  runner.walking_restart_.accepted(47);
  runner.walking_restart_.completed(46);
  PsramString out;
  if (!out.reserve(4096)) return 1;
  out+="{\\"schema\\":56";
  runner.appendWalkingRestartMetadata(out);
  out+="}";
  if (!out.ok()) return 1;
  std::puts(out.c_str());
}
''', encoding="utf8")
    subprocess.run(["g++", "-std=c++17", "-I", str(ROOT / "test/fixtures/arduino"), "-I",
                    str(ROOT / "src"), str(test), "-o", str(binary)], check=True)
    metadata = json.loads(subprocess.check_output([str(binary)], stdin=subprocess.DEVNULL))
    assert write_walking_restart(metadata, Path(tmp)) == 1
    with (Path(tmp) / "walking_restart.csv").open(newline="", encoding="utf8") as stream:
        row, = csv.DictReader(stream)
    assert (row["side"], row["width_ms"], row["elapsed_ms"], row["opposite_cross_ms"]) == ("-1", "47", "46", "0")
