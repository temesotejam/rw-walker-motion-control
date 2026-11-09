"""Execute real early adapter/offer against controlled clock and actuator doubles."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'src/experiment_runner.cpp').read_text(encoding='utf-8')
def method(name):
    pos=s.index('ExperimentRunner::'+name+'(')
    start=s.rfind('\n',0,pos)+1
    opening=s.index('{',pos);level=1;end=opening+1
    while level:
        level += (s[end]=='{') - (s[end]=='}');end+=1
    return s[start:end]
methods='\n'.join(method(n) for n in ('footEarlyPeakIdentity','offerFootEarlyCandidate','tryEarlyFootCandidate','decideEarlyFootCandidate','acceptOptionalEarlyMotion'))
# Count calls to the extracted cold function without modifying production.
methods=methods.replace('float angle_deg,float rate_dps) {\n  const uint32_t sample_now_us=micros();',
                        'float angle_deg,float rate_dps) {\n  ++cold_calls;\n  const uint32_t sample_now_us=micros();',1)
fixture=r"""
#include "early_motion_gate.h"
#include "early_clock.h"
#include <cassert>
#include <cmath>
#include <cstdint>
using std::isfinite;
namespace Config { constexpr uint32_t ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS=250; }
enum class ExperimentState { RUNNING_BATCH_SWEEP, STOPPED };
enum class EnergyControlAutonomousHalfCycleState { WAIT_PEAK, WAIT_ZERO_CROSS };
uint32_t clock_ms=0;uint32_t millis(){return clock_ms;}uint32_t micros(){return clock_ms*1000;}
uint64_t esp_timer_get_time(){return uint64_t(clock_ms)*1000;}
struct Status {uint16_t run_id=1;bool emergency_stop=false,pulse_active=false;
 ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;};
class ExperimentRunner {public:
 Status status_;early_motion_gate::Gate early_motion_gate_;uint64_t early_measurement_epoch_us_=100000;
 EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
 bool energy_control_autonomous_last_peak_valid_=true,energy_control_autonomous_zero_cross_consumed_for_peak_=false;
 uint32_t energy_control_autonomous_last_peak_ms_=900,energy_control_autonomous_last_accepted_zero_cross_ms_=800;
 bool energy_control_autonomous_last_accepted_zero_cross_valid_=true;
 int energy_control_autonomous_last_peak_side_=-1,energy_control_autonomous_input_peak_percent_=0;
 float energy_control_autonomous_target_peak_deg_=8;uint32_t optional_early_attempted_peak_ms_=0,run_start_ms_=100;
 bool optional_early_write_uncertain_=false,optional_early_event_=false,send_success=false;
 unsigned calls=0,cold_calls=0;bool wheelWorkMode()const{return true;}
 uint32_t footEarlyPeakIdentity()const;
 bool offerFootEarlyCandidate(uint16_t,uint32_t,uint32_t,uint32_t,float);
 void tryEarlyFootCandidate(uint32_t,uint32_t,float,float);
 void decideEarlyFootCandidate(uint32_t,uint32_t,float,float);
 void acceptOptionalEarlyMotion(uint32_t,float,float);
 void acceptMotionCrossing(bool,uint32_t,float,float,float,float,float){
   ++calls;if(send_success) energy_control_autonomous_zero_cross_consumed_for_peak_=true;
 }
};
// METHODS
int main(){
 ExperimentRunner r;clock_ms=1200; // run clock 1100
 assert(!r.offerFootEarlyCandidate(2,900,1040,1090,-.5f));
 assert(!r.offerFootEarlyCandidate(1,899,1040,1090,-.5f));
 assert(!r.offerFootEarlyCandidate(1,900,1040,1090,.5f));
 assert(r.offerFootEarlyCandidate(1,900,1040,1090,-.5f));
 // Before reaching the threshold, the expensive flash function is not called.
 r.tryEarlyFootCandidate(1100,micros(),-.7f,10);assert(r.cold_calls==0);
 // A stale gyro sample cannot create a command.
 r.tryEarlyFootCandidate(1100,micros()-10001,-.5f,10);assert(r.calls==0 && r.cold_calls==1 && !r.early_motion_gate_.valid);
 assert(r.offerFootEarlyCandidate(1,900,1040,1090,-.5f));
 // Valid candidate, but before the minimum interval, is retained.
 r.energy_control_autonomous_last_accepted_zero_cross_ms_=1000;
 r.tryEarlyFootCandidate(1100,micros(),-.5f,10);assert(r.calls==0 && r.cold_calls==1 && r.early_motion_gate_.valid);
 clock_ms=1350; // run time 1250; replace frame to keep it fresh
 assert(r.offerFootEarlyCandidate(1,900,1200,1240,-.5f));
 r.tryEarlyFootCandidate(1250,micros(),-.5f,10);
 assert(r.calls==1 && !r.early_motion_gate_.valid && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);
 // No send leaves ZERO eligible and a repeated early attempt blocked.
 assert(r.offerFootEarlyCandidate(1,900,1201,1240,-.5f));
 r.tryEarlyFootCandidate(1251,micros(),-.5f,10);assert(r.calls==1);
 // A measurement epoch immediately before the 32-bit microsecond wrap
 // yields the same 64-bit-relative run time after wrap, including rounding.
 ExperimentRunner w;w.early_measurement_epoch_us_=4294960000ULL;
 w.energy_control_autonomous_last_peak_ms_=1;w.energy_control_autonomous_last_accepted_zero_cross_valid_=false;
 clock_ms=4294990;uint32_t wrapped_now=uint32_t((uint64_t(clock_ms)*1000-4294960000ULL)/1000);
 assert(w.early_motion_gate_.offer(1,1,10,20,-.5f,wrapped_now));
 w.tryEarlyFootCandidate(wrapped_now,micros(),-.5f,10);assert(w.calls==1);
 // A timestamp differing by more than 5ms fails closed.
 ExperimentRunner z;z.early_measurement_epoch_us_=4294960000ULL;
 z.energy_control_autonomous_last_peak_ms_=1;z.energy_control_autonomous_last_accepted_zero_cross_valid_=false;
 assert(z.early_motion_gate_.offer(1,1,10,20,-.5f,wrapped_now));
 z.tryEarlyFootCandidate(wrapped_now+6,micros(),-.5f,10);assert(z.calls==0);
 // An insufficient-lead candidate is discarded, not reconsidered each sample.
 ExperimentRunner shortlead;clock_ms=1350;
 assert(shortlead.offerFootEarlyCandidate(1,900,1200,1240,-.5f));
 shortlead.tryEarlyFootCandidate(1250,micros(),-.1f,10);
 assert(shortlead.calls==0 && shortlead.cold_calls==1 && !shortlead.early_motion_gate_.valid);
 shortlead.tryEarlyFootCandidate(1251,micros(),-.2f,5);
 assert(shortlead.calls==0 && shortlead.cold_calls==1);
 // Expired, wrong run/peak, or outward motion each invokes cold at most
 // once and invalidates the cached candidate.
 for(int condition=0;condition<4;++condition){
   ExperimentRunner y;clock_ms=1350;
   assert(y.offerFootEarlyCandidate(1,900,1200,1240,-.5f));
   if(condition==0)clock_ms=1421; // frame age 121ms
   if(condition==1)y.status_.run_id=2;
   if(condition==2)y.energy_control_autonomous_last_peak_ms_=901;
   const uint32_t now=clock_ms-100;
   y.tryEarlyFootCandidate(now,micros(),-.5f,condition==3?-10.f:10.f);
   assert(y.calls==0 && y.cold_calls==1 && !y.early_motion_gate_.valid);
   y.tryEarlyFootCandidate(now+1,micros(),-.5f,10.f);
   assert(y.cold_calls==1);
 }
 // A definite successful early send consumes the peak.
 ExperimentRunner x;clock_ms=1350;x.send_success=true;
 assert(x.offerFootEarlyCandidate(1,900,1200,1240,-.5f));
 x.tryEarlyFootCandidate(1250,micros(),-.5f,10);
 assert(x.calls==1 && x.energy_control_autonomous_zero_cross_consumed_for_peak_ && x.footEarlyPeakIdentity()==0);
}
""".replace('// METHODS',methods)
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(fixture,encoding='utf-8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(root/'src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('actual early offer/IMU gate/adapter ownership PASS')
