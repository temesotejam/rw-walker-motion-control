"""Run the actual angle ETA precheck, decision and existing adapter with doubles."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'src/experiment_runner.cpp').read_text(encoding='utf8')
def method(name):
    pos=s.index('ExperimentRunner::'+name+'(');start=s.rfind('\n',0,pos)+1;opening=s.index('{',pos);level=1;end=opening+1
    while level:
        level+=(s[end]=='{')-(s[end]=='}');end+=1
    return s[start:end]
methods='\n'.join(method(n) for n in ('tryAngleEarlyCandidate','decideAngleEarlyCandidate','acceptOptionalEarlyMotion'))
# Count calls to the extracted cold method without changing production.
methods=methods.replace('float angle_deg,float rate_dps) {\n  const int8_t side=',
                        'float angle_deg,float rate_dps) {\n  ++cold_calls;\n  const int8_t side=',1)
fixture=r"""
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
using std::isfinite;
namespace Config {constexpr uint32_t ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS=250;}
enum class State { WAIT_PEAK, WAIT_ZERO_CROSS };
using EnergyControlAutonomousHalfCycleState=State;
uint32_t clock_ms=1300;uint32_t micros(){return clock_ms*1000;}
struct Status {bool emergency_stop=false,pulse_active=false;};
class ExperimentRunner { public:
 Status status_;State energy_control_autonomous_half_cycle_state_=State::WAIT_ZERO_CROSS;
 bool energy_control_autonomous_last_peak_valid_=true,energy_control_autonomous_zero_cross_consumed_for_peak_=false;
 uint32_t energy_control_autonomous_last_peak_ms_=900,energy_control_autonomous_last_accepted_zero_cross_ms_=800;
 bool energy_control_autonomous_last_accepted_zero_cross_valid_=true;
 int8_t energy_control_autonomous_last_peak_side_=-1;int energy_control_autonomous_input_peak_percent_=0;
 float energy_control_autonomous_target_peak_deg_=8;uint32_t optional_early_attempted_peak_ms_=0,angle_late_blocked_peak_ms_=0,angle_last_check_ms_=0;
 bool optional_early_write_uncertain_=false,optional_early_event_=false,send_success=false;
 unsigned calls=0,cold_calls=0;bool wheelWorkMode()const{return true;}
 void tryAngleEarlyCandidate(uint32_t,uint32_t,float,float);
 void decideAngleEarlyCandidate(uint32_t,uint32_t,float,float);
 void acceptOptionalEarlyMotion(uint32_t,float,float);
 void acceptMotionCrossing(bool,uint32_t,float,float,float,float,float){++calls;if(send_success)energy_control_autonomous_zero_cross_consumed_for_peak_=true;}
};
// METHODS
int main(){
 {ExperimentRunner r;r.tryAngleEarlyCandidate(1200,micros(),-.8f,32);assert(r.calls==1 && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);r.tryAngleEarlyCandidate(1201,micros(),-.7f,30);assert(r.calls==1);}
 {ExperimentRunner r;r.energy_control_autonomous_last_peak_side_=1;r.send_success=true;r.tryAngleEarlyCandidate(1200,micros(),.8f,-32);assert(r.calls==1 && r.energy_control_autonomous_zero_cross_consumed_for_peak_);}
 {ExperimentRunner r;r.tryAngleEarlyCandidate(1200,micros(),-.3f,12);assert(r.calls==0);r.tryAngleEarlyCandidate(1205,micros(),-1.51f,60);assert(r.calls==0);r.tryAngleEarlyCandidate(1210,micros(),-.4f,5);assert(r.calls==0);}
 {ExperimentRunner r;r.tryAngleEarlyCandidate(1200,micros(),-.8f,-32);assert(r.calls==0);}
 {ExperimentRunner r;r.tryAngleEarlyCandidate(1200,micros(),-1.5f,50);assert(r.calls==1);}
 {ExperimentRunner r;r.tryAngleEarlyCandidate(1200,micros(),-.8f,40);assert(r.calls==0 && r.angle_late_blocked_peak_ms_==900);r.tryAngleEarlyCandidate(1201,micros(),-.7f,28);assert(r.calls==0);}
 {ExperimentRunner r;r.tryAngleEarlyCandidate(1200,micros()-10001,-.8f,32);assert(r.calls==0 && r.cold_calls==1);r.tryAngleEarlyCandidate(1201,micros(),-.8f,32);assert(r.calls==0 && r.cold_calls==1);r.tryAngleEarlyCandidate(1205,micros(),-.8f,32);assert(r.calls==1 && r.cold_calls==2);}
 {ExperimentRunner r;r.energy_control_autonomous_last_accepted_zero_cross_ms_=1000;r.tryAngleEarlyCandidate(1200,micros(),-.8f,32);assert(r.calls==0);r.tryAngleEarlyCandidate(1250,micros(),-.8f,32);assert(r.calls==1);}
 {ExperimentRunner r;r.tryAngleEarlyCandidate(1200,micros(),-2.f,40);assert(r.cold_calls==0);r.tryAngleEarlyCandidate(1200,micros(),-.3f,12);assert(r.cold_calls==1);r.tryAngleEarlyCandidate(1201,micros(),-.4f,16);assert(r.cold_calls==1);r.tryAngleEarlyCandidate(1205,micros(),-.4f,16);assert(r.cold_calls==2 && r.calls==1);}
 {ExperimentRunner r;r.status_.emergency_stop=true;r.tryAngleEarlyCandidate(1200,micros(),-.8f,32);assert(r.calls==0);}
 {ExperimentRunner r;r.energy_control_autonomous_input_peak_percent_=1;r.tryAngleEarlyCandidate(1200,micros(),-.8f,32);assert(r.calls==0);}
 {ExperimentRunner r;auto nan=std::numeric_limits<float>::quiet_NaN();r.tryAngleEarlyCandidate(1200,micros(),nan,32);r.tryAngleEarlyCandidate(1200,micros(),-.8f,nan);assert(r.calls==0);}
}
""".replace('// METHODS',methods)
with tempfile.TemporaryDirectory() as d:
 cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(fixture,encoding='utf8')
 subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('actual angle ETA and early adapter gates PASS')
