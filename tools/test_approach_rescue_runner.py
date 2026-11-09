"""Exercise the production rescue scope with a recorded v6 sample sequence."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'src/experiment_runner.cpp').read_text(encoding='utf8')
start = '__attribute__((noinline)) bool ExperimentRunner::observeInwardStallRescue('
end = '\n__attribute__((noinline)) void ExperimentRunner::acceptMotionCrossing('
method = start + source.split(start, 1)[1].split(end, 1)[0]
fixture = r'''
#include "stall_rescue.h"
#include <cassert>
#include <cstdint>
enum class EnergyControlAutonomousHalfCycleState { WAIT_ZERO_CROSS, WAIT_PEAK };
class ExperimentRunner {
 public:
 bool wheelWorkMode() const { return true; }
 float energy_control_autonomous_target_peak_deg_=8;
 int energy_control_autonomous_input_peak_percent_=0;
 int8_t energy_control_autonomous_last_peak_side_=-1;
 bool energy_control_autonomous_last_peak_valid_=true;
 bool energy_control_autonomous_zero_cross_consumed_for_peak_=false;
 EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
 uint32_t stall_rescue_attempted_peak_ms_=0,energy_control_autonomous_last_peak_ms_=27931;
 uint32_t energy_control_autonomous_previous_detector_test_ms_=28189;
 float energy_control_autonomous_last_peak_amplitude_deg_=8.749f;
 float energy_control_autonomous_previous_detector_relative_angle_deg_=-3.11f;
 float energy_control_autonomous_previous_detector_rate_dps_=25.58f;
 stall_rescue::Detector stall_rescue_detector_,early_stall_rescue_detector_,approach_stall_rescue_detector_;
 bool early_stall_rescue_event_=false,approach_stall_rescue_event_=false;
 bool observeInwardStallRescue(uint32_t,bool,bool,float,float);
};
// METHOD
int main() {
 ExperimentRunner r;
 assert(!r.observeInwardStallRescue(28193,false,false,-3.03f,22.96f));
 r.energy_control_autonomous_previous_detector_test_ms_=28193;
 r.energy_control_autonomous_previous_detector_relative_angle_deg_=-3.03f;
 r.energy_control_autonomous_previous_detector_rate_dps_=22.96f;
 assert(!r.observeInwardStallRescue(28199,false,false,-2.95f,23.51f));
 r.energy_control_autonomous_previous_detector_test_ms_=28199;
 r.energy_control_autonomous_previous_detector_relative_angle_deg_=-2.95f;
 r.energy_control_autonomous_previous_detector_rate_dps_=23.51f;
 assert(r.observeInwardStallRescue(28204,false,false,-2.81f,22.96f));
 assert(r.approach_stall_rescue_event_ && !r.early_stall_rescue_event_);
 r.stall_rescue_attempted_peak_ms_=27931;
 assert(!r.observeInwardStallRescue(28208,false,false,-2.75f,21.f));
 assert(!r.approach_stall_rescue_event_);
 r.stall_rescue_attempted_peak_ms_=0;
 assert(!r.observeInwardStallRescue(28209,true,false,-2.74f,20.f));
 assert(!r.approach_stall_rescue_event_);
}
'''.replace('// METHOD', method)
with tempfile.TemporaryDirectory(prefix='approach_rescue_') as td:
    cpp = Path(td) / 'test.cpp'
    exe = Path(td) / 'test.exe'
    cpp.write_text(fixture, encoding='utf8')
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DSYMMETRIC_RESCUE_TRIAL=1', '-DEARLY_RESCUE_TRIAL=1',
                    '-DAPPROACH_RESCUE_TRIAL=1', '-I'+str(root / 'src'),
                    str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('approach rescue runner scope and recorded samples PASS')
