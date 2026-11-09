"""Exercise the real autonomous pulse admission guard with host doubles."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
config = (root / 'src/config.h').read_text(encoding='utf-8')
runner = (root / 'src/experiment_runner.cpp').read_text(encoding='utf-8')
def value(name):
    match = re.search(r'\b' + name + r'\s*=\s*(\d+)', config)
    assert match, name
    return int(match.group(1))

walking_ma = value('WALKING_TARGET8_MAX_CURRENT_MA')
walking_ms = value('WALKING_TARGET8_MAX_PULSE_MS')
normal_ms = value('ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS')
assert (walking_ma, walking_ms, normal_ms) == (1200, 150, 100)
method = 'bool ExperimentRunner::beginEnergyControlAutonomousPulse(' + runner.split(
    'bool ExperimentRunner::beginEnergyControlAutonomousPulse(', 1)[1].split(
    '  const uint32_t begin_t0_us = micros();', 1)[0] + '  return true;\n}\n'
fixture = r'''#include "autonomous_input_current.h"
#include <cassert>
#include <cstdint>
namespace Config {
 constexpr int WALKING_TARGET8_MAX_CURRENT_MA = WALKING_MA;
 constexpr int WALKING_TARGET8_MAX_PULSE_MS = WALKING_MS;
 constexpr int ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = NORMAL_MS;
}
enum class ExperimentState { RUNNING_BATCH_SWEEP, OTHER };
enum class EnergyControlAutonomousPhase { ENERGY_CONTROL, HOLD, OTHER };
enum class EnergyControlAutonomousHalfCycleState { WAIT_ZERO_CROSS, OTHER };
struct Status { bool emergency_stop=false, pulse_active=false;
  ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP; };
struct Roller { bool ok() const { return true; } };
class ExperimentRunner { public:
 bool work=true, energy_control_autonomous_mode_=true;
 float energy_control_autonomous_target_peak_deg_=8.f;
 Status status_; Roller roller_storage_; Roller* roller_=&roller_storage_;
 EnergyControlAutonomousPhase energy_control_autonomous_phase_=EnergyControlAutonomousPhase::ENERGY_CONTROL;
 EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
 bool wheelWorkMode() const { return work; }
 bool beginEnergyControlAutonomousPulse(uint32_t,uint32_t,int8_t,uint16_t,int16_t);
};
// METHOD
int main() {
 ExperimentRunner r;
 assert(r.beginEnergyControlAutonomousPulse(0,0,1,150,1200));
 assert(r.beginEnergyControlAutonomousPulse(0,0,-1,100,500));
 assert(!r.beginEnergyControlAutonomousPulse(0,0,1,151,1200));
 assert(!r.beginEnergyControlAutonomousPulse(0,0,1,150,1210));
 assert(!r.beginEnergyControlAutonomousPulse(0,0,0,150,1200));
 r.work=false;
 assert(r.beginEnergyControlAutonomousPulse(0,0,1,100,1200));
 assert(!r.beginEnergyControlAutonomousPulse(0,0,1,101,1200));
 r.work=true; r.status_.emergency_stop=true;
 assert(!r.beginEnergyControlAutonomousPulse(0,0,1,150,1200));
}
'''.replace('WALKING_MA', str(walking_ma)).replace('WALKING_MS', str(walking_ms)).replace(
    'NORMAL_MS', str(normal_ms)).replace('// METHOD', method)
with tempfile.TemporaryDirectory(prefix='walking_limit_') as td:
    source, exe = Path(td)/'limit.cpp', Path(td)/'limit.exe'
    source.write_text(fixture, encoding='utf-8')
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                    '-I' + str(root/'src'), str(source), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('actual autonomous walking pulse limits PASS')
