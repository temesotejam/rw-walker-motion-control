#!/usr/bin/env python3
"""Exercise fixed 3 ms projection, HTTP guards and saved metadata on host."""
from pathlib import Path
import json
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / 'src/experiment_runner.cpp').read_text()
web = (ROOT / 'src/web_ui.cpp').read_text()
logger = (ROOT / 'src/psram_logger.cpp').read_text()

def method(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

config = (ROOT/'src/config.h').read_text()
header = (ROOT/'src/experiment_runner.h').read_text()
assert not (ROOT/'src/autonomous_timing_compensation.h').exists()
for removed in ('autonomous_timing_', 'setEnergyControlAutonomousTimingCompensation', 'energyControlAutonomousRunTimingCompensationUs'):
    assert removed not in runner+header, removed
assert '/energy-control-autonomous/timing-compensation' not in web
assert 'handleSetEnergyControlAutonomousTimingCompensation' not in web
assert 'autonomous_run_timing_compensation_ms' not in web
assert 'autonomous_timing_compensation_us_ = 0;' in method(logger, 'void PsramLogger::clear(')
# Only the completed/saved run's snapshot may feed its metadata.
logger_start = method(logger, 'void PsramLogger::startRun(')
assignment = re.search(r'  autonomous_timing_compensation_us_ = .*?;', logger_start, re.S).group()
assert 'if (downloading_) return;' in logger_start
assert logger_start.index('if (downloading_) return;') < logger_start.index(assignment)
legacy_logger = logger
compact_start = legacy_logger.find("  // Compact Autonomous metadata begin\n")
if compact_start >= 0:
    compact_end = legacy_logger.index("  // Compact Autonomous metadata end\n", compact_start) + len("  // Compact Autonomous metadata end\n")
    compact_block = legacy_logger[compact_start:compact_end]
    assert 'autonomous_timing_compensation_us' in compact_block
    legacy_logger = legacy_logger[:compact_start] + legacy_logger[compact_end:]
metadata = '\n'.join(line for line in legacy_logger.splitlines() if any('\\"'+key+'\\"' in line for key in ('autonomous_timing_compensation_us','autonomous_control_prediction_enabled','autonomous_timing_prediction_formula','autonomous_timing_compensation_selectable')))
assert metadata.count('json +=') == 4
assert 'Config::ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US' not in metadata
projection = runner.split('  // V46ac delay compensation begin\n', 1)[1].split('  // V46ac delay compensation end', 1)[0]
state_enum = re.search(r'enum class ExperimentState.*?\n};', (ROOT/'src/log_types.h').read_text(), re.S).group()
source = r'''
#include <cstdint>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
using std::isfinite;
struct String : std::string {
 using std::string::string;
 explicit String(uint32_t v):std::string(std::to_string(v)){}
};
namespace Config { CONFIG_CONSTANTS }
STATE_ENUM
struct Logger {
 bool downloading_ = false;
 bool energy_control_autonomous_mode_ = true;
 uint32_t autonomous_timing_compensation_us_ = 0;
 bool downloading() const {return downloading_;}
 void snapshot(bool energy_control_autonomous_mode) {
   if(downloading_)return;
   energy_control_autonomous_mode_ = energy_control_autonomous_mode;
   SNAPSHOT
 }
 std::string metadata() const { std::string json="{";
   METADATA
   json.back()='}';return json;
 }
};
struct ImuReading {float gy_dps=100;};
struct Imu {int reads=0;void update(){++reads;}};
struct RunControl {bool active_=false,ready_=true;bool active()const{return active_;}bool ready()const{return ready_;}} run_control;
struct ExperimentRunner {
 struct Status {
   ExperimentState state=ExperimentState::READY_TO_MEASURE;
   const char* last_error="";
   float pitch_mekf_measurement_relative_deg=1.5f,mekf_bias_y_dps=2;
   float pitch_mekf_detector_relative_deg=0,pitch_mekf_deg=0;
 } status_;
 Logger* logger_;
 bool energy_control_autonomous_mode_=true;
 int starts=0;bool start_ok=true;
 bool running()const{return status_.state==ExperimentState::START_SYNC||status_.state==ExperimentState::RUNNING_BATCH_SWEEP||status_.state==ExperimentState::TRIAL_REST||status_.state==ExperimentState::END_SYNC;}
 const Status& status()const{return status_;}
 bool startEnergyControlAutonomousCapture(){++starts;return start_ok;}
 void project(const ImuReading& r){ PROJECTION }
};
struct Server {
 std::map<std::string,std::string> args;
 int response=0;
 bool hasArg(const char* k){return args.count(k)!=0;}
 std::string arg(const char* k){return args.at(k);}
 void send(int code,const char*,const std::string&){response=code;}
};
struct WebUi {
 Server* server_; ExperimentRunner* runner_; Logger* logger_; Imu* imu_;
 void handleStartEnergyControlAutonomous();
};
HTTP_START
int main(){
 Logger log; ExperimentRunner r; r.logger_=&log;Server s;Imu imu;WebUi ui{&s,&r,&log,&imu};
 static_assert(Config::ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US==3000,"fixed delay");
 // Preserve the former 3 ms calculation across motion direction and live bias.
 for(float angle:{-8.f,0.f,8.f})for(float rate:{-100.f,-0.1f,0.f,0.1f,100.f})for(float bias:{-2.f,0.f,2.f}){
   r.status_.pitch_mekf_measurement_relative_deg=angle;r.status_.mekf_bias_y_dps=bias;
   r.project(ImuReading{rate});
   const uint32_t former_run_us=3000;
   const float expected=angle+((rate-bias)*Config::MEKF_GYRO_Y_SCALE)*(static_cast<float>(former_run_us)*1.0e-6f);
   assert(r.status_.pitch_mekf_detector_relative_deg==expected);
   assert(r.status_.pitch_mekf_measurement_relative_deg==angle);
 }
 r.project(ImuReading{NAN});assert(std::isnan(r.status_.pitch_mekf_detector_relative_deg));
 r.status_.pitch_mekf_measurement_relative_deg=NAN;r.project(ImuReading{});assert(std::isnan(r.status_.pitch_mekf_detector_relative_deg));
 r.status_.pitch_mekf_measurement_relative_deg=1.5f;
 r.energy_control_autonomous_mode_=false;r.project(ImuReading{});assert(r.status_.pitch_mekf_detector_relative_deg==1.5f);
 log.snapshot(true);const std::string saved=log.metadata();std::cout<<saved<<'\n';
 log.downloading_=true;log.snapshot(false);assert(log.metadata()==saved);log.downloading_=false;
 log.snapshot(false);std::cout<<log.metadata()<<'\n';log.snapshot(true);std::cout<<log.metadata()<<'\n';
 // Ownership guard must run before any runner/logger access.
 run_control.active_=true;WebUi detached{&s,nullptr,nullptr,nullptr};
 detached.handleStartEnergyControlAutonomous();assert(s.response==409);run_control.active_=false;
 log.downloading_=true;ui.handleStartEnergyControlAutonomous();assert(s.response==409);log.downloading_=false;
 run_control.ready_=false;ui.handleStartEnergyControlAutonomous();assert(s.response==503);run_control.ready_=true;
 assert(r.starts==0&&imu.reads==0);
 for(const char* old_setting:{"0","3","6","9","","3.0","garbage"}){
   s.args={{"timing_ms",old_setting}};ui.handleStartEnergyControlAutonomous();
   assert(s.response==400&&r.starts==0&&imu.reads==0);
 }
 s.args.clear();ui.handleStartEnergyControlAutonomous();assert(s.response==200&&r.starts==1&&imu.reads==1);
 r.start_ok=false;ui.handleStartEnergyControlAutonomous();assert(s.response==409&&r.starts==2&&imu.reads==2);
}
'''
for key, value in {
 'CONFIG_CONSTANTS':'\n'.join(re.findall(r'static constexpr (?:uint32_t ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US|float MEKF_GYRO_Y_SCALE) = .*?;',config)),
 'STATE_ENUM':state_enum,'SNAPSHOT':assignment,'METADATA':metadata,'PROJECTION':projection,
 'HTTP_START':method(web,'void WebUi::handleStartEnergyControlAutonomous('),
}.items():
    source=re.sub(r"\b"+key+r"\b", lambda _: value, source)
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(source)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src'),str(cpp),'-o',str(exe)],check=True)
    rows=[json.loads(line) for line in subprocess.check_output([str(exe)],text=True).splitlines()]
    assert len(rows)==3
    for row,ms in zip(rows,[3,0,3]):
        assert row['autonomous_timing_compensation_us']==ms*1000,row
        assert row['autonomous_timing_compensation_selectable'] is False
        assert row['autonomous_control_prediction_enabled']==(ms>0),row
        assert 'autonomous_timing_compensation_us*1e-6' in row['autonomous_timing_prediction_formula']
subprocess.run(['node',str(ROOT/'tools/test_v46aj_fixed_timing_ui.js')],check=True)
print('V46aj PASS: fixed 3 ms; former 3 ms equivalence; posterior preserved; metadata; rejected legacy settings; HTTP ownership/start guards; UI races')
