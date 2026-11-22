"""Compile the real request gates and default walking selection on a host."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]


def section(file, start, end):
    code = (root / "src" / file).read_text(encoding="utf8")
    first = code.index(start)
    return code[first:code.index(end, first)]


worker = 'run_control_worker.h'
fixture = '''
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include "walking_trial_profile.h"
#include "autonomous_target.h"
#include "autonomous_input_percent.h"
#include "autonomous_input_current.h"
#include "autonomous_brake_limit.h"
#include "work_target_policy.h"
#include "lateral_cg_energy.h"
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
class Queue {
public:
  bool ready() const { return true; }
  int mux_=0;
  struct {bool running=false;} snapshot_;
''' + section(worker,'  enum class Command','  using Step') + section(worker,'  bool request(Command','  CommandState commandState()') + section(worker,'  bool active_ =','  RunControlSnapshot snapshot_;') + '''
};
using Profile=walking_trial::Profile;
class Runner {
public:
  struct Logger {
    Profile profile=Profile::None;
    Profile walkingTrialProfile() const {return profile;}
  } logger;
  Logger* logger_=&logger;
  bool energy_bridge_enabled_=false, negative_probe_mode_=false;
  uint16_t energy_control_autonomous_brake_max_ms_=0;
  float energy_control_autonomous_target_peak_deg_=0, energy_control_autonomous_input_peak_percent_=0;
  void beginStartSync(uint32_t,Profile profile){logger.profile=profile;}
''' + section('experiment_runner.h','  bool carryHistoryTrialSelected()', '  bool setEnergyControlAutonomousTarget(') + section('experiment_runner.h','  bool wheelWorkMode()', '  bool negativeProbeBatchMode()') + '''
  bool startAccepted() {
''' + section('experiment_runner.cpp','  const auto profile=walking_trial::forRun(', '\nbool ExperimentRunner::startQIdentCapture') + '''
};
struct Model : std::string {
  using std::string::string;
  bool startsWith(const char* prefix) const {return find(prefix)==0;}
};
struct Server {
  int status=0;
  void send(int code,const char*,const char*) {status=code;}
};
static void post(Queue& queue,Server* server_,const Model& model,float target=8,int current=300) {
''' + section('web_ui.cpp','  if (model.startsWith("work_control_"))', '  if (cmd == RunControlWorker::Command::Start && !work_start_policy') + '''
  assert(work_control);
  server_->status=queue.request(Queue::Command::Start,target,0,current,40,false,true,false)?202:409;
}
int main(){
  Queue q;
  Runner r;
  Server http;
  for (auto retired : {"work_control_small_step","work_control_return_history","work_control_return_prediction",
                       "work_control_left_transfer_carry","work_control_unknown"}) {
    post(q,&http,retired);
    assert(http.status==400 && q.takeCommand()==Queue::Command::None);
  }
  for(const float target : {8.f, 10.f, 12.f, 8.f}){
    post(q,&http,"work_control",target);
    assert(http.status==202);
    post(q,&http,"work_control",target);
    assert(http.status==409);
    int16_t current;bool adapt;
    assert(q.takeCommand(&r.energy_control_autonomous_target_peak_deg_,&r.energy_control_autonomous_input_peak_percent_,
                &current,&r.energy_control_autonomous_brake_max_ms_,&adapt,&r.energy_bridge_enabled_,
                &r.negative_probe_mode_)==Queue::Command::Start);
    assert(current==300 && !adapt && r.energy_control_autonomous_target_peak_deg_==target);
    assert(r.startAccepted());
    const auto measured=r.logger.walkingTrialProfile();
    if(target==8 && WALKING_SMALL_STEP_AB) {
#if WALKING_RETURN_HISTORY
      assert(!std::strcmp(walking_trial::name(measured),"left_transfer_carry_v4"));
      assert(r.isTransferPriorityProfile() && r.isLeftTransferCarryProfile() && r.carryHistoryTrialSelected() && r.returnPredictionTrialSelected());
#elif WALKING_RIGHT_ARRIVAL
      assert(measured==Profile::RightArrival);
#else
      assert(measured==Profile::Turn3);
#endif
    } else {
      assert(measured==Profile::None && !r.isLeftTransferCarryProfile());
    }
    assert(!r.smallStepBaselineSelected());
    q.completeCommand(true,"ok");
  }
  post(q,&http,"work_control",8,400);
  assert(http.status==409 && q.takeCommand()==Queue::Command::None);
  q.snapshot_.running=true;
  post(q,&http,"work_control");
  assert(http.status==409 && q.takeCommand()==Queue::Command::None);
  assert(walking_trial::forRun(false,8,0)==Profile::None);
  assert(walking_trial::forRun(true,8,1)==Profile::None);
}
'''

with tempfile.TemporaryDirectory(prefix='walking_trial_request_') as directory:
    temp = Path(directory)
    source = temp / 'queue.cpp'
    source.write_text(fixture, encoding='utf8')
    for walking, arrival, history in ((0, 0, 0), (1, 0, 0), (1, 1, 0), (1, 1, 1)):
        exe = temp / 'queue.exe'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        f'-DWALKING_SMALL_STEP_AB={walking}', f'-DWALKING_RIGHT_ARRIVAL={arrival}',
                        f'-DWALKING_RETURN_HISTORY={history}', '-I' + str(root / 'test/fixtures/arduino'),
                        '-I' + str(root / 'src'), str(source), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
print('Normal walking START selects one controller; retired requests, non-walking targets and stale state checked')
