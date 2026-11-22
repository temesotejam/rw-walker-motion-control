"""Exercise the firmware's real START queue on the host with lock calls stubbed."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
worker = (root / 'src/run_control_worker.h').read_text(encoding='utf8')


def section(start, end):
    return worker[worker.index(start):worker.index(end, worker.index(start))]


fixture = '''
#include <cassert>
#include <cstdio>
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
  int mux_ = 0;
  struct { bool running=false; } snapshot_;
''' + section('  enum class Command', '  using Step') + section('  bool request(Command', '  CommandState commandState()') + section('  bool active_ =', '  RunControlSnapshot snapshot_;') + '''
};
int main(){
  Queue q;
  using Cmd=Queue::Command;
  using Profile=walking_trial::Profile;
  const auto post=[&](const char* model, int target=8){
    return q.request(Cmd::Start,target,0,300,40,false,true,false,walking_trial::requestedProfile(model));
  };
  const auto consume=[&](Profile expected){
    auto selected=Profile::Turn3;
    assert(q.takeCommand(nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,&selected)==Cmd::Start);
    assert(selected==expected);
    q.completeCommand(true,"ok");
  };
  assert(walking_trial::requestedProfile("work_control")==Profile::None);
  assert(walking_trial::requestedProfile("work_control_small_ste")==Profile::None);
  assert(walking_trial::requestedProfile(nullptr)==Profile::None);
  assert((walking_trial::requestedProfile("work_control_small_step")==Profile::SmallStep)==bool(WALKING_SMALL_STEP_AB));
  assert((walking_trial::requestedProfile("work_control_return_history")==Profile::CarryHistory)==bool(WALKING_RETURN_HISTORY));
  assert((walking_trial::requestedProfile("work_control_return_prediction")==Profile::ReturnPrediction)==bool(WALKING_RETURN_HISTORY));
  assert(!std::strcmp(walking_trial::name(Profile::ReturnPrediction),"left_return_prediction_v1"));
  assert((walking_trial::requestedProfile("work_control_left_transfer_carry")==Profile::LeftTransferCarryV1)==bool(WALKING_RETURN_HISTORY));
  assert(!std::strcmp(walking_trial::name(Profile::LeftTransferCarryV1),"left_transfer_carry_v1"));
  for(auto profile : {Profile::SmallStep,Profile::CarryHistory,Profile::ReturnPrediction,Profile::LeftTransferCarryV1}){
    for(const int target : {10,12}) assert(!q.request(Cmd::Start,target,0,300,40,false,true,false,profile));
    assert(!q.request(Cmd::Start,8,0,300,40,true,true,false,profile));
    assert(!q.request(Cmd::Start,8,0,400,40,false,true,false,profile));
    assert(!q.request(Cmd::Clear,8,0,300,40,false,true,false,profile));
  }
  assert(!q.request(Cmd::Start,8,0,300,40,false,true,false,Profile::Turn3));
  assert(post("work_control"));
  assert(!post("work_control_small_step")); // A rejected request cannot replace the waiting start.
  consume(Profile::None);
  q.snapshot_.running=true;
  assert(!post("work_control_small_step"));
  q.snapshot_.running=false;
#if WALKING_SMALL_STEP_AB
  assert(post("work_control_small_step"));
  assert(!post("work_control"));
  consume(Profile::SmallStep);
#else
  assert(!q.request(Cmd::Start,8,0,300,40,false,true,false,Profile::SmallStep));
#endif
#if WALKING_RETURN_HISTORY
  assert(post("work_control_return_history"));
  assert(!post("work_control_small_step"));
  consume(Profile::CarryHistory);
  assert(post("work_control_return_prediction"));
  consume(Profile::ReturnPrediction);
  assert(post("work_control_left_transfer_carry"));
  consume(Profile::LeftTransferCarryV1);
#else
  assert(!q.request(Cmd::Start,8,0,300,40,false,true,false,Profile::CarryHistory));
  assert(!q.request(Cmd::Start,8,0,300,40,false,true,false,Profile::ReturnPrediction));
  assert(!q.request(Cmd::Start,8,0,300,40,false,true,false,Profile::LeftTransferCarryV1));
#endif
  assert(q.request(Cmd::Start,8,0,300,40,false,true)); // Old callers omit the new option.
  consume(Profile::None);
}
'''

with tempfile.TemporaryDirectory(prefix='walking_trial_request_') as directory:
    temp = Path(directory)
    source = temp / 'queue.cpp'
    source.write_text(fixture, encoding='utf8')
    for flag, history in ((0, 0), (1, 0), (1, 1)):
        exe = temp / 'queue.exe'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        f'-DWALKING_SMALL_STEP_AB={flag}', f'-DWALKING_RETURN_HISTORY={history}', '-I' + str(root / 'test/fixtures/arduino'),
                        '-I' + str(root / 'src'), str(source), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
print('Walking A/B START queue rejects invalid and concurrent requests and resets profile on the next start PASS')
