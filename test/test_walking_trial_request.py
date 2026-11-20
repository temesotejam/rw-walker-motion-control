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
  const auto post=[&](const char* model, int target=8){
    return q.request(Cmd::Start,target,0,300,40,false,true,false,walking_trial::requestsSmallStep(model));
  };
  const auto consume=[&](bool expected){
    bool selected=!expected;
    assert(q.takeCommand(nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,&selected)==Cmd::Start);
    assert(selected==expected);
    q.completeCommand(true,"ok");
  };
  assert(!walking_trial::requestsSmallStep("work_control"));
  assert(!walking_trial::requestsSmallStep("work_control_small_ste"));
  assert(!walking_trial::requestsSmallStep(nullptr));
  assert(walking_trial::requestsSmallStep("work_control_small_step")==bool(WALKING_SMALL_STEP_AB));
  for(const int invalid_target : {10,12}){
    assert(!q.request(Cmd::Start,invalid_target,0,300,40,false,true,false,true));
  }
  assert(!q.request(Cmd::Start,8,0,300,40,true,true,false,true));
  assert(!q.request(Cmd::Start,8,0,400,40,false,true,false,true));
  assert(!q.request(Cmd::Clear,8,0,300,40,false,true,false,true));
  assert(post("work_control"));
  assert(!post("work_control_small_step")); // A rejected request cannot replace the waiting start.
  consume(false);
  q.snapshot_.running=true;
  assert(!post("work_control_small_step"));
  q.snapshot_.running=false;
#if WALKING_SMALL_STEP_AB
  assert(post("work_control_small_step"));
  assert(!post("work_control"));
  consume(true);
#else
  assert(!q.request(Cmd::Start,8,0,300,40,false,true,false,true));
#endif
  assert(q.request(Cmd::Start,8,0,300,40,false,true)); // Old callers omit the new option.
  consume(false);
}
'''

with tempfile.TemporaryDirectory(prefix='walking_trial_request_') as directory:
    temp = Path(directory)
    source = temp / 'queue.cpp'
    source.write_text(fixture, encoding='utf8')
    for flag in (0, 1):
        exe = temp / 'queue.exe'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        f'-DWALKING_SMALL_STEP_AB={flag}', '-I' + str(root / 'test/fixtures/arduino'),
                        '-I' + str(root / 'src'), str(source), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
print('Walking A/B START queue rejects invalid and concurrent requests and resets profile on the next start PASS')
