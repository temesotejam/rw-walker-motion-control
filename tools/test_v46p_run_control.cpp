// Deterministic lifecycle/accounting tests of the actual worker implementation.
// Task creation is mocked: this is not a FreeRTOS scheduling/timing benchmark.
#include "host_v46o/Arduino.h"
#include <cassert>
#include <iostream>
#define private public
#include "../src/run_control_worker.h"
#undef private
struct Model { bool running=false; unsigned iterations=0; RunControlWorker* worker=nullptr; };
static bool step(void* ptr) {
  auto& m=*static_cast<Model*>(ptr); ++m.iterations;
  const uint32_t start=micros();
  if(m.worker->takeStopRequest()) m.running=false;
  host_us+=750;
  m.worker->recordStep(start,100,600,750);
  return m.running;
}
static void capture(void* ptr,RunControlSnapshot& s){
  const auto& m=*static_cast<Model*>(ptr);
  s.running=m.running;s.state_id=m.running?6:5;s.motor_cmd_mA=m.running?0:0;
  strcpy(s.state_name,m.running?"START_SYNC":"ESTOP");
}
int main(){
  RunControlWorker w;Model m;m.worker=&w;host_us=1000;
  assert(!w.ready()&&!w.active()&&!w.requestStop()&&!w.start());
  assert(w.begin(step,capture,&m));assert(w.ready());assert(!w.begin(step,capture,&m));
  assert(!w.start());m.running=true;assert(w.start());assert(!w.start());
  for(int i=0;i<40;++i)w.oneStep();
  assert(w.active()&&m.iterations==40&&w.snapshot().running);
  assert(w.audit_.steps==40&&w.audit_.max_runner_us==600);
  assert(w.requestStop());w.oneStep();
  assert(!w.active()&&!w.snapshot().running&&m.iterations==41);
  assert(w.audit_.stop_requests==1&&w.audit_.stop_consumed==1);
  const auto json=w.diagnosticsJson().value;
  assert(json.find("recent_steps")!=std::string::npos);
  assert(!w.requestStop());
  // A later explicit Start may reacquire ownership; statistics start a new epoch.
  // Production start gates/fault latches remain in ExperimentRunner/ImuManager.
  host_us=UINT32_MAX-200;m.running=true;assert(w.start());
  host_us=300;w.oneStep();assert(w.audit_.max_period_us==501);
  m.running=false;w.oneStep();assert(!w.active());
  std::cout<<"V46p real worker lifecycle, stop handoff, snapshots, ring and timestamp wrap PASS\n";
}
