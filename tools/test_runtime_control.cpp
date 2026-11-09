#include "host_v46o/Arduino.h"
#include <cassert>
#include <iostream>
#define private public
#include "../src/run_control_worker.h"
#undef private
struct Model {bool running=false;bool adaptation=true,probe=false;unsigned steps=0;float target=8,angle=0;int16_t current=300;uint16_t brake=50;RunControlWorker* worker=nullptr;};
static bool step(void* p) {
  auto& m=*static_cast<Model*>(p); ++m.steps;
  if(m.worker->takeStopRequest()) m.running=false;
  float target=0,angle=0;int16_t current=0;uint16_t brake=50;
  const auto command=m.worker->takeCommand(&target,&angle,&current,&brake,&m.adaptation,nullptr,&m.probe);
  if(command==RunControlWorker::Command::Start){m.target=target;m.angle=angle;m.current=current;m.brake=brake;m.running=true;m.worker->beginRunAudit();m.worker->completeCommand(true,"started");
    assert(!m.worker->commandPublished(m.worker->commandState().completed));}
  if(command==RunControlWorker::Command::Clear){m.running=false;m.worker->completeCommand(true,"cleared");}
  if(m.running){const auto t=micros();host_us+=750;m.worker->recordStep(t,100,600,750);}
  return m.running;
}
static void capture(void* p,RunControlSnapshot& s) {
  s.running=static_cast<Model*>(p)->running;s.state_id=s.running?3:2;host_us+=100;
  s.target_deg=static_cast<Model*>(p)->target;
  s.input_peak_percent=static_cast<Model*>(p)->angle;
  s.input_current_mA=static_cast<Model*>(p)->current;
  s.brake_max_ms=static_cast<Model*>(p)->brake;
  s.peak_adaptation_enabled=static_cast<Model*>(p)->adaptation;
  s.negative_probe_mode=static_cast<Model*>(p)->probe;
}
int main(){
  for(const char* bad : {"", "9", "0", "-10", "14", "10junk", "nan", "inf", "1e999", "1e-999"}) {
    float target=12;
    assert(!autonomous_target::parse(bad,target)&&target==12);
  }
  float parsed=8;
  assert(!autonomous_target::parse(nullptr,parsed)&&parsed==8);
  assert(autonomous_target::parse("8",parsed)&&parsed==8);
  assert(autonomous_target::parse("10.0",parsed)&&parsed==10);
  assert(autonomous_target::parse("12",parsed)&&parsed==12);
  for(const char* bad : {"", "-0.1", "1", "50", "100", "100.1", "nan", "inf", "1junk", "1e999", "1e-999"}) {
    float angle=2;assert(!autonomous_input_percent::parse(bad,angle)&&angle==2);
  }
  for(const char* good : {"0"}) {
    float angle=-1;assert(autonomous_input_percent::parse(good,angle));
  }
  for(const char* bad : {"", "99", "1201", "555", "100junk", "10000000000"}) {
    int16_t current=500;assert(!autonomous_input_current::parse(bad,current)&&current==500);
  }
  for(const char* good : {"100", "300", "500", "1200"}) {
    int16_t current=0;assert(autonomous_input_current::parse(good,current));
  }
  RunControlWorker w;Model m;m.worker=&w;
  assert(!w.request(RunControlWorker::Command::Start));
  assert(w.begin(step,capture,&m));assert(!w.begin(step,capture,&m));
  w.oneStep();w.oneStep();assert(m.steps==2&&!w.active()); // permanent idle owner
  assert(!w.request(RunControlWorker::Command::Start,9));
  assert(!w.request(RunControlWorker::Command::Start,NAN));
  assert(!w.request(RunControlWorker::Command::Start,INFINITY));
  assert(!w.commandState().pending&&w.commandState().submitted==0);
  assert(!w.request(RunControlWorker::Command::Start,10,NAN));
  assert(!w.request(RunControlWorker::Command::Start,10,100.1f));
  assert(!w.request(RunControlWorker::Command::Start,10,0.0f,90));
  assert(!w.request(RunControlWorker::Command::Start,10,0.0f,1210));
  assert(!w.request(RunControlWorker::Command::Start,10,0.0f,500,10));
  assert(!w.request(RunControlWorker::Command::Start,8,0.0f,300,20,false,true,true));
  assert(!w.request(RunControlWorker::Command::Start,10,0.0f,300,15,false,true,true));
  assert(!w.request(RunControlWorker::Command::Start,8,0.0f,500,15,false,true,true));
  assert(!w.request(RunControlWorker::Command::Start,8,0.0f,300,15,true,true,true));
  assert(!w.request(RunControlWorker::Command::Start,8,0.0f,300,15,false,false,true));
  assert(!w.request(RunControlWorker::Command::SetLateralCg,NAN));
  assert(!w.request(RunControlWorker::Command::SetLateralCg,2.01f));
  assert(w.request(RunControlWorker::Command::SetLateralCg,-.5f));
  float cg_shift=0;
  assert(w.takeCommand(&cg_shift)==RunControlWorker::Command::SetLateralCg && cg_shift==-.5f);
  assert(!w.request(RunControlWorker::Command::CaptureEquilibrium));
  w.completeCommand(true,"lateral_cg_saved");
  w.oneStep();
  assert(w.request(RunControlWorker::Command::CaptureUprightReference));
  assert(w.takeCommand()==RunControlWorker::Command::CaptureUprightReference);
  assert(w.commandState().pending);
  assert(!w.request(RunControlWorker::Command::Start));
  assert(!w.request(RunControlWorker::Command::Clear));
  assert(!w.request(RunControlWorker::Command::CaptureUprightReference));
  w.completeCommand(true,"upright_reference_saved");
  w.oneStep(); assert(w.commandPublished(w.commandState().completed));
  assert(w.request(RunControlWorker::Command::Start,10,0.0f,500,20,false,false));
  bool applied_adaptation=true;
  assert(!w.request(RunControlWorker::Command::Start,12)); // cannot replace accepted target
  assert(!w.request(RunControlWorker::Command::Clear)); // pending start blocks clear
  w.oneStep();assert(w.active()&&w.commandState().ok);
  assert(w.takeCommand(nullptr,nullptr,nullptr,nullptr,&applied_adaptation)==RunControlWorker::Command::None);
  assert(applied_adaptation); // No command may overwrite the caller's setting.
  assert(m.target==10&&w.snapshot().target_deg==10);
  assert(m.angle==0.0f&&w.snapshot().input_peak_percent==0.0f);
  assert(m.current==500&&w.snapshot().input_current_mA==500);
  assert(m.brake==20&&w.snapshot().brake_max_ms==20);
  assert(!m.adaptation && !w.snapshot().peak_adaptation_enabled);
  assert(w.commandPublished(w.commandState().completed));
  assert(!w.request(RunControlWorker::Command::Start));
  assert(!w.request(RunControlWorker::Command::Clear));
  for(int i=0;i<100;++i)w.oneStep();
  assert(w.requestStop());w.oneStep();assert(!w.active());
  const auto audit=w.healthSnapshot();
  const auto profile_done=control_work::profile.stages[0][static_cast<uint8_t>(control_work::Stage::Owner)];
  assert(profile_done.count>0&&profile_done.max_us==850); // includes 100-us snapshot after 750-us work
  for(int i=0;i<100;++i)w.oneStep();
  assert(w.healthSnapshot().steps==audit.steps); // idle cannot alter run deadlines
  assert(control_work::profile.stages[0][static_cast<uint8_t>(control_work::Stage::Owner)].count==profile_done.count);
  assert(w.request(RunControlWorker::Command::Start,12));
  assert(w.requestStop());w.oneStep();assert(!w.active()&&!w.commandState().pending);
  assert(m.target==10 && m.angle==0.0f); // cancelled START never changes the applied target
  assert(!w.commandState().ok); // STOP cancels queued START
  assert(w.request(RunControlWorker::Command::Clear));w.oneStep();assert(!w.active());
  w.beginRunAudit();w.last_step_start_us_=UINT32_MAX-200;host_us=300;
  w.recordStep(300,1,2,3);assert(w.healthSnapshot().max_period_us==501);
  w.recordSampleCompletion(false,true,0,9000,8000,true,true);
  w.recordSampleCompletion(true,false,0,9000,8000,true,true);
  assert(w.audit_.sample_completion.count==0); // same RUNNING/fresh gate
  const uint32_t stamp=UINT32_MAX-100;
  for(unsigned pulse=0;pulse<2;++pulse)for(unsigned accel=0;accel<2;++accel) {
    const uint32_t elapsed=2499+pulse*2+accel;
    w.recordSampleCompletion(true,true,stamp,stamp+elapsed,elapsed-10,pulse,accel);
    const auto& c=w.audit_.input_cohorts[pulse][accel];
    assert(c.count==1 && c.sum==elapsed && c.maximum==elapsed);
    assert(c.over==(elapsed>2500)); // equality is in budget, no tolerance
  }
  assert(w.audit_.sample_completion.count==4 && w.audit_.sample_completion.over==2);
  assert(w.audit_.sample_completion.sum==10002 && w.audit_.sample_completion.maximum==2502);
  assert(w.audit_.runner_work.sum==9962);
  const std::string json=w.diagnosticsJson().c_str();
  assert(json.find("\"runner_mean_us\":2490.500")!=std::string::npos);
  assert(json.find("\"deadline_input_cohorts\"")!=std::string::npos);
  w.beginRunAudit();
  assert(w.audit_.sample_completion.count==0);
  for(const auto& group:w.audit_.input_cohorts)for(const auto& c:group)assert(c.count==0&&c.sum==0);
  for(float target : Config::ENERGY_CONTROL_AUTONOMOUS_TARGET_CHOICES_DEG) {
    assert(w.request(RunControlWorker::Command::Start,target));
    w.oneStep();assert(m.target==target&&w.snapshot().target_deg==target);
    assert(!w.request(RunControlWorker::Command::Start,8)); // running target is fixed
    assert(w.requestStop());w.oneStep();
    assert(w.request(RunControlWorker::Command::Clear));w.oneStep();
  }
  for(uint16_t width : {0,15,25,30,40,50}) {
    assert(w.request(RunControlWorker::Command::Start,8,0.0f,300,width,false,true,true));
    w.oneStep();
    assert(m.probe&&w.snapshot().negative_probe_mode&&m.brake==width);
    assert(m.target==8&&m.current==300&&!m.adaptation);
    assert(w.requestStop());w.oneStep();
    assert(w.request(RunControlWorker::Command::Clear));w.oneStep();
  }
  for(float target : {8.f,10.f,12.f}) {
    assert(!w.request(RunControlWorker::Command::Start,target,0.f,300,40,true,true,false));
    assert(!w.request(RunControlWorker::Command::Start,target,0.f,500,40,false,true,false));
    if(target!=8.f) assert(!w.request(RunControlWorker::Command::Start,target,0.f,300,25,false,true,false));
    assert(w.request(RunControlWorker::Command::Start,target,0.f,300,40,false,true,false));
    w.oneStep();
    assert(!m.probe && m.brake==40 && m.target==target && m.current==300 && !m.adaptation);
    assert(w.snapshot().target_deg==target && w.commandState().ok);
    assert(w.commandPublished(w.commandState().completed));
    assert(w.requestStop());w.oneStep();
    assert(w.request(RunControlWorker::Command::Clear));w.oneStep();
  }
  assert(w.request(RunControlWorker::Command::Start));w.oneStep();
  assert(m.target==Config::ENERGY_CONTROL_AUTONOMOUS_DEFAULT_TARGET_PEAK_DEG);
  std::cout<<"permanent owner, command exclusion, STOP precedence, sealed audit and wrap PASS\n";
  std::cout<<"Completion input cohorts partition totals, preserve strict budget/wrap/gates and reset PASS\n";
  std::cout<<"8/10/12 target parsing, atomic command payload, rejected replacement, cancellation and readback PASS\n";
}
