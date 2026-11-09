// Exercise actual dispatch/driver setup and failure cleanup using RTOS doubles.
// This is not proof of ESP32 timing, interrupt priority or electrical capture.
#include <cassert>
#include <iostream>
#include "imu_poll_timer.h"
#include "camera_lifecycle.h"
static bool timerCallback(void*) { return true; }
static void execute(TaskFunction_t fn,void* arg,uint32_t stack,uint32_t priority,uint32_t core) {
  assert(core==0 && stack==8192 && priority==2);
  const int saved=host_core;host_core=core;fn(arg);host_core=saved;
}
static bool work(void* arg) {assert(host_core==0);++*static_cast<int*>(arg);return true;}
int main() {
  for(const char* failure:{"init","counter","alarm","add","start"}) {
    timer_stub::reset();timer_stub::fail=failure;host_core=1;
    ImuPollTimer timer;assert(!timer.begin(timerCallback,nullptr,1000));
    assert(timer.lastError()==ESP_FAIL && !timer_stub::started && !timer_stub::registered && !timer_stub::initialized);
    if(std::string(failure)=="start") {
      assert((timer_stub::calls==std::vector<std::string>{"init","counter","alarm","add","start","pause","remove","deinit"}));
    }
  }
  timer_stub::reset();host_core=0;ImuPollTimer wrong;
  assert(!wrong.begin(timerCallback,nullptr,1000)&&timer_stub::calls.empty());
  host_core=1;assert(!wrong.begin(timerCallback,nullptr,2500)&&timer_stub::calls.empty());
  ImuPollTimer good;assert(good.begin(timerCallback,nullptr,1000));
  assert(timer_stub::started && timer_stub::registered && good.ownerCore()==1);
  assert(timer_stub::config.divider==80 && timer_stub::config.auto_reload==TIMER_AUTORELOAD_EN);
  assert(timer_stub::config.counter_en==TIMER_PAUSE && timer_stub::alarm==1000 && timer_stub::counter==0);
  assert(timer_stub::flags==(ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_LEVEL1));
  auto calls=timer_stub::calls;assert(!good.begin(timerCallback,nullptr,1000)&&timer_stub::calls==calls);
  int count=0;host_task_hook=execute;host_core=1;
  auto result=camera_lifecycle::run(work,&count);
  assert(result.invoked&&result.ok&&count==1&&host_core==1&&host_tasks_deleted==1);
  host_core=0;auto tasks=host_tasks_created;result=camera_lifecycle::run(work,&count);
  assert(result.ok&&count==2&&host_tasks_created==tasks);
  host_core=1;host_task_result=pdFALSE;
  result=camera_lifecycle::run(work,&count);assert(!result.invoked&&!result.ok&&count==2);
  host_task_result=pdPASS;host_semaphore_fail=true;tasks=host_tasks_created;
  result=camera_lifecycle::run(work,&count);assert(!result.invoked&&host_tasks_created==tasks);
  host_semaphore_fail=false;
  result=camera_lifecycle::run([](void*){return false;},nullptr);assert(result.invoked&&!result.ok);
  std::cout<<"Core ownership, timer startup failures/IRQ cleanup, camera synchronous handoff and dispatch failures PASS\n";
}
