#include "Arduino.h"
#include <cassert>
#include <functional>
#include <iostream>

using TaskHandle_t = void*;
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) (void)(mux)
#define portEXIT_CRITICAL(mux) (void)(mux)
constexpr int pdPASS=1, pdTRUE=1, portMAX_DELAY=1000, tskIDLE_PRIORITY=0;
struct NoNotification {};
static bool notified=false, fail_task=false;
static void (*task_entry)(void*)=nullptr;
static void* task_arg=nullptr;
static std::function<void()> during_solve;
int xTaskCreatePinnedToCore(void (*entry)(void*),const char*,int,void* arg,int priority,void** handle,int core) {
  assert(core==0 && priority==tskIDLE_PRIORITY);
  if (fail_task) return 0;
  task_entry=entry; task_arg=arg; *handle=arg; return pdPASS;
}
void xTaskNotifyGive(void*) { notified=true; }
void ulTaskNotifyTake(int,int) {
  if (!notified) throw NoNotification{};
  notified=false;
}
void dispatch() {
  try { task_entry(task_arg); }
  catch(const NoNotification&) {}
}
uint32_t testMicros() {
  if (during_solve) { auto once=std::move(during_solve); during_solve=nullptr; once(); }
  return host_us;
}
#define micros testMicros
uint64_t esp_timer_get_time() { return host_us; }
#include <support_transfer_audit.cpp>
#include "periodic_foot_state.h"
#include "half_cycle_energy_budget.h"
#include "wheel_cycle_guard.h"
#include "walking_current_direction.h"
#include "control_math_cache.h"
namespace Config {
constexpr int MOTOR_DRIVER_POLARITY=-1, ENERGY_CONTROL_AUTONOMOUS_NORMAL_OUTPUT_SIGN=-1;
constexpr float MODEL_VBAT_REFERENCE_V=7.5f, MODEL_VBAT_MIN_V=6.18f, MODEL_VBAT_MAX_V=8.02f;
}
enum class EnergyControlAutonomousHalfCycleState { WAIT_ZERO_CROSS };
enum class EnergyControlAutonomousPhase { ENERGY_CONTROL, HOLD };
// No motor command, actuator or I/O API is available to the real snapshot method.
struct Roller {
  bool current_valid=true, speed_valid=true;
  uint32_t current_sample_time_us=0, speed_sample_time_us=0;
  int actual_current_mA=7, battery_mV=7400;
  float speed_rpm=0;
  const Roller& telemetrySnapshot() const {return *this;}
};
struct ExperimentRunner {
  periodic_foot::Packet periodic_foot_packet_;
  uint32_t transfer_packet_=0, transfer_previous_sample_us_=0;
  float transfer_previous_rate_dps_=NAN;
  int energy_control_autonomous_last_peak_side_=-1, energy_control_autonomous_target_peak_deg_=8,
      energy_control_autonomous_input_peak_percent_=0;
  unsigned energy_control_autonomous_last_peak_ms_=1;
  bool energy_control_autonomous_last_peak_valid_=true, energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
  EnergyControlAutonomousPhase energy_control_autonomous_phase_=EnergyControlAutonomousPhase::ENERGY_CONTROL;
  struct { bool emergency_stop=false, pulse_active=false; } status_;
  struct { float angle_deg=0; } walking_fore_aft_last_;
  struct { uint16_t timingCap(int,unsigned,uint16_t cap) const {return cap;} } tilt_priority_;
  bool allow_fore_aft=true;
  bool walkingForeAftAllowsOutput() const {return allow_fore_aft;}
  bool wheelWorkMode() const {return true;}
  float predictCurrentGoalMa(int,float) const {return 320;}
  half_cycle_energy_budget::Ledger walking_half_cycle_budget_;
  wheel_cycle_guard::Controller wheel_cycle_guard_;
  Roller roller; Roller* roller_=&roller;
  SupportTransferAudit transfer_audit_;
  void observeSupportTransfer(uint32_t,uint32_t,float,float);
  void tick(unsigned seq,float rate) {
    host_us=100000+seq*25000;
    auto& p=periodic_foot_packet_; p.count=seq; p.published_us=host_us;
    for (auto* foot : {&p.left,&p.right}) {
      foot->estimated=1; foot->rate_dps=0;foot->image_us=host_us-10000;
    }
    if (post_zero) { p.left.estimated=0; p.right.estimated=.5f; }
    if (!allow_fore_aft) p.left.image_us=host_us-80000;
    roller.current_sample_time_us=roller.speed_sample_time_us=host_us;
    observeSupportTransfer(host_us/1000,host_us,post_zero ? -energy_control_autonomous_last_peak_side_*0.3f
                                                         : energy_control_autonomous_last_peak_side_*2.f,rate);
  }
  bool post_zero = false;
};
#include "runner_snapshot.inc"

void print(const SupportTransferAudit& audit, size_t padding=0) {
  PsramString json; assert(json.reserve(foot_fast_policy::kMetadataHardBudgetBytes));
  if (padding) { std::string data(padding,' ');json.append(data.c_str(),data.size()); }
  audit.append(json); assert(json.ok());
  std::cout<<"{"<<(json.c_str()+padding+1)<<"}\n";
}
int main() {
  ExperimentRunner r;
  auto& audit=r.transfer_audit_;
  audit.begin(.05f);audit.startRun();
  r.walking_half_cycle_budget_.sourcePeak(1);
  r.energy_control_autonomous_last_peak_valid_=false; r.tick(1,10);
  r.energy_control_autonomous_last_peak_valid_=true; r.tick(2,20);
  r.observeSupportTransfer(host_us/1000,host_us,-2,20); // duplicate packet must not enqueue
  host_us+=7000;dispatch();
  r.energy_control_autonomous_last_peak_side_=1;
  r.post_zero=true; r.energy_control_autonomous_zero_cross_consumed_for_peak_=true;
  r.roller.battery_mV=8350;
  assert(r.walking_half_cycle_budget_.accepted(half_cycle_energy_budget::Kind::Main,0));
  r.tick(3,-19);r.tick(4,-20);dispatch();
  r.status_.pulse_active=true; r.allow_fore_aft=false;r.roller.current_valid=false;r.tick(5,-20);dispatch();
  r.tick(6,-19);audit.seal();dispatch();print(audit);

  SupportTransferAudit::Sample sample;sample.packet=10;sample.gate_mask=1;
  audit.startRun();audit.submit(sample);
  during_solve=[&] {audit.seal();}; dispatch();print(audit);

  audit.startRun();audit.submit(sample);
  during_solve=[&] {
    audit.seal();audit.startRun();sample.packet=12;audit.submit(sample);
  };
  dispatch();audit.seal();print(audit);

  SupportTransferAudit failed;fail_task=true;failed.begin(.05f);failed.startRun();
  failed.submit(sample);failed.seal();print(failed);
  audit.startRun();for(unsigned i=1;i<=1210;++i){sample.packet=i;audit.submit(sample);dispatch();}
  audit.seal();print(audit,foot_fast_policy::kMetadataHardBudgetBytes-16000);
  audit.startRun();
  sample.gate_mask=SupportTransferAudit::NoBudget;
  auto& in=sample.input;in.side=1;in.angle_deg=-2;in.rate_dps=40;in.accel_dps2=0;
  in.fore_aft_deg=in.left_deg=in.right_deg=in.left_rate_dps=in.right_rate_dps=0;
  in.cap_ms=0;in.initial_current_ma=0;in.goal_current_ma=320;
  sample.captured_us=UINT32_MAX-3;host_us=3;
  audit.submit(sample);dispatch();
  in.accel_dps2=-400;sample.captured_us=4;host_us=6;
  audit.submit(sample);dispatch();audit.seal();print(audit);
}
