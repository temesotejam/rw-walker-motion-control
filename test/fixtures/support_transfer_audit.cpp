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
#include "walking/wheel_input.h"
#include "models/control_math_cache.h"
namespace Config {
constexpr int MOTOR_DRIVER_POLARITY=-1, ENERGY_CONTROL_AUTONOMOUS_NORMAL_OUTPUT_SIGN=-1;
constexpr float MODEL_VBAT_REFERENCE_V=7.5f, MODEL_VBAT_MIN_V=6.18f, MODEL_VBAT_MAX_V=8.02f;
}
enum class EnergyControlAutonomousHalfCycleState { WAIT_ZERO_CROSS };
enum class EnergyControlAutonomousPhase { ENERGY_CONTROL, HOLD };
// No motor command, actuator or I/O API is available to the real snapshot method.
struct Roller {
  bool current_valid=true, speed_valid=true, roller_ok=true;
  int applied_current_mA=0, requested_current_mA=0;
  uint32_t command_sequence=0, applied_command_sequence=1, applied_command_time_us=UINT32_MAX-9999;
  uint32_t current_sample_time_us=0, speed_sample_time_us=0;
  int actual_current_mA=7, battery_mV=7400;
  float speed_rpm=0;
  const Roller& telemetrySnapshot() const {return *this;}
};
struct ExperimentRunner {
  periodic_foot::Packet periodic_foot_packet_;
  uint32_t transfer_packet_=0, transfer_previous_sample_us_=0;
  float transfer_previous_rate_dps_=NAN;
  float transfer_acceleration_dps2_=NAN;
  int energy_control_autonomous_last_peak_side_=-1, energy_control_autonomous_target_peak_deg_=8,
      energy_control_autonomous_input_peak_percent_=0;
  unsigned energy_control_autonomous_last_peak_ms_=1;
  bool energy_control_autonomous_last_peak_valid_=true, energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
  EnergyControlAutonomousPhase energy_control_autonomous_phase_=EnergyControlAutonomousPhase::ENERGY_CONTROL;
  struct { bool emergency_stop=false, pulse_active=false; uint32_t pulse_id=0; } status_;
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

bool finishMetadata(PsramString& json) {
  constexpr size_t kMetadataJsonHardBudgetBytes=foot_fast_policy::kMetadataHardBudgetBytes;
#include "metadata_footer.inc"
}
struct ExportProbe {
  size_t previous = 0, checkpoints = 0, schema_bytes = 0;
  static void progress(void* context, size_t length) {
    auto& p = *static_cast<ExportProbe*>(context);
    if (length == p.previous) ++p.checkpoints;
    p.previous = length;
  }
};
ExportProbe print(const SupportTransferAudit& audit, size_t prefix_bytes=0, bool expect_overflow=false) {
  static_assert(foot_fast_policy::kMetadataReserveBytes == foot_fast_policy::kMetadataHardBudgetBytes,
                "v39 reserves only the publishable metadata size");
  PsramString json; assert(json.reserve(foot_fast_policy::kMetadataReserveBytes));
  json += "{\"sensor_data\":\"";
  if (prefix_bytes) {
    assert(prefix_bytes > json.length());
    const std::string data(prefix_bytes - json.length() - 1, 'x');
    json.append(data.c_str(), data.size());
  }
  json += "\"";
  const size_t base = json.length();
  ExportProbe probe; probe.previous = base;
  json.setProgress(ExportProbe::progress, &probe);
  audit.append(json);
  json.setProgress(nullptr);
  const char* rows = std::strstr(json.c_str() + base, "\"rows\":[");
  if (rows) probe.schema_bytes = rows - (json.c_str() + base) + std::strlen("\"rows\":[");
  const bool complete = finishMetadata(json);
  if (expect_overflow) {
    assert(!complete && !json.ok());
    return probe;
  }
  assert(complete && json.c_str()[json.length()] == '\0');
  assert(json.length() <= foot_fast_policy::kMetadataHardBudgetBytes);
  std::cout << json.c_str() << "\n";
  return probe;
}
int main() {
  ExperimentRunner r;
  auto& audit=r.transfer_audit_;
  audit.begin(.05f);audit.startRun();
  r.walking_half_cycle_budget_.sourcePeak(1);
  r.energy_control_autonomous_last_peak_valid_=false; r.tick(1,10);
  assert(std::isnan(r.transfer_acceleration_dps2_));
  r.status_.pulse_id=17;
  r.energy_control_autonomous_last_peak_valid_=true; r.tick(2,20);
  assert(std::fabs(r.transfer_acceleration_dps2_-400.f)<.01f);
  r.observeSupportTransfer(host_us/1000,host_us,-2,22); // duplicate packet must not overwrite the derivative
  assert(std::fabs(r.transfer_acceleration_dps2_-400.f)<.01f);
  host_us+=7000;dispatch();
  r.energy_control_autonomous_last_peak_side_=1;
  r.post_zero=true; r.energy_control_autonomous_zero_cross_consumed_for_peak_=true;
  r.roller.battery_mV=8350;
  r.roller.applied_command_time_us=210000; // The frame-4 current observation predates this acknowledgement.
  ++r.status_.pulse_id;
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
  audit.startRun();
  for(unsigned i=1;i<=1210;++i) {
    sample.packet=i; sample.time_ms=i*25; sample.captured_us=UINT32_MAX-1210+i;
    sample.input.side=i%2 ? -1 : 1;
    sample.input.angle_deg=std::sin(i*.02f)*8;
    sample.input.accel_dps2=std::cos(i*.02f)*123.4567f;
    sample.input.left_deg=std::sin(i*.05f)*14;
    sample.input.right_deg=-sample.input.left_deg;
    sample.input.initial_current_ma=-17.1337f;sample.input.goal_current_ma=320;
    audit.submit(sample);dispatch();
  }
  audit.seal();
  constexpr size_t limit=foot_fast_policy::kMetadataHardBudgetBytes;
  const auto limited = print(audit,limit-16000);
  assert(limited.checkpoints>1 && limited.checkpoints<=2+16000/kMinRowBytes);
  const size_t sealed_bytes=limited.schema_bytes+kFooterBytes+kClosingBytes;
  assert(sealed_bytes>512);
  // Enough room for the schema but no row, one rejected row, or only an omission count.
  assert(print(audit,limit-sealed_bytes-(kMinRowBytes-1)).checkpoints==1);
  const auto rejected=print(audit,limit-sealed_bytes-kMinRowBytes);
  assert(rejected.checkpoints>=2 && rejected.checkpoints<=3);
  assert(print(audit,limit-250).checkpoints==1);
  assert(print(audit,limit-kFooterBytes).checkpoints==1);
  print(audit);
  audit.startRun();
  sample.gate_mask=SupportTransferAudit::NoBudget;
  auto& in=sample.input;in.side=1;in.angle_deg=-2;in.rate_dps=40;in.accel_dps2=0;
  in.fore_aft_deg=in.left_deg=in.right_deg=in.left_rate_dps=in.right_rate_dps=0;
  in.cap_ms=0;in.initial_current_ma=0;in.goal_current_ma=320;
  sample.captured_us=UINT32_MAX-3;host_us=3;
  audit.submit(sample);dispatch();
  in.accel_dps2=-400;sample.captured_us=4;host_us=6;
  audit.submit(sample);dispatch();audit.seal();print(audit);

  // First seen with the left foot already lower. A pulse can mask the
  // decision, but valid geometry can still anchor the receiving half-cycle.
  const auto receiving = [&](unsigned packet, float roll, float left, unsigned mask) {
    sample.peak_ms=400; sample.packet=packet; sample.gate_mask=mask;
    sample.time_ms=400+25*packet; sample.captured_us=host_us=1000*sample.time_ms;
    in.side=-1; in.angle_deg=roll; in.rate_dps=-35; in.accel_dps2=200;
    in.left_deg=left; in.right_deg=0; in.fore_aft_deg=0;
    in.left_rate_dps=in.right_rate_dps=0;
    in.cap_ms=60;
    audit.submit(sample); dispatch();
  };
  audit.startRun();
  receiving(1,5,15,SupportTransferAudit::StalePacket);
  receiving(2,3,8,SupportTransferAudit::PulseActive);
  receiving(3,2,17,0);
  receiving(4,-3,18,0);
  audit.seal();print(audit);
  // Reuse the same peak, side and packet numbers after restarting the run.
  audit.startRun();
  receiving(1,2,17,0);
  audit.seal();print(audit);
  audit.startRun();
  // Short accepted pulses may begin and finish between two camera packets.
  auto frame=[&](unsigned packet,unsigned gate,unsigned pulse,int side=1,unsigned peak=500,float speed=35) {
    sample.peak_ms=peak; sample.packet=packet; sample.gate_mask=gate; sample.pulse_id=pulse;
    sample.time_ms=600+25*packet; sample.captured_us=host_us=1000*sample.time_ms;
    in.side=side; in.angle_deg=.4f*side; in.rate_dps=speed*side; in.accel_dps2=0;
    in.left_deg=in.right_deg=in.fore_aft_deg=in.left_rate_dps=in.right_rate_dps=0;
    in.cap_ms=gate==SupportTransferAudit::NoBudget ? 0 : 60;
    audit.submit(sample);dispatch();
  };
  sample.current_after_zero_ms=100;
  frame(1,0,0); frame(2,0,0); frame(3,0,0); // Verified initial zero, no walking pulse yet.
  frame(4,0,21); frame(5,0,21); frame(7,0,21);
  frame(8,SupportTransferAudit::NoBudget,21); frame(9,0,21);
  frame(10,SupportTransferAudit::PulseActive,22); frame(11,0,22);
  frame(12,0,22,1,500,3); frame(13,0,22);
  frame(14,0,22,1,550); frame(15,0,22,-1,550); frame(16,0,22,-1,550);
  audit.seal();print(audit);
  audit.startRun();
  frame(17,0,22,-1,550);
  sample.current_after_zero_ms=69;frame(18,0,22,-1,550);
  sample.current_after_zero_ms=70;in.goal_current_ma=280;in.initial_current_ma=100;
  frame(19,0,22,-1,550);frame(20,0,22,-1,550);
  in.initial_current_ma=-101;frame(21,0,22,-1,550);
  in.initial_current_ma=101;frame(22,0,22,-1,550);
  sample.current_after_zero_ms=UINT16_MAX;in.initial_current_ma=0;frame(23,0,22,-1,550);
  sample.current_after_zero_ms=100;frame(24,0,22,-1,550);frame(25,0,22,-1,550);
  audit.seal();print(audit);
  audit.startRun();
  r.status_.pulse_active=false;r.allow_fore_aft=true;r.roller.current_valid=true;
  r.status_.pulse_id=0;r.roller.applied_command_time_us=1;r.roller.applied_command_sequence=0;
  r.tick(30,-20);dispatch(); // No known zero acknowledgement.
  r.roller.applied_command_sequence=5;r.roller.command_sequence=6;
  r.tick(31,-20);dispatch(); // Last write failed.
  r.roller.command_sequence=5;r.roller.requested_current_mA=300;
  r.tick(32,-20);dispatch(); // Pending nonzero request.
  r.roller.requested_current_mA=0;r.roller.applied_current_mA=300;
  r.tick(33,-20);dispatch(); // Zero has been requested but not applied.
  r.roller.applied_current_mA=0;
  r.tick(34,-20);dispatch(); // Verified initial zero, no motor pulse.
  audit.seal();print(audit);
  const size_t final_footer = std::strlen(",\"metadata_json_final_bytes\":}") + std::to_string(limit).size();
  print(audit, limit - final_footer);
  print(audit, limit - final_footer + 1, true);
}
