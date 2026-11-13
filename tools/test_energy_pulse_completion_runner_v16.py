"""Execute the actual v16 finalizer and audit struct with minimal host state.

This checks once-only finalization, analytic software-duration accounting and
pending-state identity. It does not measure Roller command application/current.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
header = (ROOT / "src/experiment_runner.h").read_text(encoding="utf8")
runner = (ROOT / "src/walking_energy_runner.inc").read_text(encoding="utf8")
start = header.index("  struct WalkingEnergyAudit {")
end = header.index("\n  };", start) + len("\n  };")
audit_struct = header[start:end]
start = runner.index("void ExperimentRunner::finishWalkingEnergyPulse(")
end = runner.index("\n}\n", start) + 3
finalizer = runner[start:end]

CPP = r'''
#include "walking_energy_policy.h"
#include "energy_pulse_completion.h"
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <iostream>
#define CHECK(x) do {if(!(x)){std::cerr<<"check failed: "<<#x<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
class ExperimentRunner {public:
AUDIT_STRUCT
 WalkingEnergyAudit walking_energy_audit_[192],walking_energy_active_pulse_;
 uint32_t walking_energy_count_=0,run_start_ms_=0;
 int16_t walking_energy_active_audit_=-1;
 bool walking_energy_completion_pending_=false;
 float predicted_signed_current_end_mA_=1234.f;
 uint32_t predicted_current_end_ms_=111;
 bool energy_control_autonomous_pending_peak_=false;
 uint32_t energy_control_autonomous_pending_zero_ms_=0;
 int8_t energy_control_autonomous_pending_next_side_=0;
 float energy_control_autonomous_pending_q_command_mA_s_=999.f;
 float energy_control_autonomous_pending_model_energy_j_=888.f;
 void finishWalkingEnergyPulse(uint32_t,uint8_t);
};
FINALIZER
static bool near(float a,float b,float rel=2e-5f,float abs=2e-8f){
 return std::isfinite(a)&&std::isfinite(b)&&std::fabs(a-b)<=abs+rel*std::fabs(b);
}
static void arm(ExperimentRunner& r,uint32_t start,int side=1,float initial=0.f,
                uint16_t planned=80,int index=0){
 r.run_start_ms_=start-1000U;
 auto& a=r.walking_energy_active_pulse_;
 a.time_ms=995;a.peak_ms=800;a.trigger_ms=993;a.side=side;
 a.output_start_ms=1000;a.guarded_ms=planned;a.requested_ms=planned;a.executed=true;
 a.start_i0_ma=initial;a.start_goal_ma=300;a.start_tau_s=.0316f;a.start_rate_dps=40;
 a.decision.available_j=.0003f;a.decision.request_j=.0002f;
 a.decision.predicted_work_j=.0002f;a.decision.predicted_arrival_j=.0005f;a.decision.q_mas=15;
 r.walking_energy_active_audit_=index;r.walking_energy_completion_pending_=true;
 r.walking_energy_count_=index>=0 && index<192?static_cast<uint32_t>(index+1):192;
 if(index>=0 && index<192)r.walking_energy_audit_[index]=a;
 r.energy_control_autonomous_pending_peak_=true;
 r.energy_control_autonomous_pending_zero_ms_=a.trigger_ms;
 r.energy_control_autonomous_pending_next_side_=side;
}
// Independently integrate the specified first-order current model; do not
// obtain the test's expected values by calling the production helper again.
static float q_expected(float initial,uint32_t elapsed){
 const double t=elapsed*.001, tau=.0316;
 return 300*t+(initial-300)*tau*(-std::expm1(-t/tau));
}
static float end_expected(float initial,uint32_t elapsed){
 return 300+(initial-300)*std::exp(-(elapsed*.001)/.0316);
}
static float work_expected(int side,float q){
 const double c1=side>0?.01259279858:.03889674250;
 const double c2=side>0?.5522071518:.2578004825;
 const double amp_seconds=q*.001,omega=40*3.14159265358979323846/180;
 return c1*omega*amp_seconds+c2*amp_seconds*amp_seconds;
}
int main(){
 unsigned cases=0;
 for(int side:{-1,1})for(float initial:{-150.f,0.f,120.f})
 for(uint32_t elapsed:{0U,1U,7U,15U,80U,123U,1000U}){
  ExperimentRunner r;const uint32_t start=100000;arm(r,start,side,initial);
  const auto planned=r.walking_energy_active_pulse_.decision;
  const uint8_t reason=elapsed<80?3:1;
  r.finishWalkingEnergyPulse(start+elapsed,reason);
  const auto& a=r.walking_energy_audit_[0];const auto& d=a.completion;
  CHECK(!r.walking_energy_completion_pending_ && r.walking_energy_active_audit_==-1);
  CHECK(a.completion_reason==reason && a.output_start_ms==1000 && a.output_end_ms==1000+elapsed);
  CHECK(a.trigger_ms==993 && a.guarded_ms==80 && a.executed);
  CHECK(d.valid && d.actual_elapsed_ms==elapsed && d.planned_ms==80);
  CHECK(d.duration_completed==(elapsed>=80));
  CHECK(d.width_supported==(elapsed<=100));
  CHECK(near(d.q_mas,q_expected(initial,elapsed)));
  CHECK(near(d.current_end_ma,end_expected(initial,elapsed)));
  CHECK(near(r.predicted_signed_current_end_mA_,side*end_expected(initial,elapsed)));
  CHECK(r.predicted_current_end_ms_==start+elapsed);
  CHECK(near(r.energy_control_autonomous_pending_q_command_mA_s_,d.q_mas));
  // Preserve the decision prediction; actual completion gets separate fields.
  CHECK(a.decision.q_mas==planned.q_mas && a.decision.predicted_work_j==planned.predicted_work_j);
  if(q_expected(initial,elapsed)<0){
   CHECK(!d.work_valid && d.extrapolated && std::isnan(d.work_j) && std::isnan(d.arrival_j));
   CHECK(std::isnan(r.energy_control_autonomous_pending_model_energy_j_));
  }else{
   CHECK(d.work_valid && near(d.work_j,work_expected(side,d.q_mas),2e-5f,2e-10f));
   CHECK(near(d.arrival_j,.0003f+d.work_j,2e-5f,2e-10f));
   CHECK(near(d.unmet_j,std::fmax(0.f,.0002f-d.work_j),2e-5f,2e-10f));
   CHECK(near(r.energy_control_autonomous_pending_model_energy_j_,d.arrival_j,2e-5f,2e-10f));
  }
  if(!elapsed){CHECK(d.q_mas==0 && d.work_j==0 && d.current_end_ma==initial && near(d.unmet_j,.0002f));}
  if(elapsed>100){CHECK(!d.width_supported && d.extrapolated && d.q_mas>q_expected(initial,80));}
  const auto saved=a;const float end_i=r.predicted_signed_current_end_mA_;
  r.finishWalkingEnergyPulse(start+elapsed+200,4);
  CHECK(r.walking_energy_audit_[0].output_end_ms==saved.output_end_ms);
  CHECK(r.walking_energy_audit_[0].completion_reason==saved.completion_reason);
  CHECK(r.predicted_current_end_ms_==start+elapsed && r.predicted_signed_current_end_mA_==end_i);
  ++cases;
 }
 // Modulo-millis arithmetic must preserve a genuinely short duration across
 // wrap. It must not become a huge elapsed time or be clamped to planned ms.
 {ExperimentRunner r;const uint32_t start=UINT32_MAX-10U;arm(r,start,-1);
  r.finishWalkingEnergyPulse(start+20U,3);const auto& a=r.walking_energy_audit_[0];
  CHECK(a.output_end_ms==1020 && a.completion.actual_elapsed_ms==20);
  CHECK(r.predicted_current_end_ms_==start+20U && a.completion_reason==3);}
 // Stop reasons describe the caller's lifecycle event, not an inference from
 // how close elapsed time happened to be to the planned duration.
 for(uint8_t reason:{1,2,3,4}){ExperimentRunner r;arm(r,100000);
  r.finishWalkingEnergyPulse(100017,reason);const auto& a=r.walking_energy_audit_[0];
  CHECK(a.completion_reason==reason && a.completion.actual_elapsed_ms==17);
  CHECK(a.completion.valid && !a.completion.duration_completed);}
 // No storage slot (or a stale storage index) must never disable control
 // accounting or write outside the bounded log. The active copy survives.
 for(int index:{-2,-1,192,250}){
  ExperimentRunner r;arm(r,100000,1,0,80,index);r.walking_energy_audit_[191].output_end_ms=777;
  r.finishWalkingEnergyPulse(100080,1);
  CHECK(r.walking_energy_active_pulse_.completion.valid);
  CHECK(r.walking_energy_active_pulse_.completion.actual_elapsed_ms==80);
  CHECK(r.walking_energy_audit_[191].output_end_ms==777);
  CHECK(r.predicted_current_end_ms_==100080 && near(r.energy_control_autonomous_pending_q_command_mA_s_,q_expected(0,80)));
 }
 // Finalize only its corresponding pending half-cycle. A same-timestamp
 // opposite side is still a different identity, not permission to overwrite.
 for(unsigned mismatch=0;mismatch<3;++mismatch){
  ExperimentRunner r;arm(r,100000);
  if(mismatch==0)r.energy_control_autonomous_pending_peak_=false;
  if(mismatch==1)++r.energy_control_autonomous_pending_zero_ms_;
  if(mismatch==2)r.energy_control_autonomous_pending_next_side_=-1;
  r.finishWalkingEnergyPulse(100080,1);
  CHECK(r.walking_energy_active_pulse_.completion.valid && r.predicted_current_end_ms_==100080);
  CHECK(r.energy_control_autonomous_pending_q_command_mA_s_==999 && r.energy_control_autonomous_pending_model_energy_j_==888);
 }
 // A corrupted arithmetic input cannot preserve a stale planned pending
 // arrival or fabricate finite delivered current. It remains explicit NAN.
 {ExperimentRunner r;arm(r,100000);r.walking_energy_active_pulse_.start_tau_s=0;
  r.finishWalkingEnergyPulse(100020,2);
  CHECK(!r.walking_energy_active_pulse_.completion.valid);
  CHECK(r.predicted_current_end_ms_==111 && r.predicted_signed_current_end_mA_==1234);
  CHECK(std::isnan(r.energy_control_autonomous_pending_q_command_mA_s_));
  CHECK(std::isnan(r.energy_control_autonomous_pending_model_energy_j_));}
 // Unsent candidates and already-completed records cannot be completed again.
 for(bool already_done:{false,true}){ExperimentRunner r;arm(r,100000);
  if(already_done)r.walking_energy_active_pulse_.completion_reason=2;
  else r.walking_energy_active_pulse_.executed=false;
  r.finishWalkingEnergyPulse(100080,1);
  CHECK(!r.walking_energy_completion_pending_ && r.walking_energy_active_audit_==-1);
  CHECK(r.predicted_current_end_ms_==111 && r.energy_control_autonomous_pending_q_command_mA_s_==999);
 }
 {ExperimentRunner r;r.finishWalkingEnergyPulse(100080,1);CHECK(r.predicted_current_end_ms_==111);}
 std::cout<<"v16 actual finalizer: "<<cases<<" analytic cases, zero/full/abort/long/wrap/overflow/identity/once-only PASS\n";
}
'''.replace("AUDIT_STRUCT", audit_struct).replace("FINALIZER", finalizer)

with tempfile.TemporaryDirectory(prefix="energy_completion_runner_v16_") as temporary:
    tmp = Path(temporary)
    (tmp / "test.cpp").write_text(CPP, encoding="utf8")
    subprocess.run(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-DPULSE_LIFECYCLE_FIX=1", "-I" + str(ROOT / "src"),
                    str(tmp / "test.cpp"), "-o", str(tmp / "test.exe")], check=True)
    subprocess.run([str(tmp / "test.exe")], check=True)
