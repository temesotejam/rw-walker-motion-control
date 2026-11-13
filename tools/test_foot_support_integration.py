"""Execute the FOOT_SUPPORT_ASSIST runner path using hardware doubles.

The candidate, ordinary selector, optional-early wrapper and rollback/dispatch
are extracted from the actual runner. Sensor/actuator transport and crossing
bookkeeping are doubles: this does not validate ESP timing or physical walking.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf8")
header = (ROOT / "src/experiment_runner.h").read_text(encoding="utf8")
fixture = (ROOT / "tools/tilt_priority_integration_fixture.cpp").read_text(encoding="utf8")


def method(name, result):
    signature = result + " ExperimentRunner::" + name + "("
    assert runner.count(signature) == 1, signature
    start = runner.index(signature)
    brace = runner.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (runner[end] == "{") - (runner[end] == "}")
        end += 1
    return runner[start:end]


# Use the established fixture's real portable controllers and minimal doubles.
# Replace its main: old tests intentionally expect the separate 15 ms path.
fixture = fixture.split("int main(){", 1)[0]
fixture = '#include "foot_phase_prediction.h"\n#include "foot_support_assist.h"\n#include "foot_support_policy.h"\n#include "foot_support_deceleration.h"\n' + fixture
fixture = fixture.replace("uint32_t millis(){return 10000;}",
                          "uint32_t mock_us=2000000,mock_ms=1300;\nuint32_t micros(){return mock_us;}\nuint32_t millis(){return mock_ms;}")
fixture = fixture.replace("struct Status {bool emergency_stop=false,pulse_active=false;",
                          "struct Status {uint16_t run_id=3;uint32_t pulse_id=0;bool emergency_stop=false,pulse_active=false;")
member_start = "  foot_phase_prediction::Projection foot_pose_;"
member_lines = [member_start]
member_depth = 1
for member_line in header.split(member_start, 1)[1].splitlines():
    directive = member_line.strip()
    if directive.startswith("#if"):
        member_depth += 1
    elif directive.startswith("#endif"):
        member_depth -= 1
        if member_depth == 0:
            break
    member_lines.append(member_line)
assert member_depth == 0
members = "\n".join(member_lines)
fixture = fixture.replace("class ExperimentRunner {public:", "class ExperimentRunner {public:\n" + members + "\n")
fixture = fixture.replace("Status status_;Imu", "float energy_control_autonomous_last_peak_amplitude_deg_=3.f;\n float simulated_rpm=20;bool simulated_side_mismatch=false;\n PsramLogger::EnergyControlAutonomousZeroCrossEvent last_finished_event;\n Status status_;Imu")
fixture = fixture.replace("void finishFootEnergyDecision(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t){++finished;}",
    "void finishFootEnergyDecision(PsramLogger::EnergyControlAutonomousZeroCrossEvent& e,uint32_t){++finished;last_finished_event=e;}")
fixture = fixture.replace("dispatchTiltPriority(e,t,prev_valid,prev_ms);",
    "e.zero_cross_abs_rate_dps=std::fabs(rate);e.pre_input_wheel_speed_rpm=simulated_rpm;\n"
    "   e.side_mismatch_diagnostic=simulated_side_mismatch;dispatchTiltPriority(e,t,prev_valid,prev_ms);")
fixture = fixture.replace("return begin_ok;", "if(begin_ok){++status_.pulse_id;status_.pulse_active=true;}return begin_ok;")
selection_start = "bool ExperimentRunner::tryTiltPriority("
selection_end = "\nbool ExperimentRunner::tryWeakSideAssist("
selection = selection_start + runner.split(selection_start, 1)[1].split(selection_end, 1)[0]
fixture = fixture.replace("// METHOD_SOURCE", selection)
fixture = fixture.replace("// DISPATCH_SOURCE", method("restoreUnsentStallRescue", "void") + "\n" + method("dispatchTiltPriority", "bool"))
fixture = fixture.replace("// EARLY_WRAPPER_SOURCE", method("acceptOptionalEarlyMotion", "void") + "\n" + method("tryFootSupportCandidate", "void"))
assert "// METHOD_SOURCE" not in fixture

fixture += r'''
#include <cstdlib>
#undef assert
#define assert(test) do { if(!(test)){std::cerr<<"assertion failed: "<<#test<<" line "<<__LINE__<<"\n";std::exit(2);} } while(false)
static void pose(ExperimentRunner& r,uint32_t now=1200) {
  r.foot_pose_=foot_phase_prediction::Projection{};
  r.foot_pose_.run_id=r.status_.run_id;r.foot_pose_.frame_ms=now-77;
  r.foot_pose_.received_ms=now-14;r.foot_pose_.reason=foot_phase_prediction::Reason::Ready;
  r.foot_pose_.confidence=1.f;r.foot_pose_sequence_=42;
  for(unsigned i=0;i<4;++i){
    r.foot_pose_.left[i]=.4f;r.foot_pose_.right[i]=2.5f;
    r.foot_pose_.left_uncertainty[i]=r.foot_pose_.right_uncertainty[i]=.35f;
  }
}
static void prepare(ExperimentRunner& r) {
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=800;
  r.tilt_priority_.peak(-1,900,3.f);pose(r);
}
static void candidate(ExperimentRunner& r,uint32_t t=1200) {
  mock_ms=r.run_start_ms_+t;
  r.tryFootSupportCandidate(t,mock_us,-1.5f,15.f);
}
static void prepare_coast(ExperimentRunner& r,bool short_timing=false,float incoming=8.f) {
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=800;
  if(short_timing){
    // A real previous paired arrival at +1 occurred 40 ms after its command:
    // the timing guard must retain a maximum of 40-30=10 ms for that side.
    r.tilt_priority_.peak(-1,100,3.f);
    tilt_priority::Choice c;c.selected=true;c.direction=1;c.width_ms=10;
    c.current_ma=1200;c.target_deg=r.tilt_priority_.target();
    r.tilt_priority_.crossing(1,600);r.tilt_priority_.confirm(1,600,c,true);
    r.tilt_priority_.peak(1,640,4.5f);
  }
  r.energy_control_autonomous_last_peak_amplitude_deg_=incoming;
  r.tilt_priority_.peak(-1,900,incoming);pose(r);
}
static void returning_pose(ExperimentRunner& r,bool forward=false) {
  pose(r);
  const float arc[4]={3.1f,2.8f,2.4f,2.f};
  for(unsigned i=0;i<4;++i)r.foot_pose_.right[i]=arc[forward?3u-i:i];
}
static void coast_sample(ExperimentRunner& r,uint32_t t,float inward) {
  pose(r,t);mock_ms=r.run_start_ms_+t;
  r.tryFootSupportCandidate(t,mock_us,-1.5f,inward);
}
static void coast_slowdown(ExperimentRunner& r) {
  for(uint32_t t=1100;t<=1140;t+=5)coast_sample(r,t,20.f);
  assert(r.calls==0 && r.optional_early_attempted_peak_ms_==0);
  for(uint32_t t=1145;t<1160;t+=5)coast_sample(r,t,14.f);
  assert(r.calls==0 && r.optional_early_attempted_peak_ms_==0);
  coast_sample(r,1160,14.f);
}
int main(){
  // Relocate the ordinary 60 ms proposal intact; the old early path was 15 ms.
  {ExperimentRunner r;prepare(r);candidate(r);
   assert(r.calls==1 && r.ma==1200 && r.sign==1 && r.width==60);
   assert(r.last_finished_event.work_control_branch==16 && r.last_finished_event.output_executed);
   assert(r.foot_support_audit_count_==1 && r.foot_support_audit_[0].executed);
   assert(r.foot_support_audit_[0].normal_width==60 && r.foot_support_audit_[0].guarded_width==60);
   assert(r.optional_early_attempted_peak_ms_==900 && r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(!r.optional_early_event_ && !r.foot_support_event_ && r.foot_support_active_audit_==-1);
   // Even manually reopening the crossing cannot repeat the same peak's trial.
   r.status_.pulse_active=false;r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
   candidate(r,1206);assert(r.calls==1 && r.foot_support_audit_count_==1);}

  // A staged overshoot coast alone does not authorize output. Sustained
  // inward deceleration can authorize one bounded 15 ms assistance pulse.
  {ExperimentRunner r;prepare_coast(r);
   tilt_priority::Input in;in.now_ms=1100;in.preset_deg=8;in.target_deg=r.tilt_priority_.target();
   in.next_side=1;in.crossing_rate_dps=20;in.hardware_cap_ms=150;in.hardware_healthy=true;
   const auto ordinary=r.tilt_priority_.propose(in);assert(ordinary.selected && ordinary.coast && !ordinary.width_ms);
   coast_slowdown(r);
   assert(r.calls==1 && r.ma==1200 && r.sign==1 && r.width==15);
   assert(r.foot_support_audit_count_==1 && r.foot_support_audit_[0].coast_assist);
   assert(r.foot_support_audit_[0].normal_width==0 && r.foot_support_audit_[0].requested_width==15);
   assert(r.foot_support_audit_[0].guarded_width==15 && r.foot_support_audit_[0].executed);
   assert(r.last_finished_event.work_control_branch==16 && r.last_finished_event.output_executed);
   assert(r.foot_support_reason_counts_[static_cast<unsigned>(foot_support_assist::Reason::CoastNeedsDeceleration)]>0);
   assert(r.optional_early_attempted_peak_ms_==900 && r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   r.status_.pulse_active=false;coast_sample(r,1165,14);assert(r.calls==1);
   r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
   coast_sample(r,1170,14);assert(r.calls==1 && r.foot_support_audit_count_==1);}

  // Shape anticipation is a separate route: no velocity history is fed here,
  // so it cannot borrow the sustained-deceleration evidence used above.
  // The lower-margin dominant foot must be Returning with >=1deg lower
  // asymmetry and an incoming peak at or below 8.5deg.
  for(int mode=0;mode<5;++mode){
   ExperimentRunner r;prepare_coast(r,false,mode==2?8.5001f:8.5f);
   returning_pose(r,mode==1);
   if(mode==3)for(auto& x:r.foot_pose_.right_uncertainty)x=.76f;
   if(mode==4)for(auto& x:r.foot_pose_.left)x=1.2f;
   const auto estimate=r.foot_pose_.predict(1200,0);
   if(mode==0)assert(estimate.phase_right==foot_phase_prediction::Phase::Returning);
   if(mode==1)assert(estimate.phase_right==foot_phase_prediction::Phase::Forward);
   candidate(r);
   if(mode==0){
     assert(r.calls==1 && r.width==15 && r.foot_support_audit_count_==1);
     const auto& audit=r.foot_support_audit_[0];
     assert(audit.coast_assist && audit.anticipated_shape_need && audit.executed);
     assert(audit.normal_width==0 && audit.requested_width==15 && audit.guarded_width==15);
     // Verify the candidate uses the current estimate, not the old +30ms lead.
     assert(std::fabs(audit.right-estimate.right)<.00001f);
     assert(std::fabs(audit.right-r.foot_pose_.predict(1200,30).right)>.1f);
     r.status_.pulse_active=false;candidate(r,1206);assert(r.calls==1);
   }else{
     assert(r.calls==0 && r.optional_early_attempted_peak_ms_==0 && r.foot_support_audit_count_==0);
     assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   }
  }

  // The same coast assistance remains subject to wheel and timing caps.
  // Any reduction declines the early pulse instead of silently weakening it.
  for(bool timing:{false,true}){ExperimentRunner r;prepare_coast(r,timing);
   if(timing)assert(r.tilt_priority_.timingCap(1,1160,150)==10);
   else {r.simulated_rpm=600;assert(r.wheel_cycle_guard_.limit(-1,600,true,1050,15)==15);}
   coast_slowdown(r);
   assert(r.calls==0 && r.foot_support_audit_count_==1);
   const auto& a=r.foot_support_audit_[0];
   assert(a.coast_assist && a.normal_width==0 && a.requested_width==15 && !a.executed);
   assert(a.guarded_width==(timing?10:12));
   assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==800);
   assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);
   coast_sample(r,1165,14);assert(r.calls==0 && r.foot_support_audit_count_==1);}

  // A real wheel guard changes 60 to 45 ms. Reject relocation, restore ZERO,
  // retain the guard history and let ordinary ZERO execute its own 45 ms pulse.
  {ExperimentRunner r;prepare(r);r.simulated_rpm=600;
   assert(r.wheel_cycle_guard_.limit(-1,600,true,1000,60)==60);
   const auto guard_before=r.wheel_cycle_guard_;candidate(r);
   assert(r.calls==0 && r.foot_support_audit_count_==1);
   assert(r.foot_support_audit_[0].normal_width==60 && r.foot_support_audit_[0].guarded_width==45);
   assert(!r.foot_support_audit_[0].executed && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==800);
   assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);
   auto copied=guard_before,restored=r.wheel_cycle_guard_;
   assert(copied.limit(1,600,true,1300,60)==restored.limit(1,600,true,1300,60));
   candidate(r,1206);assert(r.calls==0); // an early attempt is consumed once
   r.acceptMotionCrossing(true,1400,15.f,0,0,0,0);
   assert(r.calls==1 && r.width==45 && r.last_finished_event.work_control_branch==15);}

  // The upstream pose and identity checks do not consume the ordinary chance.
  for(int mode=0;mode<6;++mode){ExperimentRunner r;prepare(r);
   if(mode==0)r.foot_pose_.run_id=4;
   if(mode==1){r.foot_pose_.frame_ms=1035;r.foot_pose_.received_ms=1098;}
   if(mode==2)r.foot_pose_.confidence=.1f;
   if(mode==3)for(auto& x:r.foot_pose_.left_uncertainty)x=1.25f;
   if(mode==4)r.foot_pose_.received_ms=1201;
   if(mode==5)r.status_.emergency_stop=true;
   candidate(r);
   assert(r.calls==0 && r.optional_early_attempted_peak_ms_==0 && r.foot_support_audit_count_==0);
   assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);}

  // A side mismatch discovered by the real selector cancels the attempt and
  // restores the previous crossing timestamp, without sending motor current.
  {ExperimentRunner r;prepare(r);r.simulated_side_mismatch=true;candidate(r);
   assert(r.calls==0 && r.optional_early_attempted_peak_ms_==900);
   assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==800);}

  // A definitely unsent command restores ZERO. The retry is the ordinary path,
  // never another optional command in this same peak.
  {ExperimentRunner r;prepare(r);r.begin_ok=false;candidate(r);
   assert(r.calls==1 && !r.status_.emergency_stop);
   assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==800);
   candidate(r,1206);assert(r.calls==1);
   r.begin_ok=true;r.acceptMotionCrossing(true,1400,15.f,0,0,0,0);
   assert(r.calls==2 && r.last_finished_event.work_control_branch==15);}

  // A transport error after a possibly delivered current write must stop.
  // It may not reopen ZERO and risk a second pulse.
  {ExperimentRunner r;prepare(r);r.begin_ok=false;r.simulate_write_uncertain=true;candidate(r);
   assert(r.calls==1 && r.status_.emergency_stop && r.log_obj.events==1);
   assert(r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==1200);
   assert(!r.foot_support_audit_[0].executed);
   candidate(r,1206);assert(r.calls==1);}

  // Recheck pose identity, decision age, sample freshness and uncertainty
  // immediately in the real selector too, independent of upstream approval.
  for(bool coast:{false,true})for(int mode=0;mode<4;++mode){
   ExperimentRunner r;if(coast)prepare_coast(r);else prepare(r);
   r.optional_early_event_=true;r.foot_support_event_=true;
   r.foot_support_decision_.selected=true;r.foot_support_decision_.width_ms=coast?15:60;
   r.foot_support_decision_.coast_assist=coast;
   r.foot_support_decision_.estimated_zero_ms=100;
   r.foot_support_sample_us_=mock_us;mock_ms=r.run_start_ms_+1200;
   if(mode==0)r.foot_pose_.run_id=4;
   if(mode==1)mock_ms+=6;
   if(mode==2)r.foot_support_sample_us_=mock_us-10001;
   if(mode==3)for(auto& x:r.foot_pose_.right_uncertainty)x=1.25f;
   Event e;e.zero_cross_abs_rate_dps=15;
   assert(!r.tryTiltPriority(e,1200) && r.calls==0);}
  std::cout<<"actual support ordinary/coast candidate, deceleration, wheel/timing guards, rollback and uncertain write PASS\n";
}
'''

with tempfile.TemporaryDirectory(prefix="foot_support_runner_") as temporary:
    temporary = Path(temporary)
    cpp, exe = temporary / "fixture.cpp", temporary / "fixture.exe"
    cpp.write_text(fixture, encoding="utf8")
    subprocess.run([
        "g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-DFOOT_SUPPORT_ASSIST=1", "-DSWAY_BALANCE_TRIAL=1",
        "-I" + str(ROOT / "src"), str(cpp), "-o", str(exe)
    ], check=True)
    subprocess.run([str(exe)], check=True)
