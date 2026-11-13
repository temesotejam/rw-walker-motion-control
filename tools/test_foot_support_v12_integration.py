"""Execute the v12 real runner methods with frozen local evidence and doubles.

Shares the v11 fixture construction only, then supplies separate v12 cases.
The real source-selection helper is used without inventing a history score.
Hardware transport and crossing bookkeeping remain doubles, not physical gait.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
v11_test = (ROOT / "tools/test_foot_support_integration.py").read_text(encoding="utf8")
prefix = v11_test.split("fixture += r'''", 1)[0]
assert "TemporaryDirectory" not in prefix
exec(compile(prefix, str(ROOT / "tools/test_foot_support_integration.py"), "exec"), globals())
fixture = '#include "foot_support_pose.h"\n' + fixture
roller_double = "struct Roller {bool good=true;bool ok()const{return good;}};"
assert fixture.count(roller_double) == 1
fixture = fixture.replace(roller_double,
    "struct Telemetry {float speed_rpm=20;uint32_t speed_sample_time_us=1999900;bool speed_valid=true;};\n"
    "struct Roller {bool good=true;Telemetry telemetry;bool ok()const{return good;}Telemetry telemetrySnapshot()const{return telemetry;}};")
fixture = fixture.replace("float simulated_rpm=20;",
    "uint32_t case_now=0;int change_before_dispatch=0;float simulated_rpm=20;")
fixture = fixture.replace("e.side_mismatch_diagnostic=simulated_side_mismatch;dispatchTiltPriority(e,t,prev_valid,prev_ms);",
    "e.side_mismatch_diagnostic=simulated_side_mismatch;\n"
    "   if(change_before_dispatch==1)++status_.run_id;\n"
    "   if(change_before_dispatch==2)mock_ms+=6;\n"
    "   if(change_before_dispatch==3)mock_us+=10001;\n"
    "   if(change_before_dispatch==4){foot_pose_.confidence=1;for(unsigned i=0;i<4;++i){foot_pose_.left_uncertainty[i]=.35f;foot_pose_.right_uncertainty[i]=.35f;}}\n"
    "   if(change_before_dispatch==5)--foot_local_pose_.frame_ms;\n"
    "   if(change_before_dispatch==6)mock_ms+=4;\n"
    "   dispatchTiltPriority(e,t,prev_valid,prev_ms);")
fixture += r'''
#include <cstdlib>
#undef assert
#define assert(test) do {if(!(test)){std::cerr<<"assertion failed: "<<#test<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
static void local_pose(ExperimentRunner& r,uint32_t now){
  foot_local_prediction::Predictor local;
  local.observe(r.status_.run_id,now-263,now-200,true,-.56573f,5.75716f);
  local.observe(r.status_.run_id,now-163,now-100,true,-.47464f,5.36924f);
  local.observe(r.status_.run_id,now-63,now,true,-.58475f,3.95754f);
  r.foot_local_pose_=local.projection();
  r.foot_pose_=foot_phase_prediction::Projection{};
  r.foot_pose_.run_id=r.status_.run_id;r.foot_pose_.frame_ms=now-63;r.foot_pose_.received_ms=now;
  r.foot_pose_.reason=foot_phase_prediction::Reason::Ready;r.foot_pose_.confidence=0;
  for(unsigned i=0;i<4;++i){r.foot_pose_.left[i]=-.58475f;r.foot_pose_.right[i]=3.95754f;}
  r.foot_pose_sequence_=1001;
}
static void prepare(ExperimentRunner& r,unsigned requested_width){
  mock_us=2000000;
  r.tilt_priority_.peak(-1,100,3.f);
  uint32_t t=300;
  const unsigned pairs=(requested_width-60u)/5u;
  for(unsigned n=0;n<pairs;++n)for(int side:{1,-1}){
    tilt_priority::Choice c;c.selected=true;c.direction=side;c.current_ma=1200;
    c.width_ms=r.tilt_priority_.widthLimit(side);c.target_deg=r.tilt_priority_.target();
    r.tilt_priority_.crossing(side,t);r.tilt_priority_.confirm(side,t,c,true);
    r.tilt_priority_.peak(side,t+200,1.f);t+=300;
  }
  // A fresh last negative peak is needed even when no cap-learning loop ran.
  if(!pairs){r.tilt_priority_.peak(-1,900,3.f);t=1000;}
  r.case_now=t+300;
  r.energy_control_autonomous_last_peak_ms_=pairs?t-100:900;
  r.energy_control_autonomous_last_peak_side_=-1;
  r.energy_control_autonomous_last_peak_amplitude_deg_=pairs?1.f:3.f;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=r.case_now-500;
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  local_pose(r,r.case_now);mock_ms=r.run_start_ms_+r.case_now;
  assert(r.tilt_priority_.widthLimit(1)==requested_width);
}
static void candidate(ExperimentRunner& r,float distance=2.f,float speed=15.f){
  mock_ms=r.run_start_ms_+r.case_now;
  r.tryFootSupportCandidate(r.case_now,mock_us,-distance,speed);
}
static void deceleration(ExperimentRunner& r){
  for(uint32_t t=r.case_now-40;t<=r.case_now-20;t+=5)
    assert(!r.foot_support_deceleration_.update(r.status_.run_id,r.energy_control_autonomous_last_peak_ms_,t,20));
  for(uint32_t t=r.case_now-15;t<r.case_now;t+=5)
    assert(!r.foot_support_deceleration_.update(r.status_.run_id,r.energy_control_autonomous_last_peak_ms_,t,10));
}
int main(){
  // The common selector keeps history confidence at zero while recognizing
  // independent local Returning evidence from exactly the failure's 3 frames.
  {ExperimentRunner r;prepare(r,70);
   const auto selected=foot_support_pose::select(r.foot_pose_,r.foot_local_pose_,r.case_now);
   assert(selected.usable && selected.source==foot_support_pose::Source::Local);
   assert(selected.pose.confidence==0 && selected.local_returning_confirmed && selected.local_evidence==1.f);}

  // Preserve the complete ordinary command, including widths forbidden by v11.
  for(unsigned width:{70u,80u}){ExperimentRunner r;prepare(r,width);candidate(r);
   assert(r.calls==1 && r.width==width && r.ma==1200 && r.sign==1);
   assert(r.foot_support_audit_count_==1 && r.foot_support_audit_[0].executed);
   const auto& a=r.foot_support_audit_[0];
   assert(a.normal_width==width && a.requested_width==width && a.guarded_width==width);
   assert(a.prediction_source==static_cast<uint8_t>(foot_support_pose::Source::Local));
   assert(a.confidence==0 && a.local_returning && a.local_evidence==1.f);
   assert(a.mode==static_cast<uint8_t>(foot_support_policy::Mode::OrdinaryRelocation));
   assert(r.last_finished_event.work_control_branch==17);
   assert(r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   r.status_.pulse_active=false;r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
   r.case_now+=6;candidate(r);assert(r.calls==1 && r.foot_support_audit_count_==1);}

  // A changed guard must not truncate the relocation or consume normal ZERO.
  {ExperimentRunner r;prepare(r,70);r.simulated_rpm=600;
   assert(r.wheel_cycle_guard_.limit(-1,600,true,r.case_now-200,70)==70);
   const uint32_t previous=r.energy_control_autonomous_last_accepted_zero_cross_ms_;
   candidate(r);
   assert(r.calls==0 && r.foot_support_audit_count_==1);
   assert(r.foot_support_audit_[0].normal_width==70 && r.foot_support_audit_[0].guarded_width<70);
   assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==previous);
   assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);}

  // Preview the ordinary proposal through a COPY of the same wheel guard.
  // With this cached/actual speed pair the protected ordinary width is 68 ms,
  // although the upstream controller proposed 70 ms. It must execute once at
  // 68 ms, with no extra taper and no mutation by the earlier preview.
  {ExperimentRunner r;prepare(r,70);r.simulated_rpm=1.81f;
   r.roller_obj.telemetry.speed_rpm=1.81f;
   assert(r.wheel_cycle_guard_.limit(-1,378.91f,true,r.case_now-100,70)==70);
   candidate(r);
   assert(r.calls==1 && r.width==68 && r.ma==1200);
   const auto& a=r.foot_support_audit_[0];
   assert(a.normal_width==70 && a.normal_guarded_width==68 && a.requested_width==68);
   assert(a.guarded_width==68 && a.executed);}

  // Preview does not override a different guard result at actual selection.
  {ExperimentRunner r;prepare(r,70);r.simulated_rpm=600.f;
   r.roller_obj.telemetry.speed_rpm=1.81f;
   assert(r.wheel_cycle_guard_.limit(-1,378.91f,true,r.case_now-100,70)==70);
   const uint32_t previous=r.energy_control_autonomous_last_accepted_zero_cross_ms_;
   candidate(r);
   assert(r.calls==0 && r.foot_support_audit_count_==1);
   const auto& a=r.foot_support_audit_[0];
   assert(a.normal_width==70 && a.normal_guarded_width==68 && a.requested_width==68);
   assert(a.guarded_width==53 && !a.executed);
   assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==previous);}

  // A cached preview must itself be fresh and valid. Missing or future sample
  // timestamps cannot manufacture the protected width and consume the peak.
  for(int stale=0;stale<5;++stale){ExperimentRunner r;prepare(r,70);
   if(stale==0)r.roller_obj.telemetry.speed_valid=false;
   if(stale==1)r.roller_obj.telemetry.speed_sample_time_us=0;
   if(stale==2)r.roller_obj.telemetry.speed_sample_time_us=mock_us-50001;
   if(stale==3)r.roller_obj.telemetry.speed_sample_time_us=mock_us+1;
   if(stale==4)r.roller_obj.telemetry.speed_rpm=NAN;
   candidate(r);assert(r.calls==0 && r.foot_support_audit_count_==0);
   assert(r.optional_early_attempted_peak_ms_==0 && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);}
  {ExperimentRunner r;prepare(r,70);r.roller_obj.telemetry.speed_sample_time_us=mock_us-50000;
   candidate(r);assert(r.calls==1 && r.width==70);}

  // A 300 ms constant-rate ZERO estimate is admitted only with local Returning
  // shape and independently confirmed, sustained inward loss of speed.
  for(bool slowed:{false,true}){ExperimentRunner r;prepare(r,70);
   if(slowed)deceleration(r);
   candidate(r,3.f,10.f);
   if(slowed){assert(r.calls==1 && r.width==70);
     assert(r.foot_support_audit_[0].stall_route && r.foot_support_audit_[0].inward_decelerating);
     assert(r.foot_support_audit_[0].estimated_zero_ms>250.f);
     assert(r.foot_support_audit_[0].mode==static_cast<uint8_t>(foot_support_policy::Mode::StallOrdinaryRelocation));
   }else{assert(r.calls==0 && r.optional_early_attempted_peak_ms_==0);}}

  // Identity, decision latency, IMU age and a changed prediction source are
  // checked after the candidate was selected but before its actuator call.
  for(int change=1;change<=5;++change){ExperimentRunner r;prepare(r,70);
   const uint32_t previous=r.energy_control_autonomous_last_accepted_zero_cross_ms_;
   r.change_before_dispatch=change;candidate(r);
   assert(r.calls==0 && r.foot_support_audit_count_==1 && !r.foot_support_audit_[0].executed);
   assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==previous);}

  // Four ms is within the dispatch-age ceiling, but the propagated body pose
  // must still preserve enough estimated time for the complete ordinary pulse.
  for(bool short_lead:{false,true}){ExperimentRunner r;prepare(r,70);
   r.change_before_dispatch=6;candidate(r,short_lead?1.365f:2.f,15.f);
   if(short_lead){assert(r.calls==0 && r.foot_support_audit_count_==1);
     assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   }else{assert(r.calls==1 && r.width==70);}}

  // Valid-looking local data from a different image/run cannot fill history's
  // gap, and a single reversal cannot pretend to be two decreasing intervals.
  for(int invalid=0;invalid<3;++invalid){ExperimentRunner r;prepare(r,70);
   if(invalid==0)++r.foot_local_pose_.run_id;
   if(invalid==1)--r.foot_local_pose_.frame_ms;
   if(invalid==2){foot_local_prediction::Predictor p;
     p.observe(r.status_.run_id,r.case_now-263,r.case_now-200,true,0,3);
     p.observe(r.status_.run_id,r.case_now-163,r.case_now-100,true,0,5.7f);
     p.observe(r.status_.run_id,r.case_now-63,r.case_now,true,0,5.3f);r.foot_local_pose_=p.projection();}
   candidate(r);assert(r.calls==0 && r.optional_early_attempted_peak_ms_==0);}

  // A possibly delivered failed write owns the half-cycle and stops; it cannot
  // restore ZERO and invite a second command.
  {ExperimentRunner r;prepare(r,70);r.begin_ok=false;r.simulate_write_uncertain=true;candidate(r);
   assert(r.calls==1 && r.status_.emergency_stop);
   assert(r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   r.case_now+=6;candidate(r);assert(r.calls==1);}
  std::cout<<"actual v12 local selection, 70/80ms preservation, stall, fresh-source guards and uncertain write PASS\n";
}
'''

with tempfile.TemporaryDirectory(prefix="foot_support_v12_runner_") as temporary:
    temporary = Path(temporary)
    cpp, exe = temporary / "fixture.cpp", temporary / "fixture.exe"
    cpp.write_text(fixture, encoding="utf8")
    subprocess.run([
        "g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-DFOOT_SUPPORT_ASSIST=1", "-DFOOT_SUPPORT_LOCAL=1", "-DSWAY_BALANCE_TRIAL=1",
        "-I" + str(ROOT / "src"), str(cpp), "-o", str(exe)
    ], check=True)
    subprocess.run([str(exe)], check=True)
