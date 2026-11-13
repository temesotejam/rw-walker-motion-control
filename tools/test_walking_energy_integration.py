"""Exercise the actual v14 runner/dispatch/foot methods with driver doubles.

The real portable energy policy, wheel guard, timing caps and foot selector run
inside the actual source methods. Sensor/transport and crossing bookkeeping
remain doubles; this cannot validate ESP scheduling or physical walking.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
v12=(ROOT/'tools/test_foot_support_v12_integration.py').read_text(encoding='utf8')
prefix=v12.split("\nfixture += r'''\n",1)[0]
exec(compile(prefix,str(ROOT/'tools/test_foot_support_v12_integration.py'),'exec'),globals())
fixture += v12.split("\nfixture += r'''\n",1)[1].split('int main(){',1)[0]
fixture='#include "walking_energy_policy.h"\n'+fixture
for unused in ['static void prepare(', 'static void deceleration(']:
    begin=fixture.index(unused);end=fixture.index('{',begin)+1;depth=1
    while depth:
        depth+=(fixture[end]=='{')-(fixture[end]=='}');end+=1
    fixture=fixture[:begin]+fixture[end:]

# Use the actual energy members and both real v14 execution methods. Arduino
# metadata string serialization is outside this host-side actuator contract.
energy_start='  lateral_energy::CurrentTable walking_energy_current_;'
energy_members=energy_start+header.split(energy_start,1)[1].split('  void appendWalkingEnergyMetadata',1)[0]
energy_members+='  lateral_energy::Geometry lateral_energy_geometry_;\n'
energy_members+='  uint32_t predicted_current_end_ms_=0;float predicted_signed_current_end_mA_=0;\n'
energy_members+='  float predictCurrentGoalMa(int,float)const{return 329.547119f;}\n'
fixture=fixture.replace('class ExperimentRunner {public:','class ExperimentRunner {public:\n'+energy_members)
fixture=fixture.replace('struct Status {uint16_t run_id=3;',
                        'struct Status {float beta_model_vbat_mV=7500;uint16_t run_id=3;')
fixture=fixture.replace(' uint32_t zero_cross_time_ms=999;',
    ' uint32_t previous_peak_time_ms=900;float previous_peak_amplitude_deg=8;\n'
    ' float detector_angle_after_deg=0,zero_cross_rate_dps=35,i0_estimated_mA=0,cg_shift_mm=0;\n'
    ' float kinetic_energy_j=0,delta_energy_required_j=0,commanded_energy_deficit_j=0;\n'
    ' float corrected_target_energy_j=0,integral_energy_j=0,predicted_loss_j=0;\n'
    ' float maximum_input_work_j=0,unapplied_energy_request_j=0;\n'
    ' uint32_t zero_cross_time_ms=999;')
fixture=fixture.replace('uint32_t t,float rate,float,float,float,float)',
                        'uint32_t t,float rate,float,float after,float,float)')
marker='e.side_mismatch_diagnostic=simulated_side_mismatch;'
assert fixture.count(marker)==1
fixture=fixture.replace(marker,marker+'\n'
    '   e.zero_cross_rate_dps=rate;e.detector_angle_after_deg=after;\n'
    '   e.previous_peak_time_ms=energy_control_autonomous_last_peak_ms_;\n'
    '   e.previous_peak_amplitude_deg=energy_control_autonomous_last_peak_amplitude_deg_;\n')
energy_source=(ROOT/'src/walking_energy_runner.inc').read_text(encoding='utf8')
energy_source=energy_source.split('void ExperimentRunner::appendWalkingEnergyMetadata',1)[0]+'\n#endif\n'
fixture += '\n'+energy_source
fixture += r'''
static bool near(float a,float b,float tolerance=3e-9f){return std::fabs(a-b)<tolerance;}
static void ready(ExperimentRunner& r,float same=6,float previous=8){
  r.case_now=1200;mock_ms=r.run_start_ms_+r.case_now;mock_us=2000000;
  r.energy_control_autonomous_last_peak_ms_=900;
  r.energy_control_autonomous_last_peak_amplitude_deg_=previous;
  r.energy_control_autonomous_last_peak_side_=-1;
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=600;
  r.walking_energy_peaks_[1]=same;r.walking_energy_peak_ms_[1]=700;
  assert(r.walking_energy_current_.prepare(.0316f));
  // Ramp the real stage to 8deg using only accepted peak history.
  r.tilt_priority_.peak(-1,1,4.5f);uint32_t t=10;
  for(unsigned n=0;n<32;++n){const int side=n%2?-1:1;
    tilt_priority::Choice c;c.selected=true;c.direction=side;c.current_ma=1200;
    c.width_ms=1;c.target_deg=r.tilt_priority_.target();
    r.tilt_priority_.crossing(side,t);r.tilt_priority_.confirm(side,t,c,true,true,false);
    r.tilt_priority_.peak(side,t+10,c.target_deg);t+=25;}
  // Fresh unpaired observations clear the artificial short time-to-peak cap.
  r.tilt_priority_.peak(-1,900,previous);r.tilt_priority_.peak(1,925,same);
  assert(r.tilt_priority_.target()==8.f);
  local_pose(r,r.case_now);
}
static Event event(ExperimentRunner& r,float rate=55){
  Event e;e.zero_cross_time_ms=r.case_now;e.zero_cross_abs_rate_dps=rate;
  e.zero_cross_rate_dps=rate;e.previous_peak_time_ms=r.energy_control_autonomous_last_peak_ms_;
  e.previous_peak_amplitude_deg=r.energy_control_autonomous_last_peak_amplitude_deg_;return e;
}
static walking_energy_policy::Input model(ExperimentRunner& r,const Event& e,unsigned cap=80){
  walking_energy_policy::Input in;in.side=e.physical_next_peak_side;in.rate_dps=e.zero_cross_abs_rate_dps;
  in.target_deg=r.tilt_priority_.target();in.previous_peak_deg=e.previous_peak_amplitude_deg;
  const unsigned i=in.side>0?1:0;
  in.same_side_peak_deg=r.walking_energy_peak_ms_[i] && r.case_now>=r.walking_energy_peak_ms_[i] &&
      r.case_now-r.walking_energy_peak_ms_[i]<=3000?r.walking_energy_peaks_[i]:NAN;
  in.body_angle_deg=(r.optional_early_event_ || r.stall_rescue_event_)?e.detector_angle_after_deg:0.f;
  in.initial_current_ma=in.side*e.i0_estimated_mA;in.goal_current_ma=329.547119f;in.cap_ms=cap;return in;
}
static void energy_fields(const Event& e,const walking_energy_policy::Decision& d){
  assert(near(e.target_energy_j,d.target_j) && near(e.passive_energy_j,d.available_j));
  assert(near(e.kinetic_energy_j,d.effective_kinetic_j));
  assert(near(e.delta_energy_required_j,d.deficit_j) && near(e.commanded_energy_deficit_j,d.request_j));
  assert(near(e.corrected_target_energy_j,d.target_j+d.peak_correction_j));
  assert(near(e.predicted_input_work_j,d.predicted_work_j) && near(e.predicted_energy_j,d.predicted_arrival_j));
  assert(near(e.q_command_mA_s,d.q_mas) && near(e.unapplied_energy_request_j,d.unmet_j));
  assert(e.integral_energy_j==0 && e.cg_shift_mm==0);
}
int main(){
  // Branch19 uses the real J-domain inverse. Poison old width history: it
  // cannot replace the energy solution with +/-10ms or an ordinary-rise cap.
  {ExperimentRunner r;ready(r);r.tilt_priority_.ordinaryResult(1,1100,1,true);
   Event e=event(r);auto in=model(r,e);auto expected=walking_energy_policy::decide(r.lateral_energy_geometry_,r.walking_energy_current_,in);
   assert(expected.valid && !expected.coast && expected.width_ms>15);
   assert(r.dispatchTiltPriority(e,r.case_now,true,600));
   assert(e.work_control_branch==19 && e.valid && e.output_executed && r.calls==1);
   assert(r.width==expected.width_ms && e.pulse_width_ms==expected.width_ms && r.ma==1200);
   energy_fields(e,expected);assert(r.walking_energy_count_==1 && r.adaptive_cg_.broken);}

  // Completed peaks and current rate change the actual dispatched width.
  unsigned weak=0,strong=0,slow=0,fast=99;
  for(int k=0;k<4;++k){ExperimentRunner r;ready(r,k==0?2.f:k==1?11.f:6.f);
   Event e=event(r,k==2?25.f:k==3?80.f:55.f);
   assert(r.dispatchTiltPriority(e,r.case_now,true,600) && e.work_control_branch==19 && e.valid);
   if(k==0)weak=e.pulse_width_ms;
   if(k==1)strong=e.pulse_width_ms;
   if(k==2)slow=e.pulse_width_ms;
   if(k==3){fast=e.pulse_width_ms;assert(r.calls==0 && !e.output_executed);}
  }
  assert(weak>strong && slow>fast && fast==0);

  // Rescue retains its 15ms timing cap at5.5deg/s. It is an energy-sized
  // excitation with explicit low-rate extrapolation, not a discarded event.
  for(bool rescue:{false,true}){ExperimentRunner r;ready(r);r.stall_rescue_event_=rescue;r.optional_early_event_=!rescue;
   Event e=event(r,5.5f);e.detector_angle_after_deg=-.4f;
   auto expected=walking_energy_policy::decide(r.lateral_energy_geometry_,r.walking_energy_current_,model(r,e,15));
   assert(expected.valid && !expected.coast && expected.width_ms<=15 && expected.passive_intercept_rolloff);
   assert(r.dispatchTiltPriority(e,r.case_now,true,600));
   assert(e.valid && e.output_executed && e.work_control_branch==19 && r.width==expected.width_ms);
   assert(e.work_brake_cap_ms==15 && e.energy_model_extrapolated);energy_fields(e,expected);}

  // Invalid model/telemetry/history remains a consumed energy branch in
  // dispatch, never an invitation for a legacy angle-width fallback.
  for(unsigned bad=0;bad<8;++bad){ExperimentRunner r;ready(r);Event e=event(r);
   if(bad==0)e.zero_cross_abs_rate_dps=NAN;
   if(bad==1)e.vbat_mV=0;
   if(bad==2)e.pre_input_wheel_speed_valid=false;
   if(bad==3)e.side_mismatch_diagnostic=true;
   if(bad==4)e.previous_peak_time_ms=0;
   if(bad==5)e.previous_peak_time_ms=r.case_now+1;
   if(bad==6)e.previous_peak_time_ms=r.case_now-1501;
   if(bad==7)r.walking_energy_current_.ready=false;
   assert(r.dispatchTiltPriority(e,r.case_now,true,600));
   assert(e.work_control_branch==19 && !e.valid && !e.output_executed && r.calls==0);
   assert(r.walking_energy_total_==1);}

  // Stale/future same-side history is omitted; the current previous peak
  // remains usable. Fresh history alone enables paired energy feedback.
  for(unsigned mode=0;mode<3;++mode){ExperimentRunner r;ready(r,2);Event e=event(r);
   if(mode==1)r.walking_energy_peak_ms_[1]=r.case_now+1;
   if(mode==2){r.case_now=5000;mock_ms=r.run_start_ms_+r.case_now;e.zero_cross_time_ms=r.case_now;e.previous_peak_time_ms=4900;}
   assert(r.dispatchTiltPriority(e,r.case_now,true,600) && e.valid);
   const auto& d=r.walking_energy_audit_[0].decision;
   if(mode==0)assert(std::isfinite(d.same_side_peak_energy_j) && d.peak_correction_j!=0);
   else assert(std::isnan(d.same_side_peak_energy_j) && d.peak_correction_j==0);}

  // The guard can only shorten. Actual Q/work/arrival/residual must be
  // recomputed at that shortened width, with signed I0 preserved.
  {ExperimentRunner r;ready(r);Event e=event(r);e.i0_estimated_mA=-5;e.pre_input_wheel_speed_rpm=600;
   auto in=model(r,e);auto selected=walking_energy_policy::decide(r.lateral_energy_geometry_,r.walking_energy_current_,in);
   r.wheel_cycle_guard_.limit(-1,600,true,1000,selected.width_ms);
   auto guard=r.wheel_cycle_guard_;auto protected_ms=guard.limit(1,600,true,r.case_now,selected.width_ms);
   assert(protected_ms<selected.width_ms);
   auto actual=walking_energy_policy::evaluateWidth(r.lateral_energy_geometry_,r.walking_energy_current_,in,protected_ms);
   assert(r.dispatchTiltPriority(e,r.case_now,true,600) && e.valid && r.width==protected_ms);
   assert(r.width<=selected.width_ms && actual.predicted_work_j<selected.predicted_work_j);
   assert(actual.unmet_j>selected.unmet_j);energy_fields(e,actual);
   assert(near(e.aligned_initial_current_mA,-5));}

  // A genuine foot timing candidate executes exactly the energy-selected
  // amount. Supplying v13 boost/coast flags cannot add milliseconds or turn
  // surplus-energy coast into a forced pulse.
  {ExperimentRunner r;ready(r);candidate(r,3.f,25.f);
   assert(r.calls==1 && r.last_finished_event.work_control_branch==19);
   const auto& a=r.foot_support_audit_[0];const auto& d=r.walking_energy_audit_[0];
   assert(a.executed && a.raw_boost_ms==0 && a.requested_width==d.requested_ms && r.width==d.guarded_ms);
   assert(a.requested_width==a.boosted_guarded_width && !a.coast_assist);}
  // Observation-only legacy experiments can accumulate a pending/active
  // state. Their old overlap guard must not suppress v14's energy path.
  for(bool foot:{false,true}){ExperimentRunner r;ready(r);
   r.long_foot_balance_.observe(100,200,10,1,6,6,true,true,true,1);
   r.positive_sway_probe_.confirm(1000,60,65,true,true);
   assert(!r.long_foot_balance_.idleForNegativeTrial() && r.positive_sway_probe_.active());
   if(foot){candidate(r,3.f,25.f);assert(r.last_finished_event.work_control_branch==19);}
   else {r.optional_early_event_=true;Event e=event(r,25);e.detector_angle_after_deg=-.4f;
     assert(r.dispatchTiltPriority(e,r.case_now,true,600) && e.work_control_branch==19);}
   assert(r.calls==1 && r.walking_energy_count_==1);}
  {ExperimentRunner r;ready(r);r.foot_support_event_=true;r.optional_early_event_=true;
   r.foot_support_sample_us_=mock_us;r.foot_support_active_audit_=0;
   r.foot_support_source_=foot_support_pose::Source::Local;
   r.foot_support_decision_.selected=true;r.foot_support_decision_.width_ms=80;
   r.foot_support_decision_.coast_assist=true;r.foot_support_input_.raw_boost_ms=12;
   Event e=event(r,80);e.detector_angle_after_deg=-2;
   assert(r.tryTiltPriorityImpl(e,r.case_now,nullptr));
   assert(r.calls==0 && !e.output_executed && e.work_control_branch==19);
   assert(r.walking_energy_audit_[0].decision.coast && r.walking_energy_audit_[0].requested_ms==0);}

  // Upstream foot pose cannot bypass zero energy demand or stale peak age.
  for(unsigned mode=0;mode<4;++mode){ExperimentRunner r;ready(r);
   if(mode==1)r.energy_control_autonomous_last_peak_ms_=r.case_now+1;
   if(mode==2){r.case_now=2501;local_pose(r,r.case_now);mock_ms=r.run_start_ms_+r.case_now;}
   if(mode==3)r.foot_pose_.run_id++;
   candidate(r,2.f,mode==0?80.f:25.f);
   assert(r.calls==0 && r.foot_support_audit_count_==0 && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);}

  std::cout<<"v14 real runner: branch19 J-sized peak/rate, rescue15, coast/no fallback, guard recomputation, foot timing-only and fresh history PASS\n";
}
'''

with tempfile.TemporaryDirectory(prefix='walking_energy_runner_') as tmp:
    p=Path(tmp);(p/'fixture.cpp').write_text(fixture,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
                    '-DFOOT_SUPPORT_ASSIST=1','-DFOOT_SUPPORT_LOCAL=1','-DFOOT_SUPPORT_TRANSFER=1',
                    '-DFOOT_SUPPORT_ENERGY=1','-DSWAY_BALANCE_TRIAL=1',
                    '-I'+str(ROOT/'src'),str(p/'fixture.cpp'),'-o',str(p/'fixture.exe')],check=True)
    subprocess.run([str(p/'fixture.exe')],check=True)
