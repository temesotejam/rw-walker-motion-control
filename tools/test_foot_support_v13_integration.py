"""Compile actual v13 runner methods with driver doubles and real guards."""
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
v12=(ROOT/'tools/test_foot_support_v12_integration.py').read_text(encoding='utf8')
prefix=v12.split("\nfixture += r'''\n",1)[0]
exec(compile(prefix,str(ROOT/'tools/test_foot_support_v12_integration.py'),'exec'),globals())
fixture += v12.split("\nfixture += r'''\n",1)[1].split('int main(){',1)[0]
fixture += r'''
static void turn_pose(ExperimentRunner& r){
  auto now=r.case_now;foot_local_prediction::Predictor local;
  local.observe(r.status_.run_id,now-271,now-209,true,-.64947f,1.76019f);
  local.observe(r.status_.run_id,now-170,now-108,true,-.63555f,7.01348f);
  local.observe(r.status_.run_id,now-71,now-9,true,-.58236f,6.83693f);
  r.foot_local_pose_=local.projection();r.foot_pose_.frame_ms=now-71;r.foot_pose_.received_ms=now-9;
}
int main(){
  {ExperimentRunner r;prepare(r,60);turn_pose(r);candidate(r,4.7f,30.f);
   assert(r.calls==1 && r.last_finished_event.work_control_branch==18);
   const auto& a=r.foot_support_audit_[0];
   assert(a.local_turn && !a.local_returning && a.raw_boost_ms==10 && a.backend_reason==9);
   assert(a.requested_width==a.boosted_guarded_width && a.requested_width==a.guarded_width);
   assert(a.confidence==0 && a.prediction_source==3);
   assert(a.requested_width>a.normal_guarded_width);
   r.status_.pulse_active=false;r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
   r.case_now+=6;candidate(r,4.5f,29.f);assert(r.calls==1);}
  {ExperimentRunner r;prepare(r,70);candidate(r);
   assert(r.calls==1 && r.width==74);
   const auto& a=r.foot_support_audit_[0];assert(a.raw_boost_ms==4 && a.normal_width==70 && a.requested_width==74);}
  {ExperimentRunner r;prepare(r,80);candidate(r);assert(r.calls==1 && r.width==80 && r.foot_support_audit_[0].raw_boost_ms==0);}
  {ExperimentRunner r;prepare(r,70);deceleration(r);candidate(r,3.f,10.f);
   assert(r.calls==1 && r.foot_support_audit_[0].stall_route);}
  {ExperimentRunner r;prepare(r,70);
   r.wheel_cycle_guard_.limit(-1,378.91f,true,r.case_now-300,70);
   r.roller_->telemetry.speed_rpm=1.81f;r.simulated_rpm=1.81f;
   candidate(r);assert(r.calls==1 && r.width==72);
   assert(r.foot_support_audit_[0].normal_guarded_width==68 && r.foot_support_audit_[0].requested_width==72);}
  for(int mode:{1,2,3,4,5}){ExperimentRunner r;prepare(r,70);r.change_before_dispatch=mode;candidate(r);
   assert(r.calls==0 && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);
   assert(r.foot_support_audit_[0].backend_reason==4 || r.foot_support_audit_[0].backend_reason==5);}
  {ExperimentRunner r;prepare(r,70);r.simulated_rpm=600;
   r.roller_->telemetry.speed_rpm=1.81f;r.wheel_cycle_guard_.limit(-1,378.91f,true,r.case_now-100,70);
   candidate(r);
   assert(r.calls==0 && r.foot_support_audit_[0].backend_reason==7);
   assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);}
  {ExperimentRunner r;prepare(r,70);r.begin_ok=false;r.simulate_write_uncertain=true;candidate(r);
   assert(r.calls==1 && r.status_.emergency_stop && r.foot_support_audit_[0].backend_reason==8);}
  {ExperimentRunner r;prepare(r,70);r.simulated_side_mismatch=true;candidate(r);
   assert(r.calls==0 && r.foot_support_audit_[0].backend_reason==1);}
  {ExperimentRunner r;prepare(r,60);turn_pose(r);r.foot_local_pose_.frame_ms-=31;r.foot_pose_.frame_ms-=31;
   candidate(r,4.7f,30);assert(r.calls==0);}
  std::cout<<"v13 actual runner: early-turn, bounded boost, wheel guard, recheck, rollback and write uncertainty PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='foot_support_v13_runner_') as tmp:
    p=Path(tmp);(p/'fixture.cpp').write_text(fixture,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
                    '-DFOOT_SUPPORT_ASSIST=1','-DFOOT_SUPPORT_LOCAL=1','-DFOOT_SUPPORT_TRANSFER=1','-DSWAY_BALANCE_TRIAL=1',
                    '-I'+str(ROOT/'src'),str(p/'fixture.cpp'),'-o',str(p/'fixture.exe')],check=True)
    subprocess.run([str(p/'fixture.exe')],check=True)
