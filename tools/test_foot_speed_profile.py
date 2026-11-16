"""Exercise sensor boot transactions and all three predictors at faster cadence."""
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include <cassert>
#include <iostream>
#include "camera_speed_profile.h"
#include "foot_fast_policy.h"
#include "foot_pose_prediction.h"
#include "foot_local_prediction.h"
#include "foot_phase_prediction.h"
int main(){
#if defined(FOOT_SPEED_FIX1) && FOOT_SPEED_FIX1
 constexpr int target_hblank=128;
#else
 constexpr int target_hblank=64;
#endif
#if defined(FOOT_SPEED_FIX1) && FOOT_SPEED_FIX1
 // The corrected profile must not touch AEC, gain, flicker or exposure.
 {
  int regs[256]={},before[256]={};for(int i=0;i<256;++i)regs[i]=i;
  for(auto r:camera_speed_profile::expected)regs[r.address]=r.value;
  for(int i=0;i<256;++i)before[i]=regs[i];
  assert(camera_speed_profile::apply([&](int r){return regs[r];},[&](int r,int v){
   assert(r==0xfe || r==0x0f || r==0x01);regs[r]=v;return true;}));
  for(int i=0;i<256;++i)if(i!=0xfe && i!=0x0f && i!=0x01)assert(regs[i]==before[i]);
 }
#endif
 // A write can acknowledge before its register value is visible. Model one
 // 50ms frame of latency; the former immediate-readback policy rejected it.
 {
  int regs[256]={},pending[256],due[256];
  for(int i=0;i<256;++i){pending[i]=-1;due[i]=0;}
  for(auto r:camera_speed_profile::expected)regs[r.address]=r.value;
  regs[0xd2]=0x90;int now=0;camera_speed_profile::Report report;
  auto read=[&](int r){if(pending[r]>=0 && now>=due[r]){regs[r]=pending[r];pending[r]=-1;}return regs[r];};
  auto write=[&](int r,int v){pending[r]=v;due[r]=now+50;return true;};
  assert(camera_speed_profile::apply(read,write,[&](unsigned ms){now+=ms;},&report));
  assert(report.applied && report.settle_wait_ms>0 && regs[1]==target_hblank && regs[15]==0 && regs[0xd2]==0x90);
 }
 // Persistent old value is still rejected and its exact cause is retained.
 {
  int regs[256]={};for(auto r:camera_speed_profile::expected)regs[r.address]=r.value;
  regs[0xd2]=0x90;camera_speed_profile::Report report;
  assert(!camera_speed_profile::apply([&](int r){return regs[r];},[&](int r,int v){if(r!=1)regs[r]=v;return true;},[](unsigned){},&report));
  assert(report.reg==1 && report.expected==target_hblank && report.actual==50 && report.settle_wait_ms==100);
  assert(regs[0xd2]==0x90 && !report.applied);
 }
 // Every write/read failure is surfaced. AEC is restored after partial setup.
 for(int fault=-1;fault<100;++fault){
  int registers[256]={};for(auto r:camera_speed_profile::expected)registers[r.address]=r.value;
  registers[0xd2]=0x90;int calls=0;
  auto read=[&](int r){if(calls++==fault)return -1;return registers[r];};
  auto write=[&](int r,int v){if(calls++==fault)return false;registers[r]=v;return true;};
  bool ok=camera_speed_profile::apply(read,write);
  if(fault<0 || fault>=calls){assert(ok);assert(registers[1]==target_hblank && registers[15]==0 && registers[0xd2]==0x90);}
  else assert(!ok);
 }
 for(auto mismatch:camera_speed_profile::expected){
  int registers[256]={};for(auto r:camera_speed_profile::expected)registers[r.address]=r.value;
  registers[mismatch.address]^=1;int writes=0;
  assert(!camera_speed_profile::apply([&](int r){return registers[r];},[&](int r,int v){++writes;registers[r]=v;return true;}));
  assert(writes==1); // Page selection only; never program timing on unknown setup.
 }
 static_assert(foot_fast_policy::kWorkerPeriodUs==33333,"new worker cadence");
 static_assert(40000000/foot_fast_policy::kWorkerPeriodUs+2<foot_fast_policy::kArchiveCapacity,"40s archive fits");
 foot_pose_prediction::Predictor pose;
 foot_local_prediction::Predictor local;
 foot_phase_prediction::Predictor phase;
 for(unsigned i=0;i<12;++i){
  auto frame=1000+38*i;auto now=frame+47;
  auto p=pose.observe(1,frame,now,true,i*.1f,i*.1f);
  auto l=local.observe(1,frame,now,true,i*.1f,i*.1f);
  auto h=phase.observe(1,frame,now,true,i*.1f,i*.1f);
  if(i>=2){assert(p.ready());assert(l==foot_local_prediction::Reason::Ready);assert(h==foot_phase_prediction::Reason::Ready);}
 }
 assert(pose.observe(1,1420,1467,true,1.2f,1.2f).reason==foot_pose_prediction::Reason::Gap); // 2ms duplicate-like interval
 assert(local.observe(1,1420,1467,true,1.2f,1.2f)==foot_local_prediction::Reason::Gap);
 assert(phase.observe(1,1420,1467,true,1.2f,1.2f)==foot_phase_prediction::Reason::Gap);
 std::cout<<"PASS sensor readback/fault transactions, unknown profile rejection, 38ms predictors, capacity\n";
}
'''
def main():
    with tempfile.TemporaryDirectory(prefix='speed_profile_') as folder:
        p=Path(folder);(p/'test.cpp').write_text(CPP,encoding='utf8')
        for flags in [[],['-DFOOT_SPEED_FIX1=1']]:
            subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-DFOOT_SPEED_V22=1']+flags+['-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
            subprocess.run([str(p/'test.exe')],check=True)
    source=(ROOT/'src/camera_coexistence.cpp').read_text(encoding='utf8')
    hook=source.index('const bool timing_ok')
    assert source.index('sensor->set_hmirror')<hook<source.index('err = i2c_driver_delete',hook)
    assert 'camera_speed_timing_readback_failed' in source
if __name__=='__main__':main()
