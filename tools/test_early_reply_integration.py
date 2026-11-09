"""Execute the actual result receive function against a one-slot double."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];src=(root/'src/main.cpp').read_text(encoding='utf-8')
pos=src.index('static void __attribute__((noinline)) earlyReceive()');opening=src.index('{',pos);i=opening+1;depth=1
while depth:
    depth+=(src[i]=='{')-(src[i]=='}');i+=1
method=src[pos:i]
fixture=r"""
#include "early_geometry_diagnostics.h"
#include <cassert>
#include <cstdint>
namespace foot_geometry_worker {
struct Input{enum class Exposure{Unknown,Provisional};};
struct Diagnostic {int reason=0;uint32_t run_id=0,peak_id=0,sequence=0,generation=0;};
struct Result {uint32_t run_id=0,peak_id=0,sequence=0,generation=0,frame_ms=0,ready_ms=0;float threshold_deg=0;Input::Exposure exposure=Input::Exposure::Unknown;};
struct Reply{Diagnostic diagnostic;Result result;bool accepted=false;int async_reason=0;uint32_t compute_us=0,max_step_us=0,stack_high_water_raw=0;};
}
early_diag::Counters g_early_diag;
struct Worker {foot_geometry_worker::Reply slot;bool pending=false;
 bool receive(foot_geometry_worker::Reply& out){if(!pending)return false;out=slot;pending=false;return true;}} early_worker;
struct Runner{int offers=0;bool offerFootEarlyCandidate(uint16_t,uint32_t,uint32_t,uint32_t,float){++offers;return true;}} runner;
bool early_worker_ready=true;uint32_t early_run=7,early_peak=100,early_sequence=3,early_generation=4;
// METHOD
int main(){
 auto& q=early_worker.slot;g_early_diag.reset(7);q.diagnostic.run_id=6;q.diagnostic.peak_id=100;q.diagnostic.sequence=3;q.diagnostic.generation=4;
 q.compute_us=400;q.diagnostic.reason=2;q.accepted=true;early_worker.pending=true;earlyReceive();
 auto s=g_early_diag.snapshot();assert(s.stale_replies==1 && s.replies==0 && s.reason[2]==0 && s.max_compute_us==0 && runner.offers==0);
 q.diagnostic.run_id=7;q.diagnostic.generation=3;early_worker.pending=true;earlyReceive();
 s=g_early_diag.snapshot();assert(s.stale_replies==2 && s.replies==0 && s.reason[2]==0);
 q.diagnostic.generation=4;q.result.run_id=7;q.result.peak_id=100;q.result.sequence=3;q.result.generation=4;
 q.result.exposure=foot_geometry_worker::Input::Exposure::Provisional;early_worker.pending=true;earlyReceive();
 s=g_early_diag.snapshot();assert(s.replies==1 && s.reason[2]==1 && s.max_compute_us==400 && s.armed==1 && runner.offers==1);
}
""".replace('// METHOD',method)
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(fixture,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(root/'src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('actual early reply generation isolation PASS')
