"""Execute real camera-frame/identity functions and runner invalidation."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];s=(root/'src/main.cpp').read_text(encoding='utf8')
def function(n):
    pos=s.index(n);start=s.rfind('\n',0,pos)+1;brace=s.index('{',pos);i=brace+1;depth=1
    while depth:
        depth+=(s[i]=='{')-(s[i]=='}');i+=1
    return s[start:i]
methods='\n'.join(function(n) for n in ('earlySyncIdentity()', 'earlySubmitFrame('))
h=(root/'src/experiment_runner.h').read_text(encoding='utf8')
invalidate=next(x.strip() for x in h.splitlines() if 'void invalidateFootEarlyCandidate()' in x)
fixture=r"""
#include "early_clock.h"
#include "early_motion_gate.h"
#include "early_geometry_diagnostics.h"
#include <cassert>
#include <cmath>
#include <cstdint>
using std::isfinite;
namespace appcfg {constexpr float kFootAngleBMinCalXPx=0,kFootAngleBMaxCalXPx=200,kFootAngleAMinCalXPx=0,kFootAngleAMaxCalXPx=200;}
namespace foot_calibration {int left[1],right[1]; float angleAt(const int*,float x){return x;}}
namespace foot_geometry_worker {
struct Input{uint32_t run_id=0,peak_id=0,sequence=0,generation=0,frame_ms=0,observed_ms=0,expiry_ms=0,exposure_earliest_ms=0,exposure_latest_ms=0;int side=0;float left_deg=0,right_deg=0;enum class Exposure{Provisional};Exposure exposure;};
struct Request{uint32_t run_epoch_ms=0,max_pose_gap_ms=0;Input input;};
struct Async {int jobs=0;void generation(uint32_t,uint32_t,uint32_t){} bool submit(const Request&){++jobs;return true;}};
}
struct FootFrame{uint32_t sequence=0,run_id=0;uint64_t frame_us=0;bool timestamp_valid=false,frame_valid=false,zero_ready=false,left_valid=false,right_valid=false,left_in_range=false,right_in_range=false;float left_x=0,right_x=0;};
struct FootSnapshot{FootFrame latest;float left_zero=100,right_zero=100;};
struct Runner{struct Status{uint16_t run_id=7;}st;early_motion_gate::Gate early_motion_gate_;uint32_t peak=900;bool consumed=false;
 const Status& status()const{return st;}uint32_t footEarlyPeakIdentity()const{return peak;}int8_t footEarlyPeakSide()const{return -1;}
 // INVALIDATE
} runner;
early_diag::Counters g_early_diag;foot_geometry_worker::Async early_worker;bool early_worker_ready=true;
uint32_t early_generation=1,early_run=7,early_peak=900,early_sequence=0,early_last_pose_us=0;float early_pitch_origin=0;
// FUNCTIONS
int main(){
 FootSnapshot fs;auto& f=fs.latest;f.sequence=1;f.run_id=7;f.frame_us=2000000;
 f.timestamp_valid=f.zero_ready=f.left_valid=f.right_valid=f.left_in_range=f.right_in_range=true;
 assert(runner.early_motion_gate_.offer(7,900,900,950,-.5f,1000));
 // A newer invalid frame cancels an armed earlier candidate and submits nothing.
 f.frame_valid=false;earlySubmitFrame(fs,2080000,1000000);
 assert(!runner.early_motion_gate_.valid && !runner.consumed && early_worker.jobs==0);
 // Earliest exposure at the current peak+5ms is rejected, even if the
 // nominal frame time itself appears after the peak.
 assert(runner.early_motion_gate_.offer(7,900,900,950,-.5f,1000));
 f.sequence=2;f.frame_valid=true;f.frame_us=1925000;
 earlySubmitFrame(fs,2005000,1000000);
 assert(!runner.early_motion_gate_.valid && early_worker.jobs==0 && !runner.consumed);
 // One millisecond beyond the strict bound is permitted to be submitted.
 f.sequence=3;f.frame_us=1926000;earlySubmitFrame(fs,2006000,1000000);
 assert(early_worker.jobs==1);
 // A newer valid frame supersedes an armed earlier candidate before enqueue.
 assert(runner.early_motion_gate_.offer(7,900,900,950,-.5f,1000));
 f.sequence=4;f.frame_us=2000000;f.frame_valid=true;earlySubmitFrame(fs,2080000,1000000);
 assert(!runner.early_motion_gate_.valid && !runner.consumed && early_worker.jobs==2);
 // Re-reading the same frame does not cancel a candidate just computed for it.
 assert(runner.early_motion_gate_.offer(7,900,900,950,-.5f,1000));
 earlySubmitFrame(fs,2080000,1000000);assert(runner.early_motion_gate_.valid && early_worker.jobs==2);
 // A peak-generation change also cancels old arm without consuming ZERO.
 runner.peak=901;earlySyncIdentity();assert(!runner.early_motion_gate_.valid && !runner.consumed);
}
""".replace('// INVALIDATE',invalidate).replace('// FUNCTIONS',methods)
with tempfile.TemporaryDirectory() as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(fixture,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(root/'src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('actual camera frame and peak-generation invalidation PASS')
