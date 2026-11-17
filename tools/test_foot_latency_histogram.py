"""Execute production first-receipt bookkeeping and JSON/bucket boundaries."""
from pathlib import Path
import json
import subprocess
import tempfile
from test_foot_fast_metadata import function
ROOT=Path(__file__).resolve().parents[1]
FIXTURE=r'''
#include <cassert>
#include <iostream>
#include <string>
#include "foot_latency_histogram.h"
#define portENTER_CRITICAL(x) (++*(x))
#define portEXIT_CRITICAL(x) (--*(x))
enum class MarkerDetectionReason {Detected,LowContrast};
struct Frame {
 uint16_t run_id=7;uint32_t sequence=9;bool timestamp_valid=true,frame_valid=true;
 uint64_t frame_us=1000,delivered_us=19000,ready_us=20000,control_observed_us=0;
 MarkerDetectionReason right_reason=MarkerDetectionReason::Detected,left_reason=MarkerDetectionReason::Detected;
};
struct Status {Frame latest;bool recording=false;uint32_t count=0;};
struct FootObserver {
 mutable int mux_=0;Status status_;FootLatencyHistogram latency_;Frame* frames_=nullptr;
 uint16_t recording_run_id_=7;uint32_t control_observation_attempts_=0,control_observation_unmatched_=0;
 void recordControlObservation(uint16_t,uint32_t,uint64_t);
 FootLatencyHistogram latencySnapshot() const;
};
__FUNCTIONS__
int main(){
 FootObserver obs;
 obs.recordControlObservation(8,9,21000);obs.recordControlObservation(7,8,21000);
 obs.recordControlObservation(7,9,19999);assert(!obs.latency_.count);
 obs.recordControlObservation(7,9,21000);assert(obs.latency_.count==1 && obs.latency_.bins[80]==1);
 obs.recordControlObservation(7,9,22000);assert(obs.latency_.count==1);
 auto snap=obs.latencySnapshot();assert(!obs.mux_ && snap.both_detected==1);
 obs.status_.latest.sequence=10;obs.status_.latest.control_observed_us=0;
 obs.status_.latest.left_reason=MarkerDetectionReason::LowContrast;
 obs.recordControlObservation(7,10,21249);assert(obs.latency_.count==2 && obs.latency_.bins[80]==2 && obs.latency_.both_detected==1);
 // A frame already matched in the archive must still get only one receipt.
 Frame archive=obs.status_.latest;archive.sequence=11;archive.control_observed_us=0;
 obs.status_.latest=archive;obs.status_.recording=true;obs.status_.count=1;obs.frames_=&archive;
 obs.recordControlObservation(7,11,21250);obs.recordControlObservation(7,11,23000);
 assert(archive.control_observed_us==21250 && obs.latency_.count==3 && obs.latency_.bins[81]==1);
 assert(obs.control_observation_attempts_==2 && obs.control_observation_unmatched_==1);
 FootLatencyHistogram h;h.record(1000,1000,true);h.record(1000,1249,true);h.record(1000,1250,false);
 h.record(1000,64749,true);h.record(1000,64750,false);h.record(1000,100000,true);
 h.record(0,1000,true);h.record(2000,1999,true);h.record(1000,1000ULL+UINT32_MAX+1,true);
 assert(h.count==6 && h.both_detected==4 && h.invalid_time==3);
 assert(h.bins[0]==2 && h.bins[1]==1 && h.bins[254]==1 && h.bins[255]==2);
 assert(h.min_us==0 && h.max_us==99000 && h.sum_us==226998);
 std::string text;h.appendJson(text);std::cout<<text<<"\n";
 FootLatencyHistogram maximum;maximum.count=maximum.both_detected=maximum.invalid_time=UINT32_MAX;
 maximum.sum_us=UINT64_MAX;maximum.min_us=maximum.max_us=maximum.max_gap_us=UINT32_MAX;
 for(auto& b:maximum.bins)b=UINT32_MAX;text.clear();maximum.appendJson(text);std::cout<<text<<"\n";
}
'''
def main():
    source=(ROOT/'src/foot_observer.cpp').read_text(encoding='utf8')
    body=function(source,'void FootObserver::recordControlObservation(')+'\n'+function(source,'FootLatencyHistogram FootObserver::latencySnapshot()')
    with tempfile.TemporaryDirectory(prefix='foot_histogram_') as folder:
        p=Path(folder);(p/'test.cpp').write_text(FIXTURE.replace('__FUNCTIONS__',body),encoding='utf8')
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
            '-DFOOT_HISTOGRAM_V32=1','-DFOOT_CAPTURE_V23=1','-DFOOT_LOW_LATENCY_V20=1',
            '-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
        result=subprocess.run([str(p/'test.exe')],check=True,capture_output=True,text=True)
        first,maximum=[json.loads(s) for s in result.stdout.splitlines()]
        assert sum(first['bins'])==first['count']==6 and maximum['sum_us']==2**64-1
        assert len(maximum['bins'])==256 and all(x==2**32-1 for x in maximum['bins'])
    print('PASS actual observer receipts: exact-once, stale sequence/run rejection, archive, histogram edges/overflow, invalid time and maximal JSON')
if __name__=='__main__':main()
