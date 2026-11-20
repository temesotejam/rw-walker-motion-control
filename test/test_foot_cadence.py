"""Check valid-update gaps and complete five-second buckets without HTTP polling."""
from pathlib import Path
import json,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
CODE=r'''
#include <cassert>
#include <iostream>
#include <string>
#include "foot_latency_histogram.h"
int main(){
 FootLatencyHistogram h;
 h.record(100,1000,true);h.record(100,25000,false);h.record(100,51000,true);
 assert(h.valid_gaps==1 && h.valid_gap_bins[100]==1 && h.valid_gaps_over_50ms==0);
 h.record(100,101001,true);assert(h.valid_gaps_over_50ms==1);
 h.record(100,4999999,true);h.record(100,5000000,false);h.record(100,5023000,true);
 assert(h.windows[0].receipts==5 && h.windows[0].detected==4);
 assert(h.windows[1].receipts==2 && h.windows[1].detected==1);
 h.record(100,80000000,true);assert(h.windows[0].epoch==16 && h.windows[0].receipts==1);
 assert(h.valid_gap_bins[127]==2); // both long gaps preserved, not discarded
 h.record(0,80000001,true);assert(h.invalid_time==1 && h.windows[0].receipts==1);
 uint32_t sum=0;for(auto n:h.valid_gap_bins)sum+=n;assert(sum==h.valid_gaps && sum==5);
 std::string text;h.appendJson(text);std::cout<<text<<"\n";
 h.last_receipt_us=UINT64_MAX;h.valid_gaps=h.valid_gaps_over_50ms=UINT32_MAX;
 for(auto& n:h.valid_gap_bins)n=UINT32_MAX;
 for(auto& w:h.windows)w.epoch=w.receipts=w.detected=UINT32_MAX;
 text.clear();h.appendJson(text);std::cout<<text<<"\n";
}
'''
with tempfile.TemporaryDirectory(prefix='foot_cadence_') as folder:
    p=Path(folder);(p/'test.cpp').write_text(CODE)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-DFOOT_STABLE40_V36=1',
                    '-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
    r=subprocess.run([str(p/'test.exe')],capture_output=True,text=True,check=True)
    data,maximal=[json.loads(s) for s in r.stdout.splitlines()]
    assert sum(data['valid_gap_bins'])==data['valid_gaps']==5
    assert maximal['last_receipt_us']==2**64-1
    assert len(maximal['windows'])==16 and maximal['windows'][15]==[2**32-1]*3
print('PASS valid-only gaps, missing detection spans,50ms boundary,overflow,5s buckets,80s ring reuse and maximal JSON')
