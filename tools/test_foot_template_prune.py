"""Frozen detector equivalence and work counts; no claim of ESP32 latency."""
from pathlib import Path
import subprocess
import tempfile
import test_foot_tracking_fast as oracle
import test_foot_tracker_prepared as prepared
import test_foot_incremental_scan as incremental

ROOT = Path(__file__).resolve().parents[1]
WORK = r'''
extern unsigned prune_skipped, prune_scanned;
static void workCases(){
 for(int level:{0,54,55,56,230}){
  auto pixels=blank(0);band(pixels,150,66,10,level);band(pixels,170,160,10,level);
  WhiteMarker1DTracker a(0),b(1);
  prune_skipped=prune_scanned=0;
  auto left=a.process(pixels.data());auto right=b.process(pixels.data());
  assert(left.templates_tested==17 && right.templates_tested==17);
  assert(prune_skipped+prune_scanned==34);
  if(level<55){assert(prune_skipped==34 && prune_scanned==0);}
  else {assert(prune_scanned>0 && prune_skipped>0);}
  std::cout<<"WORK contrast="<<level<<" templates=34 skipped="<<prune_skipped
           <<" scanned="<<prune_scanned<<"\n";
 }
}
'''

def main():
    with tempfile.TemporaryDirectory(prefix='foot_prune_') as folder:
        path = Path(folder)
        (path/'legacy_tracker.h').write_text('#pragma once\n#include "white_marker_tracker.h"\n'+oracle.LEGACY_HEADER, encoding='utf8')
        (path/'legacy.cpp').write_text(oracle.LEGACY_SOURCE, encoding='utf8')
        cpp = prepared.fixture().replace('int main(int argc,char** argv){', WORK+'\nint main(int argc,char** argv){\n workCases();')
        (path/'test.cpp').write_text(cpp, encoding='utf8')
        source = (ROOT/'src/white_marker_tracker.cpp').read_text(encoding='utf8')
        # Count only in the host copy; no counters or hooks enter firmware.
        needle = 'if(strongest<kPeakMin)return;'
        assert source.count(needle) == 1
        instrumented = 'unsigned prune_skipped=0,prune_scanned=0;\n'+source.replace(needle,
            'if(strongest<kPeakMin){++prune_skipped;return;}\n ++prune_scanned;')
        (path/'counted.cpp').write_text(instrumented, encoding='utf8')
        common = ['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                  '-DFOOT_LOW_LATENCY_V20=1','-DFOOT_SPEED_V22=1','-DFOOT_CAPTURE_V23=1',
                  '-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_TEMPLATE_PRUNE_V28=1',
                  '-DFOOT_IMAGE_ROWS_TEST_READS=1',
                  '-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'src'),'-I'+str(path)]
        exe = path/'oracle.exe'
        subprocess.run(common+[str(path/'test.cpp'),str(path/'legacy.cpp'),str(path/'counted.cpp'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        # Unmodified production source: incremental stages, missing rows,
        # abandoned/wrong frames and ownership are independently exercised.
        (path/'incremental.cpp').write_text(incremental.CPP,encoding='utf8')
        exe = path/'incremental.exe'
        subprocess.run(common+['-DFOOT_NATIVE_SKIP_V25=1','-DFOOT_NATIVE_STREAM_V26=1',
                              str(path/'incremental.cpp'),str(ROOT/'src/white_marker_tracker.cpp'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        print('PASS v28 frozen oracle, prepared-state lifetime, exact threshold and incremental production path')

if __name__ == '__main__':
    main()
