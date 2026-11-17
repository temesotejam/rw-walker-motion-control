"""Run the actual observer integration cases under the faster full-log profile."""
from pathlib import Path
import subprocess
import tempfile
import re
import test_foot_fast_pipeline as pipeline
ROOT=Path(__file__).resolve().parents[1]
def main():
    code='#include "foot_latency_histogram.h"\n'+pipeline.fixture()
    code=code.replace('assert(foot_fast_policy::kArchiveCapacity==1280);',
                      'assert(foot_fast_policy::kArchiveCapacity==1600 && foot_fast_policy::kWorkerPeriodUs==26000);')
    # Keep the existing >=2s calibration dwell:70*26ms is too short.
    # Shift only this scenario's warm-up and corresponding event indices.
    indices={'69':'89','70':'90','78':'98','81':'101'}
    for signature in ['static std::vector<Plan> scenario()', 'static void fullPipeline()']:
        section=pipeline.function(code,signature)
        code=code.replace(section,re.sub(r'\b(69|70|78|81)\b',lambda m:indices[m[0]],section))
    with tempfile.TemporaryDirectory(prefix='foot_worker26_') as temp:
        p=Path(temp);(p/'Arduino.h').write_text(pipeline.ARDUINO,encoding='utf8')
        (p/'pipeline.cpp').write_text(code,encoding='utf8')
        flags=['FOOT_LOW_LATENCY_V20','FOOT_SUPPORT_ASSIST','FOOT_SUPPORT_LOCAL','WALKING_RETURN_ENERGY',
               'FOOT_IMAGE_ROWS_TEST_READS','FOOT_SPEED_V22','FOOT_SPEED_FIX1','FOOT_CAPTURE_V23',
               'FOOT_ROW_OVERLAP_V24','FOOT_NATIVE_SKIP_V25','FOOT_NATIVE_STREAM_V26','FOOT_EXPORT_WEBFIX',
               'FOOT_COPY_DELTA_V27','FOOT_TEMPLATE_PRUNE_V28','FOOT_OVERLAP_ALWAYS_V29','FOOT_RECEIVE_DIAG_V30',
               'FOOT_HISTOGRAM_V32','FOOT_WORKER26_V34']
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',*['-D'+x+'=1' for x in flags],
            '-I'+str(p),'-I'+str(ROOT/'src'),str(p/'pipeline.cpp'),str(ROOT/'src/white_marker_tracker.cpp'),
            str(ROOT/'src/foot_angle_estimator.cpp'),'-o',str(p/'test.exe')],check=True)
        subprocess.run([str(p/'test.exe')],check=True)
    print('PASS v34 actual observer: 26ms scheduling,1600-row bounds, ownership, preview, receipt and native full-frame admission')
if __name__=='__main__':main()
