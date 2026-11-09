#!/usr/bin/env python3
from pathlib import Path
import json
import subprocess
import tempfile
from analyze_v46q_acquisition import analyze_metadata
R=Path(__file__).resolve().parents[1]
subprocess.run(['python3','tools/test_v46q_acquisition_source.py'],cwd=R,check=True)
with tempfile.TemporaryDirectory() as temp:
    for opt in ('-O2','-Os'):
        exe=Path(temp)/('test'+opt)
        subprocess.run(['g++','-std=c++17',opt,'-Wall','-Wextra','-Werror','-Itools/host_v46o',
                        'tools/test_v46q_lightweight.cpp','-o',str(exe)],cwd=R,check=True)
        out=subprocess.check_output([str(exe)],text=True)
        profile=json.loads(next(line.split('=',1)[1] for line in out.splitlines() if line.startswith('profile_json=')))
        data=analyze_metadata({'v46n_imu_acquisition':{'v46q_poll_profile':profile}})
        assert data['reader_profile']['stages']['update_api']['mean_us']==400
        assert len(data['reader_profile']['worst_gap_by_100ms'])==1
        assert len(profile['gap_columns'])==14
        print(opt, out.splitlines()[-1])
        for bad in ({}, {'v46n_imu_acquisition':{'v46q_poll_profile':{'available':False}}}):
            try: analyze_metadata(bad)
            except ValueError: pass
            else: raise AssertionError('Missing/live profiles must not pass')
print('V46q JSON parser and profile column/count regression PASS')
