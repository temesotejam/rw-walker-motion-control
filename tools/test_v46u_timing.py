#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,subprocess,tempfile,copy
from v46u_timing_contract import ROOT,original_timing_file
from v46ac_delay_comp_contract import normalize_runner as normalize_v46ac_runner
from v46ab_no_prediction_contract import normalize_runner as normalize_v46ab_runner
from v46aa_control_zero_contract import normalize_runner as normalize_v46aa_runner
from v46z_comparison_zero_contract import normalize_runner as normalize_v46z_runner
from check_v46u_deadlines import assess
from v46ak_observation_contract import normalize_runner as normalize_v46ak_runner
from v46al_control_contract import normalize_runner as normalize_v46al_runner

def main():
    assert hashlib.sha256(normalize_v46z_runner(normalize_v46aa_runner(normalize_v46ab_runner(normalize_v46ac_runner(normalize_v46al_runner(normalize_v46ak_runner((ROOT/'src/experiment_runner.cpp').read_text())))))).encode()).hexdigest()=='2af5934f994c9c435ca53f4ef65ec98a5a95a4bd3355a740c0f1001e290f8c0e'
    baseline=json.loads((ROOT/'tools/v46s_audit_baseline.json').read_text())
    for p,h in baseline['protected'].items():
        assert hashlib.sha256(original_timing_file(p)).hexdigest()==h,p
    for token in ['BMI270_GYRO_ODR_HZ = 400','BMI270_ACCEL_ODR_HZ = 200','IMU_POLL_PERIOD_US = 1000UL','CURRENT_AUDIT_FAST_READ_PERIOD_US = 1000UL','BMI270_I2C_HZ = 1000000UL']:
        assert token in (ROOT/'src/config.h').read_text(),token
    imu=(ROOT/'src/imu_manager.cpp').read_text(); roller=(ROOT/'src/roller485_manager.cpp').read_text()
    assert 'M5.Imu.setClock(Config::BMI270_I2C_HZ);' in imu
    assert 'current_already_fresh = readCurrentFresh(true);' in roller
    assert 'current_already_fresh || readCurrentFresh(command_mA_ != 0)' in roller
    with tempfile.TemporaryDirectory() as d:
        exe=Path(d)/'reader'
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src'),str(ROOT/'tools/test_v46u_reader.cpp'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        cpp=Path(d)/'counters.cpp';cpp.write_text(r'''
#include "Arduino.h"
#include "run_control_worker.h"
#include "imu_poll_profile.h"
#include <iostream>
int main(){
  RunControlWorker w;
  w.recordSampleCompletion(false,true,0,9999,9000);
  w.recordSampleCompletion(true,false,0,9999,9000);
  w.recordSampleCompletion(true,true,0xffffff00U,2244U,2500);
  w.recordSampleCompletion(true,true,100,2601,2501);
  std::cout<<w.diagnosticsJson().c_str()<<'\n';
  ImuPollProfile p;p.start(100);ImuPollObservation o;
  o.start_us=100;o.fresh_gyro=true;o.mask=2;o.dt_us=2500;o.total_us=1000;
  o.driver_called=true;o.driver_data_bytes=6;p.record(o);
  o.start_us=2600;o.dt_us=2501;o.total_us=1001;p.record(o);
  if(p.poll_work_deadline.count!=2 || p.poll_work_deadline.over!=1 || p.gyro_interval_deadline.over!=1 || p.driver_calls!=2)return 2;
}
''')
        exe=Path(d)/'counters'
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'tools/host_v46o'),'-I'+str(ROOT/'src'),str(cpp),'-o',str(exe)],check=True)
        out=subprocess.check_output([str(exe)],text=True);v=json.loads(out)['v46u_deadline']
        assert v['count']==2 and v['over_budget']==1 and v['max_us']==2501 and not v['all_observed_within_budget'],v
    assert assess({})['result']=='INCOMPLETE'
    m={'v46n_imu_acquisition':{'captured':2,'delivered':2,'fault':False,'queue_drops':0,'delivery_sequence_gaps':0,
          'v46q_poll_profile':{'polls':2,'v46u_timing':{'driver_calls':2,'poll_work_count':2,'host_gyro_interval_count':2,'driver_failures':0,'poll_work_over_budget':0,'host_gyro_interval_over_budget':0}}},
       'v46p_control_worker':{'v46u_deadline':{'count':2,'over_budget':0}},
       'v46u_current_timing':{'read_count':3,'interval_count':2,'scope':'since_boot','read_over_budget':0,'interval_over_budget':0}}
    assert assess(m)['imu_control_observed_pass'] and not assess(m)['overall_hard_realtime_certified']
    for changes in [('over_budget',1),('count',1),('count',0)]:
        bad=copy.deepcopy(m);bad['v46p_control_worker']['v46u_deadline'][changes[0]]=changes[1]
        assert not assess(bad)['imu_control_observed_pass']
    bad=copy.deepcopy(m);bad['v46n_imu_acquisition']['fault']=True
    assert not assess(bad)['imu_control_observed_pass']
    print('PASS: unchanged controller/estimators/ODR/safety; V46ac frozen 1MHz IMU + 1ms current audit guards; real deadline counters')
if __name__=='__main__':main()
