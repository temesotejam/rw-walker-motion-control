#!/usr/bin/env python3
import csv
import json
import math
import tempfile
from pathlib import Path

from analyze_energy_validation import analyze, inverse, required_width


def main():
    assert inverse(1,.01,.5,-.001) is None
    q=inverse(1.2,.038,.257,-.0004)
    assert q is not None and q<0
    assert abs(.038*1.2*(q*.001)+.257*(q*.001)**2+.0004)<1e-12
    assert required_width(-100,0,300,.07) is None
    assert required_width(0,0,300,.07)==0
    assert required_width(-.8,0,300,.07) is not None
    z=dict(event_kind='energy_control_zero_cross',zero_cross_time_ms=5000,
           physical_next_peak_side=-1,valid=True,target_energy_j=.00073342,
           passive_energy_j=.00112589,integral_energy_j=-.00001078,
           work_omega_q_coefficient=.0388967425,work_q2_coefficient=.2578004825,
           zero_cross_abs_rate_dps=69.2887,predicted_energy_j=.00108748,
           pulse_start_ms=5000,pulse_end_ms=5020,pulse_width_ms=20,
           pre_input_capture_time_us=1000000,q_unclamped_mA_s=-.8203,
           q_command_mA_s=-.8203,pre_input_wheel_speed_rpm=212.53,
           vbat_mV=7820,target_peak_deg=8,q_saturated_upper=False,q_saturated_lower=True,
           predicted_next_peak_amplitude_deg=10.19)
    p=dict(peak_time_ms=5200,physical_peak_side=-1,pending_command_matched=True,
           measured_energy_j=.00108139,peak_amplitude_deg=10.15676)
    meta=dict(firmware_revision='1.0.5',energy_control_autonomous_zero_cross_events=[z],
              energy_control_autonomous_peak_events=[p,{**p,'physical_peak_side':1}],
              energy_control_autonomous_negative_side_brake_width_max_ms=20,
              peak_error_adaptation_enabled=True,terminal_state={'state':'FINISHED'})
    def sample(t,stamp,seq,current,q,active=1):
        return dict(t_test_ms=t,roller_current_sample_time_us=stamp,roller_current_sequence=seq,
                    roller_current_valid=1,pulse_active=active,roller_actual_current_mA=current,
                    roller_q_meas_observed_valid=1,roller_q_meas_observed_mA_s=q)
    samples=[sample(5000,999990,9,200,9),sample(5005,1005000,10,20,.02),
             sample(5008,1005000,10,20,.02),sample(5015,1015000,11,55,.4),
             sample(5025,1020000,12,60,.657,0)]
    with tempfile.TemporaryDirectory() as temp:
        folder=Path(temp)
        (folder/'metadata.json').write_text(json.dumps(meta),encoding='utf-8')
        with (folder/'timeseries.csv').open('w',encoding='utf-8',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(samples[0]));writer.writeheader();writer.writerows(samples)
        rows,summary=analyze(folder)
        assert len(rows)==1
        r=rows[0]
        # Old firmware's mislabeled, clipped Q must never be used as demand.
        assert abs(r['required_q_mAs']+9.018535)<.0001
        assert abs(r['unapplied_request_j']+.00036484)<1e-8
        assert r['current_unique_active_samples']==2
        assert r['current_max_sample_mA']==55
        assert r['current_observed_partial_q_mAs']==.657
        assert r['required_width_not_available']
        meta['energy_request_audit_revision']='unrestricted_inverse_v1_20261014'
        z.update(unclamped_energy_request_j=-.00040325,q_unclamped_mA_s=None,
                 unclamped_inverse_valid=False)
        (folder/'metadata.json').write_text(json.dumps(meta),encoding='utf-8')
        rows,_=analyze(folder)
        assert rows[0]['required_q_mAs'] is None
    print('energy validation: true demand, unavailable inverse, peak pairing, stale/duplicate current and partial Q PASS')


if __name__=='__main__':
    main()
