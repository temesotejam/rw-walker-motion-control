import csv
import tempfile
from pathlib import Path
from convert_rwlog_to_csv import expand_tables, write_energy_control_autonomous_events

fields=['foot_span_flags','physical_next_peak_side','long_foot_counterfactual_ms','long_foot_actual_ms','long_foot_effective']
row=[1<<14,1,22,25,True]
root={'firmware_revision':'1.0.95-positive-sway-probe','energy_control_autonomous_zero_cross_events':{'fields':fields,'rows':[row]}}
for version in ('1.0.95-positive-sway-probe','1.0.96-tapered-amplitude-recovery', '1.0.97-paired-sway-rollback', '1.0.98-foot-observation-timeout', '1.0.99-alternate-foot-trial'):
    metadata=expand_tables(dict(root,firmware_revision=version))
    with tempfile.TemporaryDirectory() as d:
        write_energy_control_autonomous_events(metadata,Path(d))
        with open(Path(d)/'energy_control_autonomous_zero_cross_events.csv',encoding='utf8') as f: out=list(csv.DictReader(f))[0]
        assert out['positive_sway_probe_baseline_ms']=='22' and out['positive_sway_probe_candidate_ms']=='25'
        assert out['positive_sway_probe_executed']=='True' and out['long_foot_actual_ms']==''
old=dict(root,firmware_revision='1.0.94-foot-trial-loss')
try: expand_tables(old)
except ValueError: pass
else: raise AssertionError('old firmware must reject bit14')
print('positive sway compact log version and CSV mapping PASS')
