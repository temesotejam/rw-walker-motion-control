"""Check causal forecast/assist metadata joins and old log compatibility."""
import csv
import importlib.util
import tempfile
from pathlib import Path

root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('converter',root/'tools/convert_rwlog_to_csv.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
raw={'foot_frames':[{'sequence':7,'run_id':2,'frame_us':100000,'delivered_us':150000,'measurement_time_us':100000}],
     'foot_pose_prediction':{'records':{'fields':['sequence','predicted_right_deg','ready_ms'],'rows':[[7,2.3,164]]}},
     'foot_support_assist':{'output_enabled':True,'records':{'fields':['sequence','time_ms','peak_ms','normal_width_ms','requested_width_ms','coast_assist','output_executed'],
                                                           'rows':[[7,180,80,0,15,True,False]]}}}
meta=c.expand_tables(raw)
with tempfile.TemporaryDirectory(prefix='foot_support_metadata_') as td:
    out=Path(td)
    assert c.write_foot_support_diagnostics({},out)==0
    assert c.write_foot_support_diagnostics(meta,out)==2
    with (out/'foot_pose_prediction.csv').open() as f:r=next(csv.DictReader(f))
    assert r['frame_us']=='100000' and r['predicted_right_deg']=='2.3' and r['ready_ms']=='164'
    with (out/'foot_support_assist.csv').open() as f:r=next(csv.DictReader(f))
    assert r['run_id']=='2' and r['normal_width_ms']=='0' and r['requested_width_ms']=='15'
    assert r['coast_assist']=='True' and r['output_executed']=='False'
    meta['foot_local_prediction']={'records':[{'sequence':7,'ready_ms':164,'local_evidence':1.0,'predicted_right_deg':3.8}]}
    assert c.write_foot_support_diagnostics(meta,out)==3
    with (out/'foot_local_prediction.csv').open() as f:r=next(csv.DictReader(f))
    assert r['run_id']=='2' and r['predicted_right_deg']=='3.8' and r['local_evidence']=='1.0'
    meta['foot_support_assist']['records'].append(dict(meta['foot_support_assist']['records'][0]))
    try:c.write_foot_support_diagnostics(meta,out)
    except ValueError:pass
    else:raise AssertionError('repeated candidate key accepted')
    meta['foot_support_assist']['records'].pop()
    meta['foot_pose_prediction']['records'][0]['sequence']=99
    try:c.write_foot_support_diagnostics(meta,out)
    except ValueError:pass
    else:raise AssertionError('unmatched frame accepted')
print('Foot forecast and assist compact metadata, identity joins and absent-old-block PASS')
