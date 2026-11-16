"""Review the second v21 log and the selected scan-template heights."""
from pathlib import Path
import csv
import hashlib
import json
import statistics
import zlib

HERE=Path(__file__).resolve().parent
TRIAL='174064087'


def main():
    p=HERE/TRIAL
    raw=Path('C:/Users/arika/Downloads/energy_control_autonomous_run_1_174064087.rwlog').read_bytes()
    h=json.loads((p/'header.json').read_text())
    assert len(raw)==h['crc_offset']+4 and zlib.crc32(raw[:-4])==int.from_bytes(raw[-4:],'little')
    m=json.loads((p/'metadata.json').read_text())
    def rows(name):
        with (p/(name+'.csv')).open(encoding='utf8',newline='') as stream:return list(csv.DictReader(stream))
    samples=rows('timeseries');assert len(samples)==h['sample_count']
    s=[x for x in samples if x['state_id']=='3']
    f=[x for x in m['foot_frames'] if x['state_id']==3]
    a=[x for x in rows('walking_energy') if x['executed']=='True']
    z=[x for x in rows('energy_control_autonomous_zero_cross_events') if x['event_kind']=='energy_control_zero_cross']
    peaks=rows('energy_control_autonomous_peak_events');last=a[-1]
    after=[float(x['pitch_mekf_abs_deg']) for x in s if float(x['time_s'])>float(last['output_end_ms'])/1000]
    quality={}
    for side in ['left','right']:
        valid=[x for x in f if x[side+'_valid']]
        y=[x[side+'_scan_y'] for x in valid]
        quality[side]={'missing':len(f)-len(valid),'selected_template_center_y':{'min':min(y),'median':statistics.median(y),'max':max(y)},
            'templates_tested':sorted(set(x[side+'_templates'] for x in f))}
    def window_quality(end):
        frames=[x for x in f if 0<=x['measurement_time_us']<end*1e6]
        return {'n':len(frames),'either_missing':sum(not(x['left_valid'] and x['right_valid']) for x in frames)}
    latency=json.loads((p/'latency/foot_latency_summary.json').read_text())['trials'][0]
    result={'trial':TRIAL,'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest(),'crc_ok':True,
        'build':m['diagnostic_build_id'],'terminal':m['terminal_state'],'samples':len(samples),'active_samples':len(s),
        'duration_s':max(float(x['time_s']) for x in s),'foot_total':len(m['foot_frames']),'active_foot':len(f),
        'quality':quality,'first_7s_quality':window_quality(7),
        'all_quality':{'n':len(f),'either_missing':sum(not(x['left_valid'] and x['right_valid']) for x in f)},
        'dma':{k:v for k,v in m['foot_observation'].items() if k.startswith('dma_')},
        'stage_ms':latency['stages_active_ms'],'latency_checks':latency['checks'],
        'recognition_us_median':statistics.median(x['recognition_us'] for x in f),
        'row_copy_us_median':statistics.median(x['row_copy_us'] for x in f),
        'last_zero_ms':int(z[-1]['zero_cross_time_ms']),'last_peak':peaks[-1],'last_output':last,
        'post_output_angle_min_deg':min(after),'post_output_angle_max_deg':max(after),
        'last_second_angle_median_deg':statistics.median(float(x['pitch_mekf_abs_deg']) for x in s if float(x['time_s'])>=29),
        'physical_state':'User confirmed self-standing foot stop after about 8s in this trial.',
        'limits':'scan_y is selected template center, not true marker centroid. Full images are not in RWLOG; cannot replay a one-row detector or establish angle accuracy.'}
    (p/'line_budget_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:result[k] for k in ['trial','bytes','sha256','build','quality','all_quality','recognition_us_median','row_copy_us_median','last_zero_ms','last_second_angle_median_deg']},indent=2))


if __name__=='__main__':main()
