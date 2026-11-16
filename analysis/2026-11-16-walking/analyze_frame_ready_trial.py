"""Compare the first v21 measurement with both v20-downloadfix2 baselines."""
from pathlib import Path
import csv
import hashlib
import json
import statistics
import zlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
TRIAL = '36545387'


def rows(path):
    with path.open(encoding='utf8', newline='') as f:
        return list(csv.DictReader(f))


def quality(frames):
    return {'n':len(frames), 'left_missing':sum(not f['left_valid'] for f in frames),
            'right_missing':sum(not f['right_valid'] for f in frames),
            'either_missing':sum(not(f['left_valid'] and f['right_valid']) for f in frames)}


def main():
    p=HERE/TRIAL
    raw=Path('C:/Users/arika/Downloads/energy_control_autonomous_run_1_36545387.rwlog').read_bytes()
    h=json.loads((p/'header.json').read_text())
    assert len(raw)==h['crc_offset']+4 and zlib.crc32(raw[:-4])==int.from_bytes(raw[-4:],'little')
    comparisons=[]
    for trial in ['52370695','202970048',TRIAL]:
        folder=HERE/trial
        m=json.loads((folder/'metadata.json').read_text())
        f=[x for x in m['foot_frames'] if x['state_id']==3]
        latency=json.loads((folder/'latency/foot_latency_summary.json').read_text())['trials'][0]
        comparisons.append({'trial':trial,'build':m['diagnostic_build_id'],
            'latency_ms':latency['stages_active_ms'],'quality_all':quality(f),
            'quality_first_10s':quality([x for x in f if 0<=x['measurement_time_us']<10000000])})
    samples=rows(p/'timeseries.csv');assert len(samples)==h['sample_count']
    active=[x for x in samples if x['state_id']=='3']
    peaks=rows(p/'energy_control_autonomous_peak_events.csv')
    zeros=[x for x in rows(p/'energy_control_autonomous_zero_cross_events.csv') if x['event_kind']=='energy_control_zero_cross']
    outputs=[x for x in rows(p/'walking_energy.csv') if x['executed']=='True']
    last=outputs[-1];after=[float(x['pitch_mekf_abs_deg']) for x in active if float(x['time_s'])>float(last['output_end_ms'])/1000]
    jumps={}
    for side in ['left','right']:
        previous=None;jumps[side]=[]
        for x in f:
            if not x[side+'_valid']:continue
            if previous:
                dt=(x['frame_us']-previous['frame_us'])/1000
                delta=x[side+'_deg']-previous[side+'_deg']
                if dt<160 and abs(delta)>4:jumps[side].append({'time_s':x['measurement_time_us']/1e6,'dt_ms':dt,'delta_deg':delta})
            previous=x
    result={'trial':TRIAL,'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw),'crc_ok':True,
        'samples':len(samples),'active_samples':len(active),'duration_s':max(float(x['time_s']) for x in active),
        'terminal':m['terminal_state'],'dma':{k:v for k,v in m['foot_observation'].items() if k.startswith('dma_')},
        'comparisons':comparisons,'last_zero_ms':int(zeros[-1]['zero_cross_time_ms']),
        'last_peak':peaks[-1],'last_output':last,'post_output_angle_min_deg':min(after),'post_output_angle_max_deg':max(after),
        'last_second_angle_median_deg':statistics.median(float(x['pitch_mekf_abs_deg']) for x in active if float(x['time_s'])>=29),
        'jumps_over_4deg_within_160ms':jumps,'user_observation':'self-standing foot stop after about 11s, confirmed this turn',
        'limitations':'Driver time is not calibrated exposure. Counters are boot lifetime; one last full-frame age is not a distribution. No video or independent angle ground truth.'}
    (p/'frame_ready_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    fig,ax=plt.subplots(3,1,figsize=(11,8),constrained_layout=True)
    for i,c in enumerate(comparisons):
        ax[0].bar(i-.17,c['latency_ms']['driver_to_delivery_ms']['median'],width=.34,color='tab:blue',label='Driver to delivery' if i==0 else None)
        ax[0].bar(i+.17,c['latency_ms']['driver_to_control_ms']['median'],width=.34,color='tab:orange',label='Driver to control' if i==0 else None)
    ax[0].set_xticks(range(3));ax[0].set_xticklabels(['v20 / 52370695','v20 / 202970048','v21 / 36545387']);ax[0].set_ylabel('Median delay (ms)');ax[0].legend()
    ax[1].plot([float(x['time_s']) for x in active],[float(x['pitch_mekf_abs_deg']) for x in active],lw=.8)
    ax[1].axhline(0,color='k',lw=.5);ax[1].set_ylabel('Body lateral angle (deg)')
    for side in ['left','right']:
        ax[2].plot([x['measurement_time_us']/1e6 for x in f],[x[side+'_deg'] if x[side+'_valid'] else float('nan') for x in f],label=side,lw=.9)
    for a in ax[1:]:a.axvline(float(last['time_ms'])/1000,color='r',ls='--',label='Last input');a.set_xlim(0,30);a.grid(alpha=.2)
    ax[2].set_ylabel('Observed foot angle (deg)');ax[2].set_xlabel('Measurement time (s)');ax[2].legend()
    fig.suptitle('v21 first trial: no meaningful latency reduction; foot stop confirmed')
    fig.savefig(p/'frame_ready_comparison.png',dpi=150);plt.close(fig)
    print(json.dumps({k:result[k] for k in ['trial','sha256','crc_ok','samples','last_zero_ms','last_second_angle_median_deg','dma']},indent=2))


if __name__=='__main__':main()
