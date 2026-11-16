"""Analyze two recovered logs; camera angles are observations, not contact truth."""
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

def rows(path):
    with path.open(encoding='utf8', newline='') as f:
        return list(csv.DictReader(f))

def extent(values):
    v = list(values)
    return dict(n=len(v), min=min(v), median=statistics.median(v), max=max(v)) if v else None

def analyze(trial, run):
    p = HERE / trial
    raw = Path(r'C:\Users\arika\Downloads') / f'energy_control_autonomous_run_{run}_{trial}.rwlog'
    data = raw.read_bytes()
    h = json.loads((p/'header.json').read_text())
    m = json.loads((p/'metadata.json').read_text())
    latency = json.loads((p/'latency/foot_latency_summary.json').read_text())['trials'][0]
    assert len(data) == h['crc_offset'] + 4
    assert zlib.crc32(data[:-4]) == int.from_bytes(data[-4:], 'little')
    samples = rows(p/'timeseries.csv')
    assert len(samples) == h['sample_count']
    s = [r for r in samples if r['state_id'] == '3']
    f = [r for r in m['foot_frames'] if r['state_id'] == 3]
    pk = rows(p/'energy_control_autonomous_peak_events.csv')
    events = rows(p/'energy_control_autonomous_zero_cross_events.csv')
    audits = rows(p/'walking_energy.csv')
    outputs = [r for r in audits if r['executed'] == 'True']
    final = outputs[-1]
    assert final['rescue'] == 'True'
    end = float(final['output_end_ms']) / 1000
    after = [r for r in s if float(r['time_s']) >= end]
    closest = min(after, key=lambda r: abs(float(r['pitch_mekf_abs_deg'])))
    valid_foot = lambda side: [r[side+'_deg'] for r in f if r[side+'_valid'] and r[side+'_deg'] is not None]
    epoch = f[0]['frame_us'] - f[0]['measurement_time_us']
    received = [r for r in f if r['control_observed_us'] and (r['control_observed_us']-epoch)/1000 <= float(final['time_ms'])]
    last_received = received[-1]
    final_details = {k: final[k] for k in ['time_ms','peak_ms','requested_ms','guarded_ms','return_evidence',
        'return_foot_age_ms','return_deceleration_dps2','request_j','output_work_pred_j',
        'unmet_after_command_j','envelope_after_ms','completion_reason','actual_elapsed_ms',
        'planned_duration_completed','completion_extrapolated','passive_rate_supported','work_rate_supported']}
    report = dict(trial=trial, source=str(raw), sha256=hashlib.sha256(data).hexdigest(), bytes=len(data), crc_ok=True,
        build=m['diagnostic_build_id'], terminal=m['terminal_state'], samples=len(samples), duration_s=max(float(r['time_s']) for r in s),
        metadata_bytes=m['metadata_json_final_bytes'], foot_rows=len(m['foot_frames']), active_frames=len(f),
        energy_rows=m['walking_energy']['stored'], energy_rows_total=m['walking_energy']['total'], energy_truncated=m['walking_energy']['truncated'],
        peak_count=len(pk), last_peak=pk[-1], final_output=final_details,
        actual_zero_events=sum(r['event_kind']=='energy_control_zero_cross' for r in events),
        last_zero_ms=max(int(r['zero_cross_time_ms']) for r in events if r['event_kind']=='energy_control_zero_cross'),
        post_last_output_angle=extent(float(r['pitch_mekf_abs_deg']) for r in after),
        closest_to_zero_after_last_output={k:closest[k] for k in ['time_s','pitch_mekf_abs_deg','physical_roll_rate_dps']},
        last_second_angle=extent(float(r['pitch_mekf_abs_deg']) for r in s if float(r['time_s'])>=29),
        left_angle=extent(valid_foot('left')), right_angle=extent(valid_foot('right')),
        lost_left=sum(not r['left_valid'] for r in f), lost_right=sum(not r['right_valid'] for r in f),
        either_lost=sum(not (r['left_valid'] and r['right_valid']) for r in f),
        last_received_foot_at_final_decision={k:last_received[k] for k in ['measurement_time_us','control_observed_us','left_deg','right_deg','left_valid','right_valid']},
        latency_ms=latency['stages_active_ms'], latency_checks=latency['checks'],
        io_failures=m['control_latency']['io_failures'], control_done=m['control_latency']['done_age'],
        user_observation='self-standing foot stop confirmed' if run==1 else 'physical end state not separately confirmed; no video',
        interpretation='one recovered file proves completed export, not a diagnosis of earlier reset or continuous forward walking')
    (p/'trial_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    t=[float(r['time_s']) for r in s]
    angle=[float(r['pitch_mekf_abs_deg']) for r in s]
    fig,ax=plt.subplots(4,2,figsize=(14,10),sharex='col',constrained_layout=True)
    for col in [0,1]:
        ax[0,col].plot(t,angle,label='MEKF lateral angle',lw=.9)
        ax[0,col].axhline(0,color='k',lw=.6)
        ax[0,col].scatter([int(r['peak_time_ms'])/1000 for r in pk],
            [int(r['physical_peak_side'])*float(r['peak_amplitude_deg']) for r in pk],s=12,c='k')
        for side,color in [('left','tab:orange'),('right','tab:green')]:
            # Camera frame timestamp; invalid observations deliberately remain gaps.
            ax[1,col].plot([r['measurement_time_us']/1e6 for r in f],
                [r[side+'_deg'] if r[side+'_valid'] else float('nan') for r in f],label=side,color=color,lw=1)
        ax[2,col].plot(t,[float(r['physical_roll_rate_dps']) for r in s],lw=.8)
        for r in outputs:
            start=float(r['output_start_ms'])/1000; stop=float(r['output_end_ms'])/1000
            ax[3,col].plot([start,stop],[int(r['side'])*float(r['actual_elapsed_ms'])]*2,
                c='tab:red' if r['rescue']=='True' else 'tab:blue',lw=3)
        ax[3,col].axhline(0,color='k',lw=.6)
        for row in range(4):
            ax[row,col].axvline(float(final['time_ms'])/1000,color='tab:red',ls='--',lw=.8)
            ax[row,col].grid(alpha=.2)
        ax[3,col].set_xlabel('Measurement time (s)')
    ax[0,0].set_title('Full record');ax[0,1].set_title('Last return aid and failed crossing')
    for i,label in enumerate(['Body angle (deg)','Observed foot angle (deg)','Body rate (deg/s)','Signed pulse duration (ms)']):ax[i,0].set_ylabel(label)
    ax[1,0].legend();ax[3,0].set_xlim(0,30);ax[3,1].set_xlim(end-1.1,end+2.2)
    fig.suptitle(f'{trial}: downloadfix2 / CRC OK / camera gaps are not interpolated')
    fig.savefig(p/'trial_timeline.png',dpi=150);plt.close(fig)
    return report

if __name__=='__main__':
    for trial,run in [('52370695',1),('202970048',2)]:
        r=analyze(trial,run)
        print(json.dumps({k:r[k] for k in ['trial','build','crc_ok','samples','duration_s','last_zero_ms',
            'closest_to_zero_after_last_output','last_second_angle','final_output']},indent=2))
