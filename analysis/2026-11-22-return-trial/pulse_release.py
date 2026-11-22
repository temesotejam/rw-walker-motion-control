"""Compare signed wheel current, RPM and body gyro around acknowledged zero-current commands."""
import argparse
from bisect import bisect_left
import csv
import json
from pathlib import Path


def trace(folder):
    meta = json.loads((folder/'metadata.json').read_bytes())
    epoch, = {f['frame_us']-f['measurement_time_us'] for f in meta['foot_frames']
              if f['state_id'] == 3 and f['timestamp_valid']}
    channels = {
        'command': ('roller_applied_command_time_us','roller_applied_current_mA', None),
        'current': ('roller_current_sample_time_us','roller_actual_current_mA','roller_current_valid'),
        'rpm': ('roller_speed_sample_time_us','roller_speed_rpm','roller_speed_valid'),
        'body': ('body_rate_sample_time_us','gy_dps',None),
    }
    unique = {k:{} for k in channels}
    with (folder/'timeseries.csv').open(encoding='utf8',newline='') as stream:
        for row in csv.DictReader(stream):
            if row['state_id'] != '3':
                continue
            for name,(clock,field,valid) in channels.items():
                if valid and row[valid] != '1' or not row[clock] or not row[field]:
                    continue
                stamp = int(row[clock])
                ms = ((stamp-epoch) & 0xffffffff)/1000
                if not stamp or ms > 60000 or ms in unique[name]:
                    continue
                value = float(row[field])
                if name == 'body':
                    value = (value-float(row['mekf_bias_y_dps']))*meta['mekf_gyro_y_scale']
                unique[name][ms] = value
    return meta, {k:sorted(v.items()) for k,v in unique.items()}


def charge(points,start,end):
    """Signed driver readback integral (mA s), linearly bracketed at both ends."""
    times = [t for t,_ in points]
    if not times or start < times[0] or end > times[-1]:
        return None
    def at(t):
        j = bisect_left(times,t)
        if times[j] == t:
            return points[j][1]
        (t0,y0),(t1,y1) = points[j-1:j+1]
        return y0+(y1-y0)*(t-t0)/(t1-t0)
    inside = [(start,at(start)), *[(t,y) for t,y in points if start<t<end], (end,at(end))]
    return sum((b-a)*(y+z)*.0005 for (a,y),(b,z) in zip(inside,inside[1:]))


def analyze(folder):
    meta, sample = trace(folder)
    events = [w for w in meta['walking_energy']['records'] if w['executed']]
    zeros = [t for t,v in sample['command'] if v == 0]
    out = []
    for e in events:
        off = min(zeros,key=lambda t: abs(t-e['output_end_ms']))
        on,cmd = min(((t,v) for t,v in sample['command'] if v),key=lambda tv: abs(tv[0]-e['output_start_ms']))
        if max(abs(on-e['output_start_ms']), abs(off-e['output_end_ms'])) > 8 or on >= off:
            raise ValueError('No matching acknowledged command pair')
        next_on = next((t for t,v in sample['command'] if t>off and v),float('inf'))
        side = 1 if cmd>0 else -1
        relative = {k:[(t-off,v) for t,v in values if on-40 <= t < min(off+235, next_on)]
                    for k,values in sample.items()}
        def segment(channel,start,end):
            return [(t,v) for t,v in relative[channel] if start <= t < end]

        before = segment('current',on-off,0)
        after = segment('current',0,60)
        rpm_before = segment('rpm',on-off,0)
        rpm_after = segment('rpm',0,40)
        if len(before) < 1 or len(after) < 1 or not rpm_before or len(rpm_after)<2:
            raise ValueError('Insufficient trace coverage for release comparison')
        # The window ends at the next acknowledged nonzero command, including same-side commands.
        residual = [v for _,v in after]
        out.append(dict(run=folder.name, side=e['side'], return_aid=e['return_event'], on=on,off=off,
                        command_mA=cmd, last_on_mA=before[-1][1], post_mA=after,
                        on_charge_mA_s=charge(relative['current'],on-off,0),
                        next_100ms_charge_mA_s=charge(relative['current'],0,100),
                        all_post_same_sign=all(v*side>0 for v in residual),
                        post_current_decays=all(side*a >= side*b for a,b in zip(residual,residual[1:])),
                        post_wheel_delta_rpm=rpm_after[-1][1]-rpm_before[-1][1],
                        wheel_keeps_driving_sign=side*(rpm_after[-1][1]-rpm_before[-1][1])>0,
                        wheel_abs_speed_falls=abs(rpm_after[-1][1])<abs(rpm_before[-1][1]),
                        first_after_rpm=rpm_after[0], last_before_rpm=rpm_before[-1],
                        wheel_at_end=rpm_after[-1], relative=relative))
    return out


def draw(runs, output):
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Meiryo','DejaVu Sans']
    fig, axes = plt.subplots(3,2,figsize=(9.5,6.6),sharex='col',layout='constrained')
    for col,events in enumerate(runs):
        e = events[-1]
        for ax,(key,label,color) in zip(axes[:,col],[
                ('current','ドライバ電流（mA）','#8b542a'),
                ('rpm','ホイール（rpm）','#17658a'),('body','胴体角速度（°/s）','#28765b')]):
            t,v=zip(*e['relative'][key])
            ax.plot(t,v,'.-', color=color,linewidth=1.4,markersize=3)
            ax.axvspan(e['on']-e['off'],0,color='#ebbf69',alpha=.3)
            ax.axvline(0,color='#61595a',linestyle='--',linewidth=1)
            ax.axhline(0,color='#7e8385',linewidth=.7)
            ax.set(ylabel=label,xlim=(-51,220))
        axes[0,col].set_title(f"{e['run']}　{'左補助' if e['return_aid'] else '右主入力'}")
        axes[-1,col].set_xlabel('0 mA指令の受理から（ms）')
    fig.suptitle('最後のパルス終了後の実測　　黄色は通電指令')
    fig.savefig(output,dpi=135)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders',nargs='+',type=Path)
    parser.add_argument('--plot',type=Path)
    args = parser.parse_args()
    runs = [analyze(f) for f in args.folders]
    events = [e for run in runs for e in run]
    print('Total pulses excluding start kick:', len(events))
    for key in ('all_post_same_sign','post_current_decays','wheel_keeps_driving_sign','wheel_abs_speed_falls'):
        print(key, sum(e[key] for e in events))
    for events in runs:
        e=events[-1]
        print(e['run'], 'acknowledged on/off (ms):', e['on'],e['off'])
        print('Signed readback integrals (mA s), on / next 100 ms:',
              e['on_charge_mA_s'],e['next_100ms_charge_mA_s'])
        for n in ('current','rpm','body'):
            print(n,[(round(t,1),round(v,1)) for t,v in e['relative'][n]])
    if args.plot:
        if len(runs)!=2:
            parser.error('--plot requires the two run folders in experimental order')
        draw(runs,args.plot)
