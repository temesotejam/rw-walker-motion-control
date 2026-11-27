"""Replay v38 on recorded first receipts. Run the compiled test_periodic_foot executable."""
import argparse
import bisect
import json
import math
import subprocess
from pathlib import Path


def replay(folder, exe):
    frames = json.loads((folder/'metadata.json').read_text(encoding='utf8'))['foot_frames']
    active = [f for f in frames if f['state_id']==3]
    frames = [f for f in active if f['control_observed_us']]
    epoch = frames[0]['frame_us'] - frames[0]['measurement_time_us']
    events = []
    for f in frames:
        t = f['control_observed_us']
        values = [f['sequence'], f['frame_us'], int(f['timestamp_valid'] and f['zero_ready'])]
        for side in ['left','right']:
            values.extend([int(f[side+'_valid']),int(f[side+'_control_range_ok']),f[side+'_deg'] or 0])
        events.append((t,0,'F '+str(t)+' '+' '.join(map(str,values))))
    for tick in range(epoch+25000,frames[-1]['control_observed_us']+1,25000):
        events.append((tick,1,'P '+str(tick)))
    stream = str(epoch)+'\n'+'\n'.join(x[2] for x in sorted(events))
    output = subprocess.run([exe],input=stream,text=True,capture_output=True,check=True).stdout.splitlines()
    count, missed, both, *_ = output[-1].split()
    assert int(count)==len(output) and int(missed)==0
    print(folder.name, 'scheduled',count,'both_estimates',both,'coverage_pct',round(100*int(both)/int(count),2))
    # Only the offline comparison uses later images. Compare both methods on
    # identical ticks bracketed by valid images at most 30 ms apart.
    captures = [f['frame_us'] for f in active]
    receipts = [f['control_observed_us'] for f in frames]
    for column, side in enumerate(('left','right'),3):
        errors, held_errors = [], []
        for tick, line in enumerate(output,1):
            now = epoch + 25000*tick
            value = float(line.split()[column])
            i = bisect.bisect_left(captures,now)
            if not math.isfinite(value) or i==0 or i==len(active):
                continue
            a,b = active[i-1:i+1]
            if not all(f[side+'_valid'] and f[side+'_control_range_ok'] for f in (a,b)):
                continue
            dt = b['frame_us']-a['frame_us']
            if not 0<dt<=30000:
                continue
            ref = a[side+'_deg']+(b[side+'_deg']-a[side+'_deg'])*(now-a['frame_us'])/dt
            j = bisect.bisect_right(receipts,now)-1
            while j>=0 and not frames[j][side+'_valid']:
                j-=1
            assert j>=0
            errors.append(abs(value-ref))
            held_errors.append(abs(frames[j][side+'_deg']-ref))
        print(side,'scored',len(errors),'MAE_est_deg',round(sum(errors)/len(errors),3),
              'MAE_hold_deg',round(sum(held_errors)/len(errors),3))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable',type=Path)
    parser.add_argument('folders',type=Path,nargs='+')
    args = parser.parse_args()
    for folder in args.folders:
        replay(folder,args.executable.resolve())
