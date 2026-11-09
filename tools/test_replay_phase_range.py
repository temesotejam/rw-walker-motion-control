#!/usr/bin/env python3
"""Exercise the real raw phase gate through replay's recorded metadata."""
from replay_phase_cap import events_for
import subprocess
import sys

frame={'measurement_time_us':1000,'frame_us':2000,'delivered_us':2200,
       'processing_us':100,'timestamp_valid':True,'frame_valid':True,'state_id':3,'run_id':1,
       'left_reason':'detected','right_reason':'detected','left_x':202.5,'right_x':204.0,
       'zero_ready':False,'left_valid':False,'right_valid':False,'left_deg':None,'right_deg':None}
def encoded(f):
    m={'led_sync_start_total_ms':0,'foot_frames':[f],
       'energy_control_autonomous_peak_events':[],'energy_control_autonomous_zero_cross_events':[]}
    e=events_for({'run_id':1},m,0,0)
    return e[0][2] if e else None
def flags(**updates):
    line=encoded({**frame,**updates})
    if line is None:return None
    tokens=line.split();assert tokens[0]=='B'
    return int(tokens[3]),int(tokens[4])
assert flags()==(1,1)
assert flags(zero_ready=True,left_valid=True,right_valid=True)==(1,1)
assert flags(left_reason='ambiguous')==(0,1)
assert flags(right_reason='no_frame')==(1,0)
assert flags(left_x=float('nan'))==(0,1)
assert flags(right_x=None)==(1,0)
assert flags(frame_valid=False)==(0,0)
assert flags(state_id=6)==(0,0)
assert flags(timestamp_valid=False) is None
assert flags(run_id=2) is None

def replay(left_x,right_x,zero_change=False,**updates):
    lines=['R 1']
    for i,side in enumerate((-1,1,-1,1,-1)):
        t=1000+i*400;lines.append(f'P {t} {t} {side}')
        for offset,active in [(10,True),(110,False)]:
            f={**frame,**updates,'measurement_time_us':(t+offset)*1000,
               'frame_us':(t+offset)*1000,'delivered_us':(t+offset)*1000,'processing_us':0,
               'left_x':left_x,'right_x':right_x,
               'left_reason': updates.get('left_reason','detected') if active else 'no_frame',
               'right_reason':updates.get('right_reason','detected') if active else 'no_frame'}
            if zero_change and i>=2:
                f.update(zero_ready=True,left_valid=True,right_valid=True,left_deg=99,right_deg=-99)
            line=encoded(f)
            if line:lines.append(line)
    result=subprocess.run([sys.argv[1]],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    return [s.split() for s in result.stdout.splitlines()]

assert len(sys.argv)==2, 'pass compiled test_foot_phase_run_replay executable'
base=replay(201.6569+3,203.0819+3)
assert len(base)==2 and base==replay(201.6569+3,203.0819+3,zero_change=True)
assert len(replay(63.6918-3,67.5192-3))==2
assert len(replay(201.6569+3.01,203.0819))==2
assert len(replay(201.6569,203.0819+3.01))==2
assert len(replay(201.6569+20,203.0819))==2
assert len(replay(205,203.0))==2 and len(replay(201.0,205))==2
assert not replay(float('nan'),203.0819)
assert not replay(201.6569,float('nan'))
assert not replay(None,203.0819)
assert not replay(201.6569,203.0819,frame_valid=False)
assert not replay(201.6569,203.0819,right_reason='ambiguous')
assert not replay(201.6569,203.0819,timestamp_valid=False)
assert not replay(201.6569,203.0819,run_id=2)
print('raw phase replay: full image, detection, run/time and zero independence PASS')
