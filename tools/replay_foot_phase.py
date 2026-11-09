#!/usr/bin/env python3
"""Replay logged foot/peak observation only; does not model motor intervention.

Timing approximations: peak notification is peak-time + --peak-delay-ms;
camera notification is delivered_us + processing_us + --poll-delay-ms. The
camera frame and peak times are both relative to measurement start. The frame
timestamp's relationship to exposure has not been independently checked. Vary the two
delays to test sensitivity; the actual peak-confirmation timestamps are not
recorded in this export. This script refuses corrupted input.
"""
import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path

from convert_rwlog_to_csv import expand_tables, parse_header, verify_crc

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('rwlog',type=Path)
    parser.add_argument('observer_executable',type=Path)
    parser.add_argument('--peak-delay-ms',type=int,default=100)
    parser.add_argument('--poll-delay-ms',type=int,default=25)
    args=parser.parse_args()
    if args.peak_delay_ms<0 or args.poll_delay_ms<0: parser.error('delays must be nonnegative')
    data=args.rwlog.read_bytes();header=parse_header(data)
    if not verify_crc(data,header): raise ValueError('RWLOG CRC mismatch')
    if header['crc_offset']+4!=len(data): raise ValueError('unexpected trailing bytes')
    start=header['header_size'];end=start+header['metadata_json_size']
    if end!=header['samples_offset'] or end>header['crc_offset']: raise ValueError('invalid metadata boundaries')
    meta=expand_tables(json.loads(data[start:end].decode('utf-8')))
    frames=meta.get('foot_frames',[]);peaks=meta.get('energy_control_autonomous_peak_events',[])
    events=[];run_id=None
    for f in frames:
        m=f.get('measurement_time_us');delivered=f.get('delivered_us');exposure=f.get('frame_us')
        rid=f.get('run_id');processing=f.get('processing_us',0)
        if not all(isinstance(x,int) for x in (m,delivered,exposure,rid,processing)) or m<0 or not f.get('timestamp_valid'):continue
        if run_id is None:run_id=rid
        if rid!=run_id:raise ValueError('mixed run IDs')
        if delivered<exposure or processing<0:continue
        frame_ok=bool(f.get('frame_valid') and f.get('state_id')==3)
        lv=frame_ok and f.get('left_reason')=='detected';rv=frame_ok and f.get('right_reason')=='detected'
        lx=f.get('left_x');rx=f.get('right_x')
        if not isinstance(lx,(int,float)) or not math.isfinite(lx):lv=False;lx=0
        if not isinstance(rx,(int,float)) or not math.isfinite(rx):rv=False;rx=0
        at=(m+delivered-exposure+processing)//1000+args.poll_delay_ms
        events.append((at,0,f"B {at} {m//1000} {int(lv)} {int(rv)} {lx} {rx}"))
    for p in peaks:
        ms=p.get('peak_time_ms');side=p.get('physical_peak_side')
        if not isinstance(ms,int) or ms<0 or side not in (-1,1):continue
        at=ms+args.peak_delay_ms
        events.append((at,1,f"P {at} {ms} {side}"))
    if run_id is None or not events:raise ValueError('no measurement-time foot frames/peaks')
    events.sort()
    lines=[f'R {run_id}']+[line for _,__,line in events]
    result=subprocess.run([str(args.observer_executable.resolve())],input='\n'.join(lines)+'\n',
                          check=True,capture_output=True,text=True)
    print(f"input_sha256={hashlib.sha256(data).hexdigest()} firmware={meta.get('firmware_revision')}")
    print(f'peak_delay_ms={args.peak_delay_ms} poll_delay_ms={args.poll_delay_ms}')
    print('notification_ms cycle_start_ms cycle_end_ms cycle_count left_mean_deg right_mean_deg left_latest_deg right_latest_deg valid')
    print(result.stdout,end='')
    print('Historical observations only; no claim about post-intervention dynamics.')

if __name__=='__main__':main()
