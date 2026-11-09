#!/usr/bin/env python3
"""Export V46q reader-stage timing from a CRC-verified RWLOG or metadata JSON.
Only host timings are measured. Details are worst-per-100ms representatives,
not a complete list of missed sensor frames or all long acquisition intervals.
"""
import argparse
import csv
import json
from pathlib import Path

def analyze_metadata(meta):
    a=meta.get('v46n_imu_acquisition',meta)
    p=a.get('v46q_poll_profile',{})
    if not p.get('available'):
        raise ValueError('V46q profile unavailable: '+p.get('reason','missing field'))
    for key,columns in (('seconds','second_columns'),('worst_gap_by_100ms','gap_columns')):
        if any(len(row)!=len(p[columns]) for row in p[key]):
            raise ValueError('Invalid column count: '+key)
    summary={k:a.get(k) for k in ('revision','firmware_version','duration_us','captured',
        'delivered','dt_mean_us','dt_max_us','over_4ms','over_5ms','over_10ms',
        'delivery_age_mean_us','delivery_age_max_us','queue_drops','delivery_sequence_gaps','fault')}
    summary['reader_profile']=p
    summary['limits']=[
        'Wall times include preemption; update_api includes library work, not pure I2C bus duration.',
        'Notify age is from the latest callback snapshot, not the oldest coalesced wake or ISR alarm time.',
        'poll_total excludes profile accounting and overrun wait; those are reported separately.',
        'Gap details retain the worst per 100ms bucket; full long-gap counts are separate.',
        'FIFO/sensor clock loss count and sensor internal filter delay are not measured.',
        'Stage means use their own counts; conversion and publication do not happen on every poll.'
    ]
    return summary

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('input',type=Path)
    ap.add_argument('--output',type=Path,default=Path('v46q_analysis'))
    args=ap.parse_args()
    if args.input.suffix.lower()=='.rwlog':
        import convert_rwlog_to_csv as c
        raw=args.input.read_bytes();h=c.parse_header(raw)
        if not c.verify_crc(raw,h): raise ValueError('RWLOG CRC mismatch')
        meta=json.loads(raw[h['header_size']:h['header_size']+h['metadata_json_size']])
    else:
        meta=json.loads(args.input.read_text(encoding='utf-8'))
    summary=analyze_metadata(meta);p=summary['reader_profile']
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    for key,columns,name in (('seconds','second_columns','reader_per_second.csv'),
                            ('worst_gap_by_100ms','gap_columns','representative_gaps.csv')):
        with (args.output/name).open('w',newline='',encoding='utf-8') as f:
            writer=csv.writer(f);writer.writerow(p[columns]);writer.writerows(p[key])
    with (args.output/'stages.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(['stage','count','mean_us','max_us'])
        for name,s in p['stages'].items():writer.writerow([name,s['count'],s['mean_us'],s['max_us']])
    print('Saved stage timings and bounded gap representatives to',args.output)

if __name__=='__main__':
    main()
