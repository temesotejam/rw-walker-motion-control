"""Causal replay of compiled firmware foot estimators on recorded camera data.

Camera measurements are delayed inputs, never ground truth. Future camera
measurements are used only by the scorer. Both interpolated-target and direct
held-out-frame scoring are reported to expose interpolation/hold bias.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CASES = ['137267573', '267557373', '27908524', '64040782', '84815132', '57814129', '199015154']
QUALITY = ['timestamp_valid', 'frame_valid', 'zero_ready', 'left_valid', 'right_valid', 'left_in_range', 'right_in_range']
CPP = r'''
#include "foot_phase_prediction.h"
#include "foot_pose_prediction.h"
#include <cmath>
#include <iostream>
#include <iomanip>
int main() {
  foot_phase_prediction::Predictor candidate;
  foot_pose_prediction::Predictor previous;
  foot_pose_prediction::Result old;
  unsigned kind,id,run,frame,now,valid,lead;
  float l=0,r=0;unsigned last_frame=0;bool hold_ready=false;
  std::cout<<std::setprecision(10);
  while(std::cin>>kind>>id>>run>>frame>>now>>valid>>l>>r>>lead) {
    if(kind==0) {
      candidate.observe(run,frame,now,valid,l,r);
      old=previous.observe(run,frame,now,valid,l,r,0);
      hold_ready=valid && frame<=now;
      last_frame=frame;
      continue;
    }
    auto q=candidate.predict(now,lead);
    const unsigned age=now>=last_frame ? now-last_frame : 100000;
    const unsigned horizon=age+lead;
    bool h=hold_ready && age<=180 && lead<=50 && horizon<=200;
    bool o=h && old.ready();
    // The old header's recorded backward difference is projected to each
    // actual query time, rather than resetting it through duplicate observe.
    const float old_l=l+old.left_rate*horizon*.001f;
    const float old_r=r+old.right_rate*horizon*.001f;
    std::cout<<id<<' '<<int(q.reason)<<' '<<q.ready()<<' '
      <<q.left<<' '<<q.right<<' '<<q.confidence<<' '
      <<int(q.phase_left)<<' '<<int(q.phase_right)<<' '
      <<h<<' '<<l<<' '<<r<<' '<<o<<' '<<old_l<<' '<<old_r<<' '
      <<age<<' '<<horizon<<' '<<q.left_uncertainty<<' '<<q.right_uncertainty<<'\n';
  }
}
'''


def read(path):
    with path.open(encoding='utf8', newline='') as f:
        return list(csv.DictReader(f))


def good(row):
    try:
        return all(row[k] == 'True' for k in QUALITY) and all(math.isfinite(float(row[k])) for k in ['left_deg', 'right_deg'])
    except (ValueError, KeyError):
        return False


def metrics(rows):
    out = {'scored_angle_values': len(rows)}
    common = [r for r in rows if r['candidate_ready'] and r['old_ready'] and r['hold_ready']]
    out['common_angle_values'] = len(common)
    out['coverage'] = {name: sum(r[name + '_ready'] for r in rows) / len(rows) if rows else None for name in ['candidate', 'old', 'hold']}
    for subset_name, subset in [('common', common), ('reversal_common', [r for r in common if r['reversal_near_target']]),
                                ('candidate_history_common', [r for r in common if r['confidence'] > 0]),
                                ('candidate_confidence_ge_0_2_common', [r for r in common if r['confidence'] >= .2])]:
        group = {'angle_values': len(subset)}
        for name in ['candidate', 'old', 'hold']:
            e = [r[name + '_abs_error_deg'] for r in subset]
            group[name] = {'mae_deg': float(np.mean(e)), 'p95_deg': float(np.percentile(e, 95)), 'max_deg': max(e)} if e else None
        out[subset_name] = group
    direction = [r for r in common if r['confidence'] >= .2 and r['candidate_phase'] in [2,4]
                 and r['camera_centered_rate_dps'] is not None and abs(r['camera_centered_rate_dps']) >= .8]
    out['coarse_direction_agreement'] = {
        'angle_values': len(direction),
        'fraction': sum((r['candidate_phase'] == 2) == (r['camera_centered_rate_dps'] > 0) for r in direction)/len(direction) if direction else None,
        'definition': 'candidate Forward/Returning, confidence >=.2, compared to future camera central difference (not independent velocity truth)'}
    uncertainty = [r for r in common if r['candidate_uncertainty_deg'] is not None]
    out['heuristic_uncertainty_coverage'] = {
        'angle_values': len(uncertainty),
        'fraction_error_within_margin': sum(r['candidate_abs_error_deg'] <= r['candidate_uncertainty_deg'] for r in uncertainty)/len(uncertainty) if uncertainty else None,
        'definition': 'empirical fraction within heuristic degree margin; not a calibrated interval or guarantee'}
    return out


def evaluate(exe, directory):
    frames = read(directory / 'foot_angles.csv')
    cad_path = directory / 'cad_support_shadow.csv'
    cad = {int(r['sequence']): int(float(r['compute_us'])) for r in read(cad_path)} if cad_path.exists() else {}
    origin = min(int(r['frame_us']) for r in frames) - 1000000
    frame_times = [(int(r['frame_us']) - origin) // 1000 for r in frames]
    received_times = [(int(r['delivered_us']) + int(r['processing_us']) + cad.get(int(r['sequence']), 0) - origin + 999) // 1000 for r in frames]
    events = []
    queries = {}
    for i, (row, t, arrived) in enumerate(zip(frames, frame_times, received_times)):
        valid = good(row) and row['state_id'] == '3' and int(row['measurement_time_us']) >= 0
        l, r = [float(row[k]) if row[k] and math.isfinite(float(row[k])) else 0 for k in ['left_deg', 'right_deg']]
        events.append((arrived, 0, i, f"0 {i} {row['run_id']} {t} {arrived} {int(valid)} {l} {r} 0"))
        if not valid:
            continue
        # Query held-out actual camera samples using only earlier arrivals.
        for lead in [0, 30]:
            ident = len(queries)
            queries[ident] = {'truth_kind': 'direct_frame', 'target_ms': t, 'now_ms': t - lead, 'lead_ms': lead,
                              'reuse_ms': -1, 'truth_frame_index': i}
        # Query between frames at actual publication/reuse times, then score
        # future measurements by interpolation, with explicit caveats.
        for reuse in [0, 30, 60]:
            for lead in [0, 30]:
                ident = len(queries)
                queries[ident] = {'truth_kind': 'interpolated', 'target_ms': arrived + reuse + lead,
                                  'now_ms': arrived + reuse, 'lead_ms': lead, 'reuse_ms': reuse,
                                  'truth_frame_index': None}
    # Feed observation events at their completion time; all estimates for a
    # query are generated by C++ against only data that has arrived by then.
    for ident, q in queries.items():
        events.append((q['now_ms'], 1, ident, None))
    lines = []; sent_queries = 0
    latest = None
    for when, kind, ident, payload in sorted(events, key=lambda e: (e[0], e[1], e[2])):
        if kind == 0:
            lines.append(payload)
            latest = frames[ident]
        else:
            if latest is None:
                continue
            q = queries[ident]
            l, r = [float(latest[k]) if latest[k] and math.isfinite(float(latest[k])) else 0 for k in ['left_deg', 'right_deg']]
            lines.append(f"1 {ident} {latest['run_id']} 0 {when} 0 {l} {r} {q['lead_ms']}")
            sent_queries += 1
    proc = subprocess.run([str(exe)], input='\n'.join(lines) + '\n', capture_output=True, text=True, check=True)
    assert len(proc.stdout.splitlines()) == sent_queries, 'C++ replay did not consume all events'
    results = []
    for line in proc.stdout.splitlines():
        p = line.split()
        ident = int(p[0]); q = queries[ident]; target = q['target_ms']
        if q['truth_kind'] == 'direct_frame':
            j = q['truth_frame_index']; truth = [float(frames[j][s + '_deg']) for s in ['left', 'right']]
        else:
            j = bisect.bisect_left(frame_times, target)
            if not 0 < j < len(frames) or not all(good(frames[k]) and frames[k]['state_id'] == '3' for k in [j-1, j]) or not 0 < frame_times[j] - frame_times[j-1] <= 200:
                continue
            alpha = (target - frame_times[j-1]) / (frame_times[j] - frame_times[j-1])
            truth = [(1-alpha)*float(frames[j-1][s + '_deg']) + alpha*float(frames[j][s + '_deg']) for s in ['left', 'right']]
        for side_id, side in enumerate(['left', 'right']):
            # Reversal label is hindsight for analysis only, never a predictor
            # input. A sign change of adjacent observed increments is coarse.
            reversal = False; camera_rate = None
            if 0 < j < len(frames)-1 and all(good(frames[k]) for k in [j-1, j, j+1]):
                vals = [float(frames[k][side+'_deg']) for k in [j-1, j, j+1]]
                reversal = (vals[1]-vals[0])*(vals[2]-vals[1]) < 0
                if 0 < frame_times[j+1]-frame_times[j-1] <= 400:
                    camera_rate = (vals[2]-vals[0])*1000/(frame_times[j+1]-frame_times[j-1])
            row = {'case': directory.name, **q, 'side': side, 'actual_camera_deg': truth[side_id], 'reversal_near_target': reversal,
                   'camera_centered_rate_dps': camera_rate,
                   'candidate_reason': int(p[1]), 'candidate_ready': p[2] == '1', 'confidence': float(p[5]),
                   'candidate_phase': int(p[6+side_id]), 'hold_ready': p[8] == '1', 'old_ready': p[11] == '1',
                   'age_ms': int(p[14]), 'horizon_ms': int(p[15])}
            uncertainty = float(p[16+side_id])
            row['candidate_uncertainty_deg'] = uncertainty if math.isfinite(uncertainty) else None
            for name, index in [('candidate', 3), ('hold', 9), ('old', 12)]:
                value = float(p[index+side_id])
                row[name+'_deg'] = value if math.isfinite(value) else None
                row[name+'_abs_error_deg'] = abs(value-truth[side_id]) if math.isfinite(value) else None
            results.append(row)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=ROOT / 'analysis/2026-11-11-walking')
    parser.add_argument('--cases', nargs='+', default=DEFAULT_CASES)
    parser.add_argument('--out', type=Path, default=ROOT / 'analysis/2026-11-11-walking/foot_phase_prediction_comparison.json')
    parser.add_argument('--details', type=Path)
    args = parser.parse_args()
    all_rows = []
    candidate_bytes = (ROOT/'src/foot_phase_prediction.h').read_bytes()
    with tempfile.TemporaryDirectory(prefix='foot_phase_eval_') as td:
        cpp = Path(td) / 'replay.cpp'; exe = Path(td) / 'replay.exe'
        # Freeze the candidate while other agents integrate the live source.
        (Path(td)/'foot_phase_prediction.h').write_bytes(candidate_bytes)
        cpp.write_text(CPP, encoding='utf8')
        subprocess.run(['g++', '-std=c++11', '-O2', '-I'+str(ROOT/'src'), str(cpp), '-o', str(exe)], check=True)
        for case in args.cases:
            rows = evaluate(exe, args.base/case); all_rows.extend(rows)
    output = {'scope': 'Causal compiled C++ replay; future camera data is scorer only, not independent angular/contact ground truth. No motor-output simulation.',
              'arrival_model': 'delivered_us + processing_us + saved cad compute_us where present, rounded up to ms; publication/first-control scheduling latency not fully observed',
              'caveats': ['Linear interpolation between future camera samples can favor holding; direct_frame avoids that interpolation but uses the same recognition algorithm.',
                          'Original camera-driver frame timestamps have not been calibrated to exposure time.',
                          'Candidate, old linear predictor and hold are compared on identical jointly-ready target/side values; coverage is also reported.',
                          'Reversal labels are hindsight from three neighboring camera values and are only scoring strata.',
                          'Multiple targets from the same trajectory are correlated, not independent trials.',
                          'These are retrospective development recordings, not a prospectively held-out validation set.'],
              'candidate_header_sha256': hashlib.sha256(candidate_bytes).hexdigest(), 'cases': {}}
    for case in args.cases:
        rows = [r for r in all_rows if r['case'] == case]
        output['cases'][case] = {}
        for kind in ['direct_frame', 'interpolated']:
            for lead in [0,30]:
                reuses = [-1] if kind == 'direct_frame' else [0,30,60]
                for reuse in reuses:
                    group = [r for r in rows if r['truth_kind'] == kind and r['lead_ms'] == lead and r['reuse_ms'] == reuse]
                    output['cases'][case][f'{kind}_lead{lead}_reuse{reuse}'] = metrics(group)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf8')
    report = ['# Causal foot prediction comparison', '',
              'The candidate was evaluated by compiling the actual C++ header. Observations enter only at recorded image-delivery + recognition-processing + saved CAD-compute completion. Future camera data is used only by the scorer.', '',
              'Primary score: prediction at the timestamp of a not-yet-arrived camera frame, compared directly to that frame. This avoids linear interpolation of the target and still uses the same camera recognition algorithm, not independent angle truth.', '',
              '| Log | Common angle values | Candidate MAE | Hold MAE | Old linear MAE | Candidate p95 | Candidate max | Candidate coverage |',
              '|---|---:|---:|---:|---:|---:|---:|---:|']
    for case in args.cases:
        m=output['cases'][case]['direct_frame_lead0_reuse-1'];g=m['common']
        report.append(f"| {case} | {g['angle_values']} | {g['candidate']['mae_deg']:.4f} | {g['hold']['mae_deg']:.4f} | {g['old']['mae_deg']:.4f} | {g['candidate']['p95_deg']:.4f} | {g['candidate']['max_deg']:.4f} | {100*m['coverage']['candidate']:.1f}% |")
    wins=[c for c in args.cases if output['cases'][c]['direct_frame_lead0_reuse-1']['common']['candidate']['mae_deg'] < output['cases'][c]['direct_frame_lead0_reuse-1']['common']['hold']['mae_deg']]
    worse=[c for c in args.cases if c not in wins]
    report += ['', 'All angles are degrees. Common angle values include both feet and correlated targets from the same trajectory; they are not independent experiments.', '',
               f'The candidate has lower primary MAE than holding on {len(wins)} of these {len(args.cases)} recordings. Non-improved recordings: '+(', '.join(worse) if worse else 'none')+'. This is an internal retrospective diagnostic, not a prospective held-out trial, and does not establish walking improvement.', '',
               'Large errors remain. Confidence is an uncalibrated history-match score, not a bound on angle error or a contact probability. Near-reversal performance does not improve uniformly over holding.', '',
               'JSON also contains lead0/30ms, post-publication reuse0/30/60ms, common-sample coverage, confidence strata, reversal strata and coarse direction agreement. Publication-to-first-control scheduling latency is not fully recorded; camera frame timestamps are not exposure-calibrated. No intervention or ground-contact simulation is performed.', '',
               'Compiled candidate SHA-256: `'+output['candidate_header_sha256']+'`.']
    args.out.with_suffix('.md').write_text('\n'.join(report)+'\n',encoding='utf8')
    if args.details and all_rows:
        with args.details.open('w', encoding='utf8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(all_rows[0])); writer.writeheader(); writer.writerows(all_rows)
    for case in args.cases:
        for label in ['direct_frame_lead0_reuse-1', 'interpolated_lead30_reuse0']:
            m=output['cases'][case][label]
            print(case, label, 'common', m['common']['angle_values'], 'MAE', {name: m['common'][name]['mae_deg'] if m['common'][name] else None for name in ['candidate','old','hold']}, 'coverage', m['coverage'])


if __name__ == '__main__':
    main()
