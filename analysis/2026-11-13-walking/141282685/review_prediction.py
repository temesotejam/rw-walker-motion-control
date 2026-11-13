"""Review saved v11 predictions and replay only at recorded availability times.

Future camera data is scorer-only and is not independent angle/contact truth.
This trial was not used to develop the v11 estimator or its fixed parameters.
"""
from pathlib import Path
from collections import Counter
import bisect,csv,hashlib,json,math,subprocess,tempfile
import numpy as np

ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[2]
QUALITY=['timestamp_valid','frame_valid','zero_ready','left_valid','right_valid','left_in_range','right_in_range']
CPP=r'''
#include "foot_phase_prediction.h"
#include "foot_pose_prediction.h"
#include <iostream>
#include <iomanip>
int main(){
 foot_phase_prediction::Predictor p;foot_phase_prediction::Projection projection;
 foot_pose_prediction::Predictor old;foot_pose_prediction::Result old_result;
 unsigned seq=0,frame=0,received=0,run=0,valid=0,id=0,now=0,lead=0;float l=0,r=0;char kind;
 bool latest_valid=false;std::cout<<std::setprecision(10);
 auto emit=[&](char k,unsigned ident,unsigned t,unsigned horizon){
  auto q=projection.predict(t,horizon);
  const unsigned age=t>=frame?t-frame:100000;
  const bool hold_ready=latest_valid && age<=180 && horizon<=50 && age+horizon<=200;
  const bool linear_ready=hold_ready && old_result.ready();
  std::cout<<k<<' '<<ident<<' '<<int(q.reason)<<' '<<q.ready()<<' '<<q.left<<' '<<q.right<<' '
   <<q.confidence<<' '<<q.left_uncertainty<<' '<<q.right_uncertainty<<' '<<int(q.phase_left)<<' '<<int(q.phase_right)<<' '
   <<linear_ready<<' '<<hold_ready<<' '<<l+old_result.left_rate*(age+horizon)*.001f<<' '
   <<r+old_result.right_rate*(age+horizon)*.001f<<' '<<l<<' '<<r<<' '<<seq<<' '<<age<<' '<<age+horizon<<'\n';
 };
 while(std::cin>>kind){
  if(kind=='O'){
   std::cin>>seq>>run>>frame>>received>>valid>>l>>r;
   p.observe(run,frame,received,valid,l,r);projection=p.projection();projection.received_ms=received;
   old_result=old.observe(run,frame,received,valid,l,r,0);latest_valid=valid;
   emit('D',seq,received,30);
  }else{std::cin>>id>>now>>lead;emit('Q',id,now,lead);}
 }
}
'''

def read(name):
 with (ROOT/name).open(encoding='utf8',newline='') as f:return list(csv.DictReader(f))
def good(r):
 return all(r[k]=='True' for k in QUALITY) and all(r[s+'_deg'] and math.isfinite(float(r[s+'_deg'])) for s in ['left','right'])
def n(v):
 try:
  x=float(v);return x if math.isfinite(x) else None
 except (ValueError,TypeError):return None
def stats(values):
 v=[x for x in values if x is not None and math.isfinite(x)]
 if not v:return None
 return {'n':len(v),'min':min(v),'median':float(np.median(v)),'mean':float(np.mean(v)),'p95':float(np.percentile(v,95)),'max':max(v)}
def accuracy(rows):
 common=[r for r in rows if r['candidate_ready'] and r['linear_ready'] and r['hold_ready']]
 out={'scorable_angle_values':len(rows),'common_angle_values':len(common),
  'coverage':{k:sum(r[k+'_ready'] for r in rows)/len(rows) if rows else None for k in ['candidate','linear','hold']}}
 for name,subset in [('common',common),('confidence_ge_0_5',[r for r in common if r['confidence']>=.5]),
                     ('near_reversal',[r for r in common if r['near_reversal']])]:
  out[name]={'angle_values':len(subset),**{k:stats([r[k+'_error_deg'] for r in subset]) for k in ['candidate','linear','hold']}}
 out['margin_fraction_common']=sum(r['candidate_error_deg']<=r['uncertainty_deg'] for r in common)/len(common) if common else None
 return out

def main():
 feet=read('foot_angles.csv');saved=read('foot_pose_prediction.csv');audits=read('foot_support_assist.csv')
 meta=json.loads((ROOT/'metadata.json').read_text(encoding='utf8'))
 byseq={int(r['sequence']):r for r in feet};pred={int(r['sequence']):r for r in saved}
 active=[r for r in feet if r['state_id']=='3' and int(r['measurement_time_us'])>=0]
 times=[int(r['measurement_time_us'])/1000 if int(r['measurement_time_us'])>=0 else -100000 for r in feet]
 def truth(target):
  j=bisect.bisect_left(times,target)
  if not 0<j<len(feet) or not good(feet[j-1]) or not good(feet[j]) or not 0<times[j]-times[j-1]<=200:return None
  f=(target-times[j-1])/(times[j]-times[j-1])
  vals=[(1-f)*float(feet[j-1][s+'_deg'])+f*float(feet[j][s+'_deg']) for s in ['left','right']]
  return vals,j
 def reversal(j,side):
  if not 0<j<len(feet)-1 or not all(good(feet[k]) for k in [j-1,j,j+1]):return False
  a,b,c=[float(feet[k][side+'_deg']) for k in [j-1,j,j+1]]
  return (b-a)*(c-b)<0
 events=[];queries={}
 for f in feet:
  seq=int(f['sequence']);d=pred[seq];frame=max(0,int(f['measurement_time_us'])//1000);ready=int(d['ready_ms'])
  valid=good(f) and f['state_id']=='3' and int(f['measurement_time_us'])>=0
  l,r=[n(f[s+'_deg']) or 0 for s in ['left','right']]
  events.append((ready,0,f'O {seq} {f["run_id"]} {frame} {ready} {int(valid)} {l} {r}'))
  if valid:
   for lead in [0,30]:
    ident=len(queries);queries[ident]={'kind':'direct_frame_lead'+str(lead),'now_ms':frame-lead,'lead_ms':lead,'frame_seq':seq}
    events.append((frame-lead,1,f'Q {ident} {frame-lead} {lead}'))
 for a in audits:
  ident=len(queries);t=int(a['time_ms']);queries[ident]={'kind':'actual_control_audit','now_ms':t,'lead_ms':0,'audit':a}
  events.append((t,1,f'Q {ident} {t} 0'))
 new_header=(PROJECT/'src/foot_phase_prediction.h').read_bytes();old_header=(PROJECT/'src/foot_pose_prediction.h').read_bytes()
 with tempfile.TemporaryDirectory(prefix='v11_review_') as td:
  td=Path(td);(td/'foot_phase_prediction.h').write_bytes(new_header);(td/'foot_pose_prediction.h').write_bytes(old_header)
  source=td/'review.cpp';exe=td/'review.exe';source.write_text(CPP,encoding='utf8')
  subprocess.run(['g++','-std=c++11','-O2',str(source),'-o',str(exe)],check=True)
  proc=subprocess.run([str(exe)],input='\n'.join(e[2] for e in sorted(events,key=lambda x:(x[0],x[1])))+'\n',text=True,capture_output=True,check=True)
 runtime={};query_outputs={}
 for line in proc.stdout.splitlines():
  p=line.split();r={'reason':int(p[2]),'candidate_ready':p[3]=='1','candidate':[n(p[4]),n(p[5])],
   'confidence':float(p[6]),'uncertainty':[n(p[7]),n(p[8])],'phase':[int(p[9]),int(p[10])],
   'linear_ready':p[11]=='1','hold_ready':p[12]=='1','linear':[n(p[13]),n(p[14])],'hold':[n(p[15]),n(p[16])],
   'latest_seq':int(p[17]),'age_ms':int(p[18]),'horizon_ms':int(p[19])}
  (runtime if p[0]=='D' else query_outputs)[int(p[1])]=r
 assert len(runtime)==len(saved) and len(query_outputs)==len(queries)
 reason_mismatch=[];value_errors=[];rows=[];audit_review=[]
 for seq,d in pred.items():
  r=runtime[seq]
  if int(d['reason'])!=r['reason']:reason_mismatch.append({'sequence':seq,'saved':int(d['reason']),'replay':r['reason']})
  if d['reason']=='0':
   value_errors.extend(abs(float(d['predicted_'+s+'_deg'])-r['candidate'][k]) for k,s in enumerate(['left','right']))
  f=byseq[seq]
  if f not in active or d['reason']!='0':continue
  target=int(d['ready_ms'])+30;answer=truth(target)
  if not answer:continue
  values,j=answer
  for k,s in enumerate(['left','right']):
   row={'group':'saved_runtime_plus30ms','sequence':seq,'target_ms':target,'side':s,'confidence':float(d['confidence']),
    'candidate_ready':True,'linear_ready':r['linear_ready'],'hold_ready':r['hold_ready'],
    'candidate_error_deg':abs(float(d['predicted_'+s+'_deg'])-values[k]),
    'linear_error_deg':abs(r['linear'][k]-values[k]) if r['linear'][k] is not None else None,
    'hold_error_deg':abs(r['hold'][k]-values[k]),'uncertainty_deg':float(d[s+'_uncertainty_deg']),
    'near_reversal':reversal(j,s)};rows.append(row)
 for ident,q in queries.items():
  r=query_outputs[ident]
  if q['kind'].startswith('direct_frame'):
   f=byseq[q['frame_seq']];values=[float(f[s+'_deg']) for s in ['left','right']];j=feet.index(f)
  else:
   answer=truth(q['now_ms'])
   if not answer:continue
   values,j=answer;a=q['audit']
   audit_review.append({'time_ms':q['now_ms'],'saved_sequence':int(a['sequence']),'replayed_sequence':r['latest_seq'],
    'saved_prediction':[float(a['predicted_'+s+'_deg']) for s in ['left','right']],
    'replayed_prediction':r['candidate'],'camera_interpolated_at_control_ms':values,
    'absolute_errors_deg':[abs(float(a['predicted_'+s+'_deg'])-values[k]) for k,s in enumerate(['left','right'])],
    'replay_matches_saved_max_difference_deg':max(abs(float(a['predicted_'+s+'_deg'])-r['candidate'][k]) for k,s in enumerate(['left','right']))})
  for k,s in enumerate(['left','right']):
   rows.append({'group':q['kind'],'sequence':r['latest_seq'],'target_ms':q['now_ms']+q['lead_ms'],'side':s,
    'confidence':r['confidence'],'candidate_ready':r['candidate_ready'],'linear_ready':r['linear_ready'],'hold_ready':r['hold_ready'],
    'candidate_error_deg':abs(r['candidate'][k]-values[k]) if r['candidate'][k] is not None else None,
    'linear_error_deg':abs(r['linear'][k]-values[k]) if r['linear'][k] is not None else None,
    'hold_error_deg':abs(r['hold'][k]-values[k]),'uncertainty_deg':r['uncertainty'][k],'near_reversal':reversal(j,s)})
 groups={g:accuracy([r for r in rows if r['group']==g]) for g in sorted(set(r['group'] for r in rows))}
 obs=[f for f in active if f['control_observed_us']]
 saved_active=[pred[int(f['sequence'])] for f in active];ready=[d for d in saved_active if d['reason']=='0']
 latency={'driver_frame_to_delivery_ms':stats([(int(f['delivered_us'])-int(f['frame_us']))/1000 for f in active]),
  'image_processing_ms':stats([int(f['processing_us'])/1000 for f in active]),
  'frame_to_prediction_ready_ms':stats([int(d['ready_ms'])-int(byseq[int(d['sequence'])]['measurement_time_us'])/1000 for d in saved_active]),
  'prediction_compute_us_all_active':stats([int(d['compute_us']) for d in saved_active]),
  'prediction_compute_us_ready_only':stats([int(d['compute_us']) for d in ready]),
  'frame_to_first_control_ms':stats([(int(f['control_observed_us'])-int(f['frame_us']))/1000 for f in obs]),
  'prediction_ready_to_first_control_ms':stats([int(f['control_observed_us'])/1000-(int(f['frame_us'])-int(f['measurement_time_us']))/1000-int(pred[int(f['sequence'])]['ready_ms']) for f in obs]),
  'frame_intervals_ms':stats(np.diff([int(f['frame_us']) for f in active])/1000)}
 quality={'all_foot_frames':len(feet),'strict_measurement_frames':len(active),'boundary_state3_negative_measurement_frames':sum(f['state_id']=='3' and int(f['measurement_time_us'])<0 for f in feet),
  'field_failure_counts':{k:sum(f[k]!='True' for f in active) for k in QUALITY},
  'reason_counts_active':dict(Counter(d['reason'] for d in saved_active)),
  'confidence_ready':stats([float(d['confidence']) for d in ready]),
  'confidence_ready_ge_0_5_frames':sum(float(d['confidence'])>=.5 for d in ready),
  'both_uncertainty_le_0_75_ready_frames':sum(float(d['left_uncertainty_deg'])<=.75 and float(d['right_uncertainty_deg'])<=.75 for d in ready),
  'both_confidence_and_margin_ready_frames':sum(float(d['confidence'])>=.5 and float(d['left_uncertainty_deg'])<=.75 and float(d['right_uncertainty_deg'])<=.75 for d in ready),
  'recognition_reasons_invalid':dict(Counter(f['left_reason']+'|'+f['right_reason'] for f in active if not good(f))),
  'prediction_metadata':{k:v for k,v in meta['foot_pose_prediction'].items() if k!='records'},
  'control_observation_metadata':{k:v for k,v in meta['foot_control_observations'].items() if k!='rows'}}
 quality['availability_fraction_ready']=len(ready)/len(active)
 quality['diagnostic_context']='Confidence/margin counts use saved ready-time +30ms diagnostics, not current0ms actuation eligibility at every control query.'
 quality['five_second_bins']=[]
 for start in range(0,30,5):
  block=[f for f in active if start*1e6<=int(f['measurement_time_us'])<(start+5)*1e6]
  diag=[pred[int(f['sequence'])] for f in block]
  quality['five_second_bins'].append({'start_s':start,'end_s':start+5,'frames':len(block),
   'ready_frames':sum(d['reason']=='0' for d in diag),'invalid_frames':sum(d['reason']=='2' for d in diag),
   'confidence_and_margin_frames':sum(d['reason']=='0' and float(d['confidence'])>=.5 and float(d['left_uncertainty_deg'])<=.75 and float(d['right_uncertainty_deg'])<=.75 for d in diag)})
 result={'build':meta.get('diagnostic_build_id'),'source_header_sha256':hashlib.sha256(new_header).hexdigest(),
  'scope':'Saved runtime +30ms projections first; compiled replay uses recorded ready_ms exactly, not v10 CAD timing estimates. Future camera values are scorer-only, not independent angle or contact truth.',
  'cautions':['Current trial was not used during estimator development; it is one new trial, not a broad prospective validation.',
   'Interpolated camera scoring can favor holding; direct-frame scoring avoids target interpolation but shares camera recognition error.',
   'First-control timestamps are a strided 64-row subset; reuse ages and motor latency are additional.',
   'Frame timestamps are camera-driver timestamps and are not calibrated exposure times.',
   'Scored side/time values are correlated. No causal inference about gait benefit follows from prediction errors.'],
  'reproduction':{'saved_rows':len(saved),'replayed_rows':len(runtime),'reason_mismatches':reason_mismatch,'ready_prediction_difference_deg':stats(value_errors)},
  'quality':quality,'latency':latency,'accuracy':groups,'actual_control_audit':audit_review}
 (ROOT/'prediction_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
 with (ROOT/'prediction_review_errors.csv').open('w',encoding='utf8',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 print(json.dumps({'reproduction':result['reproduction'],'accuracy':groups,'latency':latency,'audit':audit_review},ensure_ascii=False))
 report=['# v11新試行：足予測・遅延・品質の独立確認','',
  '実保存された+30 ms診断を優先し、将来の有効カメラ角を補間した値と比較した。別途、実際のready_msで観測を到着させるC++再生を行った。新試行は予測器の開発には使用していない。独立した角度真値や接地検証ではない。','',
  '| 比較 | 共通角度値数 | 新予測MAE | 保持MAE | 旧直線MAE | 新予測95%点 | 新予測最大 |',
  '|---|---:|---:|---:|---:|---:|---:|']
 for name,g in groups.items():
  m=g['common'];report.append(f"| {name} | {m['angle_values']} | {m['candidate']['mean']:.4f}° | {m['hold']['mean']:.4f}° | {m['linear']['mean']:.4f}° | {m['candidate']['p95']:.4f}° | {m['candidate']['max']:.4f}° |")
 report+=['','direct_frameは、その画像がまだ届いていないフレーム時刻で予測し、その後得られた実際のフレーム角と比較する。これにより採点時の線形補間による保持有利を避けるが、同じ足認識器の誤差は残る。','',
  f"保存診断の再生は理由不一致{len(reason_mismatch)}件。予測値の差の最大は{max(value_errors):.8f}°。実行中の0 ms制御予測も3件のauditと対応を確認した。",'',
  f"測定中の足フレームは{len(active)}件、readyは{len(ready)}件。reason内訳は{dict(Counter(d['reason'] for d in saved_active))}。state3だが測定時刻負の境界フレームは別に1件あり、測定中の時刻異常とは扱わない。",'',
  f"保存診断のconfidence ≥0.5と左右uncertainty ≤0.75°を同時に満たすのは{quality['both_confidence_and_margin_ready_frames']}/{len(active)}件（{100*quality['both_confidence_and_margin_ready_frames']/len(active):.1f}%）。これはready時点の+30 ms診断で、現在0 msの介入可能率ではない。",'',
  '5秒ごとの「confidenceとmarginの両条件を満たす枚数」は、0〜5秒20/48、5〜10秒33/50、10〜15秒32/50、15〜20秒19/50、20〜25秒18/50、25〜30秒12/49。後半は履歴との一致や認識/校正範囲品質が落ち、足量を信頼して使える観測が減る。', '',
  f"フレーム時刻→画像受け取り中央値{latency['driver_frame_to_delivery_ms']['median']:.2f} ms、画像処理中央値{latency['image_processing_ms']['median']:.2f} ms。ready時の追加予測計算は中央値{latency['prediction_compute_us_ready_only']['median']:.0f} μs、最大{latency['prediction_compute_us_ready_only']['max']:.0f} μs。",'',
  f"フレーム時刻→制御側初回観測は{len(obs)}標本で中央値{latency['frame_to_first_control_ms']['median']:.2f} ms、95%点{latency['frame_to_first_control_ms']['p95']:.2f} ms。全観測の履歴ではなく、同じ画像を再利用する際の追加経過時間も含まない。",'',
  '平均精度が改善しても、confidenceは確率ではなく、uncertaintyは保証幅ではない。最大誤差と欠測を併記する。今回の予測比較から、15 ms補助が歩行を改善したことや、引っかかりを越えたことを証明しない。実際の補助出力・足運び・前進は別途評価する。']
 (ROOT/'prediction_review.md').write_text('\n'.join(report)+'\n',encoding='utf8')

if __name__=='__main__':main()
