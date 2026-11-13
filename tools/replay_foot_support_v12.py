"""Causal v12 ablations on unchanged recorded trajectories, not new dynamics.

The recorded old input/peak history force-updates the ordinary controller.
Each hypothetical first candidate is independent; later old data is not the
response to that candidate. Hardware/wheel/current guards are not simulated.
"""
from pathlib import Path
import argparse,bisect,csv,hashlib,importlib.util,inspect,json,math,subprocess,tempfile
from collections import Counter
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
CASES=['141282685','137267573','267557373','27908524','64040782','84815132','57814129','199015154']
VARIANTS=['v11_history60','v12_local60_normal','v12_local80_normal','v12_local80_stall']
REASONS=['Selected','Inactive','Identity','Time','PoseUnavailable','PoseUncertain','StaleFrame','Nonfinite','SmallForward','WrongMotion','OutsideWindow','Speed','NoOrdinaryDrive','Width','InsufficientLead','TooEarly','CoastNeedsEvidence','StallNeedsEvidence','StallTooEarly']

CPP=r'''
#include "foot_phase_prediction.h"
#include "foot_local_prediction.h"
#include "foot_pose_prediction.h"
#include "foot_support_assist.h"
#include "foot_support_assist_v12.h"
#include "foot_support_deceleration.h"
#include "tilt_priority.h"
#include "wheel_cycle_guard.h"
/*SELECTION_INCLUDE*/
#include <iostream>
#include <iomanip>
#include <cmath>
/*SELECTION_HELPER*/
int main(){
 foot_phase_prediction::Predictor history;
 foot_phase_prediction::Projection hp;
 foot_local_prediction::Predictor local;
 foot_local_prediction::Projection lp;
 foot_pose_prediction::Predictor linear;
 foot_pose_prediction::Result linear_result;
 foot_support_deceleration::Tracker deceleration;
 tilt_priority::Controller controller;
 wheel_cycle_guard::Controller wheel;
 unsigned type,id,run=0,frame=0,received=0,now,valid,seq=0,last_check=0,peak=0,last_accepted=0;
 int peak_side=0;float left=0,right=0,angle,rate,peak_amplitude=0;
 bool consumed=false,attempted=false,decelerating=false,last_drive=false;int last_direction=0;unsigned last_input_time=0;
 bool used[4]={false,false,false,false};
 unsigned checks=0,mismatches=0;
 std::cout<<std::setprecision(10);
 while(std::cin>>type){
  if(type==0){
   std::cin>>run>>frame>>received>>valid>>left>>right;
   history.observe(run,frame,received,valid,left,right);hp=history.projection();
   local.observe(run,frame,received,valid,left,right);lp=local.projection();++seq;
   linear_result=linear.observe(run,frame,received,valid,left,right,0);
  }else if(type==1){float target;unsigned matched;
   std::cin>>now>>peak_side>>peak_amplitude>>target>>matched;peak=now;
   for(auto& x:used)x=false;consumed=attempted=false;decelerating=false;deceleration.reset();
   ++checks;if(std::fabs(controller.target()-target)>.01f)++mismatches;
   wheel.peak(peak_side,peak_amplitude,now,matched && last_drive && last_direction==peak_side && now>last_input_time && now-last_input_time<=1500);
   controller.peak(peak_side,now,peak_amplitude);
  }else if(type==3){unsigned at,width,temporary,wvalid;int direction;float crossing_rate,rpm;
   std::cin>>at>>direction>>width>>temporary>>crossing_rate>>rpm>>wvalid;
   tilt_priority::Input in;in.now_ms=at;in.preset_deg=8;in.target_deg=controller.target();in.next_side=direction;
   in.crossing_rate_dps=crossing_rate;in.hardware_cap_ms=temporary?15:controller.ordinaryRiseCap(direction,at,150);in.hardware_healthy=true;
   auto choice=controller.propose(in);choice.selected=true;choice.coast=width==0;
   choice.current_ma=width?1200:0;choice.width_ms=width;choice.direction=direction;choice.target_deg=controller.target();
   controller.crossing(direction,at);
   if(!temporary)controller.ordinaryResult(direction,at,width,width!=0);
   controller.confirm(direction,at,choice,true,temporary,!temporary);
   wheel.limit(direction,rpm,wvalid,at,width);
   last_drive=width>0;last_direction=direction;last_input_time=at;
   consumed=true;last_accepted=at;if(temporary)attempted=true;
  }else if(type==5){unsigned at;std::cin>>at;attempted=true;
  }else if(type==4){unsigned at,pulse;float body_rate,body_angle;
   std::cin>>at>>body_rate>>body_angle>>pulse;
   if(pulse || consumed || body_angle*peak_side<=0 || body_rate*peak_side>=0){deceleration.reset();decelerating=false;}
  }else if(type==6){
   std::cin>>id>>now;
   auto h=hp.predict(now,0);auto l=lp.predict(now,0);auto s=select_pose(hp,lp,now);
   const unsigned age=now>=frame?now-frame:100000;
   std::cout<<'E'<<' '<<id<<' '<<h.ready()<<' '<<h.left<<' '<<h.right<<' '
    <<l.ready()<<' '<<l.left<<' '<<l.right<<' '<<l.left_uncertainty<<' '<<l.right_uncertainty<<' '
    <<int(l.phase_left)<<' '<<int(l.phase_right)<<' '<<s.source<<' '<<s.ready<<' '<<s.left<<' '<<s.right<<' '
    <<s.lu<<' '<<s.ru<<' '<<left<<' '<<right<<' '<<(valid && age<=160)<<' '<<age<<' '
    <<(valid && age<=160 && linear_result.ready())<<' '<<left+linear_result.left_rate*age*.001f<<' '<<right+linear_result.right_rate*age*.001f<<'\n';
  }else{
   unsigned wheel_valid;float wheel_rpm;std::cin>>id>>run>>now>>angle>>rate>>wheel_rpm>>wheel_valid;
   if(consumed || attempted || (last_accepted && (now<=last_accepted || now-last_accepted<250)))continue;
   if(last_check && now>=last_check && now-last_check<5)continue;last_check=now;
   decelerating=deceleration.update(run,peak,now,-rate*peak_side);
   auto h=hp.predict(now,0);auto l=lp.predict(now,0);auto chosen=select_pose(hp,lp,now);
   tilt_priority::Input ordinary;ordinary.now_ms=now;ordinary.preset_deg=8;ordinary.target_deg=controller.target();ordinary.next_side=-peak_side;
   ordinary.crossing_rate_dps=std::fabs(rate);ordinary.hardware_cap_ms=controller.ordinaryRiseCap(-peak_side,now,150);ordinary.hardware_healthy=true;
   auto drive=controller.propose(ordinary);
   if(drive.selected&&!drive.coast)drive.width_ms=controller.balanceTrimCap(now,-peak_side,drive.width_ms);
   auto wheel_preview=wheel;
   const unsigned normal_guarded=wheel_preview.limit(-peak_side,wheel_rpm,wheel_valid,now,drive.width_ms);
   for(unsigned k=0;k<4;++k){
    if(used[k])continue;
    auto pose=k==0?history_pose(hp,now):chosen;
    foot_support_assist_v12::Input in;
    in.active=true;in.run_id=run;in.foot_run_id=pose.run;in.peak_id=peak;in.peak_side=peak_side;
    in.now_ms=now;in.frame_ms=pose.frame;in.pose_ready=pose.ready;in.pose_confident=pose.usable;
    in.left_deg=pose.left;in.right_deg=pose.right;in.left_uncertainty_deg=pose.lu;in.right_uncertainty_deg=pose.ru;
    in.body_angle_deg=angle;in.body_rate_dps=rate;
    in.normal_drive_selected=drive.selected&&!drive.coast;in.normal_coast_selected=drive.selected&&drive.coast;
    in.normal_width_ms=k==0?drive.width_ms:normal_guarded;
    if(k>0 && !wheel_valid)in.normal_drive_selected=in.normal_coast_selected=false;
    in.inward_decelerating=decelerating;
    const bool left_dominant=pose.left-pose.lu>=pose.right-pose.ru;
    const int dominant=left_dominant?pose.phase_left:pose.phase_right;
    const float asym=std::fabs(pose.left-pose.right)-pose.lu-pose.ru;
    in.anticipated_shape_need=dominant==4 && asym>=1.f && peak_amplitude<=8.5f;
    in.local_returning_confirmed=pose.source==2 && dominant==4;
    in.prediction_source=static_cast<foot_support_assist_v12::PredictionSource>(pose.source);
    bool selected=false,stall=false,coast=false;unsigned reason=0,width=0;float ttc=NAN,threshold=NAN;
    if(k==0){auto d=foot_support_assist::decide(in);selected=d.selected;coast=d.coast_assist;reason=int(d.reason);width=d.width_ms;ttc=d.estimated_zero_ms;threshold=d.threshold_deg;}
    else{foot_support_assist_v12::Limits limits;limits.max_ordinary_width_ms=k==1?60:80;limits.allow_stall_route=k==3;
     auto d=foot_support_assist_v12::decide(in,limits);selected=d.selected;stall=d.stall_route;coast=d.coast_assist;reason=int(d.reason);width=d.width_ms;ttc=d.estimated_zero_ms;threshold=d.threshold_deg;}
    auto postguard=wheel;
    const unsigned backend_input=coast?width:drive.width_ms;
    const unsigned guarded_again=selected?postguard.limit(-peak_side,wheel_rpm,wheel_valid,now,backend_input):0;
    const bool width_preserved=selected && guarded_again==width;
    std::cout<<'C'<<' '<<id<<' '<<k<<' '<<selected<<' '<<reason<<' '<<peak<<' '<<peak_side<<' '
     <<pose.source<<' '<<pose.ready<<' '<<pose.usable<<' '<<pose.left<<' '<<pose.right<<' '<<pose.lu<<' '<<pose.ru<<' '
     <<dominant<<' '<<decelerating<<' '<<in.anticipated_shape_need<<' '<<in.local_returning_confirmed<<' '
     <<drive.width_ms<<' '<<drive.selected<<' '<<drive.coast<<' '<<width<<' '<<stall<<' '<<coast<<' '<<ttc<<' '<<threshold<<' '
     <<h.confidence<<' '<<h.left_uncertainty<<' '<<h.right_uncertainty<<' '<<l.ready()<<' '<<l.left<<' '<<l.right<<' '
     <<l.left_uncertainty<<' '<<l.right_uncertainty<<' '<<int(l.phase_left)<<' '<<int(l.phase_right)<<' '<<pose.frame<<' '
     <<normal_guarded<<' '<<wheel_preview.legacyWidth()<<' '<<wheel_preview.extraReduction()<<' '<<wheel_rpm<<' '<<wheel_valid<<' '
     <<guarded_again<<' '<<width_preserved<<'\n';
    if(selected)used[k]=true;
   }
  }
 }
 std::cerr<<checks<<' '<<mismatches<<'\n';
}
'''

def read(p):
 with p.open(encoding='utf8',newline='') as f:return list(csv.DictReader(f))
def good(r):
 return all(r[k]=='True' for k in ['timestamp_valid','frame_valid','zero_ready','left_valid','right_valid','left_in_range','right_in_range'])
def number(v):
 x=float(v);return x if math.isfinite(x) else None
def summary(values):
 v=[x for x in values if x is not None]
 return {'n':len(v),'mae_deg':float(np.mean(v)),'p95_deg':float(np.percentile(v,95)),'max_deg':max(v)} if v else None
def case_dir(case):
 for date in ['2026-11-13-walking','2026-11-11-walking']:
  p=ROOT/'analysis'/date/case
  if p.exists():return p
 raise FileNotFoundError(case)

def event_data(path,delay):
 """Reuse the reviewed v11 adapter, stopping before its subprocess/parser."""
 spec=importlib.util.spec_from_file_location('old_foot_replay',ROOT/'tools/replay_foot_support_assist.py')
 old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
 source=inspect.getsource(old.replay)
 source=source.replace("feet=read(path/'foot_angles.csv');samples=read(path/'timeseries.csv')", "feet=read(path/'foot_angles.csv');samples=read(path/'timeseries.csv');pose_path=path/'foot_pose_prediction.csv';saved_pose={int(r['sequence']):r for r in read(pose_path)} if pose_path.exists() else {}")
 original="received=(int(f['delivered_us'])+int(f['processing_us'])+cad.get(int(f['sequence']),0)-epoch+999)//1000"
 assert original in source
 source=source.replace(original,original+";received=int(saved_pose[int(f['sequence'])]['ready_ms']) if saved_pose else received")
 cut=source.index(' proc=subprocess.run')
 source=source[:cut]+' return lines,queries,feet,peaks,old,saved_pose\n'
 scope=dict(old.__dict__);exec(source,scope)
 return scope['replay'](None,path,delay,5)

def run_case(exe,path,delay):
 lines,queries,feet,peaks,old,saved_pose=event_data(path,delay)
 first=next(f for f in feet if int(f['measurement_time_us'])>=0)
 log_epoch=int(first['frame_us'])-int(first['log_time_us'])
 samples={int(float(s['t_test_ms'])):s for s in read(path/'timeseries.csv') if s['state_id']=='3'}
 accepted={(int(float(e['zero_cross_time_ms'])),int(e['physical_next_peak_side'])):e for e in old if e['valid']=='True' and e['event_kind']!='strong_start_kick'}
 events=[];evals={}
 for serial,line in enumerate(lines):
  t=line.split();kind=int(t[0]);when=int(t[3])+delay if kind==0 else int(t[3]) if kind==2 else int(t[1])
  if kind==3:
   e=accepted[(int(t[1]),int(t[2]))];rpm=float(e['pre_input_wheel_speed_rpm'] or 'nan')
   wvalid=e['pre_input_wheel_speed_valid']=='True' and math.isfinite(rpm) and int(e['pre_input_wheel_speed_age_us'])<=50000
   line+=f' {rpm if math.isfinite(rpm) else 0} {int(wvalid)}'
  elif kind==2:
   s=samples[int(t[3])];rpm=float(s['roller_speed_rpm'] or 'nan')
   sample_time=int(s['roller_speed_sample_time_us'] or 0)
   age=log_epoch+int(float(s['log_time_s'])*1e6)-sample_time
   wvalid=s['roller_speed_valid'] in ['1','True'] and math.isfinite(rpm) and sample_time>0 and 0<=age<=50000
   queries[int(t[1])].update({'wheel_rpm':rpm if math.isfinite(rpm) else None,'wheel_valid':wvalid,'wheel_sample_age_us':age})
   line+=f' {rpm if math.isfinite(rpm) else 0} {int(wvalid)}'
  events.append((when,0 if kind==0 else 2,serial,line))
 for i,f in enumerate(feet):
  if f['state_id']!='3' or int(f['measurement_time_us'])<0 or not good(f):continue
  ident=len(evals);now=int(f['measurement_time_us'])//1000;evals[ident]={'index':i,'time_ms':now}
  events.append((now,3,ident,f'6 {ident} {now}'))
 data='\n'.join(e[3] for e in sorted(events,key=lambda x:(x[0],x[1],x[2])))+'\n'
 proc=subprocess.run([str(exe)],input=data,text=True,capture_output=True,check=True)
 counts={k:Counter() for k in VARIANTS};candidates=[];critical=[];scores=[]
 for line in proc.stdout.splitlines():
  p=line.split()
  if p[0]=='C':
   ident,k,selected,reason,peak,side=map(int,p[1:7]);q=queries[ident]
   counts[VARIANTS[k]][REASONS[reason]]+=1
   row={'case':path.name,'control_delay_ms':delay,'variant':VARIANTS[k],**q,'selected':bool(selected),'reason':REASONS[reason],
    'peak_ms':peak,'peak_side':side,'source':int(p[7]),'pose_ready':p[8]=='1','pose_usable':p[9]=='1',
    'left_deg':number(p[10]),'right_deg':number(p[11]),'left_margin_deg':number(p[12]),'right_margin_deg':number(p[13]),
    'dominant_phase':int(p[14]),'decelerating':p[15]=='1','anticipated_shape_need':p[16]=='1','local_returning_confirmed':p[17]=='1',
    'ordinary_proposal_ms':int(p[18]),'ordinary_selected':p[19]=='1','ordinary_coast':p[20]=='1',
    'requested_width_ms':int(p[21]),'stall_route':p[22]=='1','coast_assist':p[23]=='1','zero_eta_ms':number(p[24]),'body_threshold_deg':number(p[25]),
    'history_score':number(p[26]),'history_margins':[number(p[27]),number(p[28])],
    'local_ready':p[29]=='1','local_angles':[number(p[30]),number(p[31])],'local_margins':[number(p[32]),number(p[33])],
    'local_phases':[int(p[34]),int(p[35])],'frame_ms':int(p[36])}
   row.update({'normal_guarded_width_ms':int(p[37]),'normal_preview_legacy_width_ms':int(p[38]),'normal_preview_extra_reduction_ms':int(p[39]),
    'backend_guarded_width_ms':int(p[42]),'width_preserved_after_guard':p[43]=='1'})
   if path.name=='141282685' and 27622<q['time_ms']<27958:critical.append(row)
   if selected:
    refs=[e for e in old if int(float(e['previous_peak_time_ms'] or 0))==peak and (e['valid']=='True' or e['event_kind']=='angle_rate_early_trial')]
    refs.sort(key=lambda e:float(e['zero_cross_time_ms']));ref=refs[0] if refs else None
    row.update({'old_next_decision_ms':float(ref['zero_cross_time_ms']) if ref else None,
     'ms_before_old_decision':float(ref['zero_cross_time_ms'])-q['time_ms'] if ref else None,
     'old_event_kind':ref['event_kind'] if ref else None})
    frame_times=[int(f['measurement_time_us'])/1000 if int(f['measurement_time_us'])>=0 else -100000 for f in feet]
    j=bisect.bisect_left(frame_times,q['time_ms'])
    if 0<j<len(feet) and good(feet[j-1]) and good(feet[j]) and 0<frame_times[j]-frame_times[j-1]<=200:
     alpha=(q['time_ms']-frame_times[j-1])/(frame_times[j]-frame_times[j-1])
     camera=[(1-alpha)*float(feet[j-1][s+'_deg'])+alpha*float(feet[j][s+'_deg']) for s in ['left','right']]
     row['future_camera_interpolated_deg']=camera
     row['camera_comparison_abs_error_deg']=[abs(row['left_deg']-camera[0]),abs(row['right_deg']-camera[1])]
     row['scorer_camera_interval_ms']=[frame_times[j-1],frame_times[j]]
    candidates.append(row)
  else:
   ident=int(p[1]);q=evals[ident];i=q['index'];f=feet[i]
   for side_id,side in enumerate(['left','right']):
    value=float(f[side+'_deg']);margin=number(p[8+side_id]);phase=int(p[10+side_id]);local=number(p[6+side_id]);hold=number(p[18+side_id])
    rate=None
    if 0<i<len(feet)-1 and good(feet[i-1]) and good(feet[i+1]):
     dt=(int(feet[i+1]['frame_us'])-int(feet[i-1]['frame_us']))/1e6
     if 0<dt<=.4:rate=(float(feet[i+1][side+'_deg'])-float(feet[i-1][side+'_deg']))/dt
    scores.append({'case':path.name,'time_ms':q['time_ms'],'side':side,'history_ready':p[2]=='1','history_error_deg':abs(number(p[3+side_id])-value) if number(p[3+side_id]) is not None else None,
     'local_ready':p[5]=='1','local_error_deg':abs(local-value) if local is not None else None,'local_phase':phase,'local_margin_deg':margin,
     'local_both_margins_le075':all(number(p[z]) is not None and number(p[z])<=.75 for z in [8,9]),
     'combined_source':int(p[12]),'combined_ready':p[13]=='1','combined_error_deg':abs(number(p[14+side_id])-value) if number(p[14+side_id]) is not None else None,
     'hold_ready':p[20]=='1','hold_error_deg':abs(hold-value) if hold is not None else None,'age_ms':int(p[21]),'camera_centered_rate_dps':rate})
    scores[-1]['linear_ready']=p[22]=='1'
    linear=number(p[23+side_id]);scores[-1]['linear_error_deg']=abs(linear-value) if linear is not None else None
 checks,mismatch=map(int,proc.stderr.split())
 return {'arrival_model':'actual saved ready_ms' if saved_pose else 'delivered+processing+CAD compute approximate',
  'peak_target_checks':checks,'peak_target_mismatches':mismatch,
  'reason_counts':{k:dict(v) for k,v in counts.items()},'candidate_counts':{k:sum(c['variant']==k for c in candidates) for k in VARIANTS},
  'wheel_guard_width_preserved_counts':{k:sum(c['variant']==k and c['width_preserved_after_guard'] for c in candidates) for k in VARIANTS},
  'candidates':candidates,'critical_window':critical},scores

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--cases',nargs='+',default=CASES)
 ap.add_argument('--control-delays-ms',nargs='+',type=int,default=[0,15]);ap.add_argument('--out',type=Path,default=ROOT/'analysis/2026-11-13-walking/v12_causal_replay.json');args=ap.parse_args()
 headers=['foot_phase_prediction.h','foot_local_prediction.h','foot_pose_prediction.h','foot_support_assist.h','foot_support_assist_v12.h','foot_support_deceleration.h','tilt_priority.h','wheel_cycle_guard.h']
 source=CPP
 # The parent supplies the same pure selection helper used by the runner.
 source=source.replace('/*SELECTION_INCLUDE*/',SELECTION_INCLUDE).replace('/*SELECTION_HELPER*/',SELECTION_HELPER)
 headers+=SELECTION_HEADERS
 frozen={n:(ROOT/'src'/n).read_bytes() for n in headers};results={};all_scores=[]
 with tempfile.TemporaryDirectory(prefix='support_v12_') as td:
  td=Path(td)
  for n,b in frozen.items():(td/n).write_bytes(b)
  cpp=td/'replay.cpp';exe=td/'replay.exe';cpp.write_text(source,encoding='utf8')
  subprocess.run(['g++','-std=c++11','-O2',str(cpp),'-o',str(exe)],check=True)
  for case in args.cases:
   results[case]={}
   for delay in args.control_delays_ms:
    result,scores=run_case(exe,case_dir(case),delay);results[case][str(delay)]=result
    if delay==0:all_scores+=scores
    print(case,delay,result['candidate_counts'],'target_mismatch',result['peak_target_mismatches'])
 score_summary={}
 for case in args.cases:
  case_rows=[r for r in all_scores if r['case']==case];groups={}
  for name,filter_fn in [('local_ready',lambda r:r['local_ready']),('local_returning',lambda r:r['local_ready'] and r['local_phase']==4),
    ('local_side_margin_le075',lambda r:r['local_ready'] and r['local_margin_deg'] is not None and r['local_margin_deg']<=.75),
    ('local_both_margins_le075',lambda r:r['local_ready'] and r['local_both_margins_le075']),
    ('combined_local_selected',lambda r:r['combined_ready'] and r['combined_source']==2)]:
   rows=[r for r in case_rows if filter_fn(r) and r['hold_ready'] and r['linear_ready']]
   direction=[r for r in rows if r['local_phase']==4 and r['camera_centered_rate_dps'] is not None and abs(r['camera_centered_rate_dps'])>=.8]
   groups[name]={'angle_values':len(rows),'local':summary([r['local_error_deg'] for r in rows]),'hold':summary([r['hold_error_deg'] for r in rows]),'linear':summary([r['linear_error_deg'] for r in rows]),
    'margin_coverage_fraction':sum(r['local_error_deg']<=r['local_margin_deg'] for r in rows)/len(rows) if rows else None,
    'returning_direction_check_values':len(direction),'returning_opposite_direction_values':sum(r['camera_centered_rate_dps']>.8 for r in direction)}
  score_summary[case]=groups
 output={'scope':'Causal eligibility on unchanged old trajectories; ordinary controller and portable wheel guard forced by recorded input/peak history; not new physical response, gait benefit or complete downstream guards.',
  'variant_order':VARIANTS,'source_sha256':{n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
  'compiled_harness_sha256':hashlib.sha256(source.encode('utf8')).hexdigest(),
  'event_adapter_sha256':hashlib.sha256((ROOT/'tools/replay_foot_support_assist.py').read_bytes()).hexdigest(),
  'cautions':['Future camera values are used only for scoring and are not independent angle or contact truth.',
   'Ready timing for v11 is actual; old logs use an approximate image/CAD computation model. Additional15ms delay is sensitivity only.',
   'Each hypothetical peak candidate is independent; subsequent old data is not its response.',
   'Portable wheel-cycle guard is replayed with recorded accepted-command RPM/validity, matched drive peaks, and current-query RPM freshness. Full branch health, current/battery/driver guards and failed-write uncertainty are not reproduced.',
   'The5ms check schedule reconstructed from logged body samples and old-input-first timestamp ties do not reproduce all real runner audit counts.',
   'New v12 local computation/publication overhead is not measured by v11 ready_ms; timing sensitivities do not prove deadlines.',
   'Local margins are heuristic disagreement allowances, not error guarantees. Returning means evidence from past frames, not guaranteed instantaneous motion.'],
  'cases':results,'direct_frame_accuracy':score_summary}
 args.out.write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
 if all_scores:
  with args.out.with_name(args.out.stem+'_prediction_errors.csv').open('w',encoding='utf8',newline='') as f:
   w=csv.DictWriter(f,fieldnames=list(all_scores[0]));w.writeheader();w.writerows(all_scores)
 report=['# v12 causal eligibility and local-evidence review','',
  'This is a compiled-header replay on unchanged recorded trajectories, with the same source-selection helper as the runner. Recorded old commands and peaks force-update the ordinary controller and portable wheel-cycle guard; each hypothetical first candidate is independent. This does not predict the response to a new pulse or prove walking improvement. Remaining current/battery/hardware/driver guards and full branch-health context are not simulated.','',
  'The cumulative ablations are: v11 historical prediction /60ms cap /normal ETA; add local Returning evidence only; add80ms cap without truncating the ordinary proposal; add the bounded near-stall ETA route. All retain source quality, motion, one-attempt-per-peak and short-lead limits.','',
  '| Log | Extra availability delay | v11 | +Local /60 | +Local /80 | +Local /80 /stall |',
  '|---|---:|---:|---:|---:|---:|']
 for case,delays in results.items():
  for delay,result in delays.items():
   c=result['candidate_counts'];report.append('| '+case+' | '+delay+'ms | '+' | '.join(str(c[k]) for k in VARIANTS)+' |')
 report+=['','V12 uses a copy of the wheel guard to preview the same raw ordinary proposal; its selector receives the already-guarded width, rather than ignoring a later reduction or applying it twice. Every selected candidate also records a separate backend guard evaluation and width-preservation result. This preserves the existing wheel guard rather than bypassing it.','',
  '| Log | Delay | V12 full selector candidates | Wheel-width-preserved candidates |', '|---|---:|---:|---:|']
 for case,delays in results.items():
  for delay,result in delays.items():
   report.append(f"| {case} | {delay}ms | {result['candidate_counts']['v12_local80_stall']} | {result['wheel_guard_width_preserved_counts']['v12_local80_stall']} |")
 if '141282685' in results:
  report+=['','## Fixed failure window:141282685,27.622-27.958s','',
   '| Delay | First full candidate | Width | ETA | Old rescue advance |', '|---|---:|---:|---:|---:|']
  for delay,result in results['141282685'].items():
   c=next((r for r in result['candidates'] if r['variant']=='v12_local80_stall' and 27622<r['time_ms']<27958),None)
   report.append(f"| {delay}ms | "+(f"{c['time_ms']/1000:.3f}s | {c['requested_width_ms']}ms | {c['zero_eta_ms']:.1f}ms | {27958-c['time_ms']}ms |" if c else 'none | - | - | - |'))
  report+=['','At the same query near27.921s, v11 rejects PoseUncertain; local-only rejects Width for the guarded68ms ordinary proposal (raw70ms); width80 without stall rejects TooEarly because ETA exceeds250ms; all three changes permit a68ms candidate. The old15ms rescue at27.958s is not evidence that this hypothetical68ms action succeeds.','',
   'The wheel preview uses the last accepted negative pre-input378.91rpm and current1.81rpm at27.921s. The existing midpoint-bias guard reduces raw70ms to68ms (no extra peak-qualified reduction). The selector requests68ms and a separate evaluation of the same raw70ms returns68ms again, so width preservation succeeds. Sending raw70ms as the requested width would instead fail this existing guard.','',
   'A reconstructed v11 full-run count can differ from its3 real audits: sampled body times, publication scheduling and conservative old-command-first ties are not the exact runner. The failure-window comparison is performed identically across variants.']
 report+=['','## Local prediction checks','',
  'Returning is evidence from three already-observed frames, not guaranteed current physical direction. Future camera frames are scorer-only and are not independent angular truth. Current-state point accuracy is mostly similar to holding; the purpose is to separate usable local observations from missing historical matches.','',
  '| Log | Local side margin<=.75: values | Local MAE | Hold MAE | Empirical margin coverage | Actual local-fallback values | Fallback MAE | Fallback linear MAE | Fallback max | Coarse opposite Returning |',
  '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
 for case,groups in score_summary.items():
  g=groups['local_side_margin_le075'];f=groups['combined_local_selected']
  if not g['local'] or not f['local']:continue
  report.append(f"| {case} | {g['angle_values']} | {g['local']['mae_deg']:.4f} | {g['hold']['mae_deg']:.4f} | {100*g['margin_coverage_fraction']:.1f}% | {f['angle_values']} | {f['local']['mae_deg']:.4f} | {f['linear']['mae_deg']:.4f} | {f['local']['max_deg']:.4f} | {f['returning_opposite_direction_values']}/{f['returning_direction_check_values']} |")
 report+=['','The local correction is limited to25ms and0.5degree instead of extrapolating the full observation age. Across all Returning strata it reduces mean error compared with old straight-line extrapolation; in the small actual-fallback strata, straight-line prediction can have lower MAE than the conservative local point estimate. No general accuracy superiority is claimed.','',
  'Margins are heuristic allowances, not calibrated error bounds. Coarse direction uses future camera central differences, which can span a reversal and share recognition error. Small fallback strata do not establish a reliable error probability. Historical score is preserved rather than replaced by invented confidence.','',
  'V11 logs use actual saved ready_ms; older logs use approximate delivered+image-processing+CAD times. Added delays are timing sensitivity, not measured v12 control deadlines. New local computation/publication overhead remains unmeasured. Header hashes and the compiled harness hash are in the JSON.']
 args.out.with_suffix('.md').write_text('\n'.join(report)+'\n',encoding='utf8')


SELECTION_INCLUDE='#include "foot_support_pose.h"'
SELECTION_HEADERS=['foot_support_pose.h']
SELECTION_HELPER=r'''
struct SelectedPose {
 unsigned run=0,frame=0,source=0;
 bool ready=false,usable=false;
 float left=NAN,right=NAN,lu=INFINITY,ru=INFINITY;
 int phase_left=0,phase_right=0;
};
SelectedPose convert_pose(const foot_support_pose::Selection& s,const foot_phase_prediction::Projection& h){
 SelectedPose out;out.run=h.run_id;out.frame=h.frame_ms;out.source=static_cast<unsigned>(s.source);
 out.ready=s.pose.ready();out.usable=s.usable;out.left=s.pose.left;out.right=s.pose.right;
 out.lu=s.pose.left_uncertainty;out.ru=s.pose.right_uncertainty;
 out.phase_left=static_cast<int>(s.pose.phase_left);out.phase_right=static_cast<int>(s.pose.phase_right);return out;
}
SelectedPose select_pose(const foot_phase_prediction::Projection& h,const foot_local_prediction::Projection& l,unsigned now){
 return convert_pose(foot_support_pose::select(h,l,now),h);
}
SelectedPose history_pose(const foot_phase_prediction::Projection& h,unsigned now){
 foot_local_prediction::Projection empty;
 return convert_pose(foot_support_pose::select(h,empty,now),h);
}
'''

if __name__=='__main__':main()
