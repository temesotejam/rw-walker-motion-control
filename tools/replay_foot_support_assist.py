"""Counterfactual eligibility replay; does not simulate response to new drive.

Uses compiled estimator and selector on unchanged old camera/IMU trajectories.
Ordinary proposal widths are explicit fixed-width sensitivities, not a replay
of tilt_priority adaptation or downstream wheel guards.
"""
import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path
import subprocess
import tempfile
from collections import Counter

ROOT=Path(__file__).resolve().parents[1]
CASES=['137267573','267557373','27908524']
WIDTHS=[20,30,40,50,55,60,0,65534,65535]
def mode_name(w):return 'controller_shadow' if w==0 else 'coast15_diagnostic' if w==65534 else 'coast15_evidence_diagnostic' if w==65535 else str(w)
REASONS=['Selected','Inactive','Identity','Time','PoseUnavailable','PoseUncertain','StaleFrame','Nonfinite','SmallForward','WrongMotion','OutsideWindow','Speed','NoOrdinaryDrive','Width','InsufficientLead','TooEarly','CoastNeedsDeceleration']
CPP=r'''
#include "foot_phase_prediction.h"
#include "foot_support_assist.h"
#include "tilt_priority.h"
#include "foot_support_deceleration.h"
#include <iostream>
#include <iomanip>
#include <deque>
#include <utility>
int main(){
 foot_phase_prediction::Predictor p;foot_phase_prediction::Projection projection;
 tilt_priority::Controller controller;unsigned checks=0,mismatches=0;
 foot_support_deceleration::Tracker deceleration;
 float incoming_peak_amplitude=0;
 const unsigned pose_lead=30;const float min_confidence=.5f;const bool tracker_imu=false;
 unsigned type,id,run=0,frame,received,now,valid,side;float l,r,angle,rate;
 unsigned peak=0,last_accepted=0;int peak_side=0;unsigned last_check=0;bool consumed=false,attempted=false;bool used[9]={false,false,false,false,false,false,false,false,false};
 const unsigned widths[9]={20,30,40,50,55,60,0,65534,65535};
 std::deque<std::pair<unsigned,float>> speeds;bool decelerating=false;float speed_drop=0;
 std::cout<<std::setprecision(10);
 while(std::cin>>type){
  if(type==0){std::cin>>run>>frame>>received>>valid>>l>>r;p.observe(run,frame,received,valid,l,r);projection=p.projection();}
  else if(type==1){float amp,target;unsigned matched;std::cin>>now>>peak_side>>amp>>target>>matched;peak=now;for(auto& x:used)x=false;
   incoming_peak_amplitude=amp;
   speeds.clear();decelerating=false;speed_drop=0;deceleration.reset();consumed=attempted=false;
   ++checks;if(std::fabs(controller.target()-target)>.01f)++mismatches;controller.peak(peak_side,now,amp);}
  else if(type==3){unsigned at,width,temporary;int direction;float crossing_rate;
   std::cin>>at>>direction>>width>>temporary>>crossing_rate;
   tilt_priority::Input input;input.now_ms=at;input.preset_deg=8;input.target_deg=controller.target();input.next_side=direction;
   input.crossing_rate_dps=crossing_rate;input.hardware_cap_ms=temporary?15:controller.ordinaryRiseCap(direction,at,150);input.hardware_healthy=true;
   auto choice=controller.propose(input);choice.selected=true;choice.coast=width==0;choice.current_ma=width?1200:0;
   choice.width_ms=width;choice.direction=direction;choice.target_deg=controller.target();
   controller.crossing(direction,at);
   if(!temporary)controller.ordinaryResult(direction,at,width,width!=0);
   controller.confirm(direction,at,choice,true,temporary,!temporary);
   consumed=true;last_accepted=at;if(temporary)attempted=true;
  }
  else if(type==5){unsigned at;std::cin>>at;attempted=true;}
  else if(type==4){unsigned at,pulse;float body_rate,body_angle;std::cin>>at>>body_rate>>body_angle>>pulse;
   const float speed=-body_rate*peak_side;
   if(pulse || consumed || body_angle*peak_side<=0 || speed<=0){deceleration.reset();decelerating=false;}
   else if(tracker_imu)decelerating=deceleration.update(run,peak,at,speed);
   if(!peak || speed<=0){speeds.clear();decelerating=false;continue;}
   while(!speeds.empty() && at-speeds.front().first>100)speeds.pop_front();
   speeds.push_back({at,speed});float maximum=speed;
   for(auto v:speeds)if(v.second>maximum)maximum=v.second;
   speed_drop=maximum-speed;
  }
  else{
   std::cin>>id>>run>>now>>angle>>rate;
   if(consumed || attempted || (last_accepted && (now<=last_accepted || now-last_accepted<250)))continue;
   if(last_check && now>=last_check && now-last_check<5)continue;
   last_check=now;
   if(!tracker_imu)decelerating=deceleration.update(run,peak,now,-rate*peak_side);
   auto q=projection.predict(now,pose_lead);
   const float lower_left=q.left-q.left_uncertainty,lower_right=q.right-q.right_uncertainty;
   const auto dominant_phase=lower_left>=lower_right?q.phase_left:q.phase_right;
   const float forward=std::fmax(0.f,std::fmax(lower_left,lower_right));
   const float asymmetry=std::fmax(0.f,std::fabs(q.left-q.right)-q.left_uncertainty-q.right_uncertainty);
   const bool anticipated=dominant_phase==foot_phase_prediction::Phase::Returning && forward>=1.f && asymmetry>=1.f && incoming_peak_amplitude<=8.5f;
   for(unsigned k=0;k<9;++k){
    if(used[k])continue;
    foot_support_assist::Input in;
    in.active=true;in.run_id=run;in.foot_run_id=projection.run_id;in.peak_id=peak;in.peak_side=peak_side;
    in.now_ms=now;in.frame_ms=projection.frame_ms;in.pose_ready=q.ready();
    in.pose_confident=q.confidence>=min_confidence && q.method==foot_phase_prediction::Method::History;
    in.left_deg=q.left;in.right_deg=q.right;in.left_uncertainty_deg=q.left_uncertainty;in.right_uncertainty_deg=q.right_uncertainty;
    /*ANTICIPATED_ASSIGN*/
    in.body_angle_deg=angle;in.body_rate_dps=rate;in.normal_drive_selected=true;in.normal_width_ms=widths[k];
    bool coast=false;
    if(widths[k]==0 || widths[k]>=65534){
     tilt_priority::Input ordinary;ordinary.now_ms=now;ordinary.preset_deg=8;ordinary.target_deg=controller.target();ordinary.next_side=-peak_side;
     ordinary.crossing_rate_dps=std::fabs(rate);ordinary.hardware_cap_ms=controller.ordinaryRiseCap(-peak_side,now,150);ordinary.hardware_healthy=true;
     auto choice=controller.propose(ordinary);coast=choice.coast;in.normal_drive_selected=choice.selected&&!choice.coast;
     in.normal_coast_selected=choice.selected&&choice.coast;in.inward_decelerating=decelerating;
     in.normal_width_ms=choice.width_ms;
     if(in.normal_drive_selected)in.normal_width_ms=controller.balanceTrimCap(now,-peak_side,in.normal_width_ms);
     if(widths[k]>=65534){in.normal_drive_selected=false;in.normal_coast_selected=choice.selected&&choice.coast;in.normal_width_ms=0;
      in.inward_decelerating=widths[k]==65534 || decelerating;}
    }
    auto d=foot_support_assist::decide(in);
    std::cout<<id<<' '<<widths[k]<<' '<<int(d.reason)<<' '<<peak<<' '<<peak_side<<' '
      <<q.left<<' '<<q.right<<' '<<q.confidence<<' '<<q.left_uncertainty<<' '<<q.right_uncertainty<<' '
      <<q.age_ms<<' '<<d.forward_deg<<' '<<d.threshold_deg<<' '<<d.estimated_zero_ms<<' '
      <<(d.selected?d.width_ms:in.normal_width_ms)<<' '<<in.normal_drive_selected<<' '<<coast<<' '<<controller.target()<<' '
      <<decelerating<<' '<<speed_drop<<' '<<anticipated<<' '<<incoming_peak_amplitude<<' '<<in.normal_width_ms<<'\n';
    if(d.selected)used[k]=true;
   }
  }
 }
 std::cerr<<checks<<' '<<mismatches<<'\n';
}
'''

def read(p):
 with p.open(encoding='utf8',newline='') as f:return list(csv.DictReader(f))
def finite(s):
 try:
  v=float(s);return v if math.isfinite(v) else 0
 except (ValueError,TypeError):return 0
def good(r):
 return all(r[k]=='True' for k in ['timestamp_valid','frame_valid','zero_ready','left_valid','right_valid','left_in_range','right_in_range'])
def number(s):
 v=float(s);return v if math.isfinite(v) else None

def replay(exe,path,delay,body_window):
 feet=read(path/'foot_angles.csv');samples=read(path/'timeseries.csv')
 peaks=read(path/'energy_control_autonomous_peak_events.csv');old=read(path/'energy_control_autonomous_zero_cross_events.csv')
 cadpath=path/'cad_support_shadow.csv'
 cad={int(r['sequence']):int(float(r['compute_us'])) for r in read(cadpath)} if cadpath.exists() else {}
 f0=next(r for r in feet if int(r['measurement_time_us'])>=0)
 epoch=int(f0['frame_us'])-int(f0['measurement_time_us']);logepoch=int(f0['frame_us'])-int(f0['log_time_us'])
 events=[];queries={}
 for f in feet:
  t=(int(f['frame_us'])-epoch)//1000
  if t<0:continue
  received=(int(f['delivered_us'])+int(f['processing_us'])+cad.get(int(f['sequence']),0)-epoch+999)//1000
  valid=good(f) and f['state_id']=='3'
  events.append((received+delay,0,f"0 {f['run_id']} {t} {received} {int(valid)} {finite(f['left_deg'])} {finite(f['right_deg'])}"))
 for p in peaks:
  t=int(p['peak_time_ms']);events.append((t,1,f"1 {t} {p['physical_peak_side']} {p['peak_amplitude_deg']} {p['target_peak_deg']} {int(p['pending_command_matched']=='True')}"))
 for e in old:
  if e['event_kind']=='angle_rate_early_trial':events.append((int(float(e['zero_cross_time_ms'])),2,f"5 {e['zero_cross_time_ms']}"))
  if e['event_kind']=='strong_start_kick' or e['valid']!='True':continue
  t=int(float(e['zero_cross_time_ms']));width=int(float(e['pulse_width_ms']))
  temporary=int(e['event_kind']!='energy_control_zero_cross');rate=finite(e['zero_cross_abs_rate_dps'])
  events.append((t,2,f"3 {t} {e['physical_next_peak_side']} {width} {temporary} {rate}"))
 for s in samples:
  if s['state_id']!='3' or float(s['log_time_s'])<4:continue
  t=int(float(s['t_test_ms']));angle=finite(s['pitch_mekf_detector_relative_deg']);rate=finite(s['physical_roll_rate_dps'])
  absolute=logepoch+int(float(s['log_time_s'])*1e6)
  if s['body_rate_sample_time_us'] and absolute-int(s['body_rate_sample_time_us'])>10000:continue
  events.append((t,3,f"4 {t} {rate} {angle} {s['pulse_active']}"))
  # Match the wrapper's cold-check window and exclude active old pulses.
  if abs(angle)>body_window or s['pulse_active']!='0':continue
  ident=len(queries);queries[ident]={'time_ms':t,'body_angle_deg':angle,'body_rate_dps':rate}
  events.append((t,4,f"2 {ident} {f0['run_id']} {t} {angle} {rate}"))
 lines=[e[2] for e in sorted(events,key=lambda e:(e[0],e[1]))]
 proc=subprocess.run([str(exe)],input='\n'.join(lines)+'\n',capture_output=True,text=True,check=True)
 counts={w:Counter() for w in WIDTHS};selected=[];near_misses=[];small_forward_opportunities=[]
 for line in proc.stdout.splitlines():
  p=line.split();ident,width,reason,peak,side=map(int,p[:5]);counts[width][REASONS[reason]]+=1
  q=queries[ident];t=q['time_ms']
  if width==50 and reason in [8,10,11,14,15]:
   forward=number(p[11]);distance=q['body_angle_deg']*side;speed=-q['body_rate_dps']*side
   near={'reason':REASONS[reason],**q,'peak_ms':peak,'peak_side':side,'left_deg':number(p[5]),'right_deg':number(p[6]),
    'confidence':number(p[7]),'left_uncertainty_deg':number(p[8]),'right_uncertainty_deg':number(p[9]),'frame_age_ms':int(p[10]),
    'forward_minus_uncertainty_deg':forward,'body_distance_deg':distance,'inward_speed_dps':speed,
    'estimated_zero_ms':1000*distance/speed if speed>0 else None}
   if reason!=8:near_misses.append(near)
   if reason==8 and .8<=distance<=3 and 6<=speed<=35 and 70<=1000*distance/speed<=250:small_forward_opportunities.append(near)
  if reason:continue
  samepeak=[e for e in old if int(float(e['previous_peak_time_ms'] or 0))==peak and e['event_kind']!='strong_start_kick']
  accepted=[e for e in samepeak if e['output_executed']=='True' or e['valid']=='True']
  accepted.sort(key=lambda e:float(e['zero_cross_time_ms']))
  reference=accepted[0] if accepted else None
  preceding=[e for e in accepted if float(e['zero_cross_time_ms'])<=t]
  selected.append({'case':path.name,'control_delay_ms':delay,**q,'normal_width_mode':mode_name(width),'normal_width_assumed_ms':int(p[14]),'peak_ms':peak,'peak_side':side,'next_side':-side,
   'requested_width_ms':int(p[14]),'ordinary_proposal_width_ms':int(p[22]),
   'left_deg':number(p[5]),'right_deg':number(p[6]),'confidence':number(p[7]),'left_uncertainty_deg':number(p[8]),'right_uncertainty_deg':number(p[9]),
   'frame_age_ms':int(p[10]),'forward_minus_uncertainty_deg':number(p[11]),'threshold_deg':number(p[12]),'estimated_zero_ms':number(p[13]),
   'compiled_tracker_confirmed':p[18]=='1','raw_inward_speed_drop_from_100ms_max_dps':number(p[19]),
   'anticipated_shape_need':p[20]=='1','incoming_peak_amplitude_deg':number(p[21]),
   'old_reference_event_kind':reference['event_kind'] if reference else None,
   'old_reference_time_ms':float(reference['zero_cross_time_ms']) if reference else None,
   'old_reference_width_ms':int(float(reference['pulse_width_ms'])) if reference else None,
   'old_reference_output_executed':reference['output_executed']=='True' if reference else None,
   'ms_before_old_reference':float(reference['zero_cross_time_ms'])-t if reference else None,
   'old_event_already_accepted_in_peak':bool(preceding)})
 checks,mismatches=map(int,proc.stderr.split())
 return {'sampled_near_center_queries':len(queries),'recorded_peak_target_checks':checks,'shadow_target_mismatches':mismatches,
  'reason_counts':{str(w):dict(v) for w,v in counts.items()},
  'near_misses_after_forward_gate':near_misses,
  'max_forward_among_small_forward_lead_opportunities':max(small_forward_opportunities,key=lambda r:r['forward_minus_uncertainty_deg']) if small_forward_opportunities else None,
  'selected_counts':{mode_name(w):sum(r['normal_width_mode']==mode_name(w) for r in selected) for w in WIDTHS},
  'before_old_accepted_counts':{mode_name(w):sum(r['normal_width_mode']==mode_name(w) and not r['old_event_already_accepted_in_peak'] for r in selected) for w in WIDTHS},'candidates':selected}

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--cases',nargs='+',default=CASES)
 ap.add_argument('--base',type=Path,default=ROOT/'analysis/2026-11-11-walking');ap.add_argument('--control-delays-ms',nargs='+',type=int,default=[0,15])
 ap.add_argument('--out',type=Path,default=ROOT/'analysis/2026-11-11-walking/foot_support_assist_v11_replay.json')
 ap.add_argument('--pose-lead-ms',type=int,default=0);ap.add_argument('--min-confidence',type=float,default=.5)
 ap.add_argument('--tracker-cadence',choices=['candidate','imu'],default='candidate')
 ap.add_argument('--tracker-drop-dps',type=float,default=4);ap.add_argument('--tracker-confirm-ms',type=int,default=10)
 ap.add_argument('--anticipated-shape-need',action='store_true',default=True)
 ap.add_argument('--without-shape-anticipation',dest='anticipated_shape_need',action='store_false')
 ap.add_argument('--profile',choices=['default','legacy_window_diagnostic','wide_window_diagnostic','expanded_window_diagnostic'],default='default');args=ap.parse_args()
 headers={n:(ROOT/'src'/n).read_bytes() for n in ['foot_phase_prediction.h','foot_support_assist.h','tilt_priority.h','foot_support_deceleration.h']}
 if args.anticipated_shape_need:
  policy=headers['foot_support_assist.h'].decode('utf8')
  if 'anticipated_shape_need' not in policy:
   policy=policy.replace('bool inward_decelerating=false;','bool inward_decelerating=false,anticipated_shape_need=false;')
   policy=policy.replace('coast && !in.inward_decelerating','coast && !(in.inward_decelerating || in.anticipated_shape_need)')
   headers['foot_support_assist.h']=policy.encode('utf8')
 tracker=headers['foot_support_deceleration.h'].decode('utf8')
 if args.tracker_cadence=='imu':tracker=tracker.replace('capacity=24','capacity=64')
 if args.tracker_drop_dps!=4:tracker=tracker.replace('highest-filtered<4.f',f'highest-filtered<{float(args.tracker_drop_dps)}f')
 if args.tracker_confirm_ms!=10:tracker=tracker.replace('now-pending_since_ms_>=10u',f'now-pending_since_ms_>={args.tracker_confirm_ms}u')
 headers['foot_support_deceleration.h']=tracker.encode('utf8')
 body_window=5
 if args.profile!='default':
  # Exploratory sensitivity only: transform a TEMPORARY compiled copy,
  # leaving firmware source untouched. This is not a recommended calibration.
  text=headers['foot_support_assist.h'].decode('utf8')
  new='std::fmax(1.5f,std::fmin(3.f,1.f+.45f*out.forward_deg+.25f*out.asymmetry_deg))' if args.profile=='legacy_window_diagnostic' else 'std::fmax(2.f,std::fmin(5.f,1.5f+out.forward_deg+.5f*out.asymmetry_deg))' if args.profile=='wide_window_diagnostic' else 'std::fmax(1.5f,std::fmin(5.f,1.f+1.2f*out.forward_deg+.3f*out.asymmetry_deg))'
  text,n=re.subn(r'out.threshold_deg=std::fmax\(.*?\);',lambda m:'out.threshold_deg='+new+';',text,count=1,flags=re.S);assert n==1
  speed=45 if args.profile=='expanded_window_diagnostic' else 35
  text,n=re.subn(r'kMaxInwardSpeedDps=\d+\.f',f'kMaxInwardSpeedDps={speed}.f',text,count=1);assert n==1
  headers['foot_support_assist.h']=text.encode('utf8');body_window=3 if args.profile=='legacy_window_diagnostic' else 5
 output={'scope':'Counterfactual eligibility on unchanged old trajectories; no prediction of response to relocated drive.',
  'normal_proposal':'Fixed 20/30/40/50/55/60ms sensitivities plus width mode 0 compiled tilt_priority shadow. Shadow is force-updated by recorded valid old cross/confirm/peak events, with staged controller target, actual width/coast, temporary early flags, ordinaryRiseCap and balanceTrimCap; not full runner/wheel/foot-guard reproduction.',
  'arrival_model':'delivered + processing + saved CAD compute, then explicit control-delay sensitivity; camera driver timestamp is not calibrated exposure time',
  'selection_model':'At most one first eligible selector candidate per recorded peak per width; rows after an old accepted event are flagged and counted separately',
  'coast15_diagnostic':'Mode65534 forces decelerating=true only when shadow ordinary selected coast. Mode65535 and controller_shadow use actual compiled 3-sample-median Tracker at the 5ms selector cadence. It needs4dps drop from100ms filtered maximum for10ms/3samples; gaps>15ms reset. Normal width remains zero for coast; compiled selector assigns15ms.',
  'profile':args.profile,'body_wrapper_window_deg':body_window,
  'pose_lead_ms':args.pose_lead_ms,'min_confidence':args.min_confidence,'tracker_cadence':args.tracker_cadence,
  'tracker_drop_dps':args.tracker_drop_dps,'tracker_confirm_ms':args.tracker_confirm_ms,
  'anticipated_shape_need_enabled':args.anticipated_shape_need,
  'anticipated_definition':'Dominant conservative-lower-margin foot Returning; forward lower>=1deg, absolute foot asymmetry minus both margins>=1deg, incoming recorded body peak amplitude<=8.5deg. This is a shape proxy, not evidence of future contact or loss. Other gates remain.',
  'profile_caution':'wide: min(5,max(2,1.5+forward+.5*asymmetry)), speed35. expanded: min(5,max(1.5,1+1.2*forward+.3*asymmetry)), speed45. Both transform only a temporary header, not CAD calibrated or evidence of useful drive.',
  'header_sha256':{k:hashlib.sha256(v).hexdigest() for k,v in headers.items()},'cases':{}}
 with tempfile.TemporaryDirectory(prefix='support_replay_') as td:
  for n,b in headers.items():(Path(td)/n).write_bytes(b)
  source=CPP.replace('pose_lead=30',f'pose_lead={args.pose_lead_ms}').replace('min_confidence=.5f',f'min_confidence={args.min_confidence}f').replace('tracker_imu=false',f'tracker_imu={str(args.tracker_cadence=="imu").lower()}')
  source=source.replace('/*ANTICIPATED_ASSIGN*/','in.anticipated_shape_need=anticipated;' if args.anticipated_shape_need else '')
  cpp=Path(td)/'replay.cpp';exe=Path(td)/'replay.exe';cpp.write_text(source,encoding='utf8')
  subprocess.run(['g++','-std=c++11','-O2',str(cpp),'-o',str(exe)],check=True)
  for c in args.cases:
   output['cases'][c]={}
   for delay in args.control_delays_ms:
    result=replay(exe,args.base/c,delay,body_window);output['cases'][c][str(delay)]=result
    print(c,'delay',delay,'candidates',result['selected_counts'],'before_old_input',result['before_old_accepted_counts'])
 args.out.write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
 rows=[r for c in output['cases'].values() for d in c.values() for r in d['candidates']]
 if rows:
  with args.out.with_suffix('.csv').open('w',encoding='utf8',newline='') as f:
   w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 report=['# Foot support assist counterfactual eligibility', '',
  'This is a compiled-estimator/selector replay on unchanged recorded camera and body trajectories. It is not a simulation of the physical response to a new motor command. Every candidate remains subject to runner timing, driver, battery, wheel, current and width-preservation guards that are not fully reproduced here.', '',
  f'Profile: {args.profile}. Pose lead: {args.pose_lead_ms} ms. Minimum confidence: {args.min_confidence}. Tracker cadence: {args.tracker_cadence}. Shape anticipation: {args.anticipated_shape_need}.', '',
  'Ordinary controller shadow is causally force-updated with recorded accepted old crossings, command widths/coast results and peaks. Later old data is still used after an earlier hypothetical candidate: candidates from different peaks are separate counterfactual eligibility checks, not one simulated new run.', '',
  '| Log | Extra control delay | Controller-shadow candidates | Coast additions | Ordinary-drive relocations | Logged-stage mismatches |',
  '|---|---:|---:|---:|---:|---:|']
 for c,d in output['cases'].items():
  for delay,result in d.items():
   actual=[r for r in result['candidates'] if r['normal_width_mode']=='controller_shadow']
   coast=sum(r['ordinary_proposal_width_ms']==0 for r in actual)
   report.append(f"| {c} | {delay} ms | {len(actual)} | {coast} | {len(actual)-coast} | {result['shadow_target_mismatches']}/{result['recorded_peak_target_checks']} |")
 report+=['', 'The final default shape evidence uses the Returning phase of the foot with the larger conservative forward angle, conservative forward >=1 degree, conservative inter-foot asymmetry >=1 degree, and incoming body peak <=8.5 degrees. It does not infer contact, a calibrated CAD transition, future deceleration or required mechanical work.', '',
  'The strict camera confidence, margin and age gates remain. A coast exception requests 15 ms. Actual deceleration is separate causal evidence. The earlier two-consecutive-weak-body-peak idea was not adopted.', '',
  'Primary timestamps below use the zero-extra-delay assumption. A reference width of zero is an old coast decision, not a delivered pulse.', '',
  '| Log | Candidate time | Requested width | Old decision time | Old width | Advance |', '|---|---:|---:|---:|---:|---:|']
 for c,d in output['cases'].items():
  for r in d.get('0',{}).get('candidates',[]):
   if r['normal_width_mode']=='controller_shadow':
    report.append(f"| {c} | {r['time_ms']/1000:.3f} s | {r['requested_width_ms']} ms | {r['old_reference_time_ms']/1000:.3f} s | {r['old_reference_width_ms']} ms | {r['ms_before_old_reference']:.0f} ms |")
 report+=['', 'The former run 27908524 has no candidate immediately before its approximately26.5-second stopping episode in this replay. An eligibility result does not establish that this failure is prevented. Camera-frame timestamps are not calibrated exposure timestamps; publication/control latency remains approximate.']
 args.out.with_suffix('.md').write_text('\n'.join(report)+'\n',encoding='utf8')

if __name__=='__main__':main()
