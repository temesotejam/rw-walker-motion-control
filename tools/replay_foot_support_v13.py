"""v12/v13 cumulative ablations on recorded trajectories, not new dynamics.

Uses the v12 event adapter/parser, actual shared selection and policy headers.
Recorded commands force-update control/guard state. Each new candidate is an
independent eligibility comparison; it cannot prove the response to that input.
"""
from pathlib import Path
import argparse,hashlib,inspect,json,subprocess,tempfile
import replay_foot_support_v12 as old

ROOT=Path(__file__).resolve().parents[1]
VARIANTS=['v12','v13_turn_only','v13_amount_only','v13_turn_and_amount']


def source():
    s=old.CPP
    s=s.replace('/*SELECTION_INCLUDE*/',old.SELECTION_INCLUDE+'\n#include "foot_support_assist_v13.h"')
    s=s.replace('/*SELECTION_HELPER*/',old.SELECTION_HELPER)
    s=s.replace('auto pose=k==0?history_pose(hp,now):chosen;',
                'auto pose=(k==1||k==3)?convert_pose(foot_support_pose::select(hp,lp,now,true),hp):chosen;')
    s=s.replace('foot_support_assist_v12::Input in;', 'foot_support_assist_v13::Input in;')
    s=s.replace('in.normal_width_ms=k==0?drive.width_ms:normal_guarded;', 'in.normal_width_ms=normal_guarded;')
    s=s.replace('if(k>0 && !wheel_valid)', 'if(!wheel_valid)')
    s=s.replace('in.prediction_source=static_cast<foot_support_assist_v12::PredictionSource>(pose.source);',
      '''in.prediction_source=static_cast<foot_support_assist_v13::PredictionSource>(pose.source);
    in.local_turn_confirmed=pose.source==3;in.raw_normal_width_ms=drive.width_ms;
    in.raw_boost_ms=k>=2?foot_support_assist_v13::boostMs(in):0;
    auto boosted=wheel;
    in.boosted_guarded_width_ms=in.normal_drive_selected?boosted.limit(-peak_side,wheel_rpm,wheel_valid,now,drive.width_ms+in.raw_boost_ms):0;''')
    s=s.replace('if(k==0){auto d=foot_support_assist::decide(in);', 'if(k==0){auto d=foot_support_assist_v12::decide(in);')
    a='else{foot_support_assist_v12::Limits limits;limits.max_ordinary_width_ms=k==1?60:80;limits.allow_stall_route=k==3;\n     auto d=foot_support_assist_v12::decide(in,limits);'
    b='else{foot_support_assist_v13::Limits limits;limits.allow_first_turn=k!=2;limits.allow_amount_boost=k>=2;\n     auto d=foot_support_assist_v13::decide(in,limits);'
    assert a in s;s=s.replace(a,b)
    s=s.replace('const unsigned backend_input=coast?width:drive.width_ms;', 'const unsigned backend_input=coast?width:drive.width_ms+in.raw_boost_ms;')
    a="<<guarded_again<<' '<<width_preserved<<'\\n';";assert a in s
    s=s.replace(a,"<<guarded_again<<' '<<width_preserved<<' '<<in.raw_boost_ms<<' '<<backend_input<<'\\n';")
    return s


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases',nargs='+',default=old.CASES+['611622951','43750941'])
    p.add_argument('--delays-ms',nargs='+',type=int,default=[0,10,25])
    p.add_argument('--out',type=Path,default=ROOT/'analysis/2026-11-13-walking/v13_replay.json')
    a=p.parse_args();old.VARIANTS=VARIANTS
    # The established parser keeps all diagnostic columns, plus the new raw
    # increment and the actual raw command passed to the backend wheel guard.
    run_src=inspect.getsource(old.run_case)
    marker="   if path.name=='141282685'"
    assert marker in run_src
    run_src=run_src.replace(marker,"   row.update({'raw_boost_ms':int(p[44]),'backend_raw_width_ms':int(p[45])})\n"+marker)
    exec(compile(run_src,str(Path(old.__file__)),'exec'),old.__dict__)
    results={}
    headers=['foot_phase_prediction.h','foot_local_prediction.h','foot_pose_prediction.h','foot_support_assist.h',
             'foot_support_assist_v12.h','foot_support_assist_v13.h','foot_support_pose.h',
             'foot_support_deceleration.h','tilt_priority.h','wheel_cycle_guard.h']
    frozen={name:(ROOT/'src'/name).read_bytes() for name in headers}
    with tempfile.TemporaryDirectory(prefix='v13_replay_') as tmp:
        tmp=Path(tmp)
        for name,data in frozen.items():(tmp/name).write_bytes(data)
        (tmp/'replay.cpp').write_text(source(),encoding='utf8')
        subprocess.run(['g++','-std=c++11','-O2',str(tmp/'replay.cpp'),'-o',str(tmp/'replay.exe')],check=True)
        for case in a.cases:
            results[case]={}
            for delay in a.delays_ms:
                result,_=old.run_case(tmp/'replay.exe',old.case_dir(case),delay)
                results[case][str(delay)]=result
                print(case,delay,result['candidate_counts'],'stage_mismatch',result['peak_target_mismatches'])
    data={'scope':__doc__,'source_sha256':{name:hashlib.sha256(b).hexdigest() for name,b in frozen.items()},'cases':results}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(data,indent=2),encoding='utf8')
    lines=['# v13 recorded-trajectory candidate comparison','',
           'Recorded old commands/peaks force-update the controller and wheel guard. New physical response, full driver/current/hardware health and precise scheduling are not simulated. Each first candidate is independent.','',
           '| Log | Added publication delay | v12 | Turn only | Amount only | Both |','|---|---:|---:|---:|---:|---:|']
    for case,delays in results.items():
        for delay,result in delays.items():
            lines.append('| '+case+' | '+delay+' ms | '+' | '.join(str(result['candidate_counts'][name]) for name in VARIANTS)+' |')
    lines+=['','## First candidates after final large negative peak','',
            '| Log | Delay | Variant | Time | Source | Raw ordinary | Added | Raw backend | Guarded | Width match |','|---|---:|---|---:|---:|---:|---:|---:|---:|---|']
    for case,lo in [('611622951',28558),('141282685',27622)]:
        if case not in results:continue
        for delay,result in results[case].items():
            for name in VARIANTS:
                found=next((c for c in result['candidates'] if c['variant']==name and c['time_ms']>lo),None)
                if not found:lines.append(f'| {case} | {delay} | {name} | none | - | - | - | - | - | - |');continue
                c=found;lines.append(f"| {case} | {delay} | {name} | {c['time_ms']/1000:.3f} s | {c['source']} | {c['ordinary_proposal_ms']} | {c['raw_boost_ms']} | {c['backend_raw_width_ms']} | {c['backend_guarded_width_ms']} | {c['width_preserved_after_guard']} |")
    lines+=['','Source 3 is tentative first reversal, not confirmed Returning or contact. Width addition is an experimental heuristic, not identified required work. Counts on already assisted v12 trajectories are not an unassisted control or a proof of benefit.','']
    a.out.with_suffix('.md').write_text('\n'.join(lines),encoding='utf8')


if __name__=='__main__':main()
