"""Native v19 admission tests and optional frozen-state replay of a v18 trial.

No coefficients, energy amounts or widths are fitted. Rejected relocation
preserves the ordinary ZERO opportunity; replay does not predict its changed
trajectory or prove that the physical fall would have been prevented.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CPP = r'''
#include "energy_admission_v19.h"
#include "half_cycle_energy_budget.h"
#include <cassert>
#include <cstring>
#include <iomanip>
#include <iostream>
namespace a=energy_admission_v19;
namespace e=walking_return_energy;
namespace b=half_cycle_energy_budget;
e::Input input(){
 e::Input in;in.side=1;in.rate_dps=32;in.body_angle_deg=-.7f;in.target_deg=6.5f;
 in.previous_peak_deg=6;in.same_side_peak_deg=6;in.initial_current_ma=0;
 in.goal_current_ma=345;in.arrival_bias_j=.00013f;return in;
}
void tests(){
 lateral_energy::Geometry g;lateral_energy::CurrentTable table;assert(table.prepare(.031652216f));
 auto in=input();const auto d=e::decide(g,table,in);
 assert(e::earlyAdmissible(d) && d.passive_rate_supported && d.work_rate_supported && d.charge_supported);
 unsigned char before[sizeof(d)];std::memcpy(before,&d,sizeof(d));
 const auto accepted=a::assess(a::Scope::OptionalMain,d);
 assert(accepted.applies && accepted.allowed && !accepted.unsupported_mask && accepted.reason==a::Reason::Allowed);
 assert(std::memcmp(before,&d,sizeof(d))==0); // No adjustment of J/current/width.
 for(unsigned mask=0;mask<8;++mask){auto x=d;
  x.passive_rate_supported=!(mask&1);x.work_rate_supported=!(mask&2);x.charge_supported=!(mask&4);
  const auto q=a::assess(a::Scope::OptionalMain,x);
  assert(q.applies && q.allowed==(mask==0) && q.unsupported_mask==mask);
  assert(a::optionalMainAdmissible(x)==q.allowed);
  for(auto scope:{a::Scope::OrdinaryZero,a::Scope::ExplicitReturnAid}){
   const auto exempt=a::assess(scope,x);assert(!exempt.applies && exempt.allowed && exempt.unsupported_mask==mask);}
 }
 for(unsigned bad=0;bad<9;++bad){auto x=d;
  if(bad==0)x.valid=false;
  if(bad==1)x.selected=false;
  if(bad==2)x.width_ms=0;
  if(bad==3)x.coast=true;
  if(bad==4)x.saturated=true;
  if(bad==5)x.request_j=0;
  if(bad==6)x.predicted_work_j=x.request_j-.000001f;
  if(bad==7)x.q_mas=-1;
  if(bad==8)x.predicted_work_j=NAN;
  assert(!a::optionalMainAdmissible(x));}
 assert(!a::assess(static_cast<a::Scope>(99),d).allowed);
 // Exact inherited range endpoints are inclusive, with no added margins.
 for(int side:{-1,1}){const auto p=walking_energy_policy::passiveCoefficients(side);
  const auto w=lateral_energy::coefficients(side);
  for(float rate:{p.rate_min_dps,p.rate_max_dps,w.rate_min_dps,w.rate_max_dps}){
   auto x=in;x.side=side;x.rate_dps=rate;const auto q=e::decide(g,table,x);
   assert(q.passive_rate_supported==(rate>=p.rate_min_dps && rate<=p.rate_max_dps));
   assert(q.work_rate_supported==(rate>=w.rate_min_dps && rate<=w.rate_max_dps));
   assert(a::optionalMainAdmissible(q)==(e::earlyAdmissible(q)&&q.passive_rate_supported&&q.work_rate_supported&&q.charge_supported));}
  for(unsigned k=1;k<=400;++k){auto x=in;x.side=side;x.rate_dps=k*.25f;
   const auto q=e::decide(g,table,x);const auto gate=a::assess(a::Scope::OptionalMain,q);
   assert(gate.allowed==(e::earlyAdmissible(q)&&q.passive_rate_supported&&q.work_rate_supported&&q.charge_supported));
   if(side<0)assert(!gate.allowed);}
 }
 // Negative passive/work rate fits have no overlap: keep ZERO available,
 // never invent a new fitted rate interval to force a relocated main.
 assert(walking_energy_policy::passiveCoefficients(-1).rate_max_dps<lateral_energy::coefficients(-1).rate_min_dps);
 {auto x=in;x.side=1;x.rate_dps=39.2354f;x.body_angle_deg=-.99409f;x.target_deg=8;
  x.previous_peak_deg=5.21523f;x.same_side_peak_deg=6.47749f;x.initial_current_ma=-3.9053f;
  x.goal_current_ma=345.0051f;x.arrival_bias_j=.0001457f;
  const auto q=e::decide(g,table,x);assert(q.width_ms==64 && e::earlyAdmissible(q));
  const auto gate=a::assess(a::Scope::OptionalMain,q);
  assert(!gate.allowed && gate.unsupported_mask==1 && gate.reason==a::Reason::PassiveRateUnsupported);
  b::Ledger ledger;assert(ledger.sourcePeak(15779));
  assert(ledger.allowMain(15779) && ledger.remaining()==80 && !ledger.mainUsed());
  assert(a::assess(a::Scope::OrdinaryZero,q).allowed);}
 // Explicit-J aids keep their separate extrapolation/evidence contract and
 // shared budget; the relocation support gate does not veto them.
 {auto x=in;x.side=-1;x.rate_dps=15;x.body_angle_deg=9.4f;x.target_deg=8;
  x.return_work_j=.0001f;const auto q=b::decideReturnAid(g,table,x);
  assert(q.valid && !q.work_rate_supported);
  const auto gate=a::assess(a::Scope::ExplicitReturnAid,q);assert(!gate.applies && gate.allowed);
  b::Ledger ledger;assert(ledger.sourcePeak(900));assert(ledger.accepted(b::Kind::ReturnAid,q.width_ms));
  assert(ledger.complete(q.width_ms+1));assert(ledger.remaining()==79-q.width_ms);}
 std::cout<<"v19 admission policy: scoped support, inclusive boundaries, signedQ/budget rejection, unchangedJ/width, ZERO/aid preservation PASS\n";
}
int main(){tests();std::cout<<std::setprecision(12);
 lateral_energy::Geometry g;unsigned id,width,scope;int side;float tau,same;char command;
 while(std::cin>>command>>id>>scope>>side){
  e::Input in;std::cin>>in.rate_dps>>in.body_angle_deg>>in.target_deg>>in.previous_peak_deg>>same
   >>in.initial_current_ma>>in.goal_current_ma>>tau>>in.cap_ms>>width>>in.arrival_bias_j>>in.return_work_j;
  in.side=side;in.same_side_peak_deg=same<0?NAN:same;
  lateral_energy::CurrentTable table;if(!table.prepare(tau))return 2;
  const bool aid=scope==2;
  const auto selected=aid?b::decideReturnAid(g,table,in):e::decide(g,table,in);
  const auto actual=aid?b::evaluateReturnAidWidth(g,table,in,width):e::evaluateWidth(g,table,in,width);
  const auto gate=a::assess(static_cast<a::Scope>(scope),actual);
  std::cout<<"P "<<id<<' '<<selected.width_ms<<' '<<actual.predicted_work_j
   <<' '<<e::earlyAdmissible(actual)<<' '<<gate.applies<<' '<<gate.allowed
   <<' '<<unsigned(gate.unsupported_mask)<<' '<<unsigned(gate.reason)<<'\n';
 }
}
'''


def replay(trial, rwlog, exe):
    from analyze_v18_trial_energy import analyze
    from analyze_v14_trial_energy import potential, tau_from_metadata
    prior, rows, _, _, _ = analyze(trial, rwlog, 'Recorded v18 states reused for admission comparison only.')
    meta = json.loads((trial / 'metadata.json').read_text(encoding='utf8'))
    tau = tau_from_metadata(meta)
    commands = []
    for i, r in enumerate(rows):
        scope = 2 if r['return_event'] else 1 if r['optional'] else 0
        values = [i, scope, r['side'], r['rate_dps'], r['model_body_deg'], r['target_deg'],
                  r['previous_peak_deg'], r['same_side_peak_deg'] if r['same_side_peak_deg'] is not None else -1,
                  r['aligned_i0_ma'], r['goal_current_ma'], tau, r['cap_ms'], r['guarded_ms'],
                  r['arrival_bias_j'], r['return_work_j']]
        commands.append('P ' + ' '.join(map(str, values)))
    result = subprocess.run([str(exe)], input='\n'.join(commands) + '\n', text=True, capture_output=True, check=True)
    output = [line for line in result.stdout.splitlines() if line.startswith('P ')]
    assert len(output) == len(rows)
    for line in output:
        _, idx, width, work, old, applies, allowed, mask, reason = line.split()
        r = rows[int(idx)]
        assert int(width) == r['requested_ms'] and abs(float(work)-r['predicted_work_j']) < 2e-8
        assert bool(int(old)) == r['reconstructed_early_admissible']
        r.update({'v19_gate_applies': bool(int(applies)), 'v19_gate_allows': bool(int(allowed)),
                  'v19_unsupported_mask': int(mask), 'v19_admission_reason': int(reason)})
        if r['same_side_peak_deg'] is not None:
            # Firmware uses float geometry and a rounded serialized energy;
            # this Python diagnostic uses double, as does the v18 peak audit.
            assert abs(potential(r['same_side_peak_deg'])-r['same_side_peak_energy_j']) < 6e-9
    optional = [r for r in rows if r['optional']]
    executed = [r for r in optional if r['executed']]
    changed = [r for r in executed if not r['v19_gate_allows']]
    detail_fields = ['time_ms', 'side', 'executed', 'requested_ms', 'guarded_ms', 'model_body_deg', 'rate_dps',
        'passive_rate_supported', 'work_rate_supported', 'charge_supported', 'request_j', 'predicted_work_j',
        'q_mas', 'v19_gate_applies', 'v19_gate_allows', 'v19_unsupported_mask', 'v19_admission_reason']
    summary = {
        'trial_build': meta['diagnostic_build_id'], 'comparison': 'new admission gate at unchanged recorded v18 states',
        'header_sha256': hashlib.sha256((ROOT/'src/energy_admission_v19.h').read_bytes()).hexdigest(),
        'prior_source_replay': prior['native_source_replay'],
        'optional_candidates': len(optional), 'previously_executed_optional': len(executed),
        'previously_executed_still_allowed': sum(r['v19_gate_allows'] for r in executed),
        'previously_executed_now_deferred': len(changed),
        'deferred_recorded_outputs': [{k:r[k] for k in detail_fields} for r in changed],
        'allowed_recorded_outputs': [{k:r[k] for k in detail_fields} for r in executed if r['v19_gate_allows']],
        'all_optional_reasons': dict(Counter(r['v19_admission_reason'] for r in optional)),
        'normal_zero_candidates_exempt': sum(not r['optional'] and not r['return_event'] and r['v19_gate_allows'] for r in rows),
        'explicit_return_aids_exempt': sum(r['return_event'] and r['v19_gate_allows'] for r in rows),
        'negative_optional_support_intersection': 'empty: passive20.313..35.5657dps versus work35.7375..91.4938dps',
        'same_side_peak_energy_checks': sum(r['same_side_peak_deg'] is not None for r in rows),
        'arrival_feedback_checks': prior['native_source_replay']['arrival_feedback'],
        'limits': ['This gate changes acceptance only; no energy coefficients, requested J, predicted J or width are refitted.',
            'No transport/ledger reservation should occur for rejected optional inputs; parent runner must preserve real ZERO.',
            'Ordinary ZERO and explicit-J return aid remain subject to their existing health, energy, evidence and shared80ms checks.',
            'The empty negative-rate support intersection defers every optional negative main to real ZERO.',
            'Recorded states after the first changed input are not predictions of the v19 physical trajectory or fall prevention.']}
    (trial/'v19_energy_admission_replay.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf8')
    with (trial/'v19_energy_admission_candidates.csv').open('w', encoding='utf8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=detail_fields);writer.writeheader()
        writer.writerows({k:r[k] for k in detail_fields} for r in rows)
    print(json.dumps({k:summary[k] for k in ['optional_candidates','previously_executed_optional',
        'previously_executed_still_allowed','previously_executed_now_deferred','deferred_recorded_outputs',
        'normal_zero_candidates_exempt','explicit_return_aids_exempt']}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path)
    parser.add_argument('--rwlog', type=Path)
    args = parser.parse_args()
    if bool(args.trial) != bool(args.rwlog):
        parser.error('--trial and --rwlog are required together')
    with tempfile.TemporaryDirectory(prefix='energy_admission_v19_') as folder:
        temp = Path(folder);source = temp/'test.cpp';exe = temp/'test.exe'
        source.write_text(CPP, encoding='utf8')
        subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(ROOT/'src'),
                        str(source),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        if args.trial:
            replay(args.trial,args.rwlog,exe)


if __name__ == '__main__':
    main()
