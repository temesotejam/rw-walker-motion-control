#include "../src/foot_phase_cap.h"
#include <cassert>
#include <iostream>
using namespace foot_phase_cap;
static NegativeDecision powered() {return classifyNegative(true,true,true,false,300,-1,25,10);}
static NegativeDecision coast() {return classifyNegative(true,false,false,true,0,-1,0,11);}
static void pair(Controller& c,unsigned end,float left=2.2f,float right=-.1f,float neg=4.7f,bool neg_output=true) {
  c.pair(end,end-800,left,right,neg,neg_output ? powered() : coast(),true);
}
static unsigned limit(Controller& c,unsigned now,unsigned cap=80,float target=.4f){return c.limit(now,80,400,1,cap,true,target);}
static void confirm(Controller& c,unsigned now,unsigned width){c.confirm(now,true,400,1,width);}
static void baseline(Controller& c,unsigned now){assert(limit(c,now)==80);confirm(c,now,80);}
static void prepare(Controller& c){baseline(c,900);pair(c,1000);baseline(c,1700);pair(c,1800);}
static void heldCycle(Controller& c,unsigned end,unsigned width,float l,float r,float n=4.7f) {
  unsigned t=end-750;assert(limit(c,t)==width);confirm(c,t,width);pair(c,end,l,r,n);
}
int main(){
  Controller waiting;baseline(waiting,900);pair(waiting,1000);
  assert(limit(waiting,1100)==80);baseline(waiting,1700);pair(waiting,1800);
  assert(waiting.limit(1850,80,400,1,80,false,.4f)==80 && waiting.state()==State::Waiting);
  // Start after two valid consecutive pairs and an actual 80ms baseline.
  Controller c;prepare(c);assert(limit(c,1850)==75);confirm(c,1850,75);
  pair(c,2600); // starts before change: cannot be used as counterfactual evidence
  heldCycle(c,3400,75,1.8f,.4f,4.6f);heldCycle(c,4200,75,1.8f,.4f,4.6f);
  assert(c.state()==State::Hold);assert(limit(c,4250)==75);confirm(c,4250,75);
  pair(c,5000,1.8f,.4f,4.6f);
  heldCycle(c,5800,75,1.4f,.8f,4.6f);heldCycle(c,6600,75,1.4f,.8f,4.6f);
  assert(c.state()==State::Hold);assert(limit(c,6650)==75);confirm(c,6650,75);
  pair(c,7400,1.4f,.8f,4.6f);
  heldCycle(c,8200,75,1.2f,1.f,4.6f);heldCycle(c,9000,75,1.2f,1.f,4.6f);
  assert(c.state()==State::Hold);assert(limit(c,9050)==75);confirm(c,9050,75);
  c.tick(11500);assert(c.state()==State::Release && c.reason()==Reason::Stale);
  assert(limit(c,11501)==80);confirm(c,11501,80);assert(c.state()==State::Locked);
  // The one-step reduction is relative to the actual starting baseline,
  // rather than a hard-coded 75 ms limit.
  Controller eighty_five;
  assert(eighty_five.limit(900,85,400,1,85,true,.4f)==85);confirm(eighty_five,900,85);pair(eighty_five,1000);
  assert(eighty_five.limit(1700,85,400,1,85,true,.4f)==85);confirm(eighty_five,1700,85);pair(eighty_five,1800);
  assert(eighty_five.limit(1850,85,400,1,85,true,.4f)==80);confirm(eighty_five,1850,80);pair(eighty_five,2600);
  assert(eighty_five.limit(2650,85,400,1,85,true,.4f)==80);confirm(eighty_five,2650,80);
  pair(eighty_five,3400,1.8f,.4f,4.6f);
  assert(eighty_five.limit(3450,85,400,1,85,true,.4f)==80);confirm(eighty_five,3450,80);
  pair(eighty_five,4200,1.8f,.4f,4.6f);
  assert(eighty_five.state()==State::Hold);
  assert(eighty_five.limit(4250,85,400,1,85,true,.4f)==80);
  // A new ordinary candidate ends identification before the old fixed cap
  // can continue to diverge from it. Increases restore by at most 5 ms per
  // issued pulse; a decrease follows the new tighter candidate immediately.
  Controller rise;prepare(rise);
  assert(limit(rise,1850)==75);confirm(rise,1850,75);pair(rise,2600);
  heldCycle(rise,3400,75,1.8f,.4f);heldCycle(rise,4200,75,1.8f,.4f);
  assert(rise.state()==State::Hold);
  assert(rise.limit(4250,90,400,1,90,true,.4f)==80);
  assert(rise.state()==State::Release && rise.reason()==Reason::InputFailed);
  confirm(rise,4250,80);
  assert(rise.limit(5050,90,400,1,90,true,.4f)==85);confirm(rise,5050,85);
  assert(rise.limit(5850,90,400,1,90,true,.4f)==90);confirm(rise,5850,90);
  assert(rise.state()==State::Locked);
  assert(rise.limit(6650,90,400,1,90,true,.4f)==90);
  Controller fall;prepare(fall);
  assert(limit(fall,1850)==75);confirm(fall,1850,75);pair(fall,2600);
  heldCycle(fall,3400,75,1.8f,.4f);heldCycle(fall,4200,75,1.8f,.4f);
  assert(fall.state()==State::Hold);
  assert(fall.limit(4250,70,400,1,70,true,.4f)==70);
  assert(fall.state()==State::Release && fall.reason()==Reason::InputFailed);
  confirm(fall,4250,70);assert(fall.state()==State::Locked);
  assert(fall.limit(5050,70,400,1,70,true,.4f)==70);
  Controller tighter;prepare(tighter);
  assert(limit(tighter,1850)==75);confirm(tighter,1850,75);
  assert(tighter.limit(1900,90,400,1,60,true,.4f)==60);
  assert(tighter.state()==State::Release);confirm(tighter,1900,60);
  assert(tighter.limit(2700,90,400,1,60,true,.4f)==60);confirm(tighter,2700,60);
  // No response after two complete cycles restores the baseline.
  Controller flat;prepare(flat);assert(limit(flat,1850)==75);confirm(flat,1850,75);pair(flat,2600);
  heldCycle(flat,3400,75,2.2f,-.1f);heldCycle(flat,4200,75,2.2f,-.1f);
  assert(flat.state()==State::Release && flat.reason()==Reason::Inconclusive);
  assert(limit(flat,4250)==80);confirm(flat,4250,80);
  // Retain signed measurements, reject the false success where the weak foot
  // gets worse and both feet's forward movement collapses.
  Controller weakfoot;prepare(weakfoot);assert(limit(weakfoot,1850)==75);confirm(weakfoot,1850,75);pair(weakfoot,2600);
  heldCycle(weakfoot,3400,75,.5f,-.3f);
  assert(weakfoot.state()==State::Release && weakfoot.reason()==Reason::Worsened);
  Controller signedfoot;baseline(signedfoot,900);pair(signedfoot,1000,2,-.63f);baseline(signedfoot,1700);
  pair(signedfoot,1800,2,-.21f);assert(limit(signedfoot,1850)==75);
  // No probing after negative no-output, low-side trend collapse, duplicated
  // or overlapping pairs, absent baseline confirmation or missing sensors.
  Controller no_neg;baseline(no_neg,900);pair(no_neg,1000,2,0,4.7,false);baseline(no_neg,1700);
  pair(no_neg,1800,2,0,4.7,false);assert(limit(no_neg,1850)==75);
  Controller trend;baseline(trend,900);pair(trend,1000,2,0,4.7);baseline(trend,1700);
  pair(trend,1800,2,0,4.2);assert(limit(trend,1850)==80);
  Controller duplicate;baseline(duplicate,900);pair(duplicate,1000);pair(duplicate,1000);
  assert(limit(duplicate,1100)==80);duplicate.pair(1800,400,2,0,4.7,powered(),true);
  assert(limit(duplicate,1850)==80);
  Controller no_actual;pair(no_actual,1000);pair(no_actual,1800);assert(limit(no_actual,1850)==80);
  Controller lost;prepare(lost);assert(limit(lost,1850)==75);confirm(lost,1850,75);
  lost.negativeDecision(2000,NegativeDecision{});
  assert(lost.state()==State::Release && lost.reason()==Reason::Invalid);
  Controller weak;prepare(weak);assert(limit(weak,1850)==75);confirm(weak,1850,75);pair(weak,2600);
  heldCycle(weak,3400,75,2.2f,-.1f,4.1f);heldCycle(weak,4200,75,2.2f,-.1f,4.0f);
  assert(weak.state()==State::Release && weak.reason()==Reason::WeakPeak);
  // Hold must continue monitoring deterioration, rather than holding forever.
  Controller later;prepare(later);assert(limit(later,1850)==75);confirm(later,1850,75);pair(later,2600);
  heldCycle(later,3400,75,1.8f,.4f);heldCycle(later,4200,75,1.8f,.4f);
  assert(later.state()==State::Hold);later.negativeDecision(4250,NegativeDecision{});
  assert(later.state()==State::Release && later.reason()==Reason::Invalid);
  // Run 147: 25ms powered baseline; 30ms powered then valid zero-width
  // coast. Coast is not an immediate failure, but neither changed cycle can
  // qualify the earlier powered-25ms reference as an improvement.
  const auto powered30=classifyNegative(true,true,true,false,300,-1,30,10);
  Controller logged;prepare(logged);assert(limit(logged,1850)==75);confirm(logged,1850,75);
  pair(logged,2600);
  assert(limit(logged,2650)==75);confirm(logged,2650,75);
  logged.negativeDecision(3000,powered30);
  logged.pair(3400,2600,1.0f,1.0f,4.7f,powered30,true);
  assert(logged.state()==State::Trial);
  assert(limit(logged,3450)==75);confirm(logged,3450,75);
  logged.negativeDecision(3800,coast());
  assert(logged.state()==State::Trial);
  logged.pair(4200,3400,1.0f,1.0f,4.7f,coast(),true);
  assert(logged.state()==State::Release && logged.reason()==Reason::Inconclusive);
  assert(limit(logged,4250)==80);confirm(logged,4250,80);
  assert(logged.state()==State::Locked);
  // Coast is only the specific valid, zero-width and zero-current no-output
  // result. A failed command cannot masquerade as a safe coast.
  assert(coast().mode==NegativeMode::Coast);
  assert(classifyNegative(false,false,false,true,0,-1,0,11).mode==NegativeMode::Invalid);
  assert(classifyNegative(true,false,false,false,0,-1,0,11).mode==NegativeMode::Invalid);
  assert(classifyNegative(true,false,false,true,300,-1,0,11).mode==NegativeMode::Invalid);
  assert(classifyNegative(true,false,false,true,0,-1,10,11).mode==NegativeMode::Invalid);
  assert(classifyNegative(true,true,false,true,300,-1,25,10).mode==NegativeMode::Invalid);
  const auto unmodeled=classifyNegative(true,false,false,true,0,-1,0,12);
  assert(unmodeled.mode==NegativeMode::UnmodeledCoast);
  // Missing prediction alone never invalidates feedback or counts as evidence.
  // Real peak deterioration still immediately releases the restriction.
  Controller out_of_model;prepare(out_of_model);
  assert(limit(out_of_model,1850)==75);confirm(out_of_model,1850,75);pair(out_of_model,2600);
  for(unsigned end=3400;end<=4200;end+=800) {
    assert(limit(out_of_model,end-750)==75);confirm(out_of_model,end-750,75);
    out_of_model.negativeDecision(end-350,unmodeled);
    out_of_model.pair(end,end-800,1.8f,.4f,4.7f,unmodeled,true);
    assert(out_of_model.state()==State::Trial);
  }
  assert(limit(out_of_model,4250)==75);confirm(out_of_model,4250,75);
  out_of_model.negativeDecision(4600,unmodeled);
  out_of_model.pair(5000,4200,1.8f,.4f,3.8f,unmodeled,true);
  assert(out_of_model.state()==State::Release && out_of_model.reason()==Reason::WeakPeak);
  // Two powered baseline cycles must not be pooled with one coast cycle.
  // A lone unmatched coast does not contribute to successful identification;
  // two following *matching* powered cycles can still provide evidence.
  Controller resume;prepare(resume);assert(limit(resume,1850)==75);confirm(resume,1850,75);pair(resume,2600);
  assert(limit(resume,2650)==75);confirm(resume,2650,75);
  resume.negativeDecision(3000,coast());
  resume.pair(3400,2600,1.8f,.4f,4.7f,coast(),true);
  assert(resume.state()==State::Trial);
  heldCycle(resume,4200,75,1.8f,.4f);
  assert(resume.state()==State::Trial);
  heldCycle(resume,5000,75,1.8f,.4f);
  assert(resume.state()==State::Hold);
  // Two valid coasting baseline cycles can themselves be compared with
  // coasting trial cycles; the two modes are never mixed.
  Controller coast_base;baseline(coast_base,900);pair(coast_base,1000,2.2f,-.1f,4.7f,false);
  baseline(coast_base,1700);pair(coast_base,1800,2.2f,-.1f,4.7f,false);
  assert(limit(coast_base,1850)==75);confirm(coast_base,1850,75);
  pair(coast_base,2600,2.2f,-.1f,4.7f,false);
  assert(limit(coast_base,2650)==75);confirm(coast_base,2650,75);
  coast_base.pair(3400,2600,1.8f,.4f,4.7f,coast(),true);
  assert(limit(coast_base,3450)==75);confirm(coast_base,3450,75);
  coast_base.pair(4200,3400,1.8f,.4f,4.7f,coast(),true);
  assert(coast_base.state()==State::Hold);
  Controller deadline;prepare(deadline);assert(limit(deadline,1850)==75);confirm(deadline,1850,75);
  deadline.tick(6851);
  assert(deadline.state()==State::Release && deadline.reason()==Reason::Inconclusive);
  assert(limit(deadline,6852)==80);confirm(deadline,6852,80);
  assert(deadline.state()==State::Locked);
  // A stricter existing cap can force an immediate drop. After confirming its
  // actual 60ms pulse, any increase must start from 60ms, not the old 75ms.
  Controller safety;prepare(safety);assert(limit(safety,1850)==75);confirm(safety,1850,75);
  assert(limit(safety,1900,60)==60);confirm(safety,1900,60);
  assert(limit(safety,1901)==65);confirm(safety,1901,65);assert(limit(safety,1902)==70);
  // An unexpected width must also become the restoration reference.
  Controller mismatch;prepare(mismatch);assert(limit(mismatch,1850)==75);mismatch.confirm(1850,true,400,1,70);
  assert(mismatch.state()==State::Release);assert(limit(mismatch,1851)==75);
  Controller bypass;prepare(bypass);assert(limit(bypass,1850)==75);confirm(bypass,1850,75);
  bypass.confirm(1900,true,400,1,80);assert(bypass.state()==State::Release);
  // A changed current makes a width-only safe restoration impossible; report
  // neutral-required so the caller can stop output rather than invent a pulse.
  Controller current;prepare(current);assert(limit(current,1850)==75);current.confirm(1850,true,350,1,75);
  assert(current.requireNeutral() && limit(current,1851)==0);
  // A changed baseline current must be blocked *before* a different-current
  // pulse can be sent; a change in target ends identification and restores.
  Controller current_before;prepare(current_before);assert(limit(current_before,1850)==75);confirm(current_before,1850,75);
  assert(current_before.limit(1900,80,350,1,80,true,.4f)==0 && !current_before.requireNeutral());
  current_before.confirm(1900,false,0,1,0);
  assert(current_before.limit(2700,80,350,1,80,true,.4f)==80);
  Controller branch;prepare(branch);assert(limit(branch,1850)==75);confirm(branch,1850,75);
  branch.modeChanged();branch.confirm(1900,false,0,1,0);
  assert(branch.state()==State::Locked && branch.reason()==Reason::ModeChanged && !branch.requireNeutral());
  assert(limit(branch,2700)==80);
  assert(branch.limit(2800,80,350,1,80,false,.4f)==80 && branch.state()==State::Locked);
  assert(branch.limit(2900,80,400,1,80,true,NAN)==80 && branch.state()==State::Locked);
  assert(limit(branch,3000)==80 && branch.state()==State::Locked);
  Controller changed_target;prepare(changed_target);assert(limit(changed_target,1850)==75);confirm(changed_target,1850,75);
  assert(limit(changed_target,1900,80,.41f)==80 && changed_target.reason()==Reason::TargetChanged);
  confirm(changed_target,1900,80);assert(changed_target.state()==State::Locked);
  // Different startup targets cannot be pooled as the two-cycle baseline.
  Controller differing;baseline(differing,900);pair(differing,1000);
  assert(limit(differing,1700,80,.42f)==80);confirm(differing,1700,80);pair(differing,1800);
  assert(limit(differing,1850,80,.42f)==80);
  // Two baseline cycles must both contain the same actual pulse width as the
  // current unmodified candidate. A prior 75 ms pulse cannot be pooled with
  // a recent 80 ms pulse; one additional 80 ms cycle makes it eligible.
  Controller differing_width;
  assert(differing_width.limit(900,75,400,1,75,true,.4f)==75);
  confirm(differing_width,900,75);pair(differing_width,1000);
  baseline(differing_width,1700);pair(differing_width,1800);
  assert(limit(differing_width,1850)==80 && differing_width.state()==State::Waiting);
  baseline(differing_width,2500);pair(differing_width,2600);
  assert(limit(differing_width,2650)==75 && differing_width.state()==State::Trial);
  // The reverse order is blocked as well, even though the latest actual
  // pulse matches the new ordinary candidate.
  Controller differing_width_reverse;baseline(differing_width_reverse,900);pair(differing_width_reverse,1000);
  assert(differing_width_reverse.limit(1700,75,400,1,75,true,.4f)==75);
  confirm(differing_width_reverse,1700,75);pair(differing_width_reverse,1800);
  assert(differing_width_reverse.limit(1850,75,400,1,75,true,.4f)==75 &&
         differing_width_reverse.state()==State::Waiting);
  // A stale pulse from a previous cycle cannot qualify a fresh baseline pair.
  Controller repeated;baseline(repeated,900);pair(repeated,1000);pair(repeated,1800);
  assert(limit(repeated,1850)==80);
  // A delayed completed image cycle can be delivered after the next positive
  // pulse. Match by decision timestamp, not whichever pulse arrived last.
  Controller delayed;prepare(delayed);assert(limit(delayed,1850)==75);confirm(delayed,1850,75);
  assert(limit(delayed,2650)==75);confirm(delayed,2650,75);pair(delayed,2600);
  assert(limit(delayed,3450)==75);confirm(delayed,3450,75);pair(delayed,3400,1.8f,.4f);
  assert(limit(delayed,4250)==75);confirm(delayed,4250,75);pair(delayed,4200,1.8f,.4f);
  assert(delayed.state()==State::Hold);
  // A decision exactly on the cycle boundary cannot be counted twice.
  Controller boundary;baseline(boundary,200);pair(boundary,1000);
  assert(limit(boundary,1000)==80);confirm(boundary,1000,80);pair(boundary,1800);
  assert(limit(boundary,1850)==80);
  Controller nan_wait;assert(nan_wait.limit(1000,80,400,1,80,true,NAN)==80 && !nan_wait.requireNeutral());
  Controller neutral;prepare(neutral);assert(limit(neutral,1850)==75);confirm(neutral,1850,75);
  assert(limit(neutral,1900)==75);neutral.confirm(1900,true,350,1,75);
  assert(neutral.requireNeutral());
  assert(neutral.limit(1901,80,400,1,80,false,.4f)==0);
  std::cout<<"Foot phase cap: consecutive measured cycles, executed actions, signed feet, response and recovery PASS\n";
}
