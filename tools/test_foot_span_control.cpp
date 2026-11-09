#define FOOT_TREND_CYCLES 2
#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;
static uint16_t tick(Controller& x,unsigned t,float l,float r,float peak=5.f,
                     uint16_t base=100,uint16_t cap=100,unsigned n=30,
                     bool complete=true,float target=1.f,int16_t ma=400,unsigned period=100) {
  const auto w=x.limit(t+period/2,base,ma,1,cap,true,target);
  x.confirm(t+period/2,true,ma,1,w);
  Cycle c;c.start_ms=t;c.end_ms=t+period;c.positive_ms=t+period/2;
  c.left=l;c.right=r;c.negative_peak=peak;c.target_mj=target;
  c.negative_key=0x80000000u | (300u<<16) | 0x8000u | n;
  c.positive_width=w;c.positive_ma=ma;c.positive_sign=1;
  c.complete=complete;c.negative_known=c.positive_executed=true;
  x.cycle(c);return w;
}
int main(){
  // A peak concentrated in one camera frame can be a real fast swing, as in
  // run132; it must not be treated as a confirmed small-foot observation.
  assert(!spanReliable(4.02f,.11f));assert(!spanReliable(3.10f,.69f));
  assert(spanReliable(3.70f,3.35f));assert(spanReliable(.4f,.1f));
  assert(!spanReliable(NAN,1.f));
  // Consecutive complete cycles with alternating side differences do not
  // justify an offset. A persistent side difference does.
  Controller signs;signs.preferHighCurrentFirst();
  tick(signs,100,3,1,5,30,60,80,true,1.f,1200);
  tick(signs,200,1,3,5,30,60,80,true,1.f,1200);
  tick(signs,300,3,1,5,30,60,80,true,1.f,1200);
  assert(signs.offset()==0);
  tick(signs,400,3,1,5,30,60,80,true,1.f,1200);
  tick(signs,500,3,1,5,30,60,80,true,1.f,1200);
  assert(signs.offset()==5);
  // The unmodeled 1200 mA walking branch can use the same actual-command
  // confirmation and two completed-cycle feedback as the 400 mA branch.
  // Changing a selected width never claims a measured physical improvement.
  Controller high;high.preferHighCurrentFirst();
  assert(tick(high,100,3,1,5,30,60,80,true,1.f,1200)==30);
  assert(tick(high,200,3,1,5,30,60,80,true,1.f,1200)==30);
  assert(high.offset()==5 && high.acceptedProbeCycles()==0);
  assert(tick(high,300,2.5f,1.1f,5,30,60,80,true,1.f,1200)==35);
  tick(high,400,2.5f,1.1f,5,30,60,80,true,1.f,1200);
  tick(high,500,2.5f,1.1f,5,30,60,80,true,1.f,1200);
  assert(high.offset()==10 && !high.fault());
  // When +5 ms is blocked at a hard cap, explore -5 ms instead.
  Controller high_cap;high_cap.preferHighCurrentFirst();
  tick(high_cap,100,3,1,5,30,30,80,true,1.f,1200);
  tick(high_cap,200,3,1,5,30,30,80,true,1.f,1200);
  assert(high_cap.offset()==5);
  assert(tick(high_cap,300,3,1,5,30,30,80,true,1.f,1200)==25);
  // A retained 1200 mA offset is not carried over to another current.
  Controller switched;switched.preferHighCurrentFirst();
  tick(switched,100,3,1,5,30,60,80,true,1.f,1200);
  tick(switched,200,3,1,5,30,60,80,true,1.f,1200);
  tick(switched,300,1.2f,1.2f,5,30,60,80,true,1.f,1200);
  tick(switched,400,1.2f,1.2f,5,30,60,80,true,1.f,1200);
  tick(switched,500,1.2f,1.2f,5,30,60,80,true,1.f,1200);
  assert(switched.offset()==5);
  auto changed_current=switched.limit(650,30,400,1,60,true,1.f);
  assert(changed_current==30 && switched.offset()==0);
  switched.confirm(650,true,400,1,changed_current);
  assert(!switched.fault());
  Controller high_bad;high_bad.preferHighCurrentFirst();
  tick(high_bad,100,3,1,5,30,60,80,true,1.f,1200);
  tick(high_bad,200,3,1,5,30,60,80,true,1.f,1200);
  tick(high_bad,300,1.5f,.5f,5,30,60,80,true,1.f,1200);
  tick(high_bad,400,1.5f,.5f,5,30,60,80,true,1.f,1200);
  tick(high_bad,500,1.5f,.5f,5,30,60,80,true,1.f,1200);
  assert(high_bad.offset()==0 && !high_bad.fault());

  // A change in negative-side trigger, crossing rate or wheel speed cannot
  // be misreported as the effect of a changed positive-side pulse. Missing
  // comparable evidence discards the experiment; it is not a stop request.
  for(int changed=0;changed<3;++changed) {
    Controller x;x.preferHighCurrentFirst();tick(x,100,3,1,5,30,60,80,true,1.f,1200);
    tick(x,200,3,1,5,30,60,80,true,1.f,1200);
    tick(x,300,2.5f,1.1f,5,30,60,80,true,1.f,1200);
    const auto width=x.limit(450,30,1200,1,60,true,1.f);
    x.confirm(450,true,1200,1,width);
    Cycle c;c.start_ms=400;c.end_ms=500;c.positive_ms=450;
    c.left=2.5f;c.right=1.1f;c.negative_peak=5;c.target_mj=1;
    c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 80u;
    c.negative_trigger_deg=changed==0 ? 1.f : 0.f;
    c.negative_rate_dps=changed==1 ? 11.f : 0.f;
    c.negative_rpm=changed==2 ? 101.f : 0.f;
    c.positive_width=width;c.positive_ma=1200;c.positive_sign=1;
    c.complete=c.negative_known=c.positive_executed=true;
    x.cycle(c);assert(x.offset()==0 && !x.fault());
  }
  // Positive-side pre-pulse speed and body rate must also be comparable;
  // they affect motor work even with the same current and pulse duration.
  for(int changed=0;changed<2;++changed) {
    Controller x;x.preferHighCurrentFirst();tick(x,100,3,1,5,30,60,80,true,1.f,1200);
    tick(x,200,3,1,5,30,60,80,true,1.f,1200);
    tick(x,300,2.5f,1.1f,5,30,60,80,true,1.f,1200);
    const auto w=x.limit(450,30,1200,1,60,true,1.f);x.confirm(450,true,1200,1,w);
    Cycle c;c.start_ms=400;c.end_ms=500;c.positive_ms=450;
    c.left=2.5f;c.right=1.1f;c.negative_peak=5;c.target_mj=1;
    c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 80u;
    c.positive_rate_dps=changed==0 ? 16.f : 0.f;
    c.positive_rpm=changed==1 ? 151.f : 0.f;
    c.positive_width=w;c.positive_ma=1200;c.positive_sign=1;
    c.complete=c.negative_known=c.positive_executed=true;
    x.cycle(c);assert(x.offset()==0 && !x.fault());
    // Recovery reacquires two fresh cycles; stale reference and evidence
    // cannot immediately propose again after an incomparable observation.
    assert(tick(x,500,3,1,5,30,60,80,true,1.f,1200)==30);
    assert(x.offset()==0);
    tick(x,5600,3,1,5,30,60,80,true,1.f,1200);
    assert(x.offset()==0);
    tick(x,5700,3,1,5,30,60,80,true,1.f,1200);
    assert(x.offset()==5 && !x.fault());
  }
  // A different state on the very first changed command must prevent the
  // candidate from being sent, even if subsequent cycles might match again.
  for(int changed=0;changed<2;++changed) {
    Controller x;x.preferHighCurrentFirst();tick(x,100,3,1,5,30,60,80,true,1.f,1200);
    tick(x,200,3,1,5,30,60,80,true,1.f,1200);
    assert(x.offset()==5);
    const auto w=x.limit(350,30,1200,1,60,true,1.f,
        changed==0 ? 16.f : 0.f,changed==1 ? -151.f : 0.f);
    assert(w==30 && x.offset()==0 && !x.fault());
    x.confirm(350,true,1200,1,w);
  }
  // A good normalized-ratio response with both weaker foot and body peak
  // retained permits another step. The observed timing includes one cycle
  // overlapping the command and two full later cycles.
  Controller good;assert(tick(good,100,3,1)==100);assert(tick(good,200,3,1)==100);
  assert(good.offset()==-10 && good.acceptedProbeCycles()==0);
  assert(tick(good,300,2.5,1.1)==90 && good.acceptedProbeCycles()==1);
  tick(good,400,2.5,1.1);assert(good.offset()==-10);
  tick(good,500,2.5,1.1);assert(good.offset()==-20);
  // Reducing both feet halves absolute disparity, but does not improve the
  // normalized ratio; the weaker foot is also lost. Undo the trial.
  Controller shrink;tick(shrink,100,3,1);tick(shrink,200,3,1);
  tick(shrink,300,1.5,.5);tick(shrink,400,1.5,.5);tick(shrink,500,1.5,.5);
  assert(shrink.offset()==0 && tick(shrink,600,3,1)==100);
  // Ratio improves slightly while weaker foot falls: must still unwind.
  Controller weak;tick(weak,100,3,1);tick(weak,200,3,1);
  tick(weak,300,2,.7f);tick(weak,400,2,.7f);tick(weak,500,2,.7f);
  assert(weak.offset()==0);
  // Ratio improvement caused only by shrinking the larger foot does not
  // satisfy the goal of increasing the weaker foot's observed motion.
  Controller strong_shrinks;tick(strong_shrinks,100,3,1);tick(strong_shrinks,200,3,1);
  tick(strong_shrinks,300,2,1);tick(strong_shrinks,400,2,1);tick(strong_shrinks,500,2,1);
  assert(strong_shrinks.offset()==0 && !strong_shrinks.fault());
  // Negative side conditions changing by more than 10 ms make a response
  // incomparable. Discard the candidate and keep normal walking available.
  Controller changed_negative;tick(changed_negative,100,3,1);tick(changed_negative,200,3,1);
  tick(changed_negative,300,2.5f,1.1f,5,100,100,30);
  tick(changed_negative,400,2.5f,1.1f,5,100,100,50);
  assert(changed_negative.offset()==0 && !changed_negative.fault());
  // The observed 20-to-30 ms variation remains comparable.
  Controller ten_ms;tick(ten_ms,100,3,1,5,100,100,20);tick(ten_ms,200,3,1,5,100,100,20);
  tick(ten_ms,300,2.5f,1.1f,5,100,100,20);tick(ten_ms,400,2.5f,1.1f,5,100,100,30);
  tick(ten_ms,500,2.5f,1.1f,5,100,100,30);assert(ten_ms.offset()==-20);
  // Body oscillation falls although observed ratio improves.
  Controller peak;tick(peak,100,3,1);tick(peak,200,3,1);
  tick(peak,300,2.5,1.1,4);tick(peak,400,2.5,1.1,4);tick(peak,500,2.5,1.1,4);
  assert(peak.offset()==0);
  // Stagnation reverses the next proposal. Under a non-saturated 80ms
  // ordinary baseline this is an actual -10 followed by +10 experiment.
  Controller reverse;tick(reverse,100,3,1,5,80,100);tick(reverse,200,3,1,5,80,100);
  assert(tick(reverse,300,3,1,5,80,100)==70);
  tick(reverse,400,3,1,5,80,100);tick(reverse,500,3,1,5,80,100);
  assert(reverse.offset()==0);assert(tick(reverse,600,3,1,5,80,100)==80);
  tick(reverse,700,3,1,5,80,100);assert(reverse.offset()==10);
  assert(tick(reverse,800,3,1,5,80,100)==90);
  // Fully blocked +10 at the upper cap explicitly tests the alternative.
  Controller capped;tick(capped,100,1,3,5,80,80);tick(capped,200,1,3,5,80,80);
  assert(capped.offset()==10);assert(tick(capped,300,1,3,5,80,80)==70);
  // Missing observation abandons the unverified candidate, then reacquires
  // without requesting an emergency stop.
  Controller missing;tick(missing,100,3,1);tick(missing,200,3,1);
  tick(missing,300,2.5,1.1);tick(missing,400,2.5,1.1,5,100,100,30,false);
  assert(missing.offset()==0 && !missing.fault());assert(tick(missing,500,3,1)==100);
  tick(missing,600,3,1);tick(missing,700,3,1);assert(!missing.fault());
  // 42830511 sequence: a valid cycle proposes a new -20ms offset, then an
  // edge-unverified cycle arrives before its positive command. That unsent
  // candidate must be discarded instead of sent without a reference. Exact
  // 22017/22737/23476 cycle boundaries and 22220/23697 positive times come
  // from the log; the values are constructed to exercise this state path.
  Controller pending_invalid;
  tick(pending_invalid,21617,3,1);tick(pending_invalid,21717,3,1);
  tick(pending_invalid,21817,2.5f,1.1f);tick(pending_invalid,21917,2.5f,1.1f);
  assert(pending_invalid.limit(22220,100,400,1,100,true,1.f)==90);
  pending_invalid.confirm(22220,true,400,1,90);
  Cycle prior;prior.start_ms=22017;prior.end_ms=22737;prior.positive_ms=22220;
  prior.left=2.5f;prior.right=1.1f;prior.negative_peak=5;prior.target_mj=1;
  prior.negative_key=0x80000000u | (300u<<16) | 0x8000u | 30u;prior.positive_width=90;prior.positive_ma=400;prior.positive_sign=1;
  prior.complete=prior.negative_known=prior.positive_executed=true;
  pending_invalid.cycle(prior); // analogous to the 23296ms event snapshot
  assert(pending_invalid.offset()==-20 && pending_invalid.acceptedProbeCycles()==1);
  Cycle invalid;invalid.start_ms=22737;invalid.end_ms=23476;invalid.complete=false;
  pending_invalid.cycle(invalid);
  assert(pending_invalid.offset()==0 && !pending_invalid.fault());
  assert(pending_invalid.limit(23697,100,400,1,100,true,1.f)==100);
  pending_invalid.confirm(23697,true,400,1,100);
  Cycle restored;restored.start_ms=23476;restored.end_ms=24205;restored.positive_ms=23697;
  restored.left=3;restored.right=1;restored.negative_peak=5;restored.target_mj=1;
  restored.negative_key=1;restored.positive_width=100;restored.positive_ma=400;
  restored.positive_sign=1;restored.complete=restored.negative_known=restored.positive_executed=true;
  pending_invalid.cycle(restored);
  assert(tick(pending_invalid,24205,3,1)==100);
  assert(pending_invalid.offset()==0); // changed negative mode restarts the trend
  assert(tick(pending_invalid,24305,3,1)==100);
  assert(pending_invalid.offset()==-10);
  assert(tick(pending_invalid,24405,3,1)==90 && !pending_invalid.fault());
  // Selector changes have the same discard/reacquire behavior.
  Controller selector;tick(selector,100,3,1);tick(selector,200,3,1);
  assert(selector.offset()==-10 && selector.selectorUnavailable(250));
  assert(selector.offset()==0 && !selector.fault());
  assert(tick(selector,300,3,1)==100);
  // Stale observations while a -30ms offset is applied must immediately
  // restore the whole offset, not transmit another residual -20ms pulse.
  Controller stale;tick(stale,100,3,1);tick(stale,200,3,1);
  tick(stale,300,2.5f,1.1f);tick(stale,400,2.5f,1.1f);tick(stale,500,2.5f,1.1f);
  tick(stale,600,2.2f,1.2f);tick(stale,700,2.2f,1.2f);tick(stale,800,2.2f,1.2f);
  assert(stale.offset()==-30 && tick(stale,900,2.2f,1.2f)==70);
  assert(stale.limit(3601,100,400,1,100,true,1.f)==100);
  assert(stale.offset()==0 && !stale.fault());
  stale.confirm(3601,true,400,1,100);assert(!stale.fault());
  // The ordinary selector can jump or change current without the experiment
  // asserting a hardware fault. Incomparable observations undo the pending
  // candidate and later valid data may restart it.
  Controller modes;tick(modes,100,3,1,5,80,100);tick(modes,200,3,1,5,80,100);
  assert(tick(modes,300,2.5,1.1,5,80,100)==70);
  assert(tick(modes,400,2.5,1.1,5,95,100,45,true,1.f,300)>0);
  assert(!modes.fault());
  assert(tick(modes,500,3,1,5,95,100,30,true,1.f,300)>0);
  assert(!modes.fault());
  // A canceled candidate is not recorded as sent and cannot count as evidence.
  Controller cancel;tick(cancel,100,3,1);tick(cancel,200,3,1);
  assert(cancel.limit(350,100,400,1,100,true,1)==90);
  assert(cancel.cancelUnsent(100));cancel.confirm(350,true,400,1,100);
  assert(cancel.acceptedProbeCycles()==0 && !cancel.fault());
  // A failed actual experimental command keeps the pre-existing fault path.
  Controller fault;tick(fault,100,3,1);tick(fault,200,3,1);
  assert(fault.limit(350,100,400,1,100,true,1)==90);
  fault.confirm(350,false,400,1,0);assert(fault.requireNeutral());
  assert(fault.limit(450,100,400,1,100,true,1)==0);
  // Delayed old cycle cannot be counted after a newer confirmed adjustment.
  Controller late;tick(late,100,3,1);tick(late,200,3,1);
  assert(late.limit(350,100,400,1,100,true,1)==90);late.confirm(350,true,400,1,90);
  Cycle old;old.start_ms=200;old.end_ms=300;old.positive_ms=250;
  old.left=2;old.right=1;old.negative_peak=5;old.target_mj=1;old.negative_key=1;
  old.positive_width=100;old.positive_ma=400;old.positive_sign=1;
  old.complete=old.negative_known=old.positive_executed=true;late.cycle(old);
  assert(late.offset()==-10 && late.acceptedProbeCycles()==1);
  // Approximate 30-second event cadence and changing negative/ordinary inputs
  // based on 38921428. Values are replayed as fixed observations only; this
  // cannot predict how the physical robot would respond to changed outputs.
  Controller loglike;unsigned changes=0,held=0;
  const float left[]={3.104f,1.79119f,2.56245f,2.06484f,2.65809f,1.98468f,2.5591f,
    3.02064f,2.91008f,2.91475f,3.07414f,2.72823f,2.91936f,3.32827f,3.28772f,
    3.61458f,2.25551f,2.8821f,2.63781f,2.81539f,2.61764f,2.68611f,2.8618f,
    2.57976f,3.42457f,3.87727f,3.71694f,3.39015f,3.75461f};
  const float right[]={.86756f,.71473f,.65353f,.53508f,.61538f,.54957f,.40704f,
    .50698f,.38408f,.29898f,.34116f,.5016f,.36277f,.36078f,.27911f,.28798f,
    .3378f,.29122f,.3698f,.30871f,.15963f,.21935f,.23246f,.22305f,.47784f,
    .3307f,.30971f,.39436f,.5821f};
  for(unsigned i=0;i<29;++i){
    const uint16_t base=i<4?85:i<9?90:i<14?95:100;
    const unsigned neg=i<4?25:i<9?30:i<14?40:i<23?30:45;
    const auto w=tick(loglike,100+i*800,left[i],right[i],4.5f,base,100,neg,true,1.f,400,800);
    assert(w>0 && w<=100 && !loglike.fault());
    if(w!=base)++changes;else ++held;
  }
  assert(changes>0 && held>0 && loglike.acceptedProbeCycles()>0);
  std::cout<<"foot span: ratio response, weak side, time matching and recovery PASS\n";
}
