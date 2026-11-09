#include "negative_cap_recovery.h"
#include <cassert>
using negative_cap_recovery::Controller;

static void initial(Controller& c,bool neutral_tail=true,bool balanced_tail=false) {
  // Actual late alternating peaks and camera spans from run 43115247.
  // Neutral flags are synthetic and exercise attribution separately; the
  // real run still had foot adjustment active for some of these periods.
  const unsigned neg[]={21453,22197,22991,23740,24559,25308};
  const float plus[]={6.13978f,6.22655f,6.7383f,5.75969f,7.00764f,6.23365f};
  const float minus[]={4.67483f,5.74341f,3.86244f,5.52389f,3.71856f,5.28546f};
  const float left[]={3.41713f,2.66652f,4.54057f,2.47194f,4.48832f,2.5132f};
  const float right[]={1.14175f,.39109f,1.25691f,.23364f,1.0346f,.19395f};
  for(unsigned i=0;i<6;++i) {
    c.peak(1,neg[i]-380,plus[i],true,true);
    c.peak(-1,neg[i],minus[i],true,true);
    const float r=balanced_tail && i>=4 ? left[i]*.95f : right[i];
    c.footCycle(i ? neg[i-1] : neg[i]-700,neg[i],left[i],r,true,true,neutral_tail && i>=4);
  }
  c.peak(1,25680,7.0662f,true,true);
}
static void drive(Controller& c,unsigned ms,unsigned t,float p,float m,float right=1.f,float left=3.f) {
  c.command(ms,30,35,true,false);
  c.peak(-1,t,m,true,true);
  c.footCycle(t-700,t,left,right,true,true,true);
  c.peak(1,t+350,p,true,true);
}
int main() {
  using negative_cap_recovery::normalFootReferenceAllowed;
  assert(!normalFootReferenceAllowed(true,false,100,95)); // ending cycle
  assert(!normalFootReferenceAllowed(false,true,100,95));
  assert(!normalFootReferenceAllowed(false,false,90,95)); // crosses last input
  assert(!normalFootReferenceAllowed(false,false,95,95));
  assert(normalFootReferenceAllowed(false,false,96,95));
  assert(normalFootReferenceAllowed(false,false,1,0));
  Controller mixed;initial(mixed,false);
  assert(!mixed.candidate(25944,8.f,30,true,true)); // old foot trial isn't neutral reference
  Controller recovered;initial(recovered,true,true);
  assert(!recovered.candidate(25944,8.f,30,true,true)); // newest pair balanced despite older imbalance
  Controller early;
  assert(!early.candidate(1000,8.f,30,true,true));
  Controller c;initial(c);
  assert(c.candidate(25944,8.f,30,true,true));
  assert(!c.candidate(25944,8.f,30,false,true));
  assert(!c.candidate(25944,8.f,40,true,true));
  assert(!c.candidate(25944,7.5f,30,true,true));
  c.command(25944,30,30,true,true);assert(!c.attempted()); // clipped
  c.command(25944,30,35,false,true);assert(!c.attempted()); // failed send
  c.command(25944,30,36,true,true);assert(!c.attempted()); // over 5 ms
  c.command(25944,30,35,true,true);assert(c.active() && c.attempted());
  c.peak(-1,26097,4.8f,true,true);c.footCycle(25308,26097,3.f,1.f,true,true,true);
  c.peak(1,26440,5.36f,true,true);
  drive(c,26648,26807,7.5f,5.6f);
  drive(c,27455,27606,5.18f,4.7f);
  drive(c,28139,28300,7.35f,5.5f);
  drive(c,28924,29075,5.7f,4.8f);
  drive(c,29637,29794,5.7f,5.6f);
  assert(c.active() && c.holding() && c.stateCode()==2 && c.evaluated() && c.improved());
  assert(!c.candidate(30100,8.f,30,true,true)); // no second trial
  drive(c,30300,30470,6.2f,4.8f);
  drive(c,31000,31170,6.2f,5.6f);
  drive(c,31700,31870,6.2f,4.8f);
  drive(c,32400,32570,6.2f,5.6f);
  assert(c.active() && c.holding()); // persist while two-pair response remains good
  Controller bad_hold=c;
  drive(bad_hold,33100,33270,6.2f,4.8f,.01f);
  drive(bad_hold,33800,33970,6.2f,5.6f,.01f);
  drive(bad_hold,34500,34670,6.2f,4.8f,.01f);
  drive(bad_hold,35200,35370,6.2f,5.6f,.01f);
  assert(!bad_hold.active() && bad_hold.completed() && !bad_hold.improved()); // sustained weak-foot collapse
  Controller missing_hold=c;
  drive(missing_hold,33100,33270,6.2f,4.8f);
  drive(missing_hold,33800,33970,6.2f,5.6f);
  drive(missing_hold,34500,34670,6.2f,4.8f);
  missing_hold.command(35200,30,35,true,false);
  missing_hold.peak(-1,35370,5.6f,true,true);
  missing_hold.footCycle(34670,35370,0.f,0.f,false,true,true);
  assert(!missing_hold.active() && missing_hold.completed() && !missing_hold.evaluated() && !missing_hold.improved());
  Controller shrinking=c;
  drive(shrinking,33100,33270,6.2f,4.8f,.60f,1.f);
  drive(shrinking,33800,33970,6.2f,5.6f,.60f,1.f);
  drive(shrinking,34500,34670,6.2f,4.8f,.60f,1.f);
  drive(shrinking,35200,35370,6.2f,5.6f,.60f,1.f);
  assert(!shrinking.active() && shrinking.evaluated() && !shrinking.improved());
  assert(c.extraCap(39000,true)==0 && !c.active() && c.completed());
  assert(!c.candidate(39100,8.f,30,true,true)); // once per run
  Controller clipped;initial(clipped);clipped.command(25944,30,35,true,true);
  clipped.peak(-1,26097,4.8f,true,true);clipped.footCycle(25308,26097,3.f,1.f,true,true,true);
  clipped.peak(1,26440,5.36f,true,true);
  clipped.command(26648,30,30,true,false);clipped.peak(-1,26807,5.6f,true,true);
  assert(clipped.responses()==1); // no credit for an equal guarded width
  clipped.interrupt();assert(!clipped.active() && clipped.attempted());
  Controller over;initial(over);over.command(25944,30,35,true,true);
  over.peak(-1,26097,10.f,true,true);assert(!over.active() && over.completed());
  Controller stale;initial(stale);stale.command(25944,30,35,true,true);
  assert(stale.extraCap(28000,true)==0 && !stale.active());
  Controller missing;initial(missing);missing.footCycle(25308,25500,0,0,false,true,false);
  assert(missing.candidate(25944,8.f,30,true,true)); // a miss is not zero
  Controller unsafe;initial(unsafe);unsafe.command(25944,30,35,true,true);
  unsafe.peak(-1,26097,1.f,true,true);assert(!unsafe.active());
  Controller footdrop;initial(footdrop);footdrop.command(25944,30,35,true,true);
  footdrop.peak(-1,26097,4.8f,true,true);footdrop.footCycle(25308,26097,3.f,.01f,true,true,true);
  footdrop.peak(1,26440,5.4f,true,true);
  for(unsigned i=0;i<4;++i) {
    const unsigned ms=26600+i*700,t=ms+180;
    footdrop.command(ms,30,35,true,false);footdrop.peak(-1,t,4.7f,true,true);
    footdrop.footCycle(t-500,t,3.f,.01f,true,true,true);
    if(i<3)footdrop.peak(1,t+300,6.f,true,true);
  }
  assert(!footdrop.active() && footdrop.completed() && !footdrop.improved());
}
