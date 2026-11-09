#include "long_foot_balance.h"
#include <cassert>
using long_foot_balance::Controller;
static void sample(Controller& c,unsigned n,float l,float r,float p=7.f,float m=7.f) {
  c.observe(n-100,n,l,r,p,m,true,true);
}
static void beginPairs(Controller& c) {
  for(unsigned i=0;i<6;++i)c.observeCycle(1000+i*700,1700+i*700,3,1,6,5,true,true,true);
  c.confirm(5300,60,65,true);
  c.observeCycle(5200,5900,3,1,6,5,true,true,true);
  c.observeCycle(5900,6600,3,1,6,5,true,true,true);
}
static void trialPair(Controller& c,unsigned start,float l,float r,float p=6,float m=5,bool valid=true,bool effective=true) {
  c.observeCycle(start,start+700,l,r,p,m,valid,true,effective);
  c.observeCycle(start+700,start+1400,l,r,p,m,valid,true,effective);
}
int main(){
  Controller missing;beginPairs(missing);
  assert(missing.state()==Controller::State::Trial);
  missing.observeCycle(6600,7300,3,1,6,5,false,true,true);
  assert(missing.state()==Controller::State::Trial);
  missing.observeCycle(7300,8000,3,1,6,5,true,true,true);
  missing.observeCycle(8000,8700,3,1,6,5,false,true,true);
  assert(missing.state()==Controller::State::Trial);
  missing.observeCycle(8700,9400,3,1,6,5,false,true,true);
  assert(missing.state()==Controller::State::Restore);
  assert(missing.propose(9410,60,100,true)==60);
  missing.confirm(9410,65,60,false);
  assert(missing.offset()==5 && missing.state()==Controller::State::Restore);
  missing.confirm(9410,65,60,true);
  assert(missing.offset()==0 && missing.state()==Controller::State::Collect);
  for(unsigned i=0;i<8;++i)
    missing.observeCycle(9400+i*700,10100+i*700,3,1,6,5,true,true,true);
  assert(missing.propose(15010,60,100,true)==55);
  Controller c;
  for(unsigned n=1000;n<=4000;n+=1000)sample(c,n,3.f,1.f);
  assert(c.propose(4100,60,100,true)==60); // first four periods untouched
  c.observe(4100,4500,0,0,0,0,false,true); // missing is not zero
  sample(c,5000,3.f,1.f);
  assert(c.propose(5100,60,100,true)==65);
  c.confirm(5100,45,45,true); // guard removed candidate
  assert(c.offset()==0 && c.state()==Controller::State::Collect);
  c.confirm(5200,45,49,false); // command not accepted
  assert(c.offset()==0);
  c.confirm(5300,45,49,true);
  assert(c.offset()==5 && c.state()==Controller::State::Trial);
  c.observe(5000,5400,3.f,1.f,7.f,7.f,true,true); // overlaps intervention
  assert(c.samples()==0);
  for(unsigned n=6000;n<=10000;n+=1000)sample(c,n,3.f,1.5f);
  assert(c.propose(10100,60,100,true)==65); // improved: hold for a further five periods
  for(unsigned n=11000;n<=15000;n+=1000)sample(c,n,3.f,1.5f);
  assert(c.propose(15100,60,100,true)==70);
  assert(c.trialActive() && c.trialReferenceOffset()==0 && c.proposedTrialReferenceOffset()==5);
  c.confirm(15100,49,49,true); // next proposal blocked; active +5 is still compared to zero
  assert(c.trialActive() && c.trialReferenceOffset()==0 && c.trialGeneration()==1);
  c.observe(15001,15120,3.f,1.5f,7.f,7.f,true,true,false);
  assert(c.trialActive() && c.trialReferenceOffset()==0);
  c.confirm(15200,49,53,true);
  assert(c.trialReferenceOffset()==5 && c.trialGeneration()==2);
  for(unsigned n=16000;n<=20000;n+=1000)sample(c,n,3.f,.7f);
  assert(c.state()==Controller::State::Restore);
  assert(c.propose(20100,60,100,true)==65);
  c.confirm(20150,49,49,true); // same physical output; discard stale offset
  assert(c.offset()==5 && c.state()==Controller::State::Hold);
  assert(c.trialActive() && c.trialReferenceOffset()==0 && c.trialGeneration()==3);
  c.observe(20200,20300,3.f,1.f,7.f,7.f,true,true,false);
  assert(c.samples()==0); // old +5 must still be effective after restore
  for(unsigned n=21000;n<=25000;n+=1000)sample(c,n,3.f,1.f);
  assert(c.propose(25100,60,100,true)==60); // reverse failed direction
  c.confirm(25200,49,45,true);
  sample(c,26000,3.f,1.f,3.f,7.f); // serious peak drop unwinds immediately
  assert(c.state()==Controller::State::Collect);
  assert(c.propose(26100,60,100,true)==60);
  assert(c.propose(40000,60,100,true)==60); // stale observations: baseline
  assert(c.offset()==0);
  c.observe(40100,40200,3.f,1.f,7.f,7.f,true,false);
  assert(c.propose(40300,60,100,true)==60);
  // Two accepted but completely guard-clipped candidates contain no
  // response evidence. Do not increment a trial or disturb the setting;
  // require a fresh full window before testing the opposite direction.
  Controller blocked;
  for(unsigned n=1000;n<=5000;n+=1000)sample(blocked,n,3.f,1.f);
  assert(blocked.propose(5100,60,60,true)==60);
  blocked.confirm(5100,60,60,false); // failed command does not count
  blocked.confirm(5200,60,60,true);
  assert(blocked.trialGeneration()==0 && blocked.offset()==0);
  blocked.confirm(5300,60,60,true);
  assert(blocked.trialGeneration()==0 && blocked.offset()==0);
  for(unsigned n=6000;n<=9000;n+=1000)sample(blocked,n,3.f,1.f);
  assert(blocked.propose(9100,60,60,true)==60);
  sample(blocked,10000,3.f,1.f);
  assert(blocked.propose(10100,60,60,true)==55);
  blocked.confirm(10100,60,55,true);
  assert(blocked.trialGeneration()==1 && blocked.offset()==-5);
  Controller recovered;
  for(unsigned n=1000;n<=5000;n+=1000)sample(recovered,n,3.f,1.f);
  recovered.confirm(5100,60,60,true);recovered.confirm(5200,60,60,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(recovered,n,2.f,2.f);
  assert(recovered.propose(10100,60,100,true)==60 && recovered.trialGeneration()==0);
  // Actual paired-cycle path: missing cycles cannot become zero motion or
  // supply the six fresh cycles required for the opposite proposal.
  Controller paired_blocked;
  for(unsigned i=0;i<6;++i)
    paired_blocked.observeCycle(100+i*100,200+i*100,3.f,1.f,7.f,7.f,true,true);
  assert(paired_blocked.propose(710,60,60,true)==60);
  paired_blocked.confirm(710,60,60,true);paired_blocked.confirm(720,60,60,true);
  for(unsigned i=0;i<6;++i)
    paired_blocked.observeCycle(800+i*100,900+i*100,3.f,1.f,7.f,7.f,i!=2,true);
  assert(paired_blocked.propose(1410,60,60,true)==60);
  for(unsigned i=0;i<6;++i)
    paired_blocked.observeCycle(1400+i*100,1500+i*100,3.f,1.f,7.f,7.f,true,true);
  assert(paired_blocked.propose(2010,60,60,true)==55);
  // A first half observed before the second clipped command cannot be
  // paired into the new reference, while alternating phase stays intact.
  Controller half_blocked;
  for(unsigned i=0;i<6;++i)
    half_blocked.observeCycle(100+i*100,200+i*100,3.f,1.f,7.f,7.f,true,true);
  assert(half_blocked.propose(710,60,60,true)==60);
  half_blocked.confirm(710,60,60,true);
  half_blocked.observeCycle(700,800,3.f,1.f,7.f,7.f,true,true);
  half_blocked.confirm(810,60,60,true);
  for(unsigned i=0;i<5;++i)
    half_blocked.observeCycle(800+i*100,900+i*100,3.f,1.f,7.f,7.f,true,true);
  assert(half_blocked.propose(1310,60,60,true)==60);
  for(unsigned i=0;i<2;++i)
    half_blocked.observeCycle(1300+i*100,1400+i*100,3.f,1.f,7.f,7.f,true,true);
  assert(half_blocked.propose(1510,60,60,true)==55);
  // A complete historical pair can arrive after both clipped commands;
  // its recorded start, not its arrival time, determines eligibility.
  Controller delayed_block;
  for(unsigned i=0;i<6;++i)
    delayed_block.observeCycle(100+i*100,200+i*100,3.f,1.f,7.f,7.f,true,true);
  assert(delayed_block.propose(710,60,60,true)==60);
  delayed_block.confirm(710,60,60,true);delayed_block.confirm(950,60,60,true);
  delayed_block.observeCycle(700,800,3.f,1.f,7.f,7.f,true,true);
  delayed_block.observeCycle(800,900,3.f,1.f,7.f,7.f,true,true);
  for(unsigned i=0;i<4;++i)
    delayed_block.observeCycle(1000+i*100,1100+i*100,3.f,1.f,7.f,7.f,true,true);
  assert(delayed_block.propose(1410,60,60,true)==60);
  for(unsigned i=0;i<2;++i)
    delayed_block.observeCycle(1400+i*100,1500+i*100,3.f,1.f,7.f,7.f,true,true);
  assert(delayed_block.propose(1610,60,60,true)==55);
  Controller safety;
  for(unsigned n=1000;n<=5000;n+=1000)sample(safety,n,3.f,1.f);
  safety.confirm(5300,60,65,true);
  safety.observe(5900,6000,0,0,3.f,7.f,false,true); // IMU still sees weak swing with no foot frame
  assert(safety.state()==Controller::State::Restore);
  assert(safety.propose(6100,60,100,true)==60);
  Controller flip;
  for(unsigned n=1000;n<=5000;n+=1000)sample(flip,n,3.f,1.f);
  flip.confirm(5300,60,65,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(flip,n,2.f,2.f);
  assert(flip.state()==Controller::State::Hold && flip.propose(10100,60,100,true)==65);
  for(unsigned n=11000;n<=15000;n+=1000)sample(flip,n,1.f,3.f);
  assert(flip.propose(15100,60,100,true)==60); // sign reversed while holding
  Controller overlap;
  for(unsigned n=1000;n<=5000;n+=1000)sample(overlap,n,3.f,1.f);
  overlap.confirm(5300,60,65,true);
  overlap.observe(5000,5400,3.f,1.f,3.f,7.f,true,true);
  assert(overlap.state()==Controller::State::Restore && overlap.samples()==0);
  Controller age;
  sample(age,1000,3.f,1.f);sample(age,5000,3.f,1.f);sample(age,9000,3.f,1.f);
  sample(age,13000,3.f,1.f);sample(age,17000,3.f,1.f);
  assert(age.samples()<5 && age.propose(17100,60,100,true)==60);
  // The latest five measurements replace a stale startup reference while a
  // candidate waits. A blocked old proposal never gains renewed freshness.
  Controller rolling;
  for(unsigned n=1000;n<=5000;n+=1000)sample(rolling,n,1.1f,.24f);
  assert(rolling.error()>.5f && rolling.propose(5100,60,100,true)==65);
  rolling.confirm(5100,60,60,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(rolling,n,1.75f,.585f);
  assert(rolling.error()>.45f && rolling.error()<.49f);
  assert(rolling.propose(11600,60,100,true)==60); // proposal older than 1.5s
  sample(rolling,12000,1.75f,.585f);
  assert(rolling.propose(12100,60,100,true)==65);
  rolling.confirm(12100,60,65,true);
  assert(rolling.trialActive() && rolling.trialGeneration()==1 && rolling.trialReferenceOffset()==0);
  rolling.observe(12200,12500,1.75f,1.f,7.f,7.f,true,true,false);
  assert(rolling.samples()==0); // guard removed the changed output
  for(unsigned n=13000;n<=17000;n+=1000)sample(rolling,n,1.75f,1.f);
  assert(rolling.state()==Controller::State::Hold);
  assert(rolling.propose(17100,60,100,true)==65); // no immediate next step
  Controller growth;
  for(unsigned n=1000;n<=5000;n+=1000)sample(growth,n,1.f,.2f,5.f,5.f);
  growth.confirm(5100,60,65,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(growth,n,2.f,.5f,7.f,7.f);
  assert(growth.state()==Controller::State::Trial); // growth is inconclusive
  assert(growth.propose(10100,60,100,true)==65);
  Controller timed;
  for(unsigned n=1000;n<=5000;n+=1000)sample(timed,n,3.f,1.f);
  timed.confirm(5100,60,65,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(timed,n,3.f,1.5f);
  for(unsigned n=11000;n<=15000;n+=1000)sample(timed,n,3.f,1.5f);
  assert(timed.propose(15100,60,100,true)==70);
  timed.confirm(15100,65,70,true);
  for(unsigned n=16000;n<=27000;n+=1000)sample(timed,n,3.f,1.6f,9.f,9.f);
  assert(timed.propose(27101,60,100,true)==65); // inconclusive: restore +5, not zero
  timed.confirm(27101,70,65,true);
  assert(timed.offset()==5 && timed.trialActive() && timed.trialReferenceOffset()==0);
  Controller safety_ref;
  for(unsigned n=1000;n<=5000;n+=1000)sample(safety_ref,n,3.f,1.f);
  safety_ref.confirm(5100,60,65,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(safety_ref,n,3.f,1.5f);
  for(unsigned n=11000;n<=15000;n+=1000)sample(safety_ref,n,3.f,1.5f,6.f,6.f);
  assert(safety_ref.trialActive() && safety_ref.propose(15100,60,100,true)==70);
  safety_ref.observe(15120,15200,0.f,0.f,4.4f,6.f,false,true,false);
  assert(safety_ref.state()==Controller::State::Restore); // queued refs never weaken active safety
  // Actual run 38414363 peak trajectory: the early reference grows from
  // roughly 3.1 to 5.1 degrees, while the later peaks settle near 5 degrees.
  Controller replay;
  const float early[]={3.065f,3.910f,3.877f,4.499f,5.120f};
  const float early_m[]={6.237f,6.439f,5.688f,5.643f,5.974f};
  for(unsigned i=0;i<5;++i)sample(replay,1000+i*1000,1.1f,.24f,early[i],early_m[i]);
  assert(replay.propose(5100,60,100,true)==60);
  const float settled[]={5.087f,5.077f,4.924f,4.933f,4.946f};
  const float settled_m[]={5.587f,6.155f,5.282f,5.995f,5.534f};
  for(unsigned i=0;i<5;++i)sample(replay,6000+i*1000,1.75f,.585f,settled[i],settled_m[i]);
  assert(replay.propose(10100,60,100,true)==65);
  // Real runner path: three disjoint pairs = six complete camera cycles.
  // Preserve the right-small -> following-left-large order in each pair.
  Controller pairs;
  for(unsigned i=0;i<6;++i) {
    const unsigned s=1000+i*700,e=s+700;
    pairs.observeCycle(s,e,(i%2)?4.f:3.f,.5f,6.f,5.f,true,true,true);
    if(i<5)assert(pairs.propose(e+1,60,100,true)==60);
  }
  assert(pairs.propose(5201,60,100,true)==65);
  pairs.confirm(5300,60,65,true);
  // Skip the pair containing the changed input; keep its alternating phase.
  pairs.observeCycle(5200,5900,3,.5f,6,5,true,true,true);
  pairs.observeCycle(5900,6600,4,.5f,6,5,true,true,true);
  for(unsigned i=0;i<6;++i) {
    const unsigned s=6600+i*700,e=s+700;
    // Ratio and right foot improve, but next-left following weak right has
    // grown: the sequence-specific safety objective rejects the trial.
    pairs.observeCycle(s,e,(i%2)?5.f:3.f,1.f,6.f,5.f,true,true,true);
  }
  assert(pairs.state()==Controller::State::Restore);
  Controller gaps;
  gaps.observeCycle(100,200,3,.5f,6,5,true,true,true);
  gaps.observeCycle(200,300,0,0,6,5,false,true,true);
  gaps.observeCycle(300,400,4,.5f,6,5,true,true,true);
  assert(gaps.samples()==0); // never form a pair across a missing cycle
  gaps.observeCycle(400,500,4,.5f,6,5,true,true,true);
  assert(gaps.samples()==1); // phase anchored across the known invalid cycle
  Controller pairing_safety;
  for(unsigned i=0;i<6;++i)pairing_safety.observeCycle(1000+i*700,1700+i*700,3,.5f,6,5,true,true,true);
  pairing_safety.confirm(5300,60,65,true);
  pairing_safety.observeCycle(5400,6100,0,0,3,5,false,true,false);
  assert(pairing_safety.state()==Controller::State::Restore); // no waiting for pair
  Controller phase_gap;
  for(unsigned i=0;i<6;++i)phase_gap.observeCycle(1000+i*700,1700+i*700,3,.5f,6,5,true,true,true);
  phase_gap.confirm(5300,60,65,true);
  // An unknown missing boundary loses phase, so response cannot be compared
  // with the pre-change sequence window; restore the prior pulse setting.
  phase_gap.observeCycle(7000,7700,3,.5f,6,5,true,true,true);
  assert(phase_gap.state()==Controller::State::Restore);
  // Early loss needs two consecutive complete effective pairs; restore the
  // last setting rather than claiming that the tested direction is causal.
  Controller loss;beginPairs(loss);
  trialPair(loss,6600,4,.4f);assert(loss.state()==Controller::State::Trial);
  trialPair(loss,8000,4,.4f);assert(loss.state()==Controller::State::Restore);
  assert(loss.propose(9401,60,100,true)==60);
  loss.confirm(9450,65,60,false);assert(loss.offset()==5);
  loss.confirm(9500,65,60,true);assert(loss.offset()==0);
  // Reacquire observations after the confirmed restore before testing the
  // other direction; do not keep reapplying the rejected +5 candidate.
  trialPair(loss,9400,3,1);
  trialPair(loss,10800,3,1);trialPair(loss,12200,3,1);trialPair(loss,13600,3,1);
  assert(loss.propose(15001,60,100,true)==55);
  // A missing pair now restores an inconclusive trial. A non-effective pair
  // still breaks early-loss evidence without claiming a direction.
  for(int kind=0;kind<2;++kind) {
    Controller gap;beginPairs(gap);trialPair(gap,6600,4,.4f);
    trialPair(gap,8000,4,.4f,6,5,kind!=0,kind!=1);
    if(kind==0) {assert(gap.state()==Controller::State::Restore);continue;}
    trialPair(gap,9400,4,.4f);assert(gap.state()==Controller::State::Trial);
    trialPair(gap,10800,4,.4f);assert(gap.state()==Controller::State::Restore);
  }
  // A single bad pair, balanced or improving weak side, or a simultaneous
  // larger sway must not trigger the new early loss condition.
  Controller early_recovered;beginPairs(early_recovered);trialPair(early_recovered,6600,4,.4f);
  trialPair(early_recovered,8000,3,1.5f);assert(early_recovered.state()==Controller::State::Trial);
  Controller growth_pair;beginPairs(growth_pair);trialPair(growth_pair,6600,4,.4f,7.2f,6.f);
  trialPair(growth_pair,8000,4,.4f,7.2f,6.f);assert(growth_pair.state()==Controller::State::Trial);
  Controller preserved;beginPairs(preserved);trialPair(preserved,6600,3.3f,1.1f,6.6f,5.5f);
  trialPair(preserved,8000,3.3f,1.1f,6.6f,5.5f);assert(preserved.state()==Controller::State::Trial);
  // Movement becoming small in both feet does not count as a stronger
  // imbalance, even if the weak foot decreases substantially.
  Controller both_small;beginPairs(both_small);trialPair(both_small,6600,1.2f,.4f);
  trialPair(both_small,8000,1.2f,.4f);assert(both_small.state()==Controller::State::Trial);
}
