#include "upright_reference.h"
#include "upright_reference_store.h"
#include <Preferences.h>
#include <cassert>
#include <iostream>
using namespace upright_reference;
static void sample(Capture& c, uint32_t us, uint32_t seq, float angle=2.2f,
                   float gyro=.3f, bool stopped=true, bool healthy=true) {
  c.step(us+100,us,seq,healthy,stopped,angle,.038f,-.999f,1.f,gyro);
}
int main() {
  Record r;
  assert(!load(r) && !valid(r) && std::isnan(relative(0,r)));
  Capture c; c.begin(1000,0);
  for (uint32_t i=0;i<=800;++i) sample(c,1000+i*2500,i+1);
  assert(c.state()==Capture::State::Ready && c.samples()==801);
  r=c.candidate(); assert(valid(r) && r.revision==1);
  assert(std::fabs(relative(10.2f,r)-8.f)<1e-5);
  assert(std::fabs(relative(-5.8f,r)+8.f)<1e-5);
  assert(std::fabs(pitchError(.038f,1.f,r))<1e-5);
  assert(save(r));
  Record rebooted; assert(load(rebooted) && rebooted.pitch_deg==r.pitch_deg);
  assert(std::fabs(relative(3.2f,rebooted)-1.f)<1e-5);
  Record newer=r; newer.pitch_deg=3.f; ++newer.revision;
  Preferences::fail_write=true; assert(!save(newer));
  assert(load(rebooted) && rebooted.pitch_deg==2.2f);
  Preferences::fail_write=false; assert(save(newer));
  assert(load(rebooted) && rebooted.pitch_deg==3.f && rebooted.revision==2);
  c.begin(1000,2); sample(c,1000,1); sample(c,3500,2,3.0f);
  assert(c.state()==Capture::State::Failed);
  assert(load(rebooted) && rebooted.pitch_deg==3.f);
  c.begin(1000,2); sample(c,1000,1,2.2f,2.f); assert(!c.pending());
  c.begin(1000,2); sample(c,1000,1,2.2f,.3f,false); assert(!c.pending());
  c.begin(1000,2); sample(c,1000,1,2.2f,.3f,true,false); assert(!c.pending());
  c.begin(1000,2); c.step(20000,1000,1,true,true,2.2f,.038f,-1.f,1.f,.3f);
  assert(!c.pending());
  c.begin(1000,2); sample(c,1000,1); sample(c,3500,1); assert(c.samples()==1);
  c.cancel(); assert(c.state()==Capture::State::Failed);
  c.begin(1000,2); sample(c,1000,1); sample(c,3002000,2);
  assert(c.state()==Capture::State::Failed);
  const uint32_t wrap=UINT32_MAX-1000000;
  c.begin(wrap,2);
  for(uint32_t i=0;i<=800;++i) sample(c,wrap+i*2500,i+1);
  assert(c.state()==Capture::State::Ready);
  Record corrupt=r; corrupt.pitch_deg=NAN; assert(!save(corrupt));
  Preferences::blob="wrong-size"; assert(!load(rebooted) && !valid(rebooted));
  std::cout << "explicit reference, averaging, motion/stale rejection, uint32 wrap, persisted reload and failed write PASS\n";
}
