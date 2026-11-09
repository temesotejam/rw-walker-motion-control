#include "foot_fullstride_observer.h"
#include <cassert>
#include <cmath>
#include <iostream>
using foot_fullstride_observer::Observer;
static bool near(float a,float b){return std::fabs(a-b)<1e-5f;}
static const float wave[8]={0,1,2,1,0,-1,-2,-1};
static void frame(Observer &o,uint16_t id,unsigned t,bool lv=true,bool rv=true,float shift=0) {
  const unsigned i=(t/100)%8;
  assert(o.frame(id,t,lv,rv,wave[i]+shift,wave[i]*.5f+shift));
}
int main(){
  Observer o;o.reset(7);
  // Require actual alternating body peaks, complete cycles and a later frame
  // to allow delivery. The first cycle is never used as a settled stride.
  for(unsigned t=0;t<=2800;t+=100){
    if(t%800==0)assert(o.peak(7,-1,t));
    else if(t%800==400)assert(o.peak(7,1,t));
    frame(o,7,t);
    if(t==1000){auto c=o.latest();assert(c.sequence==1 && c.startup && !c.valid);}
    if(t==1800){auto c=o.latest();assert(c.sequence==2 && c.valid && !c.startup);
      assert(c.start_ms==800 && c.end_ms==1600 && c.continuous);
      assert(c.left.both_directions && c.right.both_directions);
      assert(!c.left.single_outlier && !c.right.single_outlier);
      assert(near(c.left.span,4.f) && near(c.right.span,2.f));
      assert(c.left.min_ms==1400 && c.left.max_ms==1000);
      assert(near(c.left.drift,0) && near(c.right.drift,0));
      assert(c.left.samples==9 && c.left.max_gap_ms==100);
    }
  }
  assert(o.sequence()==3);
  assert(!o.frame(8,2900,true,true,0,0));
  assert(!o.frame(7,2800,true,true,0,0));
  assert(!o.peak(7,1,2700));
  // Gaps in one marker break that side's cycle, not silently reuse prior data.
  for(unsigned t=2900;t<=4200;t+=100){
    if(t==3200 || t==4000) assert(o.peak(7,-1,t));
    if(t==3600) assert(o.peak(7,1,t));
    frame(o,7,t,true,!(t>=3300&&t<=3700));
  }
  assert(!o.latest().valid && !o.latest().right.valid);
  // Quality does not pretend that a missing cycle is adjacent evidence.
  for(unsigned t=4300;t<=5800;t+=100){
    if(t==4800 || t==5600) assert(o.peak(7,-1,t));
    if(t==4400 || t==5200) assert(o.peak(7,1,t));
    frame(o,7,t);
    if(t==5000) assert(o.latest().valid && !o.latest().continuous);
    if(t==5800) assert(o.latest().valid && o.latest().continuous);
  }
  // A noisy extreme is visible in max-min and the one-sample-trimmed span.
  Observer n;n.reset(9);assert(n.peak(9,-1,0));
  for(unsigned t=0;t<=1800;t+=100){
    if(t && t%800==0)assert(n.peak(9,-1,t));
    else if(t%800==400)assert(n.peak(9,1,t));
    const unsigned i=(t/100)%8;
    const float right=t==1100?20.f:wave[i]*.5f;
    assert(n.frame(9,t,true,true,wave[i],right));
  }
  assert(n.latest().valid && n.latest().right.span>20.f);
  assert(n.latest().right.both_directions && n.latest().right.single_outlier);
  assert(n.latest().right.spike_size>10.f && !n.latest().left.single_outlier);
  assert(n.latest().right.trimmed_span<n.latest().right.span);
  // A single spike on either period boundary can be checked using valid
  // samples outside the period; those samples never expand its span.
  for(unsigned edge : {800u,1600u}) {
    Observer boundary;boundary.reset(15);assert(boundary.peak(15,-1,0));
    for(unsigned t=0;t<=1800;t+=100){
      if(t && t%800==0)assert(boundary.peak(15,-1,t));
      else if(t%800==400)assert(boundary.peak(15,1,t));
      const float left=t==edge?20.f:wave[(t/100)%8];
      assert(boundary.frame(15,t,true,true,left,wave[(t/100)%8]*.5f));
    }
    assert(boundary.latest().valid && boundary.latest().left.single_outlier);
    assert(!boundary.latest().left.edge_unverified);
  }
  // A missing near-boundary frame must not create a falsely confident verdict.
  Observer edge_missing;edge_missing.reset(16);assert(edge_missing.peak(16,-1,0));
  for(unsigned t=0;t<=1800;t+=100){
    if(t && t%800==0)assert(edge_missing.peak(16,-1,t));
    else if(t%800==400)assert(edge_missing.peak(16,1,t));
    const float left=t==800?20.f:wave[(t/100)%8];
    assert(edge_missing.frame(16,t,t!=700,true,left,wave[(t/100)%8]*.5f));
  }
  assert(edge_missing.latest().valid && edge_missing.latest().left.edge_unverified);
  assert(!edge_missing.latest().left.single_outlier);
  // Common camera drift is not suppressed or inferred as ground motion.
  Observer d;d.reset(11);assert(d.peak(11,-1,0));
  for(unsigned t=0;t<=1800;t+=100){
    if(t && t%800==0)assert(d.peak(11,-1,t));
    else if(t%800==400)assert(d.peak(11,1,t));
    frame(d,11,t,true,true,static_cast<float>(t)*.001f);
  }
  assert(d.latest().valid && near(d.latest().left.drift,.8f) && near(d.latest().right.drift,.8f));
  // A low-amplitude side remains an observation but carries a diagnostic.
  Observer z;z.reset(12);assert(z.peak(12,-1,0));
  for(unsigned t=0;t<=1800;t+=100){
    if(t && t%800==0)assert(z.peak(12,-1,t));
    else if(t%800==400)assert(z.peak(12,1,t));
    assert(z.frame(12,t,true,true,wave[(t/100)%8],wave[(t/100)%8]*.01f));
  }
  assert(z.latest().valid && z.latest().right.low_signal && !z.latest().left.low_signal);
  assert(!z.latest().right.both_directions && z.latest().left.both_directions);
  assert(!z.latest().right.single_outlier && !z.latest().left.single_outlier);
  // A monotonic shift can have a large max-min, but does not demonstrate
  // a full oscillation; drift and the direction diagnostic make it visible.
  Observer mono;mono.reset(14);assert(mono.peak(14,-1,0));
  for(unsigned t=0;t<=1800;t+=100){
    if(t && t%800==0)assert(mono.peak(14,-1,t));
    else if(t%800==400)assert(mono.peak(14,1,t));
    assert(mono.frame(14,t,true,true,t*.01f,t*.01f));
  }
  assert(mono.latest().valid && mono.latest().continuous);
  assert(mono.latest().left.span>7.f && mono.latest().left.drift>7.f);
  assert(mono.latest().left.rise>7.f && near(mono.latest().left.fall,0.f));
  assert(!mono.latest().left.both_directions && !mono.latest().left.single_outlier);
  // Incomplete body ordering cannot produce a false cycle.
  Observer x;x.reset(13);assert(x.peak(13,-1,0));assert(x.peak(13,-1,800));
  assert(x.sequence()==0);assert(x.peak(13,1,1200));assert(x.peak(13,1,1300));
  assert(x.peak(13,-1,1600));assert(x.sequence()==0);
  std::cout<<"foot full cycle spans: startup, missing data, noise, drift and ordering PASS\n";
}
