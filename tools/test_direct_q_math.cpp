#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include "direct_q_solver.h"

static float charge(unsigned ms, float initial, float final, float tau) {
  if (!ms) return 0;
  const float t = static_cast<float>(ms) / 1000.0f;
  return fabsf(final*t + (initial-final)*tau*(1-expf(-t/tau)));
}
static uint32_t seed = 4718;
static float uniform() {
  seed = 1664525U*seed+1013904223U;
  return static_cast<float>(seed>>8)/16777216.0f;
}
static void compare(float request, float initial, float final, float tau, unsigned limit=100) {
  const auto r=direct_q::width(request,initial,final,tau,limit);
  assert(r.valid && r.ms<=limit && r.evaluations<=25);
  float best=request; unsigned best_ms=0;
  for(unsigned ms=1;ms<=limit;++ms) {
    const float error=fabsf(charge(ms,initial,final,tau)-request);
    if(error<best) {best=error;best_ms=ms;}
  }
  if(r.error!=best || r.ms!=best_ms) {
    std::printf("inverse mismatch q=%.9g i0=%.9g final=%.9g tau=%.9g limit=%u got=%u/%.9g oracle=%u/%.9g\n",
        request,initial,final,tau,limit,r.ms,r.error,best_ms,best);
    std::fflush(stdout);
    assert(false);
  }
  assert(r.charge==charge(r.ms,initial,final,tau));
}
int main() {
  for(unsigned i=0;i<20000;++i) {
    const float final=(i&1 ? -1:1)*(30+470*uniform());
    const float initial=1200*uniform()-600;
    const float tau=0.01f+0.19f*uniform();
    const float request=50*uniform();
    compare(request,initial,final,tau);
    if(i<100) {
      compare(0,initial,final,tau);
      compare(charge(100,initial,final,tau),initial,final,tau);
      compare(charge(i,initial,final,tau),initial,final,tau);
    }
  }
  // Opposed residual current: an early negative lobe and a later positive
  // lobe can reach the same |Q|. Check exact integer targets on both branches.
  for(unsigned ms=0;ms<=100;++ms) {
    compare(charge(ms,-120,250,0.06f),-120,250,0.06f);
    compare(charge(ms,120,-250,0.06f),120,-250,0.06f);
  }
  for(unsigned i=0;i<10000;++i) {
    const float free=12*uniform(), desired=8+4*uniform();
    const float base=0.1f+0.4f*uniform(), gain=0.1f+0.6f*uniform();
    const float c=6*uniform()-3, limit=1.5f, available=20*uniform();
    const float integral=10*uniform()-5;
    const auto r=direct_q::target(free,desired,base,c,gain,limit,available,integral);
    assert(r.valid);
    // Independent double-precision forward evaluation + continuous bisection.
    auto forward=[&](double q) {
      const double correction=fmax(-double(limit),fmin(double(limit),double(c)+(double(gain)-base)*q));
      return fmax(0.0,double(free)+double(base)*q+correction);
    };
    double lo=0,hi=1000;
    if(forward(0)>=desired) hi=0;
    for(unsigned k=0;k<60;++k) {
      const double mid=(lo+hi)/2;
      if(forward(mid)<desired) lo=mid; else hi=mid;
    }
    assert(fabs(r.requested_ff-hi)<=2e-5*fmax(1.0,hi));
    assert(r.feedforward>=0 && r.feedforward<=available);
    assert(r.charge>=0 && r.charge<=available);
    assert(r.upper==(r.requested_ff>available || r.unclamped>available));
    assert(r.lower==(r.unclamped<0));
  }
  assert(direct_q::target(5,8,.25f,0,.25f,1.5f,10,-2).feedforward==10);
  assert(direct_q::target(5,8,.25f,0,.25f,1.5f,10,-2).charge==8);
  assert(direct_q::target(9,8,.25f,0,.25f,1.5f,10,0).charge==0);
  const float bads[]={NAN,INFINITY,-INFINITY};
  for(float bad:bads) {
    assert(!direct_q::width(bad,0,250,.06f).valid);
    assert(!direct_q::width(1,bad,250,.06f).valid);
    assert(!direct_q::width(1,0,bad,.06f).valid);
    assert(!direct_q::width(1,0,250,bad).valid);
    for(unsigned index=0;index<8;++index) {
      float p[]={5,8,.25f,0,.25f,1.5f,13,0};p[index]=bad;
      assert(!direct_q::target(p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]).valid);
    }
  }
  assert(!direct_q::width(-1,0,250,.06f).valid);
  assert(!direct_q::width(1,0,0,.06f).valid);
  assert(!direct_q::width(1,0,250,0).valid);
  assert(!direct_q::width(1,0,250,.06f,101).valid);
  compare(0,0,250,.06f,0);
  compare(1,0,250,.06f,0);
  compare(.1f,-600,30,.2f,1);
  std::puts("Direct inverse: 20,000 full-grid width oracles, reversed-current roots, 10,000 independent angle inverses and invalid/boundary cases PASS");
}
