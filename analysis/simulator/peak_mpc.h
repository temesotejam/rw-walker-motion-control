#pragma once
#include <cmath>
#include <cstdint>

// Observational 9-run model: historical-action replay is not proof of a
// counterfactual input response. Restrict the first move to the existing
// solver or a 5 ms smaller cap; reject every out-of-domain rollout.
namespace peak_mpc {
struct State { float rate, rpm, previous, battery; int side; };
struct Prediction { float peak, next_rate, next_rpm; };
struct Result { bool valid=false; bool alternate=false; float second_peak=NAN; float mean_rpm=NAN; };
inline bool predict(const State& s, float width, Prediction& out) {
  if(!std::isfinite(s.rate)||!std::isfinite(s.rpm)||!std::isfinite(s.previous)||
     !std::isfinite(s.battery)||!std::isfinite(width)||s.battery<7420||s.battery>8060) return false;
  if(s.side==1) {
    if(s.rate<12.24f||s.rate>35.44f||s.rpm< -2.25f||s.rpm>327.81f||s.previous<.99f||s.previous>4.98f||width<80||width>100) return false;
  } else if(s.side==-1) {
    if(s.rate<14.72f||s.rate>36.51f||s.rpm<275.25f||s.rpm>601.28f||s.previous<4.67f||s.previous>7.25f||width<25||width>65) return false;
  } else return false;
  const float pos[6][3]={{.094651624f,42.4553474f,-35.307996f},{.186581482f,-.980478965f,-5.0583825f},{-.020656265f,.760153702f,62.7536162f},{-.69895928f,4.32634465f,23.4935381f},{.31093198f,-1.09181985f,34.5164716f},{.021915424f,.720539917f,11.9761038f}};
  const float neg[6][3]={{7.44141506f,7.45301569f,-214.340225f},{.088476321f,.399312805f,-1.27101545f},{-.207706879f,-.599593186f,61.2792872f},{-.338859542f,-1.89068978f,11.3411319f},{-.135374191f,2.25677564f,-11.9792803f},{-.332094951f,1.92925636f,4.59956794f}};
  const float (*c)[3]=s.side>0?pos:neg;
  const float x[6]={1,s.rate,s.rpm/100.f,s.previous,width/10.f,s.battery/1000.f};
  float y[3]={};for(int i=0;i<6;++i)for(int j=0;j<3;++j)y[j]+=x[i]*c[i][j];
  out={y[0],y[1],y[2]};
  return std::isfinite(y[0])&&std::isfinite(y[1])&&std::isfinite(y[2])&&y[0]>0&&y[0]<15&&y[1]>0&&std::fabs(y[2])<1000;
}
inline bool cost(const State& s,float width,float target1,float target2,float& best,float& best_peak,float& best_rpm) {
  Prediction a;if(!predict(s,width,a))return false;
  State next{a.next_rate,a.next_rpm,a.peak,s.battery,-s.side};
  const float positive[]={80,85,90,95,100};
  const float negative[]={25,30,35,40,45,50,55,60,65};
  const float* widths=next.side>0?positive:negative;const int n=next.side>0?5:9;
  bool found=false;best=INFINITY;
  for(int i=0;i<n;++i) {
    Prediction b;if(!predict(next,widths[i],b))continue;
    const float e1=(a.peak-target1)/8.f,e2=(b.peak-target2)/8.f;
    // Crossing RPM is a phase-specific proxy, not a measured cycle average.
    const float proxy=(a.next_rpm+b.next_rpm)/800.f;
    const float score=e1*e1+e2*e2+.2f*proxy*proxy;
    if(score<best){found=true;best=score;best_peak=b.peak;best_rpm=(a.next_rpm+b.next_rpm)*.5f;}
  }
  return found;
}
inline Result choose(const State& state,float baseline_ms,float alternate_ms,float target1=8.f,float target2=8.f) {
  Result r;
  if(alternate_ms<baseline_ms-5.f||alternate_ms>=baseline_ms||!std::isfinite(target1)||!std::isfinite(target2)||target1<3.5f||target1>10.f||target2<3.5f||target2>10.f)return r;
  float a=0,b=0,p1=0,p2=0,r1=0,r2=0;
  if(!cost(state,baseline_ms,target1,target2,a,p1,r1)||!cost(state,alternate_ms,target1,target2,b,p2,r2))return r;
  r.valid=true;r.alternate=b+.005f<a;r.second_peak=r.alternate?p2:p1;r.mean_rpm=r.alternate?r2:r1;
  return r;
}
}
