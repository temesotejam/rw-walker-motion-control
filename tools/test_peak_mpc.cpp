#include "peak_mpc.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main(int argc,char**) {
  using namespace peak_mpc;
  Prediction p;
  State s{30.f,10.f,4.f,7950.f,1};
  assert(predict(s,95,p));assert(std::isfinite(p.peak));
  State invalid=s;invalid.rpm=1001;assert(!predict(invalid,95,p));
  invalid=s;invalid.battery=8100;assert(!predict(invalid,95,p));
  assert(!predict(s,70,p));
  assert(!choose(s,100,90).valid);
  unsigned supported=0,alternate=0,total=0;
  if(argc>1) {
    State x;float width;
    while(std::cin>>x.side>>x.rate>>x.rpm>>x.previous>>x.battery>>width) {
      ++total;auto r=choose(x,width,width-5);
      if(r.valid){++supported;if(r.alternate)++alternate;assert(std::isfinite(r.second_peak));assert(std::isfinite(r.mean_rpm));}
    }
    std::cout<<"historical decision points="<<total<<" supported="<<supported<<" alternative="<<alternate<<"\n";
    assert(total>20&&supported>0);
  }
  std::cout<<"Two-peak MPC: supported domains, 5ms cap and fallback PASS\n";
}
