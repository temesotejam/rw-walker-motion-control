#include "adaptive_lateral_cg.h"
#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

int main() {
  const lateral_energy::Geometry g;
  lateral_cg::Adaptive model;
  model.reset(0,0);
  assert(model.snapshot().enabled && model.snapshot().pairs==0);
  const float truth=.65f,bias=.07f;
  uint32_t ms=5000;
  for(unsigned n=0;n<100;++n) {
    for(int side : {1,-1}) {
      const float a=side>0 ? 7.8f : 9.2f;
      const float raw=lateral_cg::potential(g,side*a,truth)*1000.f-bias;
      const auto before=model.snapshot();
      const bool updated=model.observe(g,side,a,raw,ms);
      const auto after=model.snapshot();
      assert(std::fabs(after.shift_mm-before.shift_mm)<=.02501f);
      assert(std::fabs(after.common_bias_mj-before.common_bias_mj)<=.01001f);
      if(side>0) assert(!updated && before.updates==after.updates);
      if(n<2) assert(!updated);
      ms+=450;
    }
  }
  assert(std::fabs(model.snapshot().shift_mm-truth)<.0001f);
  assert(std::fabs(model.snapshot().common_bias_mj-bias)<.0001f);
  const auto before=model.snapshot();
  assert(!model.observe(g,1,8,NAN,ms));
  assert(!model.observe(g,-1,8,1000,ms+1));
  assert(before.shift_mm==model.snapshot().shift_mm && before.updates==model.snapshot().updates);
  // One side cannot identify an odd potential term separately from common loss.
  model.reset();
  for(unsigned n=0;n<20;++n) model.observe(g,1,8,.7f,5000+450*n);
  assert(model.snapshot().updates==0 && model.snapshot().pairs==0);
  // A lone/stale opposite observation is not a completed adjacent pair.
  model.reset();model.observe(g,1,8,.7f,5000);model.observe(g,-1,8,.8f,7000);
  assert(model.snapshot().pairs==0);
  // Replay recorded inputs. Parameters are read before each observed peak;
  // this checks causality, not a new closed-loop hardware trajectory.
  model.reset(0,0);
  std::ifstream file("tools/fixtures/adaptive_cg_124463989.csv");assert(file);
  std::string line;std::getline(file,line);
  unsigned rows=0;float previous=0;
  while(std::getline(file,line)) {
    std::stringstream stream(line);std::string field;float v[5];unsigned n=0;
    while(std::getline(stream,field,',')) { assert(n<5);v[n++]=std::stof(field); }
    assert(n==5);
    const auto snapshot=model.snapshot();
    assert(std::isfinite(lateral_cg::potential(g,8,snapshot.shift_mm)));
    model.observe(g,static_cast<int8_t>(v[1]),v[2],v[3],static_cast<uint32_t>(v[0]));
    assert(std::fabs(model.snapshot().shift_mm-previous)<=.02501f);
    previous=model.snapshot().shift_mm;++rows;
  }
  assert(rows>40 && model.snapshot().updates>5 && model.snapshot().shift_mm<0.f);
  std::cout << "adaptive CG: synthetic recovery, no one-sided learning, bounded causal updates, stale/outlier rejection PASS\n"
      << "recorded-input replay: rows=" << rows << " pairs=" << model.snapshot().pairs
      << " updates=" << model.snapshot().updates << " final_mm=" << model.snapshot().shift_mm << "\n";
}
