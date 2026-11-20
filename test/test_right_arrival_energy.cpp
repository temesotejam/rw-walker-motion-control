#include <cassert>
#include <cmath>
#include "right_arrival_energy.h"

int main() {
  using namespace right_arrival_energy;
  lateral_energy::Geometry g;
  lateral_energy::CurrentTable table;
  assert(table.prepare(.031652f));
  const auto bounded=[&](float rate, float peak, float target, float i0, unsigned width){
    return limit(g,table,rate,peak,target,i0,353.2416f,width);
  };
  const auto held=bounded(29.6139f,3.63306f,4.5f,-4.1712f,28);
  assert(held.eligible && held.reason==Reason::Reduced && held.width_ms==18);
  assert(held.coast_j<held.selected_j && held.selected_j<held.original_j);
  assert(bounded(38.f,4,4.5f,0,30).width_ms==0);
  assert(bounded(38.f,4,4.5f,0,0).width_ms==0);
  assert(bounded(51.01f,4,4.5f,0,30).reason==Reason::OutsideSupport);
  assert(bounded(20.99f,4,4.5f,0,30).reason==Reason::OutsideSupport);
  assert(bounded(26.f,1.8f,4.5f,0,30).reason==Reason::OutsideSupport);
  assert(bounded(26.f,4.f,4.49f,0,30).reason==Reason::OutsideSupport);
  assert(bounded(26.f,4.f,4.5f,0,80).width_ms==80);
  assert(bounded(28.f,4.f,8.f,0,20).width_ms==20);
  assert(std::isnan(predictedJ(30,-1)) && std::isnan(predictedJ(40,9)));
  // Compare all eligible bounded selections against exhaustive integer search,
  // including negative residual current before positive charge is attained.
  for (float rate:{21.f,26.f,35.f,40.f,47.f}) for (float i0:{-18.f,0.f,10.f})
    for (unsigned raw=0;raw<=80;++raw) {
      const auto choice=bounded(rate,4.f,4.5f,i0,raw);
      if (!choice.eligible) {assert(choice.width_ms==raw);continue;}
      assert(choice.width_ms<=raw && std::isfinite(choice.selected_j));
      for (unsigned n=0;n<=raw;++n) {
        const float j=predictedJ(rate,n?lateral_energy::charge(table,n,i0,353.2416f):0);
        if (std::isfinite(j)) assert(std::fabs(choice.selected_j-choice.target_j)<=std::fabs(j-choice.target_j)+2e-9f);
      }
    }
}
