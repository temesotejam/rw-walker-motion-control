#include <cassert>
#include <cmath>
#include <initializer_list>
#include "previous_peak_control_correction.h"
int main() {
  using namespace previous_peak_control;
  for(float target : {8.f,10.f,12.f})for(int side : {-1,1})
  for(unsigned ms : {0U,9999U,10000U,29999U}) {
    const auto result=evaluate(7.5f,8.f,side,target,ms);
    assert(!result.applied && result.reason==PREV_REASON_DISABLED);
    assert(result.corrected_free_peak_deg==7.5f && result.applied_correction_deg==0);
  }
  assert(std::isnan(evaluate(NAN,8,1,8,15000).corrected_free_peak_deg));
}
