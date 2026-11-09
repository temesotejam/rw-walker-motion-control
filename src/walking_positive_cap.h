#pragma once
#include <cstdint>

namespace walking_positive_cap {
// Scope: only the otherwise-uncapped positive fallback of target-8 work mode.
// All other selectors retain their own established limits.
inline uint16_t fallback(bool work_mode,bool work_choice_valid,int side,float target_deg,
                         uint16_t gradual_cap,uint16_t wheel_cap,uint16_t maximum) {
  if(!work_mode || work_choice_valid || side<=0 || target_deg!=8.f)return maximum;
  uint16_t result=gradual_cap<wheel_cap ? gradual_cap : wheel_cap;
  return result<maximum ? result : maximum;
}
}
