#pragma once
#include <cstdint>
#include <cstring>

namespace walking_trial {
enum class Profile : uint8_t { None, Turn3, SmallStep, RightArrival };
#if WALKING_SMALL_STEP_AB
constexpr bool comparison_enabled=true;
#else
constexpr bool comparison_enabled=false;
#endif

inline bool requestsSmallStep(const char* model) {
  return comparison_enabled && model && std::strcmp(model,"work_control_small_step")==0;
}
inline constexpr Profile standardProfile() {
#if WALKING_RIGHT_ARRIVAL
  return Profile::RightArrival;
#else
  return Profile::Turn3;
#endif
}
inline bool smallStepAllowed(float target, float percent, int current, unsigned brake,
                             bool adaptation, bool bridge, bool probe) {
  return comparison_enabled && target==8.f && percent==0.f && current==300 &&
      brake==40 && !adaptation && bridge && !probe;
}
inline const char* name(Profile profile) {
  switch (profile) {
    case Profile::Turn3: return "turn3";
    case Profile::SmallStep: return "v14_small_step";
    case Profile::RightArrival: return "right_arrival_v1";
    default: return "none";
  }
}
}
