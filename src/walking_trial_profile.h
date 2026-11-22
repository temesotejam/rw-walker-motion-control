#pragma once
#include <cstdint>
#include <cstring>

namespace walking_trial {
enum class Profile : uint8_t { None, Turn3, SmallStep, RightArrival, CarryHistory };
#if WALKING_SMALL_STEP_AB
constexpr bool comparison_enabled=true;
#else
constexpr bool comparison_enabled=false;
#endif

inline Profile requestedProfile(const char* model) {
  if (comparison_enabled && model && std::strcmp(model,"work_control_small_step")==0) return Profile::SmallStep;
#if WALKING_RETURN_HISTORY
  if (model && std::strcmp(model,"work_control_return_history")==0) return Profile::CarryHistory;
#endif
  return Profile::None;
}
inline constexpr Profile standardProfile() {
#if WALKING_RIGHT_ARRIVAL
  return Profile::RightArrival;
#else
  return Profile::Turn3;
#endif
}
inline bool comparisonAllowed(Profile profile, float target, float percent, int current, unsigned brake,
                             bool adaptation, bool bridge, bool probe) {
  const bool available = profile==Profile::SmallStep
#if WALKING_RETURN_HISTORY
      || profile==Profile::CarryHistory
#endif
      ;
  return comparison_enabled && available && target==8.f && percent==0.f && current==300 &&
      brake==40 && !adaptation && bridge && !probe;
}
inline const char* name(Profile profile) {
  switch (profile) {
    case Profile::Turn3: return "turn3";
    case Profile::SmallStep: return "v14_small_step";
    case Profile::RightArrival: return "right_arrival_v1";
    case Profile::CarryHistory: return "left_return_history_v1";
    default: return "none";
  }
}
}
