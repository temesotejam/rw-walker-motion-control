#pragma once
#include <cstdint>

namespace walking_trial {
enum class Profile : uint8_t { None, Turn3, SmallStep, RightArrival, CarryHistory, ReturnPrediction, LeftTransferCarryV1 };
// The accepted run settings determine the logged and executed policy. Earlier
// names remain available for interpreting archived measurements.
inline Profile forRun(bool wheel_work, float target_deg, float input_percent) {
  if (!wheel_work || target_deg!=8.f || input_percent!=0.f) return Profile::None;
#if WALKING_SMALL_STEP_AB && WALKING_RETURN_HISTORY
  return Profile::LeftTransferCarryV1;
#elif WALKING_SMALL_STEP_AB && WALKING_RIGHT_ARRIVAL
  return Profile::RightArrival;
#elif WALKING_SMALL_STEP_AB
  return Profile::Turn3;
#else
  return Profile::None;
#endif
}
inline const char* name(Profile profile) {
  switch (profile) {
    case Profile::Turn3: return "turn3";
    case Profile::SmallStep: return "v14_small_step";
    case Profile::RightArrival: return "right_arrival_v1";
    case Profile::CarryHistory: return "left_return_history_v1";
    case Profile::ReturnPrediction: return "left_return_prediction_v1";
    case Profile::LeftTransferCarryV1: return "left_transfer_carry_v1";
    default: return "none";
  }
}
}
