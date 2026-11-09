#pragma once
#include <cmath>
namespace work_target_policy {
inline bool valid(float target) { return target==8.f || target==10.f || target==12.f; }
inline bool packageAllowed(float target,unsigned brake_ms,int current_ma,bool old_adaptation) {
  return valid(target) && current_ma==300 && !old_adaptation &&
      (brake_ms==40 || (brake_ms==25 && target==8.f));
}
// Bounded experimental envelopes for new target tests, not refitted/validated
// actuator models. Preserve the measured 8-degree envelope exactly.
inline float rateCeiling(float target,int side) {
  if (target==8.f) return side>0 ? 72.7405f : 76.2778f;
  if (target==10.f) return side>0 ? 85.f : 90.f;
  if (target==12.f) return side>0 ? 95.f : 100.f;
  return NAN;
}
inline float reserveFloor(float target) { return target-1.f; }
inline float reserveFull(float target) { return target-.2f; }
inline float previousCeiling(float target) { return target+2.f; }
// Keep completed-peak learning below the unchanged 16-degree stop boundary.
inline float observationCeiling(float target) { return target==12.f ? 15.f : 13.f; }
inline unsigned negativeFeedbackIndex(float target,bool assist) {
  return assist && target!=12.f ? 3u : 2u;
}
}
