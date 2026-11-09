#pragma once
#include <math.h>
#include "foot_tracking_config.h"

namespace marker_identity {
inline bool weakDisplaced(float last_x, float last_y, float last_contrast, float last_weight,
                          float x, float y, float contrast, float weight) {
  return fabsf(x - last_x) >= appcfg::kWhiteWeakXStepPx &&
      fabsf(y - last_y) >= appcfg::kWhiteWeakYStepPx &&
      contrast < last_contrast * appcfg::kWhiteWeakContrastRatio &&
      weight < last_weight * appcfg::kWhiteWeakWeightRatio;
}
}
