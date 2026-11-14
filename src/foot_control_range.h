#pragma once
#include "foot_calibration_table.h"
#include <cmath>

// A bounded operating tolerance, not newly measured calibration support.
// angleAt already extends the nearest measured segment without clipping.
namespace foot_control_range {
constexpr float kExtensionDeg=5.f;
struct Assessment {bool measured=false,accepted=false;float extension_deg=NAN;};
inline Assessment assess(float x,bool right){
  Assessment out;if(!std::isfinite(x))return out;
  const auto* p=right?foot_calibration::right:foot_calibration::left;
  out.measured=x>=p[0].x && x<=p[foot_calibration::count-1].x;
  const float angle=foot_calibration::angleAt(p,x);
  if(!std::isfinite(angle))return out;
  const float edge=x<p[0].x?p[0].angle:p[foot_calibration::count-1].angle;
  out.extension_deg=out.measured?0.f:std::fabs(angle-edge);
  out.accepted=out.extension_deg<=kExtensionDeg+1e-5f;
  return out;
}
inline bool allows(bool measured,float x,bool right){return measured || assess(x,right).accepted;}
}
