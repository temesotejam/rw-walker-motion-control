#pragma once
#include <cmath>
#include <cstdint>
#include "foot_control_range.h"
// Shared by the UI and the actual Start command. An angle-looking nominal
// value before zero lock must not authorize this foot-dependent experiment.
namespace foot_support_start {
enum class Reason {Ready,Unavailable,Zero,FrameZero,Frame,Timestamp,Left,Right,LeftRange,RightRange,Stale,Nonfinite};
inline const char* name(Reason reason){
  switch(reason){
    case Reason::Ready:return "ready";
    case Reason::Unavailable:return "camera_unavailable";
    case Reason::Zero:return "zero_not_ready";
    case Reason::FrameZero:return "frame_zero_not_ready";
    case Reason::Frame:return "frame_invalid";
    case Reason::Timestamp:return "frame_timestamp_invalid";
    case Reason::Left:return "left_not_detected";
    case Reason::Right:return "right_not_detected";
    case Reason::LeftRange:return "left_out_of_range";
    case Reason::RightRange:return "right_out_of_range";
    case Reason::Stale:return "frame_stale";
    case Reason::Nonfinite:return "angle_nonfinite";
  }
  return "unknown";
}
template<class Snapshot> Reason reason(const Snapshot& s,uint64_t now){
  const auto& f=s.latest;
  if(!s.available)return Reason::Unavailable;
  if(!s.zero_ready)return Reason::Zero;
  if(!f.zero_ready)return Reason::FrameZero;
  if(!f.frame_valid)return Reason::Frame;
  if(!f.timestamp_valid || !f.frame_us || f.frame_us>now)return Reason::Timestamp;
  if(!f.left_valid)return Reason::Left;
  if(!f.right_valid)return Reason::Right;
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  if(!foot_control_range::allows(f.left_in_range,f.left_x,false))return Reason::LeftRange;
  if(!foot_control_range::allows(f.right_in_range,f.right_x,true))return Reason::RightRange;
#else
  if(!f.left_in_range)return Reason::LeftRange;
  if(!f.right_in_range)return Reason::RightRange;
#endif
  if(now-f.frame_us>200000)return Reason::Stale;
  if(!std::isfinite(f.left_deg) || !std::isfinite(f.right_deg))return Reason::Nonfinite;
  return Reason::Ready;
}
template<class Snapshot> bool ready(const Snapshot& s,uint64_t now){
  return reason(s,now)==Reason::Ready;
}
}
