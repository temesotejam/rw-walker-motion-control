#pragma once
#include "cad_support_vertices.h"
#include <cmath>

namespace cad_support {
// Fixed-pose rigid geometry, not measured stance or whole-system potential.
inline float lowest(const Vertex* points,size_t n,float foot,float pitch,float roll) {
  const float rad=.01745329251994329577f;
  const float a=(foot+pitch)*rad,b=roll*rad;
  const float ca=std::cos(a),sa=std::sin(a),cb=std::cos(b),sb=std::sin(b);
  float z=INFINITY;
  for(size_t i=0;i<n;++i) {
    const auto& v=points[i];
    const float w=-sb*v.x+cb*(sa*v.y+ca*v.z);
    if(w<z)z=w;
  }
  return z;
}
inline float gap(float l,float r,float pitch,float roll) {
  return lowest(left,sizeof(left)/sizeof(left[0]),l,pitch,roll)-
         lowest(right,sizeof(right)/sizeof(right[0]),r,pitch,roll);
}
inline float support(float l,float r,float pitch,float roll) {
  return -std::fmin(lowest(left,sizeof(left)/sizeof(left[0]),l,pitch,roll),
                    lowest(right,sizeof(right)/sizeof(right[0]),r,pitch,roll));
}
inline bool switchAngle(float l,float r,float pitch,float& angle) {
  angle=NAN;
  if(!std::isfinite(l)||!std::isfinite(r)||!std::isfinite(pitch)||
     std::fabs(l)>20||std::fabs(r)>20||std::fabs(pitch)>10)return false;
  float lo=-15,hi=15,fl=gap(l,r,pitch,lo),fh=gap(l,r,pitch,hi);
  if(fl*fh>0)return false;
  for(unsigned i=0;i<18;++i) {
    const float mid=(lo+hi)*.5f,fm=gap(l,r,pitch,mid);
    if(fl*fm<=0)hi=mid;else {lo=mid;fl=fm;}
  }
  angle=(lo+hi)*.5f;return true;
}
}
