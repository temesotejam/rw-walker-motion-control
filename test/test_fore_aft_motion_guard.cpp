#include "fore_aft_motion_guard.h"
#include <cassert>
#include <iostream>
#include <iomanip>
namespace f=fore_aft_motion_guard;
mekf6::Quaternion quat(float roll,float pitch,float yaw=0){
 const float r=roll*.008726646259971648f,p=pitch*.008726646259971648f,y=yaw*.008726646259971648f;
 return {std::cos(r)*std::cos(p)*std::cos(y)+std::sin(r)*std::sin(p)*std::sin(y),
 std::sin(r)*std::cos(p)*std::cos(y)-std::cos(r)*std::sin(p)*std::sin(y),
 std::cos(r)*std::sin(p)*std::cos(y)+std::sin(r)*std::cos(p)*std::sin(y),
 std::cos(r)*std::cos(p)*std::sin(y)-std::sin(r)*std::sin(p)*std::cos(y)};
}
f::Input input(uint32_t us,float roll,float pitch=0,float yaw=0){f::Input x;
 x.now_us=x.sample_us=us;x.valid=true;x.quaternion=quat(roll,pitch,yaw);return x;}
void tests(){
 {f::Guard g;assert(!g.result().initialized && !g.result().allow_output);
 auto a=g.observe(input(1000,7));assert(a.initialized && a.allow_output && a.angle_deg==0);
 auto b=g.observe(input(3500,8));assert(std::fabs(b.angle_deg-1)<.001f && b.allow_output);
 assert(b.start_sample_us==1000 && std::fabs(b.start_roll_deg-7)<.001f);}
 for(int sign:{-1,1}){f::Guard g;bool stopped=false;uint32_t stop_us=0;
  for(uint32_t t=1000;t<700000;t+=2500){auto x=input(t,sign*40.f*(t-1000)*1e-6f);
   x.gyro_rad_s.x=sign*40.f*.0174532925199433f;const auto a=g.observe(x);
   if(a.reason==f::Reason::Candidate)assert(!a.allow_output && !a.stop);
   if(a.stop){stopped=true;stop_us=t;assert(a.reason==f::Reason::Diverging);break;}}
  assert(stopped && stop_us>=188000 && stop_us<=200000);
  assert(g.observe(input(stop_us+2500,0)).stop);g.reset();assert(!g.observe(input(1000,0)).stop);
 }
 {f::Guard g;for(uint32_t t=1000;t<2000000;t+=2500){
   const float seconds=(t-1000)*1e-6f;auto x=input(t,5.8f*std::sin(12.566370614f*seconds));
   x.gyro_rad_s.x=5.8f*12.566370614f*std::cos(12.566370614f*seconds)*.0174532925199433f;
   auto a=g.observe(x);assert(!a.stop && a.allow_output);}}
 {f::Guard g;for(uint32_t t=1000;t<2000000;t+=2500){
   const float seconds=(t-1000)*1e-6f;auto x=input(t,8.f*std::sin(6.283185307f*seconds));
   x.gyro_rad_s.x=8.f*6.283185307f*std::cos(6.283185307f*seconds)*.0174532925199433f;
   assert(!g.observe(x).stop);}}
 {f::Guard g;g.observe(input(1000,0));auto x=input(3500,7);x.gyro_rad_s.x=100*.0174532925199433f;
  auto a=g.observe(x);assert(a.reason==f::Reason::Candidate && !a.allow_output);
  for(unsigned t=6000;t<30000;t+=2500){x=input(t,7.2f);x.gyro_rad_s.x=100*.0174532925199433f;assert(!g.observe(x).stop);}
  assert(g.observe(input(31000,7.2f)).allow_output);}
 {f::Guard g;g.observe(input(1000,0));
  for(unsigned t=3500;t<110000;t+=2500){auto x=input(t,7);x.gyro_rad_s.x=100*.0174532925199433f;
   assert(!g.observe(x).stop);} // No measured angle gain: gyro alone cannot confirm.
  g.reset();auto x=input(1000,0);g.observe(x);x=input(3500,7);x.gyro_rad_s.x=100*.0174532925199433f;
  g.observe(x);for(unsigned t=4000;t<13000;t+=1000){x.now_us=t;const auto a=g.observe(x);
   assert(!a.stop && !a.sample_advanced && a.candidate_us==0);}}
 for(int sign:{-1,1}){f::Guard g;g.observe(input(1000,3));
  for(unsigned i=1;i<=880;++i){auto x=input(1000+i*2500,3,sign*i*.125f,37);
   x.gyro_rad_s.y=sign*50.f*.0174532925199433f*std::cos(3.f*.0174532925199433f);
   x.gyro_rad_s.z=-sign*50.f*.0174532925199433f*std::sin(3.f*.0174532925199433f);
   auto a=g.observe(x);assert(!a.stop);if(a.valid)assert(std::fabs(a.angle_deg)<.002f);}}
 // Verify derivative independently by advancing a quaternion with body rates.
 {f::Guard g;auto x=input(1000,0);g.observe(x);x=input(3500,12,30,41);
  x.gyro_rad_s={.4f,.7f,-.3f};auto a=g.observe(x);
  const auto q=x.quaternion;const float dt=.00001f;
  const mekf6::Quaternion d{1,.5f*dt*.4f,.5f*dt*.7f,.5f*dt*-.3f};
  x.quaternion={q.w*d.w-q.x*d.x-q.y*d.y-q.z*d.z,q.w*d.x+q.x*d.w+q.y*d.z-q.z*d.y,
    q.w*d.y-q.x*d.z+q.y*d.w+q.z*d.x,q.w*d.z+q.x*d.y-q.y*d.x+q.z*d.w};
  x.now_us=x.sample_us=3510;const auto b=g.observe(x);
  assert(std::fabs((b.angle_deg-a.angle_deg)/dt-a.rate_dps)<.2f);
  assert(std::fabs(a.rate_dps-.4f*57.2957795f)>1);}
 {f::Guard a;auto x=input(1000,0);x.now_us=11001;assert(a.observe(x).reason==f::Reason::Stale);
  f::Guard b;x=input(1000,0);x.valid=false;assert(b.observe(x).reason==f::Reason::Invalid);
  f::Guard c;c.observe(input(1000,0));auto first=c.observe(input(3500,0));
  assert(first.sample_advanced && !c.observe(input(3500,0)).sample_advanced);}
 {f::Guard g;g.observe(input(1000,3,20,41));auto x=input(3500,3,20,41);
  x.quaternion.w=-x.quaternion.w;x.quaternion.x=-x.quaternion.x;
  x.quaternion.y=-x.quaternion.y;x.quaternion.z=-x.quaternion.z;
  const auto a=g.observe(x);assert(a.allow_output && std::fabs(a.angle_deg)<.001f);}
 {f::Guard g;const uint32_t origin=UINT32_MAX-100000;
  bool stopped=false;for(uint32_t dt=0;dt<300000;dt+=2500){auto x=input(origin+dt,40.f*dt*1e-6f);
   x.gyro_rad_s.x=40*.0174532925199433f;if(g.observe(x).stop){stopped=true;break;}}
  assert(stopped);}
 std::cout<<"portable PASS\n";
}

int main() { tests(); }
