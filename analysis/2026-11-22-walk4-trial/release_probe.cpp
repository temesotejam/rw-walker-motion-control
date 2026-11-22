// Feed real, in-pulse IMU samples to the production walk5 release detector.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#include "return_observation_policy.h"
#include <iostream>

int main() {
  char tag;
  float decel=0, terminal=0;
  return_observation_policy::PostAidObserver detector;
  while (std::cin >> tag) {
    if (tag=='G') {
      float l, r, le, re, fore, angle;
      std::cin >> l >> r >> le >> re >> fore >> angle;
      std::cout << (l>r && le>re && std::fabs(fore)<=10 && std::fabs(l)<=20 && std::fabs(r)<=20 &&
                    std::fabs(le)<=20 && std::fabs(re)<=20 &&
                    support_transfer_plan::supportGap(l,r,fore,angle)<0 &&
                    support_transfer_plan::supportGap(le,re,fore,angle)<0) << '\n';
    } else if (tag=='A') {
      float pitch;
      std::cin >> pitch >> decel;
      terminal=support_transfer_plan::arcEntryDeg(-1,pitch);
      detector={};
      std::cout << terminal << '\n';
    } else if (tag=='I') {
      uint32_t stamp, age, elapsed;
      float angle, rate;
      std::cin >> stamp >> age >> elapsed >> angle >> rate;
      std::cout << detector.releaseOnCarry(stamp,age,elapsed,angle,rate,terminal,decel) << '\n';
    } else return 1;
    if (!std::cin) return 2;
  }
}
