#include "wheel_cycle_guard.h"
#include <iostream>
int main() {
  wheel_cycle_guard::Controller guard;
  int side,valid,width; unsigned ms; float rpm;
  while(std::cin>>ms>>side>>rpm>>valid>>width) {
    const auto next=guard.limit(side,rpm,valid!=0,ms,static_cast<uint16_t>(width));
    std::cout<<ms<<' '<<side<<' '<<width<<' '<<next<<'\n';
  }
}
