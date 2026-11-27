#include <iomanip>
#include <iostream>
#include "walking/transfer_speed.h"

int main() {
  lateral_energy::Geometry geometry;
  float rate,peak,target,initial,goal,tau;
  unsigned width;
  std::cout<<std::setprecision(10);
  while(std::cin>>rate>>peak>>target>>initial>>goal>>tau>>width) {
    lateral_energy::CurrentTable table;
    if (!table.prepare(tau)) return 1;
    const auto fit=right_arrival_energy::limit(geometry,table,rate,peak,target,initial,goal,width);
    const auto predicted=right_arrival_energy::predictedJ(rate,width ? lateral_energy::charge(table,width,initial,goal) : 0.f);
    std::cout<<fit.eligible<<' '<<fit.width_ms<<' '<<lateral_energy::angle(geometry,predicted)<<' '
             <<lateral_energy::angle(geometry,fit.selected_j)<<'\n';
  }
}
