// Replay the archived aid, ordinary main and peak in the order they became known.
#define SUPPORT_TRANSFER_ARC_FEEDBACK 1
#include "half_cycle_energy_budget.h"
#include "receiving_arc_aid.h"
#include "wheel_cycle_guard.h"
#include <iomanip>
#include <iostream>

int main() {
  lateral_energy::Geometry geometry;
  receiving_arc_aid::ArrivalFeedback history;
  wheel_cycle_guard::Controller guard;
  std::cout<<std::setprecision(9);
  char event;
  while (std::cin>>event) {
    if (event=='K') {
      int side,matched,matched_main; unsigned t,departure,zero;float angle;
      std::cin>>side>>t>>angle>>matched>>departure>>zero>>matched_main;
      guard.peak(side,angle,t,matched);
      history.peak(geometry,side,t,angle,departure,zero,matched_main);
    } else if (event=='M') {
      int side;unsigned t,width;float rpm;std::cin>>side>>t>>width>>rpm;
      guard.limit(side,rpm,true,t,width);
    } else if (event=='E') {
      unsigned departure,end,elapsed;float terminal,work;
      std::cin>>departure>>end>>elapsed>>terminal>>work;
      history.completedAid(departure,end,terminal,elapsed,work);
    } else if (event=='A') {
      unsigned t,reference,baseline,width,reserve,reported_raw,extended;
      float omega,i0,goal,tau,rpm,target,request;
      std::cin>>t>>reference>>baseline>>width>>reserve>>reported_raw>>extended
              >>omega>>i0>>goal>>tau>>rpm>>target>>request;
      uint16_t new_ms=width;unsigned new_raw=reported_raw,source=0;float extra=0,work=0;
      if (extended) {
        lateral_energy::CurrentTable table;if (!table.prepare(tau)) return 2;
        const auto c=lateral_energy::coefficients(-1);
        auto q=[&](unsigned ms){return lateral_energy::charge(table,ms,i0,goal);};
        auto W=[&](unsigned ms){return lateral_energy::work(c,omega*lateral_energy::kRadPerDeg,q(ms));};
        auto inverse=[&](float wanted){unsigned ms=1;while(ms<reference && (q(ms)<0 || W(ms)<wanted))++ms;return ms;};
        auto existing=guard;
        if (inverse(request)!=reported_raw ||
            receiving_arc_aid::boundWidth(baseline,existing.limit(-1,rpm,true,t,reported_raw),reference,reserve)!=width) return 3;
        work=W(width);
        if (const auto* credit=history.recent(t)) {
          source=credit->peak_ms;extra=credit->missing_j;
          auto candidate_guard=guard;
          new_raw=inverse(std::min(.5f*target,request+extra));
          const auto candidate=receiving_arc_aid::boundWidth(width,candidate_guard.limit(-1,rpm,true,t,new_raw),reference,reserve);
          if (new_raw>reported_raw && candidate>width && candidate<=credit->elapsed_ms && q(candidate)>=0 &&
              q(candidate)<=c.charge_max_mAs && W(candidate)>0 && W(candidate)<=credit->previous_work_j) {
            new_ms=candidate;work=W(candidate);
          }
        }
      }
      std::cout<<t<<' '<<source<<' '<<extra<<' '<<new_raw<<' '<<new_ms<<' '<<work<<'\n';
    } else return 4;
  }
}
