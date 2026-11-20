// Stream recorded camera receipts, publications, peaks, and commands in clock order.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#include "periodic_foot_state.h"
#include "receiving_arc_aid.h"
#include "tilt_priority.h"
#include "wheel_cycle_guard.h"
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

float number() { std::string s; std::cin>>s; return std::stof(s); }
int main() {
  uint64_t epoch; std::cin>>epoch;
  periodic_foot::Estimator feet; feet.start(epoch);
  receiving_arc_aid::MainReserve reserve;
  wheel_cycle_guard::Controller guard;
  tilt_priority::Controller timing;
  std::cout<<std::setprecision(9);
  char event;
  while (std::cin>>event) {
    if (event=='F') {
      uint32_t seq; uint64_t image,received; int ok,lv,lr,rv,rr;
      std::cin>>seq>>image>>received>>ok>>lv>>lr;
      const float left=number(); std::cin>>rv>>rr;
      feet.observe(seq,image,received,ok,lv,lr,left,rv,rr,number());
    } else if (event=='P') {
      uint64_t publish; std::cin>>publish;
      if (!feet.publish(publish)) throw std::runtime_error("Missing publication");
    } else if (event=='K') {
      int side,matched; unsigned time; std::cin>>side>>time>>matched;
      const float angle=number();
      guard.peak(side,angle,time,matched); timing.peak(side,time,angle);
    } else if (event=='M') {
      int side,driven; unsigned t,width; std::cin>>side>>t>>driven>>width;
      const float rpm=number(), target=number();
      if (driven) guard.limit(side,rpm,true,t,width);
      timing.crossing(side,t); tilt_priority::Choice c{}; c.selected=true;
      c.direction=side; c.coast=!driven; c.width_ms=width; c.current_ma=driven?1200:0;c.target_deg=target;
      timing.confirm(side,t,c,true,true,false);
    } else if (event=='E') {
      int side,assisted; unsigned elapsed; std::cin>>side>>assisted>>elapsed;
      reserve.observe(side,assisted,elapsed);
    } else if (event=='R') {
      receiving_arc_aid::Input arc;
      uint64_t now,image,received; unsigned t,packet,lseq,rseq,raw_seq,envelope,observed;
      std::cin>>t>>now>>packet>>lseq>>rseq;
      const float stored_left=number(), stored_right=number();
      std::cin>>raw_seq>>image>>received>>envelope>>observed>>arc.side;
      arc.body_deg=number();arc.speed_dps=number();arc.decel_dps2=number();arc.fore_aft_deg=number();
      arc.left_deg=number();arc.right_deg=number();arc.base_j=number();arc.target_j=number();
      const float i0=number(),goal=number(),tau=number(),rpm=number();
      const auto& p=feet.packet;
      if (p.count!=packet || p.left.sequence!=lseq || p.right.sequence!=rseq)
        throw std::runtime_error("Causal replay disagrees with recorded packet or source image");
      const auto sameAngle=[](float computed,float stored) {
        return std::isfinite(stored) ? std::isfinite(computed) && std::fabs(computed-stored)<=.001f
                                     : !std::isfinite(computed);
      };
      if (!sameAngle(p.left.estimated,stored_left) || !sameAngle(p.right.estimated,stored_right))
        throw std::runtime_error("Reconstructed estimate does not match the archived 0.001 deg value");
      periodic_foot::Side left=p.left,right=p.right;
      unsigned source=0;
      if (observed && periodic_foot::matchesPair(p,now,raw_seq,image))
        source=1;
      else if (observed && feet.rescuePair(now,epoch,raw_seq,image,received,left,right))
        source=2;
      arc.observed_shape=source!=0;
      arc.left_est=left.estimated;arc.right_est=right.estimated;
      arc.left_rate=left.rate_dps;arc.right_rate=right.rate_dps;
      lateral_energy::CurrentTable table; if (!table.prepare(tau)) return 2;
      const auto cap=timing.timingCap(arc.side,t,envelope);
      const auto width=[&](float request) {
        for (unsigned w=1;w<cap;++w) {
          const auto q=lateral_energy::charge(table,w,i0,goal);
          if(q>=0 && lateral_energy::work(lateral_energy::coefficients(arc.side),arc.speed_dps*lateral_energy::kRadPerDeg,q)>=request)
            return w;
        }
        return unsigned(cap);
      };
      const auto old=width(arc.base_j);
      auto g1=guard,g2=guard;
      const auto baseline=g1.limit(arc.side,rpm,true,t,old);
#if SUPPORT_TRANSFER_STALL_V3
      const auto old_pivot=receiving_arc_aid::evaluate(arc);
      auto old_guard=guard;
      const auto old_candidate=old_guard.limit(arc.side,rpm,true,t,width(arc.base_j+old_pivot.extra_j));
      arc.base_width_ms=baseline;
#endif
      const auto result=receiving_arc_aid::evaluate(arc);
      const auto proposal=width(arc.base_j+result.extra_j);
      const auto candidate=g2.limit(arc.side,rpm,true,t,proposal);
      const auto reserved=reserve.forSide(arc.side);
      const auto choice=receiving_arc_aid::boundWidth(baseline,candidate,envelope,reserved);
      std::cout<<t<<' '<<now-p.published_us<<' '<<source<<' '<<arc.left_est<<' '<<arc.right_est<<' '<<arc.left_rate<<' '
               <<arc.right_rate<<' '<<int(result.reason)<<' '<<result.extra_j<<' '<<cap<<' '<<old<<' '
               <<baseline<<' '<<proposal<<' '<<candidate<<' '<<reserved<<' '<<choice;
#if SUPPORT_TRANSFER_STALL_V3
      const auto clearance=[&](float s) {
        return -support_transfer_plan::supportGap(arc.left_est+arc.left_rate*s,arc.right_est+arc.right_rate*s,
                                                 arc.fore_aft_deg,arc.body_deg-arc.speed_dps*s);
      };
      const auto q=lateral_energy::charge(table,choice,i0,goal);
      const auto work=lateral_energy::work(lateral_energy::coefficients(arc.side),arc.speed_dps*lateral_energy::kRadPerDeg,q);
      std::cout<<' '<<int(old_pivot.reason)<<' '<<receiving_arc_aid::boundWidth(baseline,old_candidate,envelope,reserved)
               <<' '<<result.extension_pivot_only<<' '<<clearance(0)<<' '<<clearance(.001f*baseline)
               <<' '<<clearance(result.horizon_s)<<' '<<result.horizon_s<<' '<<work;
#endif
      std::cout<<'\n';
    } else return 3;
  }
}
