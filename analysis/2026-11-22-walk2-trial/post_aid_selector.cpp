// Host adapter for the production post-aid observer and current/work inverse.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#define WALKING_RETURN_HISTORY 1
#include "return_observation_policy.h"
#include "half_cycle_energy_budget.h"
#include "wheel_cycle_guard.h"
#include <iomanip>
#include <iostream>
#include <string>

static float number() {
  std::string token;
  std::cin >> token;
  return std::stof(token);
}

int main() {
  using namespace return_observation_policy;
  Input in;
  periodic_foot::Packet packet;
  PostAidObserver observer;
  wheel_cycle_guard::Controller wheel;
  walking_return_energy::Input inverse;
  lateral_energy::Geometry geometry;
  lateral_energy::CurrentTable current;
  unsigned remaining=0;
  char kind;
  std::cout << std::setprecision(10);
  while (std::cin >> kind) {
    if (kind=='A') {
      unsigned end;
      in={}; packet={}; wheel.reset(); inverse={};
      in.side=1; in.run_id=in.foot.run_id=1;
      std::cin >> in.peak_ms >> end >> packet.epoch_us >> remaining;
      const float onset=number(), work=number(), q=number();
      in.target_j=number(); in.current_rise_tau_s=number();
      inverse.side=-1; inverse.target_deg=number();
      inverse.previous_peak_deg=number(); inverse.same_side_peak_deg=number();
      if (!current.prepare(in.current_rise_tau_s)) return 2;
      if (q>=0 && q<=lateral_energy::coefficients(-1).charge_max_mAs) {
        observer.arm(in.peak_ms,end,onset,work);
        std::cout << "A " << in.peak_ms << '\n';
      } else observer={};
    } else if (kind=='H') {
      int side,matched; unsigned t;
      std::cin >> side >> t >> matched;
      wheel.peak(side,number(),t,matched);
    } else if (kind=='M') {
      int side; unsigned t;
      std::cin >> side >> t;
      wheel.limit(side,number(),true,t,1);
    } else if (kind=='I') {
      uint64_t at;
      std::cin >> in.now >> in.sample_us >> in.sample_age_us >> at;
      in.angle_deg=number(); in.rate_dps=number(); in.periodic=nullptr;
      observer.observe(in,at);
    } else if (kind=='S') {
      uint64_t at;
      unsigned flags,recorded_cap;
      std::cin >> at >> in.now >> in.sample_us >> in.sample_age_us >> in.active
               >> packet.count >> packet.scheduled_us >> packet.published_us;
      for (auto foot:{&packet.left,&packet.right}) {
        std::cin >> foot->sequence >> foot->image_us;
        foot->measured=number(); foot->estimated=number();
      }
      auto& f=in.foot;
      std::cin >> flags >> f.frame_ms >> f.ready_ms;
      f.timestamp_valid=flags&1; f.frame_valid=flags&2; f.zero_ready=flags&4;
      f.left_detected=flags&8; f.right_detected=flags&16;
      f.left_range_ok=flags&32; f.right_range_ok=flags&64;
      f.left_deg=number(); f.right_deg=number();
      in.angle_deg=number(); in.rate_dps=number(); in.fore_aft_deg=number();
      in.periodic_acceleration_dps2=number();
      const float i0=number(), goal=number(), rpm=number();
      std::cin >> recorded_cap;
      in.periodic=&packet;
      const auto choice=observer.observe(in,at,i0);
      if (choice.selected) {
        inverse.body_angle_deg=in.angle_deg; inverse.rate_dps=-in.rate_dps;
        inverse.initial_current_ma=i0; inverse.goal_current_ma=goal; inverse.return_work_j=choice.request_j;
        // The archive stores the full available time AFTER the wheel guard.
        // Replay every possible pre-guard cap consistent with that observed
        // bound and claim a width only when all possibilities give the same result.
        int min_width=999, max_width=-1;
        float min_work=INFINITY, max_work=0;
        for (unsigned cap=1;cap<=remaining;++cap) {
          auto envelope_guard=wheel;
          if (!std::isfinite(rpm) || envelope_guard.limit(-1,rpm,true,in.now,cap)!=recorded_cap) continue;
          inverse.cap_ms=std::min<unsigned>(cap,choice.loss_cap_ms);
          const auto selected=half_cycle_energy_budget::selectPostAid(geometry,current,inverse,choice.gross_work_limit_j);
          auto guard=wheel;
          auto width=guard.limit(-1,rpm,true,in.now,selected.width_ms);
          const auto delivered=half_cycle_energy_budget::selectPostAid(geometry,current,inverse,choice.gross_work_limit_j,width);
          if (!selected.valid || !delivered.valid || !delivered.charge_supported || !(delivered.predicted_work_j>0)) width=0;
          min_width=std::min<int>(min_width,width); max_width=std::max<int>(max_width,width);
          const float work=width?delivered.predicted_work_j-half_cycle_energy_budget::postAidCoastWork(inverse.rate_dps,i0,width):0;
          min_work=std::min(min_work,work); max_work=std::max(max_work,work);
        }
        std::cout << in.peak_ms << ' ' << in.now << ' ' << choice.request_j << ' '
          << choice.loss_cap_ms << ' ' << min_width << ' ' << max_width << ' '
          << (max_width<0?0:min_work) << ' ' << max_work << '\n';
      }
    } else return 3;
    if (!std::cin) return 4;
  }
}
