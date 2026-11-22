// Replays one proposed change to an already accepted ordinary left return aid.
// Other aids clear the reference exactly as an accepted left aid does on-board.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#include "periodic_foot_state.h"
#include "receiving_arc_aid.h"
#include "return_crossing_work.h"
#include <iomanip>
#include <iostream>
#include <string>

static float number() {
  std::string token;
  std::cin >> token;
  return std::stof(token);
}

int main() {
  uint64_t epoch;
  std::cin >> epoch;
  periodic_foot::Estimator feet;
  feet.start(epoch);
  receiving_arc_aid::MainReserve reserve;
  return_crossing_work::Reference reference;
  wheel_cycle_guard::Controller wheel;
  lateral_energy::Geometry geometry;
  std::cout << std::setprecision(9);
  char kind;
  while (std::cin >> kind) {
    if (kind=='F') {
      unsigned seq, valid, lv, la, rv, ra;
      uint64_t image, received;
      std::cin >> seq >> image >> received >> valid >> lv >> la >> rv >> ra;
      float left=number(), right=number();
      feet.observe(seq,image,received,valid,lv,la,left,rv,ra,right);
    } else if (kind=='S') {
      uint64_t published;
      unsigned seq_l, seq_r;
      std::cin >> published >> seq_l >> seq_r;
      if (!feet.publish(published)) return 2;
      // Every restored publication must identify the same source images, even
      // if the recorded trial did not export an arc-aid audit.
      if (feet.packet.left.sequence!=seq_l || feet.packet.right.sequence!=seq_r) {
        std::cerr << "packet sequence mismatch at " << published-epoch << '\n';
        return 3;
      }
    } else if (kind=='M') {
      unsigned t, width;
      int side;
      std::cin >> t >> side >> width;
      const float rpm=number();
      wheel.limit(side,rpm,true,t,width);
    } else if (kind=='R') {
      int side, assisted;
      unsigned elapsed;
      std::cin >> side >> assisted >> elapsed;
      reserve.observe(side,assisted,elapsed);
    } else if (kind=='P') {
      unsigned t, zero;
      int side, powered, matches, complete;
      std::cin >> t >> side >> powered >> matches >> zero >> complete;
      const float angle=number(), speed=number(), target=number();
      wheel.peak(side,angle,t,powered);
      if (side==-1 && matches) reference.observe(zero,t,angle,speed,target,complete);
    } else if (kind=='C') {
      reference={};
    } else if (kind=='A') {
      uint64_t stamp, image, received;
      unsigned seq, packet, t, departure, cap, envelope;
      std::cin >> t >> departure >> stamp >> seq >> image >> received >> packet >> cap >> envelope;
      receiving_arc_aid::Input arc;
      walking_return_energy::Input in;
      arc.side=in.side=-1;
      arc.left_deg=number(); arc.right_deg=number(); arc.fore_aft_deg=number();
      const float fore_rate=number();
      arc.body_deg=in.body_angle_deg=number(); arc.speed_dps=in.rate_dps=number(); arc.decel_dps2=number();
      in.target_deg=number(); in.previous_peak_deg=number(); in.same_side_peak_deg=number();
      in.initial_current_ma=number(); in.goal_current_ma=number();
      const float tau=number(), rpm=number();
      in.return_work_j=number(); in.cap_ms=cap;
      lateral_energy::CurrentTable current;
      if (!current.prepare(tau)) return 4;
      auto selected=half_cycle_energy_budget::decideReturnAid(geometry,current,in);
      if (!selected.valid || selected.coast || selected.width_ms==0) return 5;
      auto guard=wheel;
      uint16_t width=guard.limit(-1,rpm,true,t,selected.width_ms);
      const auto start_width=width, start_inverse=selected.width_ms;
      const auto& p=feet.packet;
      auto l=p.left,r=p.right;
      int source=0;
      if (p.count==packet && periodic_foot::matchesPair(p,stamp,seq,image)) source=1;
      else if (p.count==packet && feet.rescuePair(stamp,epoch,seq,image,received,l,r)) source=2;
      arc.left_est=l.estimated;arc.right_est=r.estimated;arc.left_rate=l.rate_dps;arc.right_rate=r.rate_dps;
      arc.observed_shape=source!=0;arc.base_width_ms=width;arc.base_j=selected.request_j;arc.target_j=selected.target_j;
      const auto present=receiving_arc_aid::evaluate(arc);
      return_crossing_work::Audit audit;
      if (present.reason==receiving_arc_aid::Reason::NotReceiving && source && width)
        audit=return_crossing_work::applyApproach(arc,reference,departure,t,envelope,reserve.forSide(-1),
            rpm,wheel,guard,geometry,current,in,selected,width,fore_rate);
      const auto after=half_cycle_energy_budget::evaluateReturnAidWidth(geometry,current,in,width);
      std::cout << t << ' ' << source << ' ' << static_cast<unsigned>(present.reason) << ' '
                << reserve.forSide(-1) << ' ' << start_inverse << ' ' << start_width << ' '
                << reference.zero_ms << ' ' << reference.speed_dps << ' ' << audit.transfer_ms << ' '
                << audit.terminal_deg << ' ' << audit.arc_request_j << ' ' << audit.request_j << ' '
                << selected.width_ms << ' ' << width << ' ' << after.predicted_work_j << '\n';
    } else return 6;
    if (!std::cin) return 7;
  }
}
