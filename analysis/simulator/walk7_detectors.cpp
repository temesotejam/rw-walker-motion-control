// Replay only the production observation policies. This process never
// constructs a motor transport and never uses samples beyond each decision.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#define WALKING_RETURN_HISTORY 1
#include "walking/support/return_observation_policy.h"
#include "walking/support/walking_return_energy.h"
#include "walking/support/return_crossing_work.h"
#include "walking/recovery/settled_walking_restart.h"
#include <iomanip>
#include <iostream>
#include <string>

static float number() {
  std::string token;
  std::cin >> token;
  return std::stof(token);
}

int main() {
  return_observation_policy::PostAidObserver observer;
  return_observation_policy::Input in;
  periodic_foot::Packet packet;
  lateral_energy::Geometry geometry;
  lateral_energy::CurrentTable current_model;
  walking_return_energy::Input inverse;
  settled_walking_restart::Detector restart;
  bool first_stage=false, event_allocation=false;
  unsigned remaining=0;
  return_crossing_work::MainPassages passages;
  wheel_cycle_guard::Controller wheel;
  char tag;
  std::cout << std::setprecision(9);
  while (std::cin >> tag) {
    if (tag == 'E') {
      std::cin >> event_allocation;
    } else if (tag == 'A' || tag == 'F') {
      uint32_t end;
      in = {}; packet = {}; observer = {};
      passages={}; wheel={};
      in.side = 1;
      in.run_id = in.foot.run_id = 1;
      std::cin >> in.peak_ms >> end >> packet.epoch_us;
      const float first_work = number();
      first_stage = tag == 'F';
      unsigned elapsed=0;
      float onset_rate, i0=NAN, goal=NAN, nominal_work=NAN, decel=NAN;
      if (first_stage) onset_rate=number();
      else {
        std::cin >> elapsed;
        i0=number(); goal=number(); onset_rate=number(); nominal_work=number(); decel=number();
      }
      in.target_j = number();
      in.current_rise_tau_s = number();
      inverse = {}; inverse.side = -1;
      inverse.target_deg=number(); inverse.previous_peak_deg=number(); inverse.same_side_peak_deg=number();
      std::cin >> remaining;
      if(!current_model.prepare(in.current_rise_tau_s)) return 1;
      // Walk6 accounts for the second pulse's actually observed starting
      // current. Reconstruct its modeled work for the measured elapsed time.
      if (first_stage) observer.arm(in.peak_ms,end,onset_rate,first_work);
      else {
        const float q=lateral_energy::charge(current_model, elapsed, i0, goal);
        const float post_work=std::isfinite(nominal_work) ? nominal_work :
            lateral_energy::work(lateral_energy::coefficients(-1),onset_rate*lateral_energy::kRadPerDeg,q);
        observer.armRecheck(in.peak_ms, end, first_work+post_work, decel);
      }
    } else if (tag == 'H') {
      uint32_t zero,peak,elapsed;
      std::cin >> zero >> peak;
      const float magnitude=number(), speed=number(), target=number();
      std::cin >> elapsed;
      passages.observe(zero,peak,magnitude,speed,target,elapsed,number(),true);
    } else if (tag == 'W' || tag == 'P') {
      int side; uint32_t time;
      std::cin >> side >> time;
      const float value=number();
      if (tag=='W') wheel.limit(side,value,true,time,1);
      else {
        bool matched;
        std::cin >> matched;
        wheel.peak(side,value,time,matched);
      }
    } else if (tag == 'I') {
      uint64_t now;
      std::cin >> now >> in.now >> in.sample_us >> in.sample_age_us;
      in.angle_deg = number(); in.rate_dps = number();
      in.periodic = nullptr;
      if (first_stage) observer.observe(in,now,NAN,true,event_allocation);
      else observer.observeRecheck(in,now,NAN,event_allocation);
    } else if (tag == 'S') {
      uint64_t now;
      std::cin >> now >> in.now >> in.sample_us >> in.sample_age_us >> in.active
               >> packet.count >> packet.scheduled_us >> packet.published_us;
      for (auto* foot : {&packet.left, &packet.right}) {
        std::cin >> foot->sequence >> foot->image_us;
        foot->measured = number(); foot->estimated = number();
      }
      unsigned flags;
      auto& f = in.foot;
      std::cin >> flags >> f.frame_ms >> f.ready_ms;
      f.timestamp_valid = flags & 1; f.frame_valid = flags & 2; f.zero_ready = flags & 4;
      f.left_detected = flags & 8; f.right_detected = flags & 16;
      f.left_range_ok = flags & 32; f.right_range_ok = flags & 64;
      f.left_deg = number(); f.right_deg = number();
      in.angle_deg = number(); in.rate_dps = number();
      in.fore_aft_deg = number(); in.periodic_acceleration_dps2 = number();
      const float current = number();
      const float goal = number();
      in.current_goal_ma=goal;
      unsigned guarded_cap;
      std::cin >> in.current_age_us >> guarded_cap;
      const float rpm=number();
      in.periodic = &packet;
      const auto result = first_stage ? observer.observe(in,now,current,true,event_allocation)
                                      : observer.observeRecheck(in,now,current,event_allocation);
      if (result.selected && guarded_cap > 0) {
        inverse.body_angle_deg = in.angle_deg; inverse.rate_dps = -in.rate_dps;
        inverse.initial_current_ma = current; inverse.goal_current_ma = goal;
        inverse.return_work_j=result.request_j;
        // The snapshot stores the wheel-limited complete budget. The runner
        // applies the shorter stopping horizon before the wheel limiter.
        const auto guard=[&](unsigned width){
          auto copy=wheel;
          return copy.limit(-1,rpm,true,in.now,width);
        };
        unsigned raw=remaining;
        while (raw && guard(raw)>guarded_cap) --raw;
        const bool matched=std::isfinite(rpm) && guard(raw)==guarded_cap;
        inverse.cap_ms=std::min<unsigned>(matched?raw:guarded_cap,result.loss_cap_ms);
        const auto selected=half_cycle_energy_budget::selectPostAid(geometry,current_model,inverse,result.gross_work_limit_j);
        const uint16_t before=std::min<uint16_t>(selected.width_ms,matched?guard(inverse.cap_ms):guarded_cap);
        const auto allocation=event_allocation ? return_crossing_work::planPostAidPassage(passages,in.peak_ms,
            in.now,remaining,current_model,inverse,in.target_j,result.gross_work_limit_j,
            result.deceleration_dps2,before) : return_crossing_work::Audit{};
        const uint16_t width=allocation.final_ms?allocation.final_ms:before;
        const auto final=half_cycle_energy_budget::selectPostAid(geometry,current_model,inverse,result.gross_work_limit_j,width);
        const float extra=final.valid ? final.predicted_work_j-half_cycle_energy_budget::postAidCoastWork(
            inverse.rate_dps,current,width) : 0;
        std::cout << "R " << in.peak_ms << ' ' << in.now << ' ' << result.request_j << ' '
                  << result.loss_cap_ms << ' ' << in.angle_deg << ' ' << -in.rate_dps
                  << ' ' << before << ' ' << width << ' ' << extra << ' ' << allocation.passage_reserve_ms << ' '
                  << allocation.zero_ms << ' ' << allocation.peak_ms << ' ' << matched << '\n';
      }
    } else if (tag == 'D') {
      settled_walking_restart::Input s;
      std::cin >> s.now_ms >> s.sample_us >> s.run_active >> s.guarded >> s.feet_fresh
               >> s.pulse_active >> s.had_peak >> s.crossed_zero;
      s.angle_deg = number(); s.rate_dps = number();
      s.fore_aft_deg = number(); s.fore_aft_rate_dps = number();
      s.left_deg = number(); s.right_deg = number();
      bool hardware;
      std::cin >> hardware;
      const float measured = number(), goal = number(), tau = number();
      const int direction = restart.observe(s);
      if (direction && hardware) {
        if (!current_model.prepare(tau)) return 4;
        const float reference = restart.referenceSpeed(direction);
        const auto result = settled_walking_restart::plan(current_model, direction, reference,
            s.rate_dps, direction*measured, goal, restart.remaining());
        if (!result.width) continue;
        std::cout << "D " << s.now_ms << ' ' << direction << ' ' << s.angle_deg << ' '
                  << s.rate_dps << ' ' << s.left_deg << ' ' << s.right_deg << ' ' << reference
                  << ' ' << result.width << ' ' << result.demand_j << ' ' << result.predicted_j << '\n';
        return 0;  // Changes the commanded input. Later motion is counterfactual.
      }
    } else return 2;
    if (!std::cin) return 3;
  }
}
