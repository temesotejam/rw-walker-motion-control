// Re-evaluate an archived, measured left return with the firmware's selector.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#define WALKING_RETURN_HISTORY 1
#include "return_crossing_work.h"
#include <iomanip>
#include <iostream>

int main() {
  walking_return_energy::Input in;
  unsigned now, last, envelope, reserve;
  float tau, rpm, previous_rpm, previous_decel, decel;
  while (std::cin >> now >> in.body_angle_deg >> in.rate_dps >> in.target_deg
           >> in.previous_peak_deg >> in.same_side_peak_deg >> in.initial_current_ma
           >> in.goal_current_ma >> tau >> in.return_work_j >> envelope >> reserve
           >> previous_decel >> decel >> last >> previous_rpm >> rpm) {
    in.cap_ms=envelope;
    in.side=-1;
    lateral_energy::CurrentTable current;
    if (!current.prepare(tau)) return 1;
    // Each archived case has an immediately preceding accepted right main
    // and a logged inactive four-peak trim. The legacy RPM limiter still runs.
    wheel_cycle_guard::Controller wheel;
    wheel.limit(1,previous_rpm,true,last,1);
    const lateral_energy::Geometry geometry;
    const auto choice=return_crossing_work::prioritizeMeasuredArc(
        geometry,current,in,previous_decel,decel,reserve,rpm,now,wheel);
    in.cap_ms=choice.cap_ms ? choice.cap_ms : envelope-reserve;
    const auto selected=half_cycle_energy_budget::decideReturnAid(geometry,current,in);
    if (!selected.valid) return 2;
    const auto width=wheel.limit(-1,rpm,true,now,selected.width_ms);
    const auto final=half_cycle_energy_budget::evaluateReturnAidWidth(geometry,current,in,width);
    if (!final.valid) return 3;
    std::cout << std::setprecision(9) << choice.cap_ms << ' ' << width << ' '
              << final.predicted_work_j << '\n';
  }
  return std::cin.eof()?0:4;
}
