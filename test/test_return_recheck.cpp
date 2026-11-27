#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#define WALKING_RETURN_HISTORY 1
#include "walking/support/return_observation_policy.h"
#include "walking/support/walking_return_energy.h"
#include <cassert>

using namespace return_observation_policy;

// Run 33863489. Both body samples are after the second pulse ended at
// 19.724 s. The camera observations describe the recent bearing-foot pose.
struct RecordedSlot {
  uint32_t time, count, image_age_us;
  float angle, rate, acceleration, left, right, estimate_left, estimate_right;
};
constexpr RecordedSlot first{19728,789,33237,3.286819f,-15.41953f,-175.3229f,3.15718f,-.09711f,2.896556f,-.087637f};
constexpr RecordedSlot second{19753,790,37831,3.129997f,-10.20007f,209.659f,3.07882f,-.15835f,2.982444f,-.158345f};

Input load(const RecordedSlot& s,periodic_foot::Packet& p) {
  p.epoch_us=38'866'000;
  p.count=s.count;p.scheduled_us=p.epoch_us+25000ULL*s.count;
  p.published_us=p.scheduled_us+1000;
  p.left.image_us=p.right.image_us=p.scheduled_us-s.image_age_us;
  p.left.sequence=p.right.sequence=s.count;
  p.left.measured=s.left;p.right.measured=s.right;
  p.left.estimated=s.estimate_left;p.right.estimated=s.estimate_right;
  Input in;
  in.foot.run_id=in.run_id=1;in.now=s.time;in.side=1;in.peak_ms=19395;
  in.sample_us=p.epoch_us+1000ULL*s.time-900;in.sample_age_us=900;
  in.active=true;in.angle_deg=s.angle;in.rate_dps=s.rate;
  in.fore_aft_deg=.25f;in.periodic_acceleration_dps2=s.acceleration;
  in.current_rise_tau_s=.031652216f;in.periodic=&p;in.target_j=.000432975f;
  return in;
}
int main() {
  using namespace half_cycle_energy_budget;
  Ledger budget;
  assert(budget.sourcePeak(19395) && !budget.allowRecheck(19395));
  assert(budget.accepted(Kind::ReturnAid,37) && budget.complete(37));
  assert(!budget.allowRecheck(19395));
  assert(budget.accepted(Kind::PostAid,18) && !budget.allowRecheck(19395));
  assert(budget.complete(18) && budget.allowRecheck(19395) && budget.remaining()==25);
  assert(!budget.accepted(Kind::Recheck,26));

  for(unsigned variant=0;variant<10;++variant) {
    PostAidObserver observer;
    observer.armRecheck(19395,19724,66.665e-6f+16.450e-6f,211.208725f);
    periodic_foot::Packet p;
    auto in=load(first,p);
    assert(!observer.observeRecheck(in,p.epoch_us+1000ULL*in.now,NAN).selected);
    in=load(second,p);
    float measured_current=107.f;
    switch(variant) {
      case 1: in.rate_dps=-20;break;                      // current momentum increasing
      case 2: p.right.image_us-=1000;break;                // images from different frames
      case 3: in.sample_age_us=11000;break;
      case 4: in.periodic_acceleration_dps2=1;break;        // zero reachable by coasting
      case 5: in.current_rise_tau_s=.060f;break;            // new torque too late
      case 6: measured_current=NAN;break;                  // no acknowledged zero/current
      case 7: in.angle_deg=3.45f;break;                    // already moving outward
      case 8: in.foot.run_id=2;break;
      case 9: in.active=false;break;
    }
    const auto d=observer.observeRecheck(in,p.epoch_us+1000ULL*in.now,measured_current);
    if (variant) {assert(!d.selected);continue;}
    assert(d.selected && d.evidence==Evidence::RecheckedShortfall && d.terminal_deg<0);
    assert(d.request_j>0 && d.request_j<=d.gross_work_limit_j && d.loss_cap_ms>19);
    walking_return_energy::Input model;
    model.side=-1;model.rate_dps=-in.rate_dps;model.body_angle_deg=in.angle_deg;
    model.initial_current_ma=d.aligned_current_ma;model.goal_current_ma=329.4263f;
    model.previous_peak_deg=8.8307f;model.same_side_peak_deg=2.02386f;
    model.cap_ms=19;model.target_deg=5.5f;model.return_work_j=d.request_j;
    lateral_energy::CurrentTable current;assert(current.prepare(in.current_rise_tau_s));
    const auto work=selectPostAid(lateral_energy::Geometry{},current,model,d.gross_work_limit_j);
    assert(work.valid && work.charge_supported && work.width_ms>0 && work.width_ms<=model.cap_ms);
    assert(work.predicted_work_j>postAidCoastWork(model.rate_dps,model.initial_current_ma,work.width_ms));
    assert(budget.accepted(Kind::Recheck,work.width_ms) && budget.complete(work.width_ms));
    assert(!budget.allowRecheck(19395) && budget.remaining()==25-work.width_ms && budget.allowMain(19395));
    assert(!observer.observeRecheck(in,p.epoch_us+1000ULL*in.now,measured_current).selected);
  }
}
