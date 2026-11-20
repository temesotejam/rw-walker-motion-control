#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#include "return_observation_policy.h"
#include <cassert>
#include <cmath>

using namespace return_observation_policy;

struct Fixture {
  Observer observer;
  Input in;
  periodic_foot::Packet p;

  Fixture() {
    in.active=true; in.run_id=in.foot.run_id=1; in.side=1; in.peak_ms=140; in.peak_deg=6;
    in.rate_dps=-15; in.angle_deg=1.65f; in.periodic_acceleration_dps2=75; in.fore_aft_deg=-.32f;
    in.target_j=.000343854f; in.periodic=&p;
    auto& f=in.foot;
    f.timestamp_valid=f.frame_valid=f.zero_ready=f.left_detected=f.right_detected=f.left_range_ok=f.right_range_ok=true;
    p.epoch_us=1000000; p.left.measured=f.left_deg=2.2f; p.right.measured=f.right_deg=-.22f;
  }
  void frame(unsigned count, unsigned time_ms) {
    in.now=time_ms; in.sample_us=1000000+1000*time_ms; p.count=count;
    p.scheduled_us=in.sample_us-1000; p.published_us=in.sample_us-500;
    p.left.sequence=p.right.sequence=in.foot.sequence=count;
    p.left.image_us=p.right.image_us=in.foot.image_us=in.sample_us-35000;
    in.foot.frame_ms=time_ms-35; in.foot.ready_ms=time_ms-10;
  }
  Choice check() {return observer.observe(in);}
  void seed() {frame(12,301); assert(!check().selected);}
};

int main() {
  Fixture ok; ok.seed(); ok.frame(13,326);
  const auto hit=ok.check();
  assert(hit.selected && hit.evidence==Evidence::MeasuredArcShortfall);
  assert(std::fabs(hit.stopping_distance_deg-1.5f)<.0001f);
  assert(hit.request_j>0 && hit.request_j<=ok.in.target_j*.25f);
  assert(!ok.check().selected);
  ok.observer.consume(1,140); ok.frame(14,351); assert(!ok.check().selected);
  ok.frame(15,376); assert(!ok.check().selected);

  Fixture skipped; skipped.seed(); skipped.frame(13,351); assert(!skipped.check().selected);
  skipped.frame(14,376); assert(skipped.check().selected);
  Fixture packet; packet.seed(); packet.frame(14,326); assert(!packet.check().selected);
  Fixture shared; shared.seed(); shared.frame(13,326);
  shared.p.left.image_us=shared.p.right.image_us=shared.in.foot.image_us-=25000;
  assert(!shared.check().selected); shared.frame(14,351); assert(!shared.check().selected);

  Fixture missed; missed.seed(); missed.in.foot.left_detected=false;
  assert(!missed.check().selected); missed.in.foot.left_detected=true;
  missed.frame(13,326); assert(!missed.check().selected);
  Fixture range; range.seed(); range.in.foot.right_range_ok=false; assert(!range.check().selected);
  range.in.foot.right_range_ok=true; range.frame(13,326); assert(!range.check().selected);
  Fixture reversed; reversed.seed(); reversed.in.rate_dps=2; assert(!reversed.check().selected);
  reversed.in.rate_dps=-15; reversed.frame(13,326); assert(!reversed.check().selected);
  Fixture new_raw; new_raw.seed(); new_raw.frame(13,326); ++new_raw.in.foot.sequence;
  assert(!new_raw.check().selected);
  Fixture coast; coast.seed(); coast.frame(13,326); coast.in.rate_dps=-22; assert(!coast.check().selected);
  Fixture imu; imu.seed(); imu.frame(13,326); imu.in.sample_age_us=10001; assert(!imu.check().selected);
  Fixture expired; expired.seed(); expired.frame(13,326);
  expired.p.left.image_us=expired.p.right.image_us=expired.in.foot.image_us=expired.in.sample_us-75001;
  assert(!expired.check().selected);
  Fixture stopped; stopped.seed(); stopped.frame(13,326); stopped.in.rate_dps=0; assert(!stopped.check().selected);
  Fixture other_side; other_side.seed(); other_side.frame(13,326); other_side.in.side=-1; assert(!other_side.check().selected);
  Fixture startup; startup.seed(); startup.frame(13,326); startup.in.peak_ms=310; assert(!startup.check().selected);
}
