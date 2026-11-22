#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#include "return_observation_policy.h"
#include "return_crossing_work.h"
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

// Two archived observations before the receiving sole was lower in 195727101.
static void closingGap(Fixture& f, bool second) {
  const float left=second?2.54249f:2.77112f, right=second?-.16792f:-.19631f;
  f.frame(second?159:158,second?3977:3951);
  f.p.scheduled_us=f.p.epoch_us+(second?3975000:3950000);
  f.in.peak_ms=3739; f.in.peak_deg=6.83069f;
  f.in.angle_deg=second?3.329228f:3.649064f;
  f.in.rate_dps=second?-10.25376f:-12.96226f;
  f.in.periodic_acceleration_dps2=second?108.4528f:149.6926f;
  f.in.fore_aft_deg=second?-.5500403f:-.6988953f;
  f.p.left.measured=f.in.foot.left_deg=left;
  f.p.right.measured=f.in.foot.right_deg=right;
  assert(support_transfer_plan::supportGap(left,right,f.in.fore_aft_deg,f.in.angle_deg)>0);
}

static void testPredictedReturn() {
  Fixture forecast;
  forecast.in.predict_left_return=true; forecast.in.current_rise_tau_s=.0316522f;
  closingGap(forecast,false); assert(!forecast.check().selected);
  closingGap(forecast,true); const auto choice=forecast.check();
  assert(choice.selected && choice.evidence==Evidence::PredictedArcShortfall);
  assert(choice.predicted_transfer_ms>25 && choice.predicted_transfer_ms<31.6522f);
  assert(choice.terminal_deg<-1.8f && choice.terminal_deg>-2.f);
  assert(choice.request_j>0 && choice.request_j<=.25f*forecast.in.target_j);
  assert(!forecast.check().selected);
  const auto observe_first=[](Fixture& f) {
    f.in.predict_left_return=true; f.in.current_rise_tau_s=.0316522f;
    closingGap(f,false); assert(!f.check().selected);
  };
  Fixture baseline; closingGap(baseline,false); assert(!baseline.check().selected);
  closingGap(baseline,true); assert(!baseline.check().selected);
  Fixture received; received.in.predict_left_return=true; received.in.current_rise_tau_s=.0316522f;
  received.seed(); received.frame(13,326);
  const auto existing=received.check(); assert(existing.selected && existing.evidence==Evidence::MeasuredArcShortfall);
  assert(std::isnan(existing.predicted_transfer_ms) && std::isnan(existing.terminal_deg));
  Fixture lead; observe_first(lead); closingGap(lead,true);
  lead.in.current_rise_tau_s=.010f; assert(!lead.check().selected);
  Fixture reversed; observe_first(reversed); reversed.in.rate_dps=.5f;
  assert(!reversed.check().selected);
  closingGap(reversed,false); reversed.frame(159,3976); assert(!reversed.check().selected);
  closingGap(reversed,true); reversed.frame(160,4001); assert(!reversed.check().selected);
  Fixture coast; observe_first(coast); closingGap(coast,true);
  coast.in.periodic_acceleration_dps2=5; assert(!coast.check().selected);
  Fixture opening; observe_first(opening); closingGap(opening,true);
  opening.in.angle_deg=3.9f; assert(!opening.check().selected);
  Fixture shared; observe_first(shared); const auto image=shared.in.foot.image_us; closingGap(shared,true);
  shared.in.foot.image_us=shared.p.left.image_us=shared.p.right.image_us=image; assert(!shared.check().selected);
  Fixture consumed; observe_first(consumed); consumed.observer.consume(1,3739);
  closingGap(consumed,true); assert(!consumed.check().selected);
}

// Production inverse and wheel state at the last aid in run 135284989.
// None of these observations comes from after the decision.
struct ApproachFixture {
  receiving_arc_aid::Input arc;
  return_crossing_work::Reference source;
  lateral_energy::Geometry geometry;
  lateral_energy::CurrentTable current;
  walking_return_energy::Input in;
  walking_return_energy::Decision selected;
  wheel_cycle_guard::Controller wheel,chosen;
  uint16_t width=0,reserve=20;
  float fore_rate_dps=5.238708f;
  ApproachFixture() {
    assert(current.prepare(.031652216f));
    in.side=-1;in.target_deg=5;in.body_angle_deg=3.97138f;in.rate_dps=18.35050f;
    in.previous_peak_deg=7.26871f;in.same_side_peak_deg=3.62515f;
    in.initial_current_ma=-.724452f;in.goal_current_ma=341.34189f;in.return_work_j=.000015804f;
    selected=half_cycle_energy_budget::decideReturnAid(geometry,current,in);
    assert(selected.valid && selected.width_ms==17);
    wheel.limit(1,-602.23f,true,4447,30);
    chosen=wheel;width=chosen.limit(-1,-1.79f,true,4869,selected.width_ms);
    assert(width==15);
    arc.side=-1;arc.observed_shape=true;arc.body_deg=in.body_angle_deg;arc.speed_dps=in.rate_dps;
    arc.decel_dps2=72.57216f;arc.fore_aft_deg=-.5252169f;
    arc.left_deg=2.85686f;arc.right_deg=-.18851f;arc.left_est=2.838892f;arc.right_est=-.1696107f;
    arc.left_rate=-.7188846f;arc.right_rate=.7560334f;arc.base_width_ms=width;
    arc.base_j=in.return_work_j;arc.target_j=selected.target_j;
    source.observe(3168,3321,2.86886f,31.3949f,selected.target_j,true);
    assert(receiving_arc_aid::evaluate(arc).reason==receiving_arc_aid::Reason::NotReceiving);
  }
  return_crossing_work::Audit choose() {
    return return_crossing_work::applyApproach(arc,source,4670,4869,80,reserve,-1.79f,
        wheel,chosen,geometry,current,in,selected,width,fore_rate_dps);
  }
};

static void testApproachingAidWork() {
  ApproachFixture f;
  const auto audit=f.choose();
  assert(audit.transfer_ms>37 && audit.transfer_ms<41 && audit.terminal_deg<-1.89f);
  assert(audit.base_ms==15 && audit.final_ms==f.width && f.width>audit.base_ms);
  assert(audit.zero_ms==3168 && audit.arc_request_j>f.arc.base_j && audit.request_j>audit.arc_request_j);
  assert(f.width==34 && std::fabs(audit.request_j-.000080794f)<1.e-8f);
  const auto delivered=half_cycle_energy_budget::evaluateReturnAidWidth(f.geometry,f.current,f.in,f.width);
  assert(delivered.valid && delivered.charge_supported && delivered.predicted_work_j>f.arc.base_j);

  ApproachFixture no_source;no_source.source={};
  const auto geometric=no_source.choose();
  assert(!geometric.zero_ms && geometric.request_j==geometric.arc_request_j);
  assert(no_source.width>15 && no_source.width<f.width);
  ApproachFixture room;room.reserve=62;assert(room.choose().final_ms==18);
  ApproachFixture no_room;no_room.reserve=65;assert(!no_room.choose().final_ms && no_room.width==15);
  ApproachFixture delayed;delayed.arc.left_est=1.9f;assert(!delayed.choose().final_ms && delayed.width==15);
  ApproachFixture opening;opening.arc.left_rate=-60;assert(!opening.choose().final_ms && opening.width==15);
  ApproachFixture pitching;pitching.fore_rate_dps=-15;assert(!pitching.choose().final_ms && pitching.width==15);
  ApproachFixture unknown_pitch;unknown_pitch.fore_rate_dps=NAN;assert(!unknown_pitch.choose().final_ms);
  ApproachFixture disabled;disabled.arc.observed_shape=false;assert(!disabled.choose().final_ms);
  ApproachFixture stopped;stopped.arc.decel_dps2=900;assert(!stopped.choose().final_ms);
  ApproachFixture already_lower;already_lower.arc.body_deg=2.5f;assert(!already_lower.choose().final_ms);
  ApproachFixture invalid;invalid.arc.left_est=NAN;assert(!invalid.choose().final_ms);
  ApproachFixture strong;strong.in.same_side_peak_deg=5.1f;assert(!strong.choose().final_ms);
}

static void testPassageWork() {
  ApproachFixture f;
  using namespace return_crossing_work;
  MainPassages passages;
  passages.observe(100,200,1.f,18.f,.00034f,40,85.e-6f,true); // Did not reach the receiving arc.
  passages.observe(100,200,3.f,18.f,.00034f,40,85.e-6f,false); // Unfinished or unqualified pulse.
  assert(!passages.recent[0].zero_ms);
  passages.observe(100,200,3.f,18.f,.00034f,40,85.e-6f,true);
  passages.observe(50,180,4.f,23.f,.00034f,50,110.e-6f,true); // Out of order.
  assert(passages.recent[0].zero_ms==100 && !passages.recent[1].zero_ms);
  f.arc.body_deg=f.in.body_angle_deg=4.76397f; f.arc.speed_dps=f.in.rate_dps=20.1508f;
  f.arc.decel_dps2=55.27f;f.arc.fore_aft_deg=-.45944f;
  f.arc.left_deg=3.11881f;f.arc.right_deg=f.arc.right_est=-.15315f;f.arc.left_est=3.06555f;
  f.arc.left_rate=-2.1305f;f.arc.right_rate=0;
  f.in.previous_peak_deg=7.44456f;f.in.same_side_peak_deg=3.65731f;
  f.in.return_work_j=7.947e-6f;f.in.initial_current_ma=-1.0831f;f.in.goal_current_ma=340.8839f;
  f.selected=half_cycle_energy_budget::decideReturnAid(f.geometry,f.current,f.in);
  f.wheel.reset(); f.wheel.limit(1,-596.2f,true,2631,26);
  f.chosen=f.wheel;f.width=f.chosen.limit(-1,-18.81f,true,3022,f.selected.width_ms);
  assert(f.width==11);
  const auto plan=[](ApproachFixture& data,const MainPassages& refs,unsigned now,uint16_t envelope) {
    return applyPassage(data.arc,refs,now-167,now,envelope,-18.81f,data.wheel,data.chosen,
        data.geometry,data.current,data.in,data.selected,data.width);
  };
  const float old_target=lateral_energy::potential(f.geometry,4.5f);
  passages={};passages.observe(1351,1512,3.3728f,19.13394f,old_target,40,85.456e-6f,true);
  auto copy=f;
  const auto joint=plan(copy,passages,3022,80);
  assert(joint.final_ms==21 && joint.passage_main_ms==40 && joint.passage_reserve_ms==54);
  assert(joint.zero_ms==1351 && joint.passage_main_j>=85.456e-6f+f.selected.target_j-old_target);
  assert(joint.passage_zero_dps>16 && joint.passage_zero_dps<joint.speed_dps);
  assert(joint.final_ms+joint.passage_reserve_ms<=80);
  // Without enough time for the second event, keep all original output state.
  copy=f;assert(!plan(copy,passages,3022,70).final_ms && copy.width==11);
  assert(copy.selected.width_ms==f.selected.width_ms && copy.chosen.legacyWidth()==f.chosen.legacyWidth());
  copy=f;copy.arc.left_est=copy.arc.right_est;assert(!plan(copy,passages,3022,80).final_ms);
  copy=f;copy.arc.decel_dps2=10;assert(!plan(copy,passages,3022,80).final_ms);
  MainPassages stale;stale.observe(600,1512,3.f,19.f,old_target,40,85.e-6f,true);
  assert(!plan(f,stale,5600,80).final_ms);
  MainPassages future;future.observe(2700,2950,3.f,19.f,old_target,40,85.e-6f,true);
  assert(!plan(f,future,3022,80).final_ms);
  passages.recent[0].target_j=f.selected.target_j*1.2f;
  assert(!plan(f,passages,3022,80).final_ms);
  passages.recent[0].target_j=old_target;passages.recent[0].main_j=1;
  assert(!plan(f,passages,3022,80).final_ms); // Required charge above the measured range.
}

int main() {
  testPredictedReturn();
  testApproachingAidWork();
  testPassageWork();
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
