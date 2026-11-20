#include "support_transfer_plan.h"
#include <cassert>
#include <cstdio>
#include <random>

namespace st = support_transfer_plan;

static st::Input approach() {
  st::Input in;
  in.side=1; in.angle_deg=-1; in.rate_dps=15; in.accel_dps2=-50;
  in.left_deg=in.right_deg=in.fore_aft_deg=0;
  in.left_rate_dps=in.right_rate_dps=0;
  in.initial_current_ma=0; in.goal_current_ma=330; in.cap_ms=80;
  return in;
}

static float kinetic(float speed_dps, const st::detail::Pose& p, int side) {
  const auto foot=side>0 ? p.right : p.left;
  const float x=p.cg_x_mm-foot.x_mm, z=p.cg_z_mm-foot.z_mm;
  return .5f*st::kMassKg*(x*x+z*z)*1e-6f*st::kRad*st::kRad*speed_dps*speed_dps;
}

static void receivingPhase(const lateral_energy::CurrentTable& current,int side) {
  auto in=approach();
  in.side=side; in.angle_deg=-side*.2f; in.rate_dps=side*15.f; in.accel_dps2=0;
  st::Progress p;
  p.observe(1000,1,in,true);
  assert(p.anchor==st::Anchor::None);
  in.angle_deg=side;
  p.observe(1000,2,in,true);
  assert(p.anchor==st::Anchor::Bracketed);
  const float target=p.terminal_deg;
  // Independent circular-arc tangent at 5 mm, radius 150 mm, level foot.
  assert(std::fabs(target-side*std::asin(5.f/150.f)/st::kRad)<.0001f);
  const auto coast=st::select(current,in,p);
  assert(coast.phase==st::Phase::Receiving && coast.reason==st::Reason::Coast);
  assert(std::isnan(coast.transfer_ms) && std::isnan(coast.transfer_deg));
  const auto start=st::detail::pose(0,0,0,in.angle_deg);
  const auto end=st::detail::pose(0,0,0,target);
  const float expected=kinetic(15,start,side)-kinetic(5,end,side);
  assert(std::fabs(coast.coast_margin_j-expected)<2e-9f); // No second collision or gravity loss.
  in.accel_dps2=side*-120.f;
  in.cap_ms=0;
  const auto empty=st::select(current,in,p);
  assert(empty.reason==st::Reason::Insufficient && empty.coast_margin_j<0 && empty.width_ms==0);
  in.cap_ms=80;
  const auto powered=st::select(current,in,p);
  assert(powered.reason==st::Reason::Drive || powered.reason==st::Reason::Wait);
  assert(powered.margin_j>0 && powered.width_ms>0 && powered.width_ms+powered.delay_ms<=powered.terminal_ms);
  assert(std::fabs(powered.margin_j-powered.coast_margin_j-powered.work_j)<2e-9f);
  in.cap_ms=UINT16_MAX; // Invalid wrapped/oversized budgets cannot command a pulse.
  assert(st::select(current,in,p).reason==st::Reason::Invalid);
  in.cap_ms=80;
  if (side>0) in.right_deg=12; else in.left_deg=12;
  in.angle_deg=side*1.7f;
  p.observe(1000,3,in,true);
  assert(p.terminal_deg==target && !p.complete);
  in.angle_deg=side*2.f;
  p.observe(1000,4,in,true);
  auto done=st::select(current,in,p);
  assert(done.phase==st::Phase::Complete && done.reason==st::Reason::Complete && !done.width_ms);
  in.angle_deg=side*1.6f;
  p.observe(1000,5,in,true);
  assert(p.complete); // Neither a reversal nor noise rearms the same episode.
  p.observe(1500,6,in,true);
  assert(p.anchor==st::Anchor::FirstReceiving && p.terminal_deg!=target && !p.complete);
  in.angle_deg=side*3.f;
  p.observe(1800,7,in,true);
  assert(p.anchor==st::Anchor::FirstReceiving && p.complete); // Already on the arc.
  in.side=-side;
  p.observe(1800,8,in,true);
  assert(p.anchor==st::Anchor::None && !p.complete);
}

static void observationGap() {
  auto in=approach();
  for (bool invalid : {false,true}) {
    st::Progress p;
    p.observe(100,2,in,true);
    if (invalid) p.observe(100,3,in,false);
    in.angle_deg=.5f;
    p.observe(100,4,in,true);
    assert(p.anchor==st::Anchor::FirstReceiving);
    in.angle_deg=-1;
  }
  // Foot pitch for the bracketed anchor is interpolated at the change of
  // sign, not taken from a later packet after the swing has progressed.
  st::Progress p;
  in.angle_deg=-5;
  const float old_gap=st::supportGap(0,0,0,in.angle_deg);
  p.observe(200,8,in,true);
  in.right_deg=9;
  const float next_gap=st::supportGap(0,9,0,in.angle_deg);
  p.observe(200,9,in,true);
  assert(old_gap<0 && next_gap>0 && p.anchor==st::Anchor::Bracketed);
  const float contact_pitch=9.f*(-old_gap)/(next_gap-old_gap);
  assert(std::fabs(p.terminal_deg-st::arcEntryDeg(1,contact_pitch))<.00001f);
}

static void receivingPivot(const lateral_energy::CurrentTable& current) {
  auto in=approach();
  in.angle_deg=-6.0432f; in.rate_dps=16.0146f; in.fore_aft_deg=-2.4881f;
  in.left_deg=5.0838f; in.right_deg=11.0901f;
  in.left_rate_dps=-3.3421f; in.right_rate_dps=-91.3832f;
  st::Progress p;
  p.observe(100,1,in,true);
  assert(st::detail::gap(st::detail::forecast(in,0))>.4f);
  assert(st::detail::gap(st::detail::forecast(in,75))<-1.4f);
  assert(st::detail::gap(st::detail::forecast(in,150))>.25f);
  const auto reverses=st::select(current,in,p);
  assert(reverses.phase==st::Phase::Receiving && reverses.reason==st::Reason::OutsideApproach);
  assert(std::isnan(reverses.margin_j) && reverses.width_ms==0);

  // Fixed-foot continuation cannot switch away as body roll advances
  // toward the receiver in this CAD domain: d(gap)/d(roll) = x_right-x_left.
  for (int pitch : {-10,0,10}) for (int left : {-20,-5,0,5,20})
    for (int right : {-20,-5,0,5,20}) for (int side : {-1,1}) {
      float previous=-INFINITY;
      for (int i=-30;i<=30;++i) {
        float gap=side*st::supportGap(left,right,pitch,side*.5f*i);
        assert(gap>previous);
        previous=gap;
      }
    }
  in=approach();
  in.angle_deg=-4; in.right_deg=6; in.accel_dps2=0; in.cap_ms=0;
  p=st::Progress{};
  p.observe(200,1,in,true);
  auto long_horizon=st::select(current,in,p);
  assert(long_horizon.phase==st::Phase::Receiving && long_horizon.terminal_ms>150);
  const auto start=st::detail::forecast(in,0), local=st::detail::forecast(in,150);
  const auto end=st::detail::pose(in.left_deg,in.right_deg,0,long_horizon.terminal_deg);
  float max_rise=0;
  for (float angle=in.angle_deg+in.rate_dps*.15f;angle<long_horizon.terminal_deg;angle+=.025f) {
    auto at=st::detail::pose(in.left_deg,in.right_deg,0,angle);
    max_rise=std::fmax(max_rise,st::detail::height(at)-st::detail::height(local));
  }
  const float mg=st::kMassKg*st::kGravity*.001f;
  const float required=std::fmax(mg*max_rise,
      mg*(st::detail::height(end)-st::detail::height(local))+kinetic(5,end,1));
  assert(std::fabs(long_horizon.coast_margin_j-(kinetic(15,start,1)-required))<2e-8f);
}

static void boundedPivotScan() {
  std::mt19937 rng(69728952);
  const auto draw=[&](float low,float high) {
    return std::uniform_real_distribution<float>(low,high)(rng);
  };
  int compared=0, reversals=0;
  for (int i=0;i<20000;++i) {
    auto in=approach();
    in.side=i%2 ? -1 : 1;
    in.angle_deg=draw(-15,15); in.rate_dps=in.side*draw(.1f,95);
    in.left_deg=draw(-20,20); in.left_rate_dps=draw(-180,180);
    in.right_deg=draw(-20,20); in.right_rate_dps=draw(-180,180);
    in.fore_aft_deg=draw(-10,10);
    auto limit=st::detail::withinAngle(in.angle_deg,in.rate_dps,15,150);
    limit=st::detail::withinAngle(in.left_deg,in.left_rate_dps,20,limit);
    limit=st::detail::withinAngle(in.right_deg,in.right_rate_dps,20,limit);
    const auto start=st::detail::forecast(in,0), end=st::detail::forecast(in,limit);
    if (!(limit>0) || in.side*st::detail::gap(start)<0) continue;
    auto gap=[&](float t) {return in.side*st::detail::gap(st::detail::forecast(in,t));};
    bool exact=gap(limit)>=0;
    const auto check_edge=[&](float foot,float rate) {
      if (rate==0) return;
      const float t=-1000.f*(foot+in.fore_aft_deg)/rate;
      if (t>0 && t<limit && gap(t)<0) exact=false;
    };
    check_edge(in.left_deg,in.left_rate_dps);
    check_edge(in.right_deg,in.right_rate_dps);
    for (unsigned t=5;t<limit;t+=5) if (gap(t)<0) exact=false;
    assert(st::detail::keepsReceivingPivot(in,limit,start,end)==exact);
    ++compared;
    if (!exact) ++reversals;
  }
  assert(compared>5000 && reversals>200);
}

int main() {
  lateral_energy::CurrentTable current;
  auto in=approach();
  assert(st::select(current,in).reason==st::Reason::Invalid);
  assert(current.prepare(.030f));
  boundedPivotScan();
  receivingPhase(current,1);
  receivingPhase(current,-1);
  observationGap();
  receivingPivot(current);
  // Independent v14 smoothed-GLB reference values, not a mechanical ZERO.
  assert(std::fabs(st::supportGap(0,4,0,-6.26106f))<.001f);
  assert(std::fabs(st::supportGap(4,0,0,6.26106f))<.001f);
  assert(st::supportGap(0,4,0,-7)<0 && st::supportGap(0,4,0,-5)>0);
  const auto drive=st::select(current,in);
  assert(drive.reason==st::Reason::Drive && drive.side==1 && drive.delay_ms==0);
  assert(drive.coast_margin_j<0 && drive.margin_j>0 && drive.width_ms>0);
  assert(!drive.work_supported); // Slow extrapolated positive pulse.
  assert(drive.width_ms<=in.cap_ms && drive.width_ms<=drive.transfer_ms);
  in.cap_ms=drive.width_ms-1;
  const auto limited=st::select(current,in);
  assert(limited.reason==st::Reason::Insufficient && limited.margin_j<=0);
  in.cap_ms=0;
  const auto unavailable=st::select(current,in);
  assert(unavailable.reason==st::Reason::Insufficient && unavailable.width_ms==0);
  in=approach();
  in.accel_dps2=50;
  const auto wait=st::select(current,in);
  assert(wait.reason==st::Reason::Wait && wait.delay_ms>=st::kReplanMs);
  assert(wait.margin_j>0 && wait.delay_ms+wait.width_ms<=wait.transfer_ms);
  in=approach();
  lateral_energy::CurrentTable slow,fast;
  assert(slow.prepare(.075f) && fast.prepare(.012f));
  assert(st::select(slow,in).reason==st::Reason::Insufficient);
  assert(st::select(fast,in).width_ms<drive.width_ms);
  in.accel_dps2=-1200;
  const auto stops=st::select(current,in);
  assert(stops.reason==st::Reason::Insufficient && stops.width_ms<10 && stops.delay_ms==0);
  in.initial_current_ma=-250;
  assert(st::select(current,in).width_ms==0); // Cannot count reverse residual as positive.

  in=approach();
  in.angle_deg=-2; in.rate_dps=40; in.accel_dps2=0;
  const auto coast=st::select(current,in);
  assert(coast.reason==st::Reason::Coast && coast.coast_margin_j>0 && coast.width_ms==0);
  assert(std::fabs(coast.transfer_ms-50)<.2f);
  in.right_rate_dps=20; // Moving foot shifts support equality before ZERO.
  const auto moving=st::select(current,in);
  assert(moving.transfer_deg<-.5f && moving.transfer_ms<coast.transfer_ms-10);
  assert(moving.coast_margin_j<coast.coast_margin_j);
  in.right_rate_dps=0; in.angle_deg=1;
  assert(st::select(current,in).reason==st::Reason::OutsideApproach); // Already passed.
  in.side=-1;
  assert(st::select(current,in).reason==st::Reason::OutsideApproach); // Wrong velocity sign.
  in.rate_dps=-15; in.accel_dps2=50;
  const auto other=st::select(current,in);
  assert(other.side==-1 && other.reason==st::Reason::Drive && other.width_ms>0);
  assert(std::fabs(other.transfer_ms-drive.transfer_ms)<.2f);
  in.angle_deg=8;
  assert(st::select(current,in).reason==st::Reason::OutsideApproach); // Horizon unresolved.
  in.left_deg=NAN;
  assert(st::select(current,in).reason==st::Reason::Invalid);
  in=approach(); in.cap_ms=81;
  assert(st::select(current,in).reason==st::Reason::Invalid);
  std::puts("Support-transfer approach, receiving arc, pulse budgets, support reversal and event tracking passed.");
}
