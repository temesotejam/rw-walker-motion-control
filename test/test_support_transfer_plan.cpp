#include "support_transfer_plan.h"
#include <cassert>
#include <cstdio>

namespace st = support_transfer_plan;

static st::Input approach() {
  st::Input in;
  in.side=1; in.angle_deg=-1; in.rate_dps=15; in.accel_dps2=-50;
  in.left_deg=in.right_deg=in.fore_aft_deg=0;
  in.left_rate_dps=in.right_rate_dps=0;
  in.initial_current_ma=0; in.goal_current_ma=330; in.cap_ms=80;
  return in;
}

int main() {
  lateral_energy::CurrentTable current;
  auto in=approach();
  assert(st::select(current,in).reason==st::Reason::Invalid);
  assert(current.prepare(.030f));
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
  std::puts("Support-transfer CAD, coast, start/width, prefix stall, current lag and direction checks passed.");
}
