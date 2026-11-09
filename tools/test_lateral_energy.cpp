#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include "lateral_energy_control.h"
#include "peak_error_adaptation.h"
#include "autonomous_brake_limit.h"

static bool near(float a, float b, float tolerance) {
  return std::fabs(a-b) <= tolerance;
}

int main() {
  using namespace lateral_energy;
  const Geometry g;
  for (float degrees : {0.f, .25f, 1.f, 2.f, 4.f, 8.f, 10.f, 12.f, 16.f}) {
    const float e=potential(g,degrees);
    assert(std::isfinite(e));
    assert(near(angle(g,e),degrees,.015f));
  }
  CurrentTable table;
  uint16_t selected_brake_ms = 99;
  for (const char* choice : {"0", "20", "40", "50"})
    assert(autonomous_brake_limit::parse(choice, selected_brake_ms));
  for (const char* invalid : {"", "-1", "10", "51", "20ms"})
    assert(!autonomous_brake_limit::parse(invalid, selected_brake_ms));
  assert(table.prepare(.025f));
  assert(near(charge(table,0,0,300),0,.000001f));
  assert(charge(table,100,0,300)>20.f);
  for (int side : {-1,1}) {
    const auto c=coefficients(side);
    for (float rate : {40.f,60.f,85.f}) {
      const float omega=rate*kRadPerDeg;
      for (float required : {.00001f,.00010f,.00030f}) {
        const float q=requiredCharge(c,omega,required);
        assert(std::isfinite(q) && q>0);
        assert(near(work(c,omega,q),required,1.e-7f));
      }
      const float brake_q=requiredSignedCharge(c,omega,-.00002f);
      assert(std::isfinite(brake_q) && brake_q<0);
      assert(near(work(c,omega,brake_q),-.00002f,1.e-7f));
    }
    const auto d=decide(g,table,side,65, potential(g,10),0,0,300);
    assert(d.valid && d.ms>0 && d.ms<=100);
    assert(d.evaluations<=10);
    assert(d.kinetic_j>0 && d.loss_j>0);
    assert(d.predicted_j>=d.free_j);
    assert(d.selected_q_mAs<=d.available_q_mAs);
    assert(d.energy_error_j<.00003f);
    const auto brake=decide(g,table,side,75,potential(g,8),0,0,300);
    assert(brake.valid && brake.ms>0 && brake.ms<=kBrakeMaxMs);
    assert(brake.command_sign==-1 && brake.selected_q_mAs<0);
    assert(brake.predicted_j<brake.free_j && brake.extrapolated);
    assert(brake.evaluations<=10);
    const auto no_brake=decide(g,table,side,75,potential(g,8),0,0,300,0);
    assert(no_brake.valid && no_brake.ms==0 && no_brake.command_sign==0);
    const auto short_brake=decide(g,table,side,75,potential(g,8),0,0,300,20);
    assert(short_brake.valid && short_brake.ms<=20);
    assert(short_brake.predicted_j>=brake.predicted_j);
    const auto coast=decide(g,table,side,60,
        freeEnergy(c,60*kRadPerDeg),0,0,300);
    assert(coast.valid && coast.ms==0 && coast.command_sign==0);
    assert(!coast.lower && !coast.upper);
  }
  // The bridge must preserve the historical angle predictor exactly after
  // mapping through U, and must never infer an unmeasured reverse-Q gain.
  for (int side : {-1,1}) {
    const auto c=bridgeCoefficients(side);
    for (float rate : {45.f,60.f,75.f}) {
      const float free_angle=std::fmax(0.f,c.at_65_deg+c.per_dps*(rate-65.f));
      for (float q : {0.f,1.f,3.f,6.f}) {
        const float predicted=bridgeArrivalEnergy(g,side,rate,q);
        assert(near(angle(g,predicted),free_angle+c.gain_deg_per_mAs*q,.0001f));
      }
      const auto d=decideBridge(g,table,side,rate,potential(g,10.f),0,0,300);
      assert(d.valid && d.ms<=100 && d.selected_q_mAs>=0);
      assert(!std::isfinite(d.kinetic_j) && !std::isfinite(d.loss_j));
      assert(near(d.free_j,potential(g,free_angle),1.e-7f));
      assert(near(d.predicted_j,bridgeArrivalEnergy(g,side,rate,d.selected_q_mAs),1.e-7f));
      float best=std::fabs(d.corrected_target_j-d.free_j);
      for (uint16_t ms=1;ms<=100;++ms) {
        const float q=charge(table,ms,0,300);
        if (q>d.available_q_mAs) continue;
        const float e=bridgeArrivalEnergy(g,side,rate,q);
        if (std::isfinite(e)) best=std::fmin(best,std::fabs(d.corrected_target_j-e));
      }
      assert(d.energy_error_j<=best+1.e-8f);
    }
    const auto excess=decideBridge(g,table,side,80,potential(g,8.f),0,0,300);
    assert(excess.valid && excess.ms==0 && excess.command_sign==0 && excess.lower);
    assert(!std::isfinite(excess.unclamped_q_mAs));
    assert(!std::isfinite(bridgeArrivalEnergy(g,side,65,-1.f)));
    assert(near(updateBridgeIntegral(0,.0001f,.001f,side,false,false,true),
                .00001f*c.gain_deg_per_mAs,1.e-8f));
  }
  // At the same 65 dps zero crossing, the old predictor needs positive Q
  // for an 8-degree goal, while the newer quadratic free model is above it.
  for (int side : {-1,1}) {
    const auto historical=decideBridge(g,table,side,65,potential(g,8.f),0,0,300);
    const auto quadratic=decide(g,table,side,65,potential(g,8.f),0,0,300,0);
    assert(historical.valid && historical.requested_q_mAs>1.f && historical.command_sign==1);
    assert(quadratic.valid && quadratic.free_j>potential(g,8.f) && quadratic.ms==0);
  }
  const auto reverse=decide(g,table,1,65,potential(g,10),0,-150,300);
  assert(reverse.valid && reverse.ms<=100 && reverse.selected_q_mAs>=0);
  {
    const auto c = coefficients(1);
    const float omega = 60.f * kRadPerDeg;
    const float target = freeEnergy(c, omega) + work(c, omega, 5.f);
    const auto original = decide(g,table,1,60,target,0,0,300);
    const auto corrected = decide(g,table,1,60,target,0,0,300,kBrakeMaxMs,.000025f);
    assert(original.valid && corrected.valid);
    assert(corrected.selected_q_mAs < original.selected_q_mAs);
    assert(corrected.ms < original.ms);
  }
  {
    peak_error_adaptation::Model model;
    uint8_t bin=255, count=255;
    for (int i=0;i<4;++i)
      model.observe(5000+i*500,1,peak_error_adaptation::Excite,50,true,10000,.00004f);
    assert(near(model.correction(1,peak_error_adaptation::Excite,50,true,10000,&bin,&count),
                .00004f,1.e-8f));
    assert(bin==peak_error_adaptation::Slow && count==4);
    assert(model.correction(-1,peak_error_adaptation::Excite,50,true,10000)==0);
    assert(model.correction(1,peak_error_adaptation::Brake,50,true,10000)==0);
    assert(model.correction(1,peak_error_adaptation::Excite,150,true,10000)==0);
    assert(model.correction(1,peak_error_adaptation::Excite,50,true,60000)==0);
    model.observe(7100,1,peak_error_adaptation::Excite,50,true,10000,-.00004f);
    assert(model.correction(1,peak_error_adaptation::Excite,50,true,10000)==0);
    model.reset();
    assert(model.correction(1,peak_error_adaptation::Excite,50,true,10000)==0);
  }
  // Compare the bounded inverse with every legal integer pulse width. The
  // residual current may initially oppose either requested direction.
  for (int side : {-1,1}) {
    const auto c=coefficients(side);
    for (float rate : {18.f,35.f,50.f,65.f,80.f,95.f}) {
      for (float target_deg : {8.f,10.f,12.f}) {
        for (float initial : {-200.f,0.f,200.f}) {
          for (float goal : {300.f,1200.f}) {
            const auto d=decide(g,table,side,rate,potential(g,target_deg),0,initial,goal);
            if (!d.valid || d.command_sign==0) continue;
            const uint16_t max_ms=d.command_sign<0?kBrakeMaxMs:kMaxMs;
            assert(d.ms<=max_ms);
            const float signed_goal=d.command_sign*goal;
            float best=std::fabs(d.corrected_target_j-d.free_j);
            for (uint16_t ms=1;ms<=max_ms;++ms) {
              const float q=charge(table,ms,initial,signed_goal);
              if (d.command_sign<0 ? (q>0 || q< -d.available_q_mAs) : q<0) continue;
              const float predicted=d.free_j+work(c,rate*kRadPerDeg,q);
              if (predicted<0 || predicted>potential(g,16.f)) continue;
              best=std::fmin(best,std::fabs(predicted-d.corrected_target_j));
            }
            assert(d.energy_error_j<=best+1.e-8f);
          }
        }
      }
    }
  }
  assert(near(updateIntegral(0,.0001f,.001f,false,false,true),.00001f,1.e-8f));
  assert(updateIntegral(0,.0001f,.001f,true,false,true)==0);
  assert(!decide(g,table,1,65,potential(g,10),0,0,0).valid);
  assert(!decide(g,table,1,75,potential(g,8),0,0,300,51).valid);
  // Recorded hardware case: requested removal greatly exceeds a 20 ms pulse.
  const auto limited=decide(g,table,-1,69.2887f,potential(g,8),-.00001078f,0,153.f,20);
  assert(limited.valid && limited.lower && limited.ms==20);
  assert(limited.unclamped_deficit_j < -.00039f);
  assert(limited.unclamped_q_mAs < -8.8f);
  assert(limited.requested_q_mAs > -1.f);
  assert(near(work(coefficients(-1),69.2887f*kRadPerDeg,limited.unclamped_q_mAs),
              limited.unclamped_deficit_j,1.e-7f));
  // A negative discriminant is recorded as unavailable, while bounded output
  // still works. Diagnostic failures cannot turn a valid pulse into STOP.
  const auto no_root=decide(g,table,1,95,potential(g,8),-.5f*potential(g,8),0,300,20);
  assert(no_root.valid && no_root.ms<=20);
  assert(!std::isfinite(no_root.unclamped_q_mAs));
  std::puts("lateral energy model: OK");
}
