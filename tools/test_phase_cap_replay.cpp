#include "foot_phase_observer.h"
#include "foot_phase_cap.h"
#include "wheel_work_energy_control.h"
#include "unmodeled_walk_coast.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

// Read the delivery-ordered stream from replay_phase_cap.py.  This is a
// recorded-state replay, not a model of the machine after an intervention.
// Stop at the first command that differs from the actual recorded command.
struct NegativePeak { unsigned ms; float amplitude; };
struct NegativeDecision { unsigned ms; foot_phase_cap::NegativeDecision value; };

int main() {
  foot_phase_observer::Observer observer;
  foot_phase_cap::Controller cap;
  std::vector<NegativePeak> peaks;
  std::vector<NegativeDecision> decisions;
  uint32_t seq=0;
  unsigned run=0, now=0, ms=0, paired=0, valid_pairs=0, positive=0;
  char op;
  auto process = [&](uint32_t time_ms) {
    if(observer.sequence()==seq)return;
    seq=observer.sequence();++paired;
    const uint32_t start=observer.cycleStart(),end=observer.cycleTime();
    float amplitude=NAN;bool peak_known=false;
    foot_phase_cap::NegativeDecision decision;
    unsigned count=0;
    for(const auto& p:peaks)if(p.ms==end){amplitude=p.amplitude;peak_known=true;}
    for(const auto& d:decisions)if(d.ms>start && d.ms<end){++count;decision=d.value;}
    const bool valid=peak_known && observer.valid(time_ms);
    if(valid)++valid_pairs;
    cap.pair(end,start,observer.leftLatest(),observer.rightLatest(),amplitude,
             count==1 ? decision : foot_phase_cap::NegativeDecision{},valid);
  };
  while(std::cin>>op) {
    if(op=='R') {
      if(!(std::cin>>run))return 1;
      observer.reset(static_cast<uint16_t>(run));cap.reset();continue;
    }
    if(op=='X') {
      if(!(std::cin>>now))return 1;
      std::cout<<"first_unavailable_pre_mpc_baseline_ms="<<now<<" pairs="<<paired<<" valid_pairs="<<valid_pairs<<"\n";
      return 0;
    } else if(op=='B') {
      int lv=0,rv=0;float left_x=0,right_x=0;
      if(!(std::cin>>now>>ms>>lv>>rv>>left_x>>right_x))return 1;
      observer.rawFrame(static_cast<uint16_t>(run),ms,lv,rv,left_x,right_x);
      // observeFootFrame passes the frame exposure time, not delivery time.
      process(ms);
    } else if(op=='P') {
      int side=0;float amplitude=0;
      if(!(std::cin>>now>>ms>>side>>amplitude))return 1;
      if(side<0)peaks.push_back({ms,amplitude});
      observer.peak(static_cast<uint16_t>(run),side,ms);
      process(now);
    } else if(op=='Z') {
      int side=0,baseline=0,baseline_ma=0,sign=0,existing_cap=0,allowed=0;
      int known=0,executed=0,reason=0,branch=0,actual_ma=0,actual_sign=0,actual_width=0,speed_fresh=0,mismatch=0;
      float target=0,target_deg=0,start_target=0,rate=0,rpm=0,battery=0,previous=0,initial=0,goal=0,tau=0,base_prediction=0;
      if(!(std::cin>>now>>side>>baseline>>baseline_ma>>sign>>existing_cap>>allowed>>target
           >>known>>executed>>reason>>branch>>actual_ma>>actual_sign>>actual_width
           >>speed_fresh>>mismatch>>target_deg
           >>start_target>>rate>>rpm>>battery>>previous>>initial>>goal>>tau>>base_prediction))return 1;
      cap.tick(now);
      if(side<0) {
        wheel_work_energy_control::State observed(rate,rpm,battery,speed_fresh!=0,previous);
        observed.absolute_reference=true;observed.target_deg=target_deg;
        // Old logs cannot establish live hardware health at each event.
        // This conditional replay assumes the hardware checks are satisfied.
        if(reason==15 && !executed && actual_width==0 &&
           unmodeled_walk_coast::eligible(observed,target,mismatch==0)) {
          known=1;reason=1;branch=12;
        }
        const auto decision=foot_phase_cap::classifyNegative(known!=0,executed!=0,
            reason==0,reason==1,static_cast<int16_t>(actual_ma),actual_sign,
            static_cast<uint16_t>(actual_width),static_cast<uint8_t>(branch));
        decisions.push_back({now,decision});
        cap.negativeDecision(now,decision);
      } else if(side>0) {
        ++positive;
        const auto restricted=cap.limit(now,static_cast<uint16_t>(baseline),
            static_cast<int16_t>(baseline_ma),sign,static_cast<uint16_t>(existing_cap),allowed!=0,target);
        // In the integrated controller the cap is then passed to the ordinary
        // energy selector. This test only knows the selected cap; it does not
        // assert the resulting motor width without that selector's full state.
        if(allowed && restricted!=baseline) {
          // A previously recorded phase intervention that matches the new
          // proposal is still recorded history; confirm it and continue.
          if(executed && actual_ma==400 && actual_sign==sign && actual_width==restricted) {
            cap.confirm(now,true,static_cast<int16_t>(actual_ma),actual_sign,
                        static_cast<uint16_t>(actual_width));
            continue;
          }
          // Reconstruct only the local 400mA candidate with the SAME public
          // current table, charge, positiveBoundary and consider helpers used
          // by chooseFootPhaseWidth. The method itself is a private member of
          // ExperimentRunner and is NOT executed by this host-only harness.
          lateral_energy::Geometry geometry;
          lateral_energy::CurrentTable table;
          const bool ready=table.prepare(tau);
          wheel_work_energy_control::State state(rate,rpm,battery,true,previous);
          state.absolute_reference=true;state.target_deg=8.f;
          const float qbase=lateral_energy::charge(table,static_cast<uint16_t>(baseline),initial,goal);
          const auto pbase=wheel_work_energy_control::positiveBoundary(geometry,state,qbase,0.f,0.f);
          const float inferred_bias=base_prediction-pbase.energy_mj;
          const float q=lateral_energy::charge(table,restricted,initial,goal);
          const auto p=wheel_work_energy_control::positiveBoundary(geometry,state,q,0.f,inferred_bias);
          wheel_work_energy_control::Choice choice;
          if(ready && p.valid && p.branch==5 && std::isfinite(q) && q>=0.f && q<=25.f)
            wheel_work_energy_control::consider(choice,400,restricted,q,p,target);
          wheel_work_energy_control::finish(choice,target);
          std::cout<<"first_cap_change_ms="<<now<<" baseline="<<baseline
                   <<" candidate_cap="<<restricted<<" recorded="<<actual_width
                   <<" startup_target_deg="<<start_target<<" target_mJ="<<target
                   <<" pairs="<<paired<<" valid_pairs="<<valid_pairs
                   <<" state="<<static_cast<int>(cap.state())
                   <<" helper_candidate_valid="<<choice.valid<<" helper_branch="<<static_cast<int>(choice.branch)
                   <<" helper_width="<<choice.width_ms<<" q_mAs="<<q<<" predicted_mJ="<<p.energy_mj
                   <<" inferred_baseline_bias_mJ="<<inferred_bias<<"\n";
          return 0;
        }
        // If an old additional selector changed this run's output relative to
        // the unmodified baseline, the counterfactual history ends here.
        if(allowed && executed && actual_width!=baseline) {
          std::cout<<"first_baseline_history_difference_ms="<<now<<" baseline="<<baseline
                   <<" recorded="<<actual_width<<" pairs="<<paired<<" valid_pairs="<<valid_pairs<<"\n";
          return 0;
        }
        cap.confirm(now,executed!=0,static_cast<int16_t>(actual_ma),actual_sign,
                    static_cast<uint16_t>(actual_width));
      }
    } else return 1;
  }
  std::cout<<"no_changed_candidate positive="<<positive<<" pairs="<<paired
           <<" valid_pairs="<<valid_pairs<<" state="<<static_cast<int>(cap.state())
           <<" reason="<<static_cast<int>(cap.reason())<<"\n";
  return 0;
}
