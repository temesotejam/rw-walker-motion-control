#include "walking_return_energy.h"
#include "walking_pulse.h"

#include "walking_energy_policy.h"
#include "walking_energy_loss.h"
#include "one_peak_energy_inverse.h"
#include <Arduino.h>
#if EARLY_ENERGY_MAIN
namespace active_energy=walking_energy_policy_v15;
#else
namespace active_energy=walking_energy_policy;
#endif

String eventRevisions(unsigned branch,bool coast){
  struct Event {unsigned work_control_branch;float maximum_input_work_j;} e{branch,.000045678f};
  const bool work_event=branch!=0,inverse_event=false,energy_bridge_enabled_=true;
  const unsigned control_target_cdeg_=800;
  String detail="{\"branch\":"+String(branch)+",\"coast\":"+String(coast?"true":"false");
/* PRODUCTION: EVENT_REVISION */
  auto appendAutonomousNullable=[&](const char* name,float value,unsigned digits){
    detail+=",\""+String(name)+"\":";
    detail+=std::isfinite(value)?String(value,static_cast<int>(digits)):String("null");};
/* PRODUCTION: MAXIMUM_WORK */
  detail+="}";return detail;
}

#include "psram_string.h"
#include "compact_json_table.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#define CHECK(x) do {if(!(x)){std::cerr<<"check failed: "<<#x<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
class ExperimentRunner {public:
/* PRODUCTION: AUDIT_MEMBERS */
  WalkingEnergyAudit walking_energy_audit_[192];
  uint32_t walking_energy_count_=0,walking_energy_total_=0;
  void appendWalkingEnergyMetadata(PsramString& json) const;
};
/* PRODUCTION: SERIALIZER */
int main(int argc,char** argv){
  ExperimentRunner runner;
  const bool small_step=false;
  const bool empty=argc>1 && std::string(argv[1])=="empty";
  lateral_energy::Geometry geometry;
  lateral_energy::CurrentTable table;CHECK(table.prepare(.0316f));

  for(unsigned i=0;i<7 && !empty;++i){
    active_energy::Input in;in.side=1;in.rate_dps=35.f;in.body_angle_deg=0;
    in.target_deg=8;in.previous_peak_deg=8;in.same_side_peak_deg=6;
    in.initial_current_ma=i==2?-150.f:0.f;in.goal_current_ma=329.547119f;in.cap_ms=80;
    auto decision=active_energy::decide(geometry,table,in);

    if(i==1){
      walking_return_energy::Input aid;
      static_cast<walking_energy_policy::Input&>(aid)=in;
      aid.body_angle_deg=9.4f;aid.return_work_j=.00015f;
      decision=half_cycle_energy_budget::decideReturnAid(geometry,table,aid);
    }
    CHECK(decision.valid && !decision.coast);
    CHECK(i==1 || decision.width_ms==80);

    auto& a=runner.walking_energy_audit_[i];a.decision=decision;
    a.time_ms=1200+i*300;a.peak_ms=a.time_ms-250;a.trigger_ms=a.time_ms;a.side=1;
    a.arrival_bias_j=i==1?0:.000123f;a.return_work_j=i==1?.00015f:0;a.return_event=i==1;a.return_deceleration_dps2=154.68f;a.return_stopping_distance_deg=.485f;a.return_foot_age_ms=131;a.return_evidence=i==1?2:0;a.assisted_cycle=i==1 || i==2;a.envelope_before_ms=i==2?63:80;a.envelope_after_ms=i==1?63:0;a.requested_ms=a.guarded_ms=decision.width_ms;a.executed=i!=4;
    a.optional=i==4;a.early_budget_checked=i==4;a.early_budget_admitted=false;
    if(a.executed){
      a.output_start_ms=a.time_ms+7;
      a.start_i0_ma=in.initial_current_ma;a.start_goal_ma=in.goal_current_ma;
      a.start_tau_s=.0316f;a.start_rate_dps=in.rate_dps;
      if(i!=3){
        energy_pulse_completion::Input done;done.side=a.side;done.rate_dps=a.start_rate_dps;
        done.initial_current_ma=a.start_i0_ma;done.goal_current_ma=a.start_goal_ma;done.tau_s=a.start_tau_s;
        done.available_j=decision.available_j;done.request_j=decision.request_j;done.planned_ms=a.guarded_ms;
        done.actual_elapsed_ms=i==0?80:i==5?123:i==6?0:17;
        a.completion=energy_pulse_completion::evaluate(done);
        a.completion_reason=i==0 || i==5?1:i==1?5:i==6?2:3;
        a.output_end_ms=a.output_start_ms+done.actual_elapsed_ms;
      }
    }
    ++runner.walking_energy_count_;++runner.walking_energy_total_;
  }
  PsramString json;CHECK(json.reserve(1024*1024));json+="{";

#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  const bool return_energy_mode=true;
#else
  const bool return_energy_mode=false;
#endif
/* PRODUCTION: WORK_METADATA */

  const bool walking_energy_mode=true,early_energy_mode=EARLY_ENERGY_MAIN;
  const bool work_mode=true,inverse_mode=false,energy_bridge_enabled_=true;
  const bool positive_current_batch_mode=false,negative_probe_extended_mode=false;
  const bool negative_probe_batch_mode=false,negative_probe_mode_=false;
/* PRODUCTION: MODEL_REVISION */
/* PRODUCTION: POLICY_FIELDS */
  json+="\"event_revision_checks\":[";

  for(unsigned branch:{0u,15u,19u,20u,21u,22u,23u})for(bool coast:{false,true}){
    if(branch!=0 || coast)json+=",";
    json+=eventRevisions(branch,coast);
  }
  json+="],";
  json+="\"sentinel\":0";
  runner.appendWalkingEnergyMetadata(json);json+="}";CHECK(json.ok());
  std::cout<<json.c_str()<<"\n[";
  // Independent high-precision values expose truncation of sub-mJ terms.
  for(unsigned i=0;i<runner.walking_energy_count_;++i){
    if(i)std::cout<<",";
    const auto& d=runner.walking_energy_audit_[i].decision;
    std::cout<<"{";bool first=true;
    auto number=[&](const char* key,float value){if(!first)std::cout<<",";first=false;
      std::cout<<"\""<<key<<"\":";
      if(std::isfinite(value))std::cout<<std::setprecision(17)<<value;else std::cout<<"null";};
    number("available_j",d.available_j);number("target_j",d.target_j);
    number("predicted_work_j",d.predicted_work_j);number("mean_peak_energy_j",d.mean_peak_energy_j);
    number("redistribution_j",d.redistribution_j);number("request_j",d.request_j);
    const auto& c=runner.walking_energy_audit_[i].completion;
    number("elapsed_q_pred_mas",c.q_mas);number("elapsed_current_end_pred_ma",c.current_end_ma);
    number("elapsed_work_pred_j",c.work_j);number("elapsed_arrival_pred_j",c.arrival_j);
    number("elapsed_unmet_pred_j",c.unmet_j);

    std::cout<<"}";
  }
  std::cout<<"]\n";
  // The real compact-table encoder rejects drift instead of silently
  // associating values with a previous row's different columns.
  PsramString bad;CHECK(bad.reserve(1024));CompactJsonTable<PsramString> compact(bad);
  CHECK(compact.append("{\"a\":1}"));CHECK(!compact.append("{\"b\":2}"));
  return 0;
}
