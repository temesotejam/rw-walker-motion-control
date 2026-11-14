#pragma once
#include "walking_energy_policy_v15.h"

// Admission only: keep the selected J/width/current unchanged. A relocated
// main input needs both inherited fits' recorded rate support and excitation
// charge support, as well as its existing post-guard positive energy budget.
// These coverage flags are not proof of physical prediction accuracy.
namespace energy_admission_v19 {
constexpr char kRevision[]="optional_main_empirical_support_v19";
enum class Scope : uint8_t { OrdinaryZero=0,OptionalMain=1,ExplicitReturnAid=2 };
enum class Reason : uint8_t {
  Allowed=0,OrdinaryZeroExempt=1,ExplicitReturnAidExempt=2,InvalidScope=3,
  InvalidDecision=4,NoPositiveCoveredBudget=5,PassiveRateUnsupported=6,
  WorkRateUnsupported=7,ChargeUnsupported=8
};
enum Unsupported : uint8_t { PassiveRate=1u,WorkRate=2u,Charge=4u };
struct Choice {
  bool applies=false,allowed=false;
  uint8_t unsupported_mask=0;
  Reason reason=Reason::InvalidScope;
};
inline Choice assess(Scope scope,const walking_energy_policy::Decision& post_guard){
  Choice out;
  if(!post_guard.passive_rate_supported)out.unsupported_mask|=PassiveRate;
  if(!post_guard.work_rate_supported)out.unsupported_mask|=WorkRate;
  if(!post_guard.charge_supported)out.unsupported_mask|=Charge;
  // Exempt means this additional optional-main gate has no veto. It does not
  // authorize output: the runner retains ordinary energy validity, explicit
  // return-J evidence, health, freshness, wheel guard and shared80ms checks.
  if(scope==Scope::OrdinaryZero){out.allowed=true;out.reason=Reason::OrdinaryZeroExempt;return out;}
  if(scope==Scope::ExplicitReturnAid){out.allowed=true;out.reason=Reason::ExplicitReturnAidExempt;return out;}
  out.applies=true;
  if(scope!=Scope::OptionalMain)return out;
  if(!post_guard.valid || !post_guard.selected){out.reason=Reason::InvalidDecision;return out;}
  if(!walking_energy_policy_v15::earlyAdmissible(post_guard)){
    out.reason=Reason::NoPositiveCoveredBudget;return out;
  }
  if(out.unsupported_mask&PassiveRate){out.reason=Reason::PassiveRateUnsupported;return out;}
  if(out.unsupported_mask&WorkRate){out.reason=Reason::WorkRateUnsupported;return out;}
  if(out.unsupported_mask&Charge){out.reason=Reason::ChargeUnsupported;return out;}
  out.allowed=true;out.reason=Reason::Allowed;return out;
}
inline bool optionalMainAdmissible(const walking_energy_policy::Decision& post_guard){
  return assess(Scope::OptionalMain,post_guard).allowed;
}
// On rejection, send nothing and leave the real ZERO opportunity unconsumed.
// Do not reinterpret a narrower supported Q as a newly authorized command;
// the width remains the existing energy inverse result or is not relocated.
}
