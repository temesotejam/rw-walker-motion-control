// Shared by the separately identified v14 and v15 energy builds.
#if defined(FOOT_SUPPORT_ENERGY) && FOOT_SUPPORT_ENERGY
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
void ExperimentRunner::finishWalkingEnergyPulse(uint32_t now_ms,uint8_t reason){
  const int index=walking_energy_active_audit_;
  if(!walking_energy_completion_pending_)return;
  walking_energy_completion_pending_=false;
  walking_energy_active_audit_=-1; // One completion, even through nested stop paths.
  auto& a=walking_energy_active_pulse_;
  if(!a.executed || a.completion_reason)return;
  a.output_end_ms=now_ms-run_start_ms_;a.completion_reason=reason;
  energy_pulse_completion::Input in;in.side=a.side;in.rate_dps=a.start_rate_dps;
  in.initial_current_ma=a.start_i0_ma;in.goal_current_ma=a.start_goal_ma;in.tau_s=a.start_tau_s;
  in.available_j=a.decision.available_j;in.request_j=a.decision.request_j;
  in.planned_ms=a.guarded_ms;
  in.actual_elapsed_ms=static_cast<uint32_t>(now_ms-(run_start_ms_+a.output_start_ms));
  a.completion=energy_pulse_completion::evaluate(in);
  if(index>=0 && static_cast<uint32_t>(index)<walking_energy_count_)
    walking_energy_audit_[index]=a;
  if(a.completion.valid){
    predicted_signed_current_end_mA_=a.side*a.completion.current_end_ma;
    predicted_current_end_ms_=now_ms;
  }
  if(energy_control_autonomous_pending_peak_ &&
      energy_control_autonomous_pending_zero_ms_==a.trigger_ms &&
      energy_control_autonomous_pending_next_side_==a.side){
    energy_control_autonomous_pending_q_command_mA_s_=a.completion.valid?a.completion.q_mas:NAN;
    energy_control_autonomous_pending_model_energy_j_=a.completion.work_valid?a.completion.arrival_j:NAN;
  }
}
#endif
bool ExperimentRunner::tryWalkingEnergy(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& e,uint32_t t,bool* healthy_out){
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  namespace energy=walking_energy_policy_v15;
#else
  namespace energy=walking_energy_policy;
#endif
  const int side=e.physical_next_peak_side;
  const bool healthy=wheelWorkMode() && e.target_peak_deg==8.f &&
      (side==1 || side==-1) && !status_.emergency_stop && !status_.pulse_active &&
      status_.state==ExperimentState::RUNNING_BATCH_SWEEP && roller_ && roller_->ok() &&
      imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis()) &&
      !e.side_mismatch_diagnostic && e.vbat_mV>0 && e.pre_input_wheel_speed_valid &&
      e.pre_input_wheel_speed_age_us<=50000 && std::isfinite(e.pre_input_wheel_speed_rpm);
  if(healthy_out)*healthy_out=healthy;
  noteFootSupportBackend(healthy?2:1);
  e.work_control_branch=
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
      21;
#elif defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
      20;
#else
      19;
#endif
  // Even invalid/coast stays in the energy path.
  e.valid=false;e.output_executed=false;
  e.cg_shift_mm=0;
  e.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT;
  WalkingEnergyAudit a;a.time_ms=t;a.peak_ms=energy_control_autonomous_last_peak_ms_;
  a.side=side;a.optional=optional_early_event_;a.rescue=stall_rescue_event_;
  auto save=[&](){++walking_energy_total_;int16_t stored_index=-1;
    if(walking_energy_count_<192){stored_index=static_cast<int16_t>(walking_energy_count_);
      walking_energy_audit_[walking_energy_count_++]=a;}
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
    if(a.executed){walking_energy_active_audit_=stored_index;walking_energy_active_pulse_=a;
      walking_energy_completion_pending_=true;}
#else
    (void)stored_index;
#endif
  };
  if(!healthy){save();return true;} // Never fall through to the old selector.
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  const uint32_t decision_sample_us=foot_support_event_?foot_support_sample_us_:
      energy_control_autonomous_last_motion_sample_us_;
  auto early_fresh=[&](){const uint32_t now=millis()-run_start_ms_;
    return now>=t && now-t<=5 && decision_sample_us &&
        static_cast<uint32_t>(micros()-decision_sample_us)<=10000;};
  if(optional_early_event_ && !early_fresh()){save();return true;}
#endif
  if(!e.previous_peak_time_ms || t<e.previous_peak_time_ms || t-e.previous_peak_time_ms>1500){save();return true;}
  energy::Input in;in.side=side;in.rate_dps=e.zero_cross_abs_rate_dps;
  in.body_angle_deg=(optional_early_event_ || stall_rescue_event_)?e.detector_angle_after_deg:0.f;
  in.target_deg=tilt_priority_.target();in.previous_peak_deg=e.previous_peak_amplitude_deg;
  const unsigned index=side>0?1:0;
  in.same_side_peak_deg=walking_energy_peak_ms_[index] && t>=walking_energy_peak_ms_[index] &&
      t-walking_energy_peak_ms_[index]<=3000?walking_energy_peaks_[index]:NAN;
  in.initial_current_ma=side*e.i0_estimated_mA;
  in.goal_current_ma=predictCurrentGoalMa(1200,e.vbat_mV*.001f);
  in.cap_ms=tilt_priority_.timingCap(side,t,80); // timing protection only
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  // Main-input relocation gets the same envelope as normal ZERO. The
  // separate low-speed rescue trial retains its old, provisional ceiling.
  if(stall_rescue_event_)
#else
  if((optional_early_event_ && !foot_support_event_) || stall_rescue_event_)
#endif
    if(in.cap_ms>15)in.cap_ms=15;
  if(foot_support_event_){
    noteFootSupportBackend(4);
    const uint32_t now=millis()-run_start_ms_;
    if(now<t || now-t>5 || static_cast<uint32_t>(micros()-foot_support_sample_us_)>10000){save();return true;}
    in.body_angle_deg+=e.zero_cross_rate_dps*(now-t)*.001f;
  }
  const auto selected=energy::decide(lateral_energy_geometry_,walking_energy_current_,in);
  a.decision=selected;a.requested_ms=selected.width_ms;
  if(!selected.valid){noteFootSupportBackend(6);save();return true;}
  auto guard=wheel_cycle_guard_;
  const uint16_t width=selected.coast?0:guard.limit(side,e.pre_input_wheel_speed_rpm,true,t,selected.width_ms);
  a.guarded_ms=width;
  const auto actual=energy::evaluateWidth(lateral_energy_geometry_,walking_energy_current_,in,width);
  a.decision=actual;
  if(!actual.valid || width>selected.width_ms){noteFootSupportBackend(7);save();return true;}
  if(foot_support_event_){
    noteFootSupportBackend(5);
    const uint32_t now=millis()-run_start_ms_;
    const auto pose=foot_support_pose::select(foot_pose_,foot_local_pose_,now,true);
    if(!pose.usable || pose.source!=foot_support_source_ || actual.coast){save();return true;}
    auto check=foot_support_input_;check.now_ms=now;check.frame_ms=foot_pose_.frame_ms;
    check.left_deg=pose.pose.left;check.right_deg=pose.pose.right;
    check.left_uncertainty_deg=pose.pose.left_uncertainty;check.right_uncertainty_deg=pose.pose.right_uncertainty;
    check.local_turn_confirmed=pose.local_turn_confirmed;check.local_returning_confirmed=pose.local_returning_confirmed;
    check.body_angle_deg=in.body_angle_deg;check.body_rate_dps=e.zero_cross_rate_dps;
    check.raw_normal_width_ms=selected.width_ms;check.normal_width_ms=width;
    check.boosted_guarded_width_ms=width;check.raw_boost_ms=0;
    foot_support_policy::Limits limits;limits.allow_amount_boost=false;
    const auto timing=foot_support_policy::decide(check,limits);
    noteFootSupportBackend(6,static_cast<uint8_t>(timing.reason));
    if(!timing.selected){save();return true;}
    if(foot_support_active_audit_>=0){auto& f=foot_support_audit_[foot_support_active_audit_];
      f.requested_width=selected.width_ms;f.guarded_width=width;f.checked_ms=now;}
  }
  e.target_energy_j=actual.target_j;e.passive_energy_j=actual.available_j;
  e.kinetic_energy_j=actual.effective_kinetic_j;
  e.delta_energy_required_j=actual.deficit_j;e.commanded_energy_deficit_j=actual.request_j;
  e.corrected_target_energy_j=actual.target_j+actual.peak_correction_j;
  e.integral_energy_j=0;e.predicted_loss_j=actual.inbound_loss_proxy_j;
  e.maximum_input_work_j=actual.maximum_work_j;e.unapplied_energy_request_j=actual.unmet_j;
  e.predicted_energy_j=actual.predicted_arrival_j;e.predicted_input_work_j=actual.predicted_work_j;
  e.predicted_next_peak_amplitude_deg=lateral_energy::angle(lateral_energy_geometry_,actual.predicted_arrival_j);
  e.free_next_peak_amplitude_deg=lateral_energy::angle(lateral_energy_geometry_,actual.available_j);
  e.q_command_mA_s=actual.q_mas;e.q_effective_pred_mA_s=actual.q_mas;
  e.predicted_current_goal_mA=in.goal_current_ma;e.predicted_current_rise_tau_s=walking_energy_current_.tau;
  e.aligned_initial_current_mA=in.initial_current_ma;
  e.energy_model_extrapolated=actual.extrapolated;e.work_prediction_cross_condition=actual.extrapolated;
  e.work_brake_cap_ms=in.cap_ms;e.solver_selected_integer_width_ms=width;
  e.command_current_mA=actual.coast?0:1200;e.pulse_width_ms=width;
  e.q_command_direction=actual.coast?0:side;e.command_matches_zero_cross_motion=!actual.coast;
  e.pulse_start_ms=actual.coast?0:t;e.pulse_end_ms=actual.coast?0:t+width;
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  if(optional_early_event_){
    a.early_budget_checked=true;
    a.early_budget_admitted=energy::earlyAdmissible(actual);
    if(!a.early_budget_admitted){
      // No transport, no consumed main-input opportunity. The outer
      // optional wrapper restores WAIT_ZERO_CROSS before the ordinary pass.
      e.command_current_mA=0;e.pulse_width_ms=0;e.q_command_direction=0;
      e.command_matches_zero_cross_motion=false;e.pulse_start_ms=e.pulse_end_ms=0;
      e.q_command_mA_s=e.q_effective_pred_mA_s=0;e.solver_selected_integer_width_ms=0;
      e.predicted_input_work_j=0;e.predicted_energy_j=actual.available_j;
      e.predicted_next_peak_amplitude_deg=e.free_next_peak_amplitude_deg;
      e.unapplied_energy_request_j=actual.request_j;
      save();return true;
    }
  }
#endif
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  if(optional_early_event_ && !early_fresh()){
    a.early_budget_admitted=false;
    e.command_current_mA=0;e.pulse_width_ms=0;e.q_command_direction=0;
    e.q_command_mA_s=e.q_effective_pred_mA_s=0;e.solver_selected_integer_width_ms=0;
    e.command_matches_zero_cross_motion=false;e.pulse_start_ms=e.pulse_end_ms=0;
    e.predicted_input_work_j=0;e.predicted_energy_j=actual.available_j;
    e.predicted_next_peak_amplitude_deg=e.free_next_peak_amplitude_deg;
    e.unapplied_energy_request_j=actual.request_j;save();return true;
  }
#endif
#if !defined(EARLY_ENERGY_MAIN) || !EARLY_ENERGY_MAIN
  if(foot_support_event_){
    const uint32_t now=millis()-run_start_ms_;
    if(now<t || now-t>5 || static_cast<uint32_t>(micros()-foot_support_sample_us_)>10000){noteFootSupportBackend(4);save();return true;}
  }
#endif
  noteFootSupportBackend(8);
  if(!actual.coast && !beginEnergyControlAutonomousPulse(run_start_ms_+t,t,side,width,1200)){
    e.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED;
    e.command_current_mA=0;e.pulse_width_ms=0;e.pulse_start_ms=e.pulse_end_ms=0;
    save();return true;
  }
  wheel_cycle_guard_=guard;
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  if(!actual.coast){e.pulse_start_ms=active_pulse_start_test_ms_;
    e.pulse_end_ms=e.pulse_start_ms+width;a.output_start_ms=e.pulse_start_ms;}
#endif
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
  a.trigger_ms=e.zero_cross_time_ms;a.start_rate_dps=in.rate_dps;
  a.start_i0_ma=walking_pulse_start_i0_ma_;a.start_goal_ma=walking_pulse_goal_ma_;
  a.start_tau_s=walking_pulse_tau_s_;
#endif
  tilt_priority::Choice confirmation;confirmation.selected=true;confirmation.coast=actual.coast;
  confirmation.current_ma=actual.coast?0:1200;confirmation.width_ms=width;
  confirmation.direction=side;confirmation.target_deg=in.target_deg;
  tilt_priority_.crossing(side,t);tilt_priority_.confirm(side,t,confirmation,true,true,false);
  e.valid=true;e.output_executed=!actual.coast;
  e.reason=actual.coast?Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT:Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE;
  a.executed=e.output_executed;
  pending_cg_shift_mm_=0;pending_target_energy_j_=actual.target_j;
  energy_control_autonomous_pending_peak_=true;energy_control_autonomous_pending_probe_=true;
  energy_control_autonomous_pending_next_side_=side;energy_control_autonomous_pending_target_deg_=in.target_deg;
  energy_control_autonomous_pending_q_command_mA_s_=actual.q_mas;
  energy_control_autonomous_pending_model_energy_j_=actual.predicted_arrival_j;
  energy_control_autonomous_pending_zero_ms_=e.zero_cross_time_ms;
  energy_control_autonomous_pending_wheel_rpm_=e.pre_input_wheel_speed_rpm;
  energy_control_autonomous_pending_wheel_age_us_=e.pre_input_wheel_speed_age_us;
  energy_control_autonomous_pending_wheel_valid_=true;
  energy_control_autonomous_pending_mode_=actual.coast?peak_error_adaptation::Coast:peak_error_adaptation::Excite;
  energy_control_autonomous_pending_saturated_upper_=actual.saturated;
  energy_control_autonomous_pending_saturated_lower_=actual.coast;
  work_positive_pending_=work_negative_pending_=work_walking_pending_=work_walking_low_pending_=false;
  adaptive_cg_.breakPair();save();return true;
}

void ExperimentRunner::tryWalkingEnergyFoot(uint32_t t,uint32_t sample,float angle,float rate){
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  namespace energy=walking_energy_policy_v15;
#else
  namespace energy=walking_energy_policy;
#endif
  const int side=energy_control_autonomous_last_peak_side_;
  if((side!=1 && side!=-1) || !wheelWorkMode() || energy_control_autonomous_target_peak_deg_!=8.f ||
     energy_control_autonomous_input_peak_percent_!=0 || status_.emergency_stop || status_.pulse_active ||
     energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
     !energy_control_autonomous_last_peak_valid_ || energy_control_autonomous_zero_cross_consumed_for_peak_ ||
     t<energy_control_autonomous_last_peak_ms_ || t-energy_control_autonomous_last_peak_ms_>1500 ||
     optional_early_attempted_peak_ms_==energy_control_autonomous_last_peak_ms_ ||
     !std::isfinite(angle) || !std::isfinite(rate) || std::fabs(angle)>5 ||
     static_cast<uint32_t>(micros()-sample)>10000 ||
     (energy_control_autonomous_last_accepted_zero_cross_valid_ && (t<=energy_control_autonomous_last_accepted_zero_cross_ms_ ||
      t-energy_control_autonomous_last_accepted_zero_cross_ms_<Config::ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS)) ||
     (foot_support_last_check_ms_ && t-foot_support_last_check_ms_<5))return;
  foot_support_last_check_ms_=t;
  const auto s=foot_support_pose::select(foot_pose_,foot_local_pose_,t,true);
  if(!s.usable || !roller_)return;
  const auto telemetry=roller_->telemetrySnapshot();
  if(!telemetry.speed_valid || !telemetry.speed_sample_time_us ||
      static_cast<uint32_t>(micros()-telemetry.speed_sample_time_us)>50000 || !std::isfinite(telemetry.speed_rpm))return;
  energy::Input model;model.side=-side;model.rate_dps=std::fabs(rate);model.body_angle_deg=angle;
  model.previous_peak_deg=energy_control_autonomous_last_peak_amplitude_deg_;
  const unsigned index=model.side>0?1:0;
  model.same_side_peak_deg=walking_energy_peak_ms_[index] && t>=walking_energy_peak_ms_[index] &&
      t-walking_energy_peak_ms_[index]<=3000?walking_energy_peaks_[index]:NAN;
  model.target_deg=tilt_priority_.target();
  model.initial_current_ma=predicted_current_end_ms_==0?0:model.side*predicted_signed_current_end_mA_*
      expf(-static_cast<float>(run_start_ms_+t-predicted_current_end_ms_)/70.f);
  model.goal_current_ma=predictCurrentGoalMa(1200,status_.beta_model_vbat_mV*.001f);
  model.cap_ms=tilt_priority_.timingCap(-side,t,80);
  const auto preview=energy::decide(lateral_energy_geometry_,walking_energy_current_,model);
  if(!preview.valid || preview.coast)return; // No energy deficit, no forced foot pulse.
  auto guard=wheel_cycle_guard_;
  const auto width=guard.limit(-side,telemetry.speed_rpm,true,t,preview.width_ms);
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  const auto guarded=energy::evaluateWidth(lateral_energy_geometry_,walking_energy_current_,model,width);
  if(!energy::earlyAdmissible(guarded))return;
#endif
  foot_support_policy::Input in;in.active=true;in.run_id=status_.run_id;in.foot_run_id=foot_pose_.run_id;
  in.peak_id=energy_control_autonomous_last_peak_ms_;in.peak_side=side;in.now_ms=t;in.frame_ms=foot_pose_.frame_ms;
  in.pose_ready=s.pose.ready();in.pose_confident=s.usable;in.left_deg=s.pose.left;in.right_deg=s.pose.right;
  in.left_uncertainty_deg=s.pose.left_uncertainty;in.right_uncertainty_deg=s.pose.right_uncertainty;
  in.body_angle_deg=angle;in.body_rate_dps=rate;in.normal_drive_selected=true;
  in.normal_width_ms=width;in.raw_normal_width_ms=preview.width_ms;in.boosted_guarded_width_ms=width;
  in.raw_boost_ms=0;in.local_turn_confirmed=s.local_turn_confirmed;in.local_returning_confirmed=s.local_returning_confirmed;
  in.prediction_source=static_cast<foot_support_policy::PredictionSource>(s.source);
  in.inward_decelerating=foot_support_deceleration_.update(status_.run_id,in.peak_id,t,-rate*side);
  const bool left=s.pose.left-s.pose.left_uncertainty>=s.pose.right-s.pose.right_uncertainty;
  in.anticipated_shape_need=(left?s.pose.phase_left:s.pose.phase_right)==foot_phase_prediction::Phase::Returning &&
      std::fabs(s.pose.left-s.pose.right)-s.pose.left_uncertainty-s.pose.right_uncertainty>=1 &&
      energy_control_autonomous_last_peak_amplitude_deg_<=8.5f;
  foot_support_policy::Limits limits;limits.allow_amount_boost=false;
  const auto timing=foot_support_policy::decide(in,limits);
  ++foot_support_reason_counts_[static_cast<unsigned>(timing.reason)];
  if(!timing.selected)return;
  FootSupportAudit a;a.time_ms=t;a.peak_ms=in.peak_id;a.sequence=foot_pose_sequence_;a.frame_ms=in.frame_ms;
  a.left=in.left_deg;a.right=in.right_deg;a.left_uncertainty=in.left_uncertainty_deg;a.right_uncertainty=in.right_uncertainty_deg;
  a.confidence=s.pose.confidence;a.body_angle=angle;a.body_rate=rate;a.threshold=timing.threshold_deg;
  a.estimated_zero_ms=timing.estimated_zero_ms;a.normal_width=preview.width_ms;a.normal_guarded_width=width;
  a.requested_width=preview.width_ms;a.boosted_guarded_width=width;a.next_side=-side;
  a.prediction_source=static_cast<uint8_t>(s.source);a.local_turn=s.local_turn_confirmed;a.local_returning=s.local_returning_confirmed;
  a.local_evidence=s.local_evidence;a.mode=static_cast<uint8_t>(timing.mode);a.candidate_mode=a.mode;
  a.stall_route=timing.stall_route;a.anticipated_shape_need=in.anticipated_shape_need;a.inward_decelerating=in.inward_decelerating;
  a.preview_speed_rpm=telemetry.speed_rpm;
  foot_support_active_audit_=-1;++foot_support_audit_total_;
  if(foot_support_audit_count_<96){foot_support_active_audit_=foot_support_audit_count_++;foot_support_audit_[foot_support_active_audit_]=a;}
  foot_support_source_=s.source;foot_support_input_=in;foot_support_decision_=timing;foot_support_sample_us_=sample;
  foot_support_event_=true;const auto before=status_.pulse_id;acceptOptionalEarlyMotion(t,rate,angle);
  if(foot_support_active_audit_>=0){auto& f=foot_support_audit_[foot_support_active_audit_];
    f.executed=status_.pulse_id!=before && status_.pulse_active;if(f.executed)f.backend_reason=9;}
  foot_support_event_=false;foot_support_active_audit_=-1;
}

void ExperimentRunner::appendWalkingEnergyMetadata(PsramString& json) const {
  auto n=[](float x){return std::isfinite(x)?String(x,9):String("null");};
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
  json+=",\"walking_energy\":{\"revision\":\"energy_pulse_lifecycle_v16\",\"output_enabled\":true,\"branch\":21,";
  json+="\"selector_revision\":\"walking_early_energy_main_v15\",\"completion_revision\":\"actual_elapsed_pulse_completion_v16\",\"timer_clock\":\"fresh_millis_after_motion_and_enqueue\",\"completion_reason_names\":[\"not_observed\",\"timer\",\"interrupted\",\"estop\",\"run_end\"],";
  json+="\"elapsed_semantics\":\"software_queue_accept_to_stop_request_not_physical_current_edges\",\"completion_independent_of_log_capacity\":true,";
#else
  json+=",\"walking_energy\":{\"revision\":\"walking_early_energy_main_v15\",\"output_enabled\":true,\"branch\":20,";
#endif
  json+="\"width_selection\":\"minimum_integer_predicted_work_reaches_request\",\"early_admission\":\"positive_budget_fulfilled_after_guard;otherwise_no_transport_and_keep_ordinary_opportunity\",\"generic_early_15ms_cap_active\":false,";
  json+="\"main_ceiling_ms\":80,\"main_ceiling_basis\":\"inherited_normal_experimental_envelope_not_physics_derived\",\"rescue_ceiling_ms\":15,\"rescue_ceiling_basis\":\"separate_legacy_low_speed_trial_not_derived\",\"online_reversal_cutoff_active\":false,";
  json+="\"energy_fulfillment_scope\":\"predicted_work_only;model_error_and_unmodelled_motion_can_leave_actual_deficit\",\"record_prediction_semantics\":\"postguard_candidate;output_predictions_are_zero_when_unsent\",";
#else
  json+=",\"walking_energy\":{\"revision\":\"walking_peak_rate_energy_v14\",\"output_enabled\":true,\"branch\":19,";
#endif
  json+="\"origin\":\"nominal_zero_CG_rocker_U_same_for_current_peak_target\",\"physical_body_inertia_identified\":false,\"MPC_active\":false,";
  json+="\"mechanical_reference_deg\":0,\"energy_neutral_model\":\"nominal_only;moving_foot_potential_minimum_not_yet_identified\",\"paired_feedback_is_energy_neutral_estimation\":false,";
  json+="\"paired_semantics\":\"successive_peak_mean_and_difference_not_two_simultaneous_energy_stores\",\"foot_role\":\"timing_only_no_ms_boost_no_forced_coast_override\",";
  json+="\"startup_exception\":\"fixed_start_kick_only_then_all_widths_energy_selected\",\"work_model\":\"inherited_empirical_pulse_Q_arrival_transfer_provisional\",\"coast_residual_modelled\":false,";
  json+="\"common_gain\":0.25,\"redistribution_gain\":0.125,\"each_correction_limit_target_fraction\":0.125,\"intercept_rolloff\":\"min(1,rate_dps/20)^2\",\"incoming_peak_max_age_ms\":1500,\"same_side_peak_max_age_ms\":3000,";
  json+="\"passive_coefficients\":[";
  for(int side=1;side>=-1;side-=2){if(side<0)json+=",";const auto c=walking_energy_policy::passiveCoefficients(side);
    json+="{\"side\":"+String(side)+",\"offset_j\":"+n(c.offset_j)+",\"omega2_j\":"+n(c.omega2_j)+",\"rate_min_dps\":"+n(c.rate_min_dps)+",\"rate_max_dps\":"+n(c.rate_max_dps)+"}";}
  json+="],\"records\":";
  CompactJsonTable<PsramString> table(json);uint32_t stored=0;
  for(uint32_t i=0;i<walking_energy_count_;++i){
    if(json.length()+4096U>=640U*1024U*8U/10U)break;
    const auto& a=walking_energy_audit_[i];const auto& d=a.decision;
    String row="{\"time_ms\":"+String(a.time_ms)+",\"peak_ms\":"+String(a.peak_ms)+",\"side\":"+String(a.side);
    row+=",\"output_start_ms\":"+String(a.output_start_ms);
    row+=",\"requested_ms\":"+String(a.requested_ms)+",\"guarded_ms\":"+String(a.guarded_ms);
    row+=",\"executed\":"+String(a.executed?"true":"false")+",\"optional\":"+String(a.optional?"true":"false")+",\"rescue\":"+String(a.rescue?"true":"false");
    row+=",\"early_budget_checked\":"+String(a.early_budget_checked?"true":"false")+",\"early_budget_admitted\":"+String(a.early_budget_admitted?"true":"false");
    row+=",\"valid\":"+String(d.valid?"true":"false")+",\"coast\":"+String(d.coast?"true":"false")+",\"saturated\":"+String(d.saturated?"true":"false")+",\"extrapolated\":"+String(d.extrapolated?"true":"false");
    row+=",\"potential_j\":"+n(d.potential_j)+",\"effective_rate_energy_j\":"+n(d.effective_kinetic_j)+",\"fitted_offset_j\":"+n(d.fitted_offset_j);
    row+=",\"incoming_peak_energy_j\":"+n(d.peak_energy_j)+",\"same_side_peak_energy_j\":"+n(d.same_side_peak_energy_j);
    row+=",\"mean_peak_energy_j\":"+n(d.mean_peak_energy_j)+",\"side_imbalance_j\":"+n(d.side_imbalance_j);
    row+=",\"common_correction_j\":"+n(d.common_correction_j)+",\"redistribution_j\":"+n(d.redistribution_j);
    row+=",\"available_j\":"+n(d.available_j)+",\"target_j\":"+n(d.target_j)+",\"deficit_j\":"+n(d.deficit_j)+",\"request_j\":"+n(d.request_j);
    row+=",\"q_mas\":"+n(d.q_mas)+",\"predicted_work_j\":"+n(d.predicted_work_j)+",\"predicted_arrival_j\":"+n(d.predicted_arrival_j)+",\"unmet_j\":"+n(d.unmet_j);
    row+=",\"output_q_pred_mas\":"+n(a.executed?d.q_mas:0.f)+",\"output_work_pred_j\":"+n(a.executed?d.predicted_work_j:0.f)+",\"unmet_after_command_j\":"+n(a.executed?d.unmet_j:d.request_j);
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
    row+=",\"completion_reason\":"+String(a.completion_reason)+",\"output_end_ms\":"+String(a.output_end_ms);
    row+=",\"actual_elapsed_ms\":"+String(a.completion.actual_elapsed_ms)+",\"planned_duration_completed\":"+String(a.completion.duration_completed?"true":"false");
    row+=",\"completion_valid\":"+String(a.completion.valid?"true":"false")+",\"completion_work_valid\":"+String(a.completion.work_valid?"true":"false");
    row+=",\"start_i0_ma\":"+n(a.start_i0_ma)+",\"start_goal_ma\":"+n(a.start_goal_ma)+",\"start_tau_s\":"+n(a.start_tau_s)+",\"start_rate_dps\":"+n(a.start_rate_dps);
    row+=",\"elapsed_q_pred_mas\":"+n(a.completion.q_mas)+",\"elapsed_current_end_pred_ma\":"+n(a.completion.current_end_ma);
    row+=",\"elapsed_work_pred_j\":"+n(a.completion.work_j)+",\"elapsed_arrival_pred_j\":"+n(a.completion.arrival_j)+",\"elapsed_unmet_pred_j\":"+n(a.completion.unmet_j);
    row+=",\"completion_width_supported\":"+String(a.completion.width_supported?"true":"false")+",\"completion_extrapolated\":"+String(a.completion.extrapolated?"true":"false");
#endif
    row+=",\"passive_rate_supported\":"+String(d.passive_rate_supported?"true":"false")+",\"work_rate_supported\":"+String(d.work_rate_supported?"true":"false")+",\"charge_supported\":"+String(d.charge_supported?"true":"false");
    row+=",\"passive_intercept_rolloff\":"+String(d.passive_intercept_rolloff?"true":"false")+",\"early_state_transfer\":"+String(d.early_state_transfer?"true":"false");
    row+=",\"coast_residual_q_mas\":"+n(d.coast_residual_q_mas)+"}";
    if(!table.append(row)){json.fail();return;}++stored;
  }
  if(!table.finish()){json.fail();return;}
  json+=",\"stored\":"+String(stored)+",\"total\":"+String(walking_energy_total_)+",\"truncated\":"+String(stored<walking_energy_total_?"true":"false")+"}";
}
#endif
