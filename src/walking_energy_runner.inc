// Shared by the separately identified v14 and v15 energy builds.
#if defined(FOOT_SUPPORT_ENERGY) && FOOT_SUPPORT_ENERGY
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
bool ExperimentRunner::walkingForeAftAllowsOutput()const{
  return walking_fore_aft_last_.allow_output && walking_fore_aft_last_.valid &&
      walking_fore_aft_last_.sample_us && static_cast<uint32_t>(micros()-walking_fore_aft_last_.sample_us)<=
      fore_aft_motion_guard::limits::kMaximumSampleAgeUs;
}
bool ExperimentRunner::observeWalkingForeAft(const fore_aft_motion_guard::Input& in){
  const auto before=walking_fore_aft_last_.reason;
  walking_fore_aft_last_=walking_fore_aft_guard_.observe(in);
  if(walking_fore_aft_last_.reason==fore_aft_motion_guard::Reason::Candidate &&
      before!=fore_aft_motion_guard::Reason::Candidate)++walking_fore_aft_candidates_;
  if(!walking_fore_aft_last_.stop)return false;
  energy_control_autonomous_phase_=EnergyControlAutonomousPhase::STOP;
  requestEmergencyStop(fore_aft_motion_guard::reasonName(walking_fore_aft_last_.reason));
  return true;
}
#endif
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
bool ExperimentRunner::cutReturnAidAtZero(uint32_t now_ms,uint32_t sample_us,
    float before,float after,float rate){
  const int side=energy_control_autonomous_last_peak_side_;
  if(!wheelWorkMode() || energy_control_autonomous_target_peak_deg_!=8.f ||
      energy_control_autonomous_input_peak_percent_!=0 || status_.emergency_stop ||
      !energy_control_autonomous_last_peak_valid_ || !status_.pulse_active ||
      !walking_energy_completion_pending_ || !walking_energy_active_pulse_.return_event ||
      !walking_half_cycle_budget_.active() ||
      !std::isfinite(before) || !std::isfinite(after) || !std::isfinite(rate) ||
      (side!=1 && side!=-1))return false;
  const bool centre=rate*side<0 && ((side>0 && before>0 && after<=0) || (side<0 && before<0 && after>=0));
  uint8_t stop_reason=5;
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
  // If a corrected pulse cannot avert reversal, further leftward reaction
  // torque on a body already moving right no longer adds leftward motion.
  const auto& pulse=walking_energy_active_pulse_;
  const auto evidence=static_cast<return_observation_policy::Evidence>(pulse.return_evidence);
  const bool reversal_cut=return_observation_policy::isPostAid(evidence) || pulse.arc_priority.cap_ms>0;
  const bool reversed=reversal_cut && side==1 &&
      before>0 && after>=before && rate>0 && energy_control_autonomous_previous_detector_rate_dps_>0;
  if (!centre && !reversed) {
    const uint32_t t=now_ms-run_start_ms_;
    if (evidence!=return_observation_policy::Evidence::ContinuationShortfall || side!=1 || t<pulse.output_start_ms ||
        !walking_post_aid_.releaseOnCarry(sample_us,static_cast<uint32_t>(micros()-sample_us),
            t-pulse.output_start_ms,after,rate,pulse.predicted_terminal_deg,
            pulse.post_release_deceleration_dps2)) return false;
    stop_reason=7;
  } else if (reversed) stop_reason=6;
#else
  if (!centre) return false;
#endif
  // If a passage cannot be used because its proof is stale/gapped, do not
  // later wait forever for that already-passed ZERO or issue a late main.
  if (centre) walking_aid_zero_seen_=true;
  if(!sample_us || static_cast<uint32_t>(micros()-sample_us)>10000u)return false;
  const uint32_t t=now_ms-run_start_ms_;
  if(t<energy_control_autonomous_previous_detector_test_ms_ ||
      t-energy_control_autonomous_previous_detector_test_ms_>10u)return false;
  // Stop at this fresh observed crossing, then let this very sample reach
  // normal ZERO dispatch. No deferred/backdated command or lost passage.
  walking_energy_stop_reason_=stop_reason;stopMotor();
  status_.current_mA_setting=0;status_.pulse_width_ms_setting=0;
  energy_control_autonomous_pulse_authorized_=false;
  energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
  resetEnergyControlAutonomousPeakTracker(false);
  return centre;
}
#endif
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
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
  if (return_observation_policy::isPostAid(static_cast<return_observation_policy::Evidence>(a.return_evidence)) &&
      a.completion.work_valid) {
    a.post_elapsed_increment_j=a.completion.work_j-half_cycle_energy_budget::postAidCoastWork(
        a.start_rate_dps,a.start_i0_ma,in.actual_elapsed_ms);
    a.completion.unmet_j=std::fmax(0.f,a.decision.request_j-a.post_elapsed_increment_j);
  }
#endif
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  // Active accounting runs even when the PSRAM diagnostic archive is full.
  walking_half_cycle_budget_.complete(in.actual_elapsed_ms);
  a.envelope_after_ms=walking_half_cycle_budget_.remaining();
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
  if (isLeftTransferCarryProfile() && a.return_event && a.side==-1 && reason==1 &&
      !return_observation_policy::isPostAid(static_cast<return_observation_policy::Evidence>(a.return_evidence)) &&
      a.completion.duration_completed && a.completion.work_valid && a.completion.charge_supported)
    walking_post_aid_.arm(a.peak_ms,a.output_end_ms,a.start_rate_dps,a.completion.work_j);
#endif
#if SUPPORT_TRANSFER_CARRY2
  if (!a.return_event && reason==1 && a.completion.valid)
    walking_main_reserve_.observe(a.side,a.assisted_cycle,in.actual_elapsed_ms);
#if WALKING_RETURN_HISTORY
  if (carryHistoryTrialSelected() && a.return_event && a.side==-1)
    walking_return_history_.completedAid(a.history_state,a.output_end_ms,in.actual_elapsed_ms,
        reason==1 && a.completion.duration_completed && a.completion.valid &&
        a.completion.work_valid && a.completion.charge_supported);
#endif
#if SUPPORT_TRANSFER_ARC_FEEDBACK
  if (a.return_event) {
    // Every executed aid consumes the previous credit, including an interrupted aid.
    const bool observed=a.side==-1 && a.arc_aid.reason==receiving_arc_aid::Reason::Applied &&
        reason==1 && a.completion.work_valid && a.completion.charge_supported;
    walking_arc_feedback_.completedAid(a.peak_ms,a.output_end_ms,a.arc_aid.terminal_deg,
        in.actual_elapsed_ms,observed?a.completion.work_j:0.f);
  }
#endif
#endif
#endif
  if(index>=0 && static_cast<uint32_t>(index)<walking_energy_count_
#if defined(WALKING_AUDIT_PSRAM) && WALKING_AUDIT_PSRAM
      && walking_energy_audit_.available()
#endif
      )
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
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  namespace energy=walking_return_energy;
#elif defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  namespace energy=walking_energy_policy_v15;
#else
  namespace energy=walking_energy_policy;
#endif
  const int side=e.physical_next_peak_side;
#if WALKING_SMALL_STEP_AB
  const bool small_step=smallStepBaselineSelected();
#else
  const bool small_step=false;
#endif
  const bool healthy=wheelWorkMode() && e.target_peak_deg==8.f &&
      (side==1 || side==-1) && !status_.emergency_stop && !status_.pulse_active &&
      status_.state==ExperimentState::RUNNING_BATCH_SWEEP && roller_ && roller_->ok() &&
      imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis()) &&
      !e.side_mismatch_diagnostic && e.vbat_mV>0 && e.pre_input_wheel_speed_valid &&
      e.pre_input_wheel_speed_age_us<=50000 && std::isfinite(e.pre_input_wheel_speed_rpm);
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  const bool motion_allowed=walkingForeAftAllowsOutput();
#endif
  if(healthy_out)*healthy_out=healthy;
  noteFootSupportBackend(healthy?2:1);
  e.work_control_branch=
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
      small_step?19:24;
#elif defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
      23;
#elif defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
      22;
#elif defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
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
#if FOOT_PERIODIC_V38
  a.periodic_packet=periodic_foot_packet_.count;
#endif
  a.side=side;a.optional=optional_early_event_;a.rescue=stall_rescue_event_;
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  a.fore_aft_allowed=motion_allowed;a.fore_aft_angle_deg=walking_fore_aft_last_.angle_deg;
  a.fore_aft_rate_dps=walking_fore_aft_last_.rate_dps;
  a.fore_aft_reason=static_cast<uint8_t>(walking_fore_aft_last_.reason);
#endif
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  const bool return_aid=walking_return_event_ && stall_rescue_event_;
  const bool post_aid=return_aid && return_observation_policy::isPostAid(walking_return_observation_.evidence);
  a.envelope_before_ms=walking_half_cycle_budget_.remaining();a.envelope_after_ms=a.envelope_before_ms;
  a.assisted_cycle=walking_half_cycle_budget_.aidUsed() || return_aid;
  a.return_evidence=return_aid?static_cast<uint8_t>(walking_return_observation_.evidence):0;
#if WALKING_RETURN_HISTORY
  if (return_aid && returnPredictionTrialSelected()) {
    // A guard below can veto the command before energy inversion. Preserve the observation.
    a.return_event=true;
    a.return_work_j=walking_return_observation_.request_j;
    a.return_foot_age_ms=walking_return_observation_.foot_age_ms;
    a.predicted_transfer_ms=walking_return_observation_.predicted_transfer_ms;
    a.predicted_terminal_deg=walking_return_observation_.terminal_deg;
  }
#endif
#endif
  auto save=[&](){++walking_energy_total_;int16_t stored_index=-1;
#if SUPPORT_TRANSFER_CARRY2
    if (!a.executed && a.arc_aid.reason==receiving_arc_aid::Reason::Applied)
      a.arc_aid.reason=receiving_arc_aid::Reason::CommandVeto;
#endif
    if(walking_energy_count_<192
#if defined(WALKING_AUDIT_PSRAM) && WALKING_AUDIT_PSRAM
        && walking_energy_audit_.available()
#endif
        ){stored_index=static_cast<int16_t>(walking_energy_count_);
      walking_energy_audit_[walking_energy_count_++]=a;}
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
    if(a.executed){walking_energy_active_audit_=stored_index;walking_energy_active_pulse_=a;
      walking_energy_completion_pending_=true;}
#else
    (void)stored_index;
#endif
  };
  if(!healthy || (small_step && (stall_rescue_event_ || foot_support_event_))){save();return true;}
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  if(!motion_allowed){save();return true;}
#endif
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  if(post_aid?!walking_half_cycle_budget_.allowPostAid(a.peak_ms):
      return_aid?!walking_half_cycle_budget_.allowAid(a.peak_ms):
      !walking_half_cycle_budget_.allowMain(a.peak_ms)){save();return true;}
#endif
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  const uint32_t decision_sample_us=foot_support_event_?foot_support_sample_us_:
      energy_control_autonomous_last_motion_sample_us_;
  auto early_fresh=[&](){const uint32_t now=millis()-run_start_ms_;
    return now>=t && now-t<=5 && decision_sample_us &&
        static_cast<uint32_t>(micros()-decision_sample_us)<=10000;};
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  const bool freshness_required=true;
#else
  const bool freshness_required=
      optional_early_event_
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
      || stall_rescue_event_
#endif
      ;
#endif
  if(freshness_required && !early_fresh()){save();return true;}
#endif
  if(!e.previous_peak_time_ms || t<e.previous_peak_time_ms || t-e.previous_peak_time_ms>1500){save();return true;}
  energy::Input in;in.side=side;in.rate_dps=e.zero_cross_abs_rate_dps;
  in.body_angle_deg=(optional_early_event_ || stall_rescue_event_)?e.detector_angle_after_deg:0.f;
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  in.body_angle_deg=e.detector_angle_after_deg;
#endif
  in.target_deg=tilt_priority_.target();in.previous_peak_deg=e.previous_peak_amplitude_deg;
  const unsigned index=side>0?1:0;
  in.same_side_peak_deg=walking_energy_peak_ms_[index] && t>=walking_energy_peak_ms_[index] &&
      t-walking_energy_peak_ms_[index]<=3000?walking_energy_peaks_[index]:NAN;
  in.initial_current_ma=side*e.i0_estimated_mA;
  in.goal_current_ma=predictCurrentGoalMa(1200,e.vbat_mV*.001f);
  in.cap_ms=tilt_priority_.timingCap(side,t,80); // timing protection only
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  in.cap_ms=walking_half_cycle_budget_.cap(a.peak_ms,in.cap_ms);
  if(small_step){
    if(optional_early_event_ && in.cap_ms>15)in.cap_ms=15;
    if(!optional_early_event_)in.body_angle_deg=0.f;  // v14 evaluated ordinary input at mechanical ZERO.
  }
#if SUPPORT_TRANSFER_STALL_V3
  const auto evidence=walking_return_observation_.evidence;
  if (post_aid) {
    in.initial_current_ma=walking_return_observation_.aligned_current_ma;
#if WALKING_RETURN_HISTORY
    a.post_release_deceleration_dps2=walking_return_observation_.release_deceleration_dps2;
#endif
    if (in.cap_ms>walking_return_observation_.loss_cap_ms) in.cap_ms=walking_return_observation_.loss_cap_ms;
    a.post_cap_ms=in.cap_ms;
    if (!in.cap_ms || !std::isfinite(in.initial_current_ma)) {save();return true;}
  }
  const bool reserve_for_arc_aid=return_aid && (evidence==return_observation_policy::Evidence::MeasuredArcShortfall ||
                                                evidence==return_observation_policy::Evidence::PredictedArcShortfall);
#endif
#endif
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  in.arrival_bias_j=small_step?0.f:walking_arrival_feedback_.bias(side,t,lateral_energy::potential(lateral_energy_geometry_,in.target_deg));
  in.return_work_j=walking_return_event_?walking_return_choice_.request_j:0;
  a.arrival_bias_j=in.arrival_bias_j;a.return_work_j=in.return_work_j;a.return_event=walking_return_event_;
  if(walking_return_event_){a.return_deceleration_dps2=walking_return_choice_.deceleration_dps2;
    a.return_stopping_distance_deg=walking_return_choice_.stopping_distance_deg;
    a.return_foot_age_ms=
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
        walking_return_observation_.foot_age_ms;
#else
        t-foot_local_pose_.frame_ms;
#endif
  }
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  if(return_aid){in.arrival_bias_j=0;a.arrival_bias_j=0;}
#if SUPPORT_TRANSFER_STALL_V3
  if (reserve_for_arc_aid) {
    a.main_reserve_ms=walking_main_reserve_.forSide(side);
    uint16_t cap=in.cap_ms>a.main_reserve_ms ? in.cap_ms-a.main_reserve_ms : 0;
#if WALKING_RETURN_HISTORY
    if (isTransferPriorityProfile() && evidence==return_observation_policy::Evidence::MeasuredArcShortfall) {
      a.arc_priority=return_crossing_work::prioritizeMeasuredArc(lateral_energy_geometry_,walking_energy_current_,
          in,walking_return_observation_.previous_arc_deceleration_dps2,a.return_deceleration_dps2,
          a.main_reserve_ms,e.pre_input_wheel_speed_rpm,t,wheel_cycle_guard_);
      if (a.arc_priority.cap_ms) cap=a.arc_priority.cap_ms;
    }
#endif
    in.cap_ms=cap;
  }
  if (reserve_for_arc_aid && !in.cap_ms) {
    a.arc_aid.reason=receiving_arc_aid::Reason::NoMainRoom;
    save();return true;
  }
#endif
#endif
#elif defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  // Main-input relocation gets the same envelope as normal ZERO. The
  // separate low-speed rescue trial retains its old, provisional ceiling.
  if(stall_rescue_event_)
#else
  if((optional_early_event_ && !foot_support_event_) || stall_rescue_event_)
#endif
#if !defined(WALKING_RETURN_ENERGY) || !WALKING_RETURN_ENERGY
    if(in.cap_ms>15)in.cap_ms=15;
#endif
  if(foot_support_event_){
    noteFootSupportBackend(4);
    const uint32_t now=millis()-run_start_ms_;
    if(now<t || now-t>5 || static_cast<uint32_t>(micros()-foot_support_sample_us_)>10000){save();return true;}
    in.body_angle_deg+=e.zero_cross_rate_dps*(now-t)*.001f;
  }
  // For an exhausted envelope, still calculate/audit the ordinary J demand,
  // but evaluate zero output below. Never manufacture an energy-surplus coast.
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  const bool exhausted=in.cap_ms==0;
  auto calculation=in;if(exhausted)calculation.cap_ms=1;
  auto selected=small_step?walking_energy_policy::decide(lateral_energy_geometry_,walking_energy_current_,calculation):
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
      post_aid?half_cycle_energy_budget::selectPostAid(lateral_energy_geometry_,walking_energy_current_,calculation,
          walking_return_observation_.gross_work_limit_j):
#endif
      return_aid?half_cycle_energy_budget::decideReturnAid(lateral_energy_geometry_,walking_energy_current_,calculation):
      energy::decide(lateral_energy_geometry_,walking_energy_current_,calculation);
#else
  const auto selected=energy::decide(lateral_energy_geometry_,walking_energy_current_,in);
#endif
  a.decision=selected;a.requested_ms=selected.width_ms;
  if(!selected.valid){noteFootSupportBackend(6);save();return true;}
  auto guard=wheel_cycle_guard_;
  auto guard_input=selected.width_ms;
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
  // For a continuation, the wheel guard bounds the remaining actuator
  // capacity. Select the requested energy inside that capacity: applying
  // the attenuation to the minimum inverse width itself can leave a known
  // shortage even when the bounded wheel command could cover it.
  if (post_aid && evidence==return_observation_policy::Evidence::ContinuationShortfall)
    guard_input=calculation.cap_ms;
#endif
  uint16_t width=
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
      exhausted?0:
#endif
      selected.coast?0:guard.limit(side,e.pre_input_wheel_speed_rpm,true,t,guard_input);
  if (width>selected.width_ms) width=selected.width_ms;
#if SUPPORT_TRANSFER_CARRY2
  if (return_aid && !post_aid && side==-1 && !exhausted && width>0 && energy_control_autonomous_target_peak_deg_==8.f &&
      energy_control_autonomous_input_peak_percent_==0) {
    const auto& packet=periodic_foot_packet_;
    const uint64_t now_us=esp_timer_get_time();
    receiving_arc_aid::Input arc;
    arc.side=side;arc.body_deg=in.body_angle_deg;arc.speed_dps=in.rate_dps;
    arc.decel_dps2=a.return_deceleration_dps2;arc.fore_aft_deg=a.fore_aft_angle_deg;
    const auto& raw=walking_return_frame_;
    auto left=packet.left,right=packet.right;
    if (walking_return_observation_.evidence==return_observation_policy::Evidence::ObservedShape &&
        walking_return_observation_.foot_age_ms<=periodic_foot::maximum_age_us/1000 &&
        raw.run_id && raw.run_id==status_.run_id) {
      if (periodic_foot::matchesPair(packet,now_us,raw.sequence,raw.image_us)) a.arc_foot_source=1;
      else if (periodicFeet.rescuePair(raw.run_id,now_us,packet.epoch_us,
                   raw.sequence,raw.image_us,raw.received_us,left,right)) a.arc_foot_source=2;
    }
    arc.observed_shape=a.arc_foot_source!=0;
    arc.left_deg=raw.left_deg;arc.right_deg=raw.right_deg;
    arc.left_est=left.estimated;arc.right_est=right.estimated;
    arc.left_rate=left.rate_dps;arc.right_rate=right.rate_dps;
    arc.base_j=selected.request_j;arc.target_j=selected.target_j;
#if SUPPORT_TRANSFER_STALL_V3
    arc.base_width_ms=width;
#endif
    a.arc_aid=receiving_arc_aid::evaluate(arc);
    a.base_aid_ms=a.arc_candidate_ms=width;
    a.main_reserve_ms=walking_main_reserve_.forSide(side);
    if (a.arc_aid.reason==receiving_arc_aid::Reason::Eligible) {
      auto candidate_input=calculation;
      candidate_input.return_work_j=arc.base_j+a.arc_aid.extra_j;
      const auto candidate=half_cycle_energy_budget::decideReturnAid(
          lateral_energy_geometry_,walking_energy_current_,candidate_input);
      if (candidate.valid && !candidate.coast && candidate.width_ms>=selected.width_ms) {
        auto candidate_guard=wheel_cycle_guard_; // Compare both widths against the same pre-command state.
        a.arc_candidate_ms=candidate_guard.limit(side,e.pre_input_wheel_speed_rpm,true,t,candidate.width_ms);
        const uint16_t limited=receiving_arc_aid::boundWidth(width,a.arc_candidate_ms,a.envelope_before_ms,a.main_reserve_ms);
        if (limited>width) {
          calculation=candidate_input;selected=candidate;guard=candidate_guard;width=limited;
          a.requested_ms=selected.width_ms;
          a.arc_aid.reason=receiving_arc_aid::Reason::Applied;
        }
      }
      if (a.arc_aid.reason==receiving_arc_aid::Reason::Eligible)
        a.arc_aid.reason=a.arc_candidate_ms>width ? receiving_arc_aid::Reason::NoMainRoom : receiving_arc_aid::Reason::NoWidthGain;
#if SUPPORT_TRANSFER_ARC_FEEDBACK
      // The present foot geometry must already justify a real arc extension.
      // Its ordinary wheel limit and the later main-input reserve still apply.
      const auto* credit=walking_arc_feedback_.recent(t);
      if (credit && a.arc_aid.reason==receiving_arc_aid::Reason::Applied) {
        a.arc_feedback_peak_ms=credit->peak_ms;a.arc_feedback_base_ms=width;
        a.arc_feedback_available_j=credit->missing_j;
        auto adjusted=calculation;
        adjusted.return_work_j=std::fmin(.5f*selected.target_j,calculation.return_work_j+credit->missing_j);
        const auto inverse=half_cycle_energy_budget::decideReturnAid(lateral_energy_geometry_,walking_energy_current_,adjusted);
        if (inverse.valid && inverse.width_ms>selected.width_ms) {
          auto limited_guard=wheel_cycle_guard_;
          const auto guarded=limited_guard.limit(side,e.pre_input_wheel_speed_rpm,true,t,inverse.width_ms);
          const uint16_t limited=receiving_arc_aid::boundWidth(width,guarded,a.envelope_before_ms,a.main_reserve_ms);
          if (limited>width && limited<=credit->elapsed_ms) {
            const auto checked=half_cycle_energy_budget::evaluateReturnAidWidth(
                lateral_energy_geometry_,walking_energy_current_,adjusted,limited);
            if (checked.valid && checked.charge_supported && checked.predicted_work_j>0 &&
                checked.predicted_work_j<=credit->previous_work_j) {
              a.arc_feedback_added_j=adjusted.return_work_j-calculation.return_work_j;
              calculation=adjusted;selected=inverse;guard=limited_guard;width=limited;
              a.requested_ms=selected.width_ms;
            }
          }
        }
      }
#endif
#if WALKING_RETURN_HISTORY
      // Enter here only after the same measured-foot CAD pivot certificate
      // used by the existing aid. A reserved main may have limited its width.
      if (carryHistoryTrialSelected()) {
        a.history_state={a.peak_ms,a.return_foot_age_ms,arc.body_deg,arc.speed_dps,arc.left_deg,
            arc.right_deg,arc.fore_aft_deg,arc.decel_dps2,a.arc_aid.terminal_deg,arc.target_j};
        a.history=walking_return_history_.compare(a.history_state,t);
        a.history_base_ms=a.history_final_ms=width;
        if (a.history.request_j>selected.request_j && a.history.limit_ms>width) {
          auto adjusted=calculation;
          adjusted.return_work_j=a.history.request_j;
          const auto inverse=half_cycle_energy_budget::decideReturnAid(
              lateral_energy_geometry_,walking_energy_current_,adjusted);
          if (inverse.valid && inverse.width_ms>width) {
            auto candidate_guard=wheel_cycle_guard_;
            const auto guarded=candidate_guard.limit(side,e.pre_input_wheel_speed_rpm,true,t,inverse.width_ms);
            auto limited=receiving_arc_aid::boundWidth(width,guarded,a.envelope_before_ms,a.main_reserve_ms);
            if (limited>a.history.limit_ms) limited=a.history.limit_ms;
            if (limited>width) {
              const auto check=half_cycle_energy_budget::evaluateReturnAidWidth(
                  lateral_energy_geometry_,walking_energy_current_,adjusted,limited);
              if (check.valid && check.charge_supported && check.predicted_work_j>0 &&
                  check.predicted_work_j<=.5f*check.target_j) {
                calculation=adjusted;selected=inverse;guard=candidate_guard;width=limited;
                a.requested_ms=selected.width_ms;a.history_final_ms=width;
              }
            }
          }
        }
      }
      if (returnPredictionTrialSelected() && a.arc_aid.reason==receiving_arc_aid::Reason::Applied) {
        const uint16_t history_cap=a.history_final_ms>a.history_base_ms ? a.history.limit_ms : UINT16_MAX;
        a.crossing_work=return_crossing_work::apply(walking_crossing_reference_,a.peak_ms,t,
            arc.decel_dps2,a.envelope_before_ms,a.main_reserve_ms,e.pre_input_wheel_speed_rpm,
            wheel_cycle_guard_,guard,lateral_energy_geometry_,walking_energy_current_,calculation,
            selected,width,history_cap);
        a.requested_ms=selected.width_ms;
        e.wheel_guard_legacy_ms=guard.legacyWidth();e.wheel_guard_extra_ms=guard.extraReduction();
        e.wheel_guard_trial_eligible=guard.trialEligible();
      }
#endif
    }
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
    // The measured sole is still higher, but the extra current can arrive
    // during its transition. Forecast sizing has its own audit; never mark
    // the measured-pivot certificate as fulfilled based on a prediction.
    if (isLeftTransferCarryProfile() && a.arc_aid.reason==receiving_arc_aid::Reason::NotReceiving) {
      a.crossing_work=return_crossing_work::applyApproach(arc,walking_crossing_reference_,
          a.peak_ms,t,a.envelope_before_ms,a.main_reserve_ms,e.pre_input_wheel_speed_rpm,
          wheel_cycle_guard_,guard,lateral_energy_geometry_,walking_energy_current_,calculation,selected,width,
          a.fore_aft_rate_dps);
      if (width==a.base_aid_ms) {
        const auto passage=return_crossing_work::applyPassage(arc,walking_main_passages_,a.peak_ms,t,
            a.envelope_before_ms,e.pre_input_wheel_speed_rpm,wheel_cycle_guard_,guard,
            lateral_energy_geometry_,walking_energy_current_,calculation,selected,width);
        if (passage.final_ms) a.crossing_work=passage;
      }
      a.requested_ms=selected.width_ms;
      if (a.crossing_work.final_ms>a.crossing_work.base_ms) {
        e.wheel_guard_legacy_ms=guard.legacyWidth();e.wheel_guard_extra_ms=guard.extraReduction();
        e.wheel_guard_trial_eligible=guard.trialEligible();
      }
    }
#endif
  }
#endif
  auto actual=
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
      small_step?walking_energy_policy::evaluateWidth(lateral_energy_geometry_,walking_energy_current_,calculation,width):
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
      post_aid?half_cycle_energy_budget::selectPostAid(lateral_energy_geometry_,walking_energy_current_,calculation,
          walking_return_observation_.gross_work_limit_j,width):
#endif
      return_aid?half_cycle_energy_budget::evaluateReturnAidWidth(lateral_energy_geometry_,walking_energy_current_,calculation,width):
      energy::evaluateWidth(lateral_energy_geometry_,walking_energy_current_,calculation,width);
#else
      energy::evaluateWidth(lateral_energy_geometry_,walking_energy_current_,in,width);
#endif
#if WALKING_RIGHT_ARRIVAL
  // Evaluate the new arrival fit against the unsteered, already wheel-limited
  // command. This fit was identified on positive, unassisted physical ZERO.
  bool right_startup_target=in.target_deg==4.5f;
#if WALKING_RETURN_HISTORY
  right_startup_target=right_startup_target || (isLeftTransferCarryProfile() && in.target_deg==5.f);
#endif
  if (!small_step && side==1 && e.event_kind==0 && !optional_early_event_ &&
      !stall_rescue_event_ && !foot_support_event_ && !return_aid && !a.assisted_cycle &&
      energy_control_autonomous_input_peak_percent_==0 && right_startup_target &&
      actual.valid && !actual.coast && width) {
    a.right_arrival_checked=true;
    a.right_base_ms=width;a.right_base_q=actual.q_mas;
    a.right_arrival=right_arrival_energy::limit(lateral_energy_geometry_,walking_energy_current_,
        in.rate_dps,in.previous_peak_deg,in.target_deg,in.initial_current_ma,in.goal_current_ma,width);
    if (a.right_arrival.eligible && a.right_arrival.width_ms<width) {
      width=a.right_arrival.width_ms;
      if (width) {
        actual=energy::evaluateWidth(lateral_energy_geometry_,walking_energy_current_,calculation,width);
      } else {
        // The arrival fit selects coast. Keep the inherited model's positive
        // unmet request visible alongside its zero newly-commanded work.
        guard=wheel_cycle_guard_;
        actual.width_ms=0;actual.q_mas=0;actual.predicted_work_j=0;
        actual.predicted_arrival_j=actual.available_j;actual.unmet_j=actual.request_j;
        actual.saturated=actual.request_j>0;actual.charge_supported=true;
        actual.coast=true;actual.reason=walking_energy_policy::Reason::Coast;
      }
    }
  }
#endif
#if WALKING_TURN_OBSERVER
  // Use the confirmed cycle only on an ordinary, already admissible motion
  // event. Both model inversions start with the same pre-command guard state.
  const auto& turn=gait_turn_run_.status();
  if (!small_step && e.event_kind==0 && !optional_early_event_ && !stall_rescue_event_ && !foot_support_event_ &&
      !walking_return_event_ && !a.assisted_cycle && !exhausted && width && !selected.coast &&
      energy_control_autonomous_target_peak_deg_==8.f && energy_control_autonomous_input_peak_percent_==0 &&
      turn.enabled && turn.latest.reason==gait_turn::Reason::Confirmed) {
    a.turn_cycle_end_ms=turn.latest.end_ms;
    a.turn_base_ms=a.turn_candidate_ms=width;
    const auto& raw=walking_return_frame_;
    const uint64_t now_us=esp_timer_get_time();
    const bool fresh=raw.run_id && raw.run_id==status_.run_id && raw.timestamp_valid &&
        raw.frame_valid && raw.zero_ready && raw.left_detected && raw.right_detected &&
        raw.left_range_ok && raw.right_range_ok && std::isfinite(raw.left_deg) && std::isfinite(raw.right_deg) &&
        raw.image_us && raw.image_us<=raw.received_us && raw.received_us<=now_us &&
        now_us-raw.image_us<=periodic_foot::maximum_age_us;
    if (fresh && status_.steering.gyro_valid && t>=turn.latest.end_ms &&
        t-turn.latest.end_ms<=turn.latest.end_ms-turn.latest.start_ms) {
      const float delta=gait_turn_energy::adjustment(turn,side,t,in,actual,lateral_energy_geometry_,walking_energy_current_);
      if (delta!=0.f) {
        auto adjusted=calculation;
        adjusted.turn_adjustment_j=delta;
        const auto candidate=energy::decide(lateral_energy_geometry_,walking_energy_current_,adjusted);
        if (candidate.valid && !candidate.coast) {
          auto candidate_guard=wheel_cycle_guard_;
          a.turn_candidate_ms=candidate_guard.limit(side,e.pre_input_wheel_speed_rpm,true,t,candidate.width_ms);
          const auto after=energy::evaluateWidth(lateral_energy_geometry_,walking_energy_current_,adjusted,a.turn_candidate_ms);
          if (
#if WALKING_RIGHT_ARRIVAL
              (!a.right_arrival_checked || !a.right_arrival.eligible || a.turn_candidate_ms<=a.right_arrival.width_ms) &&
#endif
              gait_turn_energy::admissible(actual,after,delta)) {
            calculation=adjusted;selected=candidate;guard=candidate_guard;width=a.turn_candidate_ms;actual=after;
            a.requested_ms=selected.width_ms;a.turn_adjustment_j=delta;
          }
        }
      }
    }
  }
#endif
#if WALKING_RIGHT_ARRIVAL
  if(a.right_arrival_checked && a.right_arrival.eligible && actual.valid)
    a.right_final_j=right_arrival_energy::predictedJ(in.rate_dps,actual.q_mas);
#endif
  a.guarded_ms=width;
  a.decision=actual;
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
  if (post_aid && (!actual.valid || !width || !actual.charge_supported || !(actual.predicted_work_j>0))) {
    save();return true;
  }
  if (post_aid) a.post_increment_j=actual.predicted_work_j-half_cycle_energy_budget::postAidCoastWork(
      in.rate_dps,in.initial_current_ma,width);
#endif
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  const auto scope=return_aid?energy_admission_v19::Scope::ExplicitReturnAid:
      !optional_early_event_?energy_admission_v19::Scope::OrdinaryZero:
      small_step?energy_admission_v19::Scope::SmallStepEarly:energy_admission_v19::Scope::OptionalMain;
  const auto admission=energy_admission_v19::assess(scope,actual);
  a.energy_admission_checked=true;a.energy_admission_applies=admission.applies;a.energy_admission_allowed=admission.allowed;
  a.energy_unsupported_mask=admission.unsupported_mask;a.energy_admission_reason=static_cast<uint8_t>(admission.reason);
#endif
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  if(exhausted && !return_aid && !(small_step && optional_early_event_)){
    // Closing a real ZERO with no allowance is not energy surplus. Preserve
    // its calculated positive deficit even though no pulse can be admitted.
    e.target_energy_j=actual.target_j;e.passive_energy_j=actual.available_j;
    e.commanded_energy_deficit_j=actual.request_j;e.delta_energy_required_j=actual.deficit_j;
    e.unapplied_energy_request_j=actual.request_j;e.predicted_energy_j=actual.available_j;
    e.work_brake_cap_ms=0;
    walking_half_cycle_budget_.accepted(half_cycle_energy_budget::Kind::Main,0);
    a.envelope_after_ms=walking_half_cycle_budget_.remaining();
    save();return true;
  }
#endif
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
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  if(return_aid)e.predicted_next_peak_amplitude_deg=e.free_next_peak_amplitude_deg=NAN;
#endif
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
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
    a.early_budget_admitted=admission.allowed;
#else
    a.early_budget_admitted=energy::earlyAdmissible(actual);
#endif
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
  if(freshness_required && !early_fresh()){
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
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  const auto kind=post_aid?half_cycle_energy_budget::Kind::PostAid:
      return_aid?half_cycle_energy_budget::Kind::ReturnAid:half_cycle_energy_budget::Kind::Main;
  if(!walking_half_cycle_budget_.accepted(kind,width)){
    requestEmergencyStop("energy_envelope_reservation_failed");save();return true;
  }
  a.envelope_after_ms=walking_half_cycle_budget_.remaining();
  if(return_aid)walking_return_observer_.consume(status_.run_id,a.peak_ms);
  else wheel_cycle_guard_=guard;
#if WALKING_RETURN_HISTORY
  // Each new left aid consumes the previous reference. Its own centre
  // passage can qualify only after the subsequent left peak has arrived.
  if (returnPredictionTrialSelected() && return_aid && side==-1) walking_crossing_reference_={};
#endif
#else
  wheel_cycle_guard_=guard;
#endif
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
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  if(!return_aid)
#endif
  {tilt_priority_.crossing(side,t);tilt_priority_.confirm(side,t,confirmation,true,true,false);}
  e.valid=true;e.output_executed=!actual.coast;
  e.reason=actual.coast?Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT:Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE;
  a.executed=e.output_executed;
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  // A local aid prediction is not a next-peak forecast or a main input.
  if(return_aid){walking_aid_zero_seen_=false;adaptive_cg_.breakPair();save();return true;}
#endif
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  walking_arrival_pending_bias_j_=in.arrival_bias_j;walking_return_pending_=walking_return_event_;
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  walking_return_pending_=walking_half_cycle_budget_.aidUsed();
#endif
#endif
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
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  if(walking_half_cycle_budget_.aidUsed())return;
#endif
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  namespace energy=walking_return_energy;
#elif defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
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
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  model.arrival_bias_j=walking_arrival_feedback_.bias(model.side,t,lateral_energy::potential(lateral_energy_geometry_,model.target_deg));
#endif
  const auto preview=energy::decide(lateral_energy_geometry_,walking_energy_current_,model);
  if(!preview.valid || preview.coast)return; // No energy deficit, no forced foot pulse.
  auto guard=wheel_cycle_guard_;
  const auto width=guard.limit(-side,telemetry.speed_rpm,true,t,preview.width_ms);
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
  const auto guarded=energy::evaluateWidth(lateral_energy_geometry_,walking_energy_current_,model,width);
  if(!energy::earlyAdmissible(guarded))return;
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  if(!energy_admission_v19::optionalMainAdmissible(guarded))return;
#endif
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
#if WALKING_SMALL_STEP_AB
  if(smallStepBaselineSelected()){
    json+=",\"walking_energy\":{\"revision\":\"small_step_v14_current_guards\",\"output_enabled\":true,\"branch\":19,";
    json+="\"selector_revision\":\"walking_peak_rate_energy_v14\",\"width_selection\":\"minimum_absolute_predicted_work_error\",";
    json+="\"early_admission\":\"positive_bounded_partial_work_within_charge_range\",\"generic_early_15ms_cap_active\":true,";
    json+="\"generic_early_ceiling_ms\":15,\"main_ceiling_ms\":80,\"return_then_main\":false,\"combined_half_cycle_envelope_ms\":80,";
    json+="\"arrival_feedback\":\"disabled\",\"completion_revision\":\"actual_elapsed_pulse_completion_v16\",";
    json+="\"elapsed_semantics\":\"software_queue_accept_to_stop_request_not_physical_current_edges\",";
  }else{
#endif
#if defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  json+=",\"walking_energy\":{\"revision\":\"walking_energy_admission_v19\",\"output_enabled\":true,\"branch\":24,";
  json+="\"selector_revision\":\"walking_energy_admission_v19\",\"optional_main_support_required\":true,";
#if WALKING_TURN_OBSERVER
  json+="\"turn_input_revision\":\"weak_right_negative_yaw_unassisted_zero_v1\",";
#endif
#if SUPPORT_TRANSFER_CARRY2
  json+="\"arc_aid_revision\":\"negative_receiving_arc_v2\",\"arc_aid_reason_names\":[\"inactive\",\"no_fresh_shape\",\"outside_model\",\"not_receiving\",\"shape_returning\",\"eligible\",\"no_main_room\",\"no_width_gain\",\"applied\",\"uncertain_pivot\",\"command_veto\"],";
  json+="\"arc_foot_source_names\":[\"unavailable\",\"periodic_packet\",\"fresh_pair_since_packet\"],";
#endif
#else
  json+=",\"walking_energy\":{\"revision\":\"walking_energy_transfer_v18\",\"output_enabled\":true,\"branch\":23,";
  json+="\"selector_revision\":\"walking_energy_transfer_v18\",";
#endif
  json+="\"arrival_feedback\":\"causal_completed_unassisted_main_only\",\"arrival_bias_limit_target_fraction\":0.25,";
  json+="\"return_budget\":\"explicit_local_rate_energy_reachability_J_only_no_nominal_U_or_passive_reservoir_credit\",\"return_budget_limit_target_fraction\":0.5,\"dropout_budget_limit_target_fraction\":0.25,";
  json+="\"return_then_main\":true,\"main_at_real_zero_after_aid\":true,\"combined_half_cycle_envelope_ms\":80,\"envelope_accounting\":\"accepted_reservation_replaced_by_all_actual_elapsed_software_ms\",";
  json+="\"aid_prediction_semantics\":\"available_J_zero;predicted_arrival_J_means_local_return_demand_fulfillment_not_next_peak\",\"aid_cutoff_at_fresh_mechanical_zero\":true,";
  json+="\"return_evidence_names\":[\"unavailable\",\"observed_shape\",\"recognition_dropout\",\"range_rejected\",\"observed_unseparated\"";
#if SUPPORT_TRANSFER_STALL_V3
  json+=",\"measured_arc_shortfall\",\"predicted_arc_shortfall\",\"post_aid_shortfall\"";
#if WALKING_RETURN_HISTORY
  if (isContinuationPriorityProfile()) json+=",\"continue_after_observed_loss\"";
#endif
#endif
  json+="],";
#elif defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  json+=",\"walking_energy\":{\"revision\":\"walking_return_energy_v17\",\"output_enabled\":true,\"branch\":22,";
  json+="\"selector_revision\":\"walking_return_energy_v17\",\"arrival_feedback\":\"causal_completed_peak_residual_plus_applied_bias_ewma_per_side\",\"arrival_bias_limit_target_fraction\":0.25,\"return_budget\":\"foot_gated_local_rate_energy_loss_extended_to_mechanical_zero_not_identified_potential\",\"return_budget_limit_target_fraction\":0.5,";
#else
  json+=",\"walking_energy\":{\"revision\":\"energy_pulse_lifecycle_v16\",\"output_enabled\":true,\"branch\":21,";
  json+="\"selector_revision\":\"walking_early_energy_main_v15\",";
#endif
  json+="\"completion_revision\":\"actual_elapsed_pulse_completion_v16\",\"timer_clock\":\"fresh_millis_after_motion_and_enqueue\",\"completion_reason_names\":[\"not_observed\",\"timer\",\"interrupted\",\"estop\",\"run_end\"";
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  json+=",\"mechanical_zero_cutoff\"";
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
  json+=",\"post_aid_reversal_cutoff\"";
  if (isContinuationPriorityProfile()) json+=",\"observed_carry_cutoff\"";
#endif
#endif
  json+="],";
  json+="\"elapsed_semantics\":\"software_queue_accept_to_stop_request_not_physical_current_edges\",\"completion_independent_of_log_capacity\":true,";
#else
  json+=",\"walking_energy\":{\"revision\":\"walking_early_energy_main_v15\",\"output_enabled\":true,\"branch\":20,";
#endif
  json+="\"width_selection\":\"minimum_integer_predicted_work_reaches_request\",\"early_admission\":\"positive_budget_fulfilled_after_guard;otherwise_no_transport_and_keep_ordinary_opportunity\",\"generic_early_15ms_cap_active\":false,";
  json+="\"main_ceiling_ms\":80,\"main_ceiling_basis\":\"inherited_normal_experimental_envelope_not_physics_derived\",";
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
  json+="\"rescue_ceiling_ms\":80,\"rescue_ceiling_basis\":\"shared_with_main_within_inherited_half_cycle_envelope_not_physics_derived\",";
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
  if (isTransferPriorityProfile())
    json+="\"online_reversal_cutoff_active\":true,\"reversal_cutoff_scope\":\"post_aid_or_priority_assist_two_fresh_outward_rates_before_zero\",";
  else if (isLeftTransferCarryProfile())
    json+="\"online_reversal_cutoff_active\":true,\"reversal_cutoff_scope\":\"post_aid_only_two_fresh_outward_rates_before_zero\",";
  else
#endif
  json+="\"online_reversal_cutoff_active\":false,";
#elif defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  json+="\"rescue_ceiling_ms\":80,\"rescue_ceiling_basis\":\"energy_inverse_with_existing_main_envelope_not_physics_derived\",\"online_reversal_cutoff_active\":false,";
#else
  json+="\"rescue_ceiling_ms\":15,\"rescue_ceiling_basis\":\"separate_legacy_low_speed_trial_not_derived\",\"online_reversal_cutoff_active\":false,";
#endif
  json+="\"energy_fulfillment_scope\":\"predicted_work_only;model_error_and_unmodelled_motion_can_leave_actual_deficit\",\"record_prediction_semantics\":\"postguard_candidate;output_predictions_are_zero_when_unsent\",";
#else
  json+=",\"walking_energy\":{\"revision\":\"walking_peak_rate_energy_v14\",\"output_enabled\":true,\"branch\":19,";
#endif
#if WALKING_SMALL_STEP_AB
  }
#endif
  json+="\"origin\":\"nominal_zero_CG_rocker_U_same_for_current_peak_target\",\"physical_body_inertia_identified\":false,\"MPC_active\":false,";
#if defined(WALKING_AUDIT_PSRAM) && WALKING_AUDIT_PSRAM
  json+="\"archive_storage\":\"psram\",\"archive_available\":"+String(walking_energy_audit_.available()?"true":"false")+",";
  json+="\"archive_bytes\":"+String(walking_energy_audit_.bytes())+",";
#endif
  json+="\"mechanical_reference_deg\":0,\"energy_neutral_model\":\"nominal_only;moving_foot_potential_minimum_not_yet_identified\",\"paired_feedback_is_energy_neutral_estimation\":false,";
  json+="\"paired_semantics\":\"successive_peak_mean_and_difference_not_two_simultaneous_energy_stores\",";
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
#if WALKING_SMALL_STEP_AB
  if(smallStepBaselineSelected())json+="\"foot_role\":\"observation_only\",";
  else
#endif
  json+="\"foot_role\":\"fresh_raw_shape_or_bounded_recognition_dropout_with_two_strong_IMU_deceleration_windows_no_ms_boost\",";
#elif defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  json+="\"foot_role\":\"fresh_observed_amount_gates_return_energy_budget_no_ms_boost\",";
#else
  json+="\"foot_role\":\"timing_only_no_ms_boost_no_forced_coast_override\",";
#endif
  json+="\"startup_exception\":\"fixed_start_kick_only_then_all_widths_energy_selected\",\"work_model\":\"inherited_empirical_pulse_Q_arrival_transfer_provisional\",\"coast_residual_modelled\":false,";
#if WALKING_RETURN_HISTORY && SUPPORT_TRANSFER_STALL_V3
  if (isLeftTransferCarryProfile()) {
    json+="\"post_aid_work_comparison\":\"modeled_pulse_work_minus_zero_command_tail_over_same_duration\",";
    json+="\"post_aid_coast_fall_tau_s\":"+String(walking_energy_policy::kCoastFallTauS,6)+",";
  }
#endif
#if WALKING_RIGHT_ARRIVAL
  if(!smallStepBaselineSelected()){
#if WALKING_RETURN_HISTORY
    if (isLeftTransferCarryProfile()) json+="\"right_arrival_model\":\"positive_unassisted_real_zero_startup_4p5_5_v1\",";
    else
#endif
      json+="\"right_arrival_model\":\"positive_unassisted_real_zero_startup_4p5_v1\",";
    json+="\"right_arrival_reason_names\":[\"not_evaluated\",\"invalid\",\"outside_support\",\"kept\",\"reduced\",\"coast\"],";
    json+="\"right_arrival_units\":\"J_of_nominal_body_peak_U;Q_in_mA_s_modelled_during_command\",";
    json+="\"right_arrival_comparison\":\"base=unsteered_wheel_limited;final=after_steering;existing_work_and_completion_fields_use_inherited_model\",";
  }
#endif
  json+="\"common_gain\":0.25,\"redistribution_gain\":0.125,\"each_correction_limit_target_fraction\":0.125,\"intercept_rolloff\":\"min(1,rate_dps/20)^2\",\"incoming_peak_max_age_ms\":1500,\"same_side_peak_max_age_ms\":3000,";
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
  const auto& cfg=walking_fore_aft_guard_.config();const auto& motion=walking_fore_aft_last_;
  json+="\"fore_aft_motion_guard\":{\"revision\":\""+String(fore_aft_motion_guard::kRevision)+"\",\"active\":true,\"physical_stability_limit_identified\":false,\"motion_only_not_J\":true,";
  json+="\"reference\":\"first_fresh_run_gravity_YZ_projection\",\"rate\":\"gravity_cross_bias_corrected_gyro_projection_derivative\",\"candidate_vetoes_new_output\":true,\"confirmed_stops_active_output\":true,";
  json+="\"candidate_angle_deg\":"+n(cfg.candidate_angle_deg)+",\"minimum_outward_rate_dps\":"+n(cfg.minimum_outward_rate_dps)+",\"predicted_boundary_deg\":"+n(cfg.predicted_boundary_deg)+",\"maximum_time_to_boundary_ms\":"+n(cfg.maximum_time_to_boundary_ms)+",";
  json+="\"confirmation_us\":"+String(cfg.confirmation_us)+",\"minimum_outward_gain_deg\":"+n(cfg.minimum_outward_gain_deg)+",\"maximum_sample_age_us\":"+String(cfg.maximum_sample_age_us)+",";
  json+="\"initialized\":"+String(motion.initialized?"true":"false")+",\"allow_output\":"+String(motion.allow_output?"true":"false")+",\"stop\":"+String(motion.stop?"true":"false")+",\"reason\":\""+String(fore_aft_motion_guard::reasonName(motion.reason))+"\",";
  json+="\"start_sample_us\":"+String(motion.start_sample_us)+",\"start_roll_deg\":"+n(motion.start_roll_deg)+",\"last_sample_us\":"+String(motion.sample_us)+",\"angle_deg\":"+n(motion.angle_deg)+",\"rate_dps\":"+n(motion.rate_dps)+",\"time_to_boundary_ms\":"+n(motion.time_to_boundary_ms)+",\"candidate_us\":"+String(motion.candidate_us)+",\"candidate_entries\":"+String(walking_fore_aft_candidates_)+"},";
#endif
  json+="\"passive_coefficients\":[";
  for(int side=1;side>=-1;side-=2){if(side<0)json+=",";const auto c=walking_energy_policy::passiveCoefficients(side);
    json+="{\"side\":"+String(side)+",\"offset_j\":"+n(c.offset_j)+",\"omega2_j\":"+n(c.omega2_j)+",\"rate_min_dps\":"+n(c.rate_min_dps)+",\"rate_max_dps\":"+n(c.rate_max_dps)+"}";}
  json+="],\"records\":";
  CompactJsonTable<PsramString> table(json);uint32_t stored=0;
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
  constexpr size_t energy_audit_soft_end=foot_fast_policy::kEnergyAuditEndBytes;
#else
  constexpr size_t energy_audit_soft_end=640U*1024U*8U/10U;
#endif
  size_t extra_table_bytes=0;
  for(uint32_t i=0;i<walking_energy_count_;++i){
    if(json.length()+4096U+extra_table_bytes>=energy_audit_soft_end)break;
    const auto& a=walking_energy_audit_[i];const auto& d=a.decision;
    String row="{\"time_ms\":"+String(a.time_ms)+",\"peak_ms\":"+String(a.peak_ms)+",\"side\":"+String(a.side);
    row+=",\"output_start_ms\":"+String(a.output_start_ms);
    row+=",\"requested_ms\":"+String(a.requested_ms)+",\"guarded_ms\":"+String(a.guarded_ms);
#if WALKING_RIGHT_ARRIVAL
    if(!smallStepBaselineSelected()){
      const auto& fit=a.right_arrival;
      row+=",\"right_arrival_checked\":"+String(a.right_arrival_checked?"true":"false");
      row+=",\"right_arrival_eligible\":"+String(a.right_arrival_checked && fit.eligible?"true":"false");
      row+=",\"right_arrival_reason\":"+String(static_cast<unsigned>(fit.reason));
      row+=",\"right_arrival_base_ms\":"+String(a.right_base_ms)+",\"right_arrival_limit_ms\":"+String(fit.width_ms);
      row+=",\"right_arrival_base_q_mas\":"+n(a.right_base_q)+",\"right_arrival_coast_j\":"+n(fit.coast_j);
      row+=",\"right_arrival_base_j\":"+n(fit.original_j)+",\"right_arrival_target_j\":"+n(fit.target_j);
      row+=",\"right_arrival_final_j\":"+n(a.right_final_j);
      row+=",\"right_arrival_elapsed_j\":"+n(fit.eligible && a.completion.valid
          ? right_arrival_energy::predictedJ(a.start_rate_dps,a.completion.q_mas) : NAN);
    }
#endif
    row+=",\"executed\":"+String(a.executed?"true":"false")+",\"optional\":"+String(a.optional?"true":"false")+",\"rescue\":"+String(a.rescue?"true":"false");
    row+=",\"early_budget_checked\":"+String(a.early_budget_checked?"true":"false")+",\"early_budget_admitted\":"+String(a.early_budget_admitted?"true":"false");
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
    row+=",\"energy_admission_checked\":"+String(a.energy_admission_checked?"true":"false")+",\"energy_admission_applies\":"+String(a.energy_admission_applies?"true":"false")+",\"energy_admission_allowed\":"+String(a.energy_admission_allowed?"true":"false");
    row+=",\"energy_unsupported_mask\":"+String(a.energy_unsupported_mask)+",\"energy_admission_reason\":"+String(a.energy_admission_reason);
    row+=",\"fore_aft_allowed\":"+String(a.fore_aft_allowed?"true":"false")+",\"fore_aft_reason\":"+String(a.fore_aft_reason)+",\"fore_aft_angle_deg\":"+n(a.fore_aft_angle_deg)+",\"fore_aft_rate_dps\":"+n(a.fore_aft_rate_dps);
#endif
#if WALKING_TURN_OBSERVER
    row+=",\"turn_cycle_end_ms\":"+String(a.turn_cycle_end_ms)+",\"turn_base_ms\":"+String(a.turn_base_ms);
    row+=",\"turn_candidate_ms\":"+String(a.turn_candidate_ms)+",\"turn_adjustment_j\":"+n(a.turn_adjustment_j);
    row+=",\"turn_command_adjustment_j\":"+n(a.executed?a.turn_adjustment_j:0.f);
#endif
    row+=",\"valid\":"+String(d.valid?"true":"false")+",\"coast\":"+String(d.coast?"true":"false")+",\"saturated\":"+String(d.saturated?"true":"false")+",\"extrapolated\":"+String(d.extrapolated?"true":"false");
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
    row+=",\"arrival_bias_j\":"+n(a.arrival_bias_j)+",\"return_work_j\":"+n(a.return_work_j)+",\"return_event\":"+String(a.return_event?"true":"false");
    row+=",\"return_deceleration_dps2\":"+n(a.return_deceleration_dps2)+",\"return_stopping_distance_deg\":"+n(a.return_stopping_distance_deg)+",\"return_foot_age_ms\":"+String(a.return_foot_age_ms);
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
    row+=",\"return_evidence\":"+String(a.return_evidence)+",\"assisted_cycle\":"+String(a.assisted_cycle?"true":"false");
#if FOOT_PERIODIC_V38
    row+=",\"periodic_foot_packet\":"+String(a.periodic_packet);
#endif
    row+=",\"envelope_before_ms\":"+String(a.envelope_before_ms)+",\"envelope_after_ms\":"+String(a.envelope_after_ms);
#endif
#endif
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
#if SUPPORT_TRANSFER_CARRY2
    if(a.return_event) extra_table_bytes+=128; // Reserve this compact audit row in the same log budget.
#if SUPPORT_TRANSFER_ARC_FEEDBACK
    if(a.return_event) extra_table_bytes+=64;
#endif
#if SUPPORT_TRANSFER_STALL_V3
    if(a.return_event) extra_table_bytes+=8;
#endif
#if WALKING_RETURN_HISTORY
    if(a.return_event && carryHistoryTrialSelected()) extra_table_bytes+=180;
    if(a.return_event && returnPredictionTrialSelected()) extra_table_bytes+=140;
    if(a.return_event && isLeftTransferCarryProfile()) extra_table_bytes+=216;
#if SUPPORT_TRANSFER_STALL_V3
    if(a.return_event && isTransferPriorityProfile()) extra_table_bytes+=144;
#endif
#endif
#endif
  }
  if(!table.finish()){json.fail();return;}
#if SUPPORT_TRANSFER_CARRY2
  json+=",\"arc_aids\":";
  CompactJsonTable<PsramString> arc_table(json);
  for(uint32_t i=0;i<stored;++i){
    const auto& a=walking_energy_audit_[i];
    if(!a.return_event)continue;
    String row="{\"row_index\":"+String(i)+",\"arc_aid_reason\":"+String(static_cast<unsigned>(a.arc_aid.reason));
    row+=",\"arc_foot_source\":"+String(a.arc_foot_source)+",\"arc_base_guarded_ms\":"+String(a.base_aid_ms);
#if SUPPORT_TRANSFER_STALL_V3
    row+=a.arc_aid.extension_pivot_only?",\"arc_extension_pivot_only\":true":",\"arc_extension_pivot_only\":false";
#endif
    row+=",\"arc_candidate_guarded_ms\":"+String(a.arc_candidate_ms)+",\"arc_main_reserve_ms\":"+String(a.main_reserve_ms);
    row+=",\"arc_extra_request_j\":"+n(a.arc_aid.extra_j)+",\"arc_terminal_deg\":"+n(a.arc_aid.terminal_deg)+",\"arc_local_s\":"+n(a.arc_aid.horizon_s);
#if SUPPORT_TRANSFER_ARC_FEEDBACK
    row+=",\"arc_feedback_peak_ms\":"+String(a.arc_feedback_peak_ms)+",\"arc_feedback_base_ms\":"+String(a.arc_feedback_base_ms);
    row+=",\"arc_feedback_available_j\":"+n(a.arc_feedback_available_j)+",\"arc_feedback_command_j\":"+n(a.executed?a.arc_feedback_added_j:0.f);
#endif
#if WALKING_RETURN_HISTORY
    if (returnPredictionTrialSelected()) {
      row+=",\"arc_predicted_transfer_ms\":"+n(a.predicted_transfer_ms)+",\"arc_predicted_terminal_deg\":"+n(a.predicted_terminal_deg);
    }
    if (carryHistoryTrialSelected()) {
      row+=",\"arc_history_body_deg\":"+n(a.history_state.body_deg)+",\"arc_history_left_deg\":"+n(a.history_state.left_deg);
      row+=",\"arc_history_right_deg\":"+n(a.history_state.right_deg);
      row+=",\"arc_history_first_peak_ms\":"+String(a.history.first_peak_ms)+",\"arc_history_second_peak_ms\":"+String(a.history.second_peak_ms);
      row+=",\"arc_history_decel_dps2\":"+n(a.history.deceleration)+",\"arc_history_request_j\":"+n(a.history.request_j);
      row+=",\"arc_history_limit_ms\":"+String(a.history.limit_ms)+",\"arc_history_base_ms\":"+String(a.history_base_ms);
      row+=",\"arc_history_selected_ms\":"+String(a.history_final_ms);
    }
    if (returnPredictionTrialSelected()) {
      row+=",\"arc_crossing_ref_zero_ms\":"+String(a.crossing_work.zero_ms)+",\"arc_crossing_ref_peak_ms\":"+String(a.crossing_work.peak_ms);
      row+=",\"arc_crossing_ref_dps\":"+n(a.crossing_work.speed_dps)+",\"arc_crossing_request_j\":"+n(a.crossing_work.request_j);
      row+=",\"arc_crossing_base_ms\":"+String(a.crossing_work.base_ms)+",\"arc_crossing_selected_ms\":"+String(a.crossing_work.final_ms);
    }
    if (isLeftTransferCarryProfile()) {
      row+=",\"arc_approach_transfer_ms\":"+n(a.crossing_work.transfer_ms)+",\"arc_approach_terminal_deg\":"+n(a.crossing_work.terminal_deg);
      row+=",\"arc_approach_request_j\":"+n(a.crossing_work.arc_request_j);
      row+=",\"arc_passage_main_ms\":"+String(a.crossing_work.passage_main_ms)+",\"arc_passage_reserve_ms\":"+String(a.crossing_work.passage_reserve_ms);
      row+=",\"arc_passage_main_j\":"+n(a.crossing_work.passage_main_j)+",\"arc_passage_zero_dps\":"+n(a.crossing_work.passage_zero_dps);
      row+=",\"arc_post_cap_ms\":"+String(a.post_cap_ms)+",\"arc_post_increment_j\":"+n(a.post_increment_j)+
           ",\"arc_post_elapsed_increment_j\":"+n(a.post_elapsed_increment_j);
    }
#if SUPPORT_TRANSFER_STALL_V3
    if (a.return_evidence==static_cast<uint8_t>(return_observation_policy::Evidence::ContinuationShortfall))
      row+=",\"arc_post_release_decel_dps2\":"+n(a.post_release_deceleration_dps2);
    if (isTransferPriorityProfile()) {
      row+=",\"arc_priority_base_ms\":"+String(a.arc_priority.base_ms)+",\"arc_priority_base_j\":"+n(a.arc_priority.base_work_j);
      row+=",\"arc_priority_zero_j\":"+n(a.arc_priority.zero_work_j)+",\"arc_priority_cap_ms\":"+String(a.arc_priority.cap_ms);
    }
#endif
#endif
    row+="}";
    if(!arc_table.append(row)){json.fail();return;}
  }
  if(!arc_table.finish()){json.fail();return;}
#endif
  json+=",\"stored\":"+String(stored)+",\"total\":"+String(walking_energy_total_)+",\"truncated\":"+String(stored<walking_energy_total_?"true":"false")+"}";
}
#endif
