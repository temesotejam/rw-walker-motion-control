// Called on new IMU samples before any ordinary peak/ZERO decision.
bool ExperimentRunner::serviceWalkingRestart(uint32_t t,uint32_t sample,float before,float angle,float rate) {
  using namespace settled_walking_restart;
  Input in;
  in.now_ms=t;in.sample_us=sample;in.angle_deg=angle;in.rate_dps=rate;
  in.run_active=energy_control_autonomous_mode_ && wheelWorkMode() &&
      energy_control_autonomous_target_peak_deg_==8.f && !status_.emergency_stop &&
      status_.state==ExperimentState::RUNNING_BATCH_SWEEP &&
      (energy_control_autonomous_phase_==EnergyControlAutonomousPhase::ENERGY_CONTROL ||
       energy_control_autonomous_phase_==EnergyControlAutonomousPhase::HOLD);
  if(!in.run_active)return false;
  const uint64_t now_us=esp_timer_get_time();
  in.guarded=imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis()) &&
      uint32_t(uint32_t(now_us)-sample)<=10000 && walkingForeAftAllowsOutput();
  in.fore_aft_deg=walking_fore_aft_last_.angle_deg;in.fore_aft_rate_dps=walking_fore_aft_last_.rate_dps;
  const auto& feet=periodic_foot_packet_;
  const auto fresh=[&](const periodic_foot::Side& s){
    return s.sequence && s.image_us>=feet.epoch_us && s.image_us<=now_us &&
        now_us-s.image_us<=periodic_foot::maximum_age_us && std::isfinite(s.estimated);
  };
  // main.cpp starts the periodic stream from the measured run clock once it
  // observes RUNNING. The legacy early-candidate epoch is not its origin.
  const uint64_t elapsed_us=uint64_t(t)*1000;
  const uint64_t epoch_age=now_us-feet.epoch_us;
  const bool this_run=feet.epoch_us && feet.epoch_us<=now_us &&
      (epoch_age>elapsed_us ? epoch_age-elapsed_us : elapsed_us-epoch_age)<=periodic_foot::period_us;
  in.feet_fresh=feet.count && this_run &&
      feet.published_us<=now_us && now_us-feet.published_us<=2*periodic_foot::period_us &&
      fresh(feet.left) && fresh(feet.right);
  in.left_deg=feet.left.measured;in.right_deg=feet.right.measured;
  in.had_peak=energy_control_autonomous_last_peak_valid_;in.pulse_active=status_.pulse_active;
  in.crossed_zero=(before>0 && angle<=0 && rate<0) || (before<0 && angle>=0 && rate>0);
  const int8_t pending=walking_restart_.pending();
  const int8_t side=walking_restart_.observe(in);
  // The first centre crossing ends a restart pulse early. Ordinary control
  // resumes at the ensuing actual peak, after a measurable opposite excursion.
  if(pending && status_.pulse_active && (!in.guarded || !in.feet_fresh ||
      (in.crossed_zero && pending*rate>0))) {
    stopActivePulse(millis());
    status_.current_mA_setting=0;status_.pulse_width_ms_setting=0;
    energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
    resetEnergyControlAutonomousPeakTracker(true);
  }
  if(!side)return walking_restart_.pending()!=0;
  if(walking_restart_checked_packet_==feet.count)return pending!=0;
  walking_restart_checked_packet_=feet.count;
  if(!roller_ || !roller_->ok() || !imu_ || t+episode_ms>=measurementTotalDurationMs() ||
      (energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_PEAK &&
       energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS))
    return pending!=0;
  const auto tel=roller_->telemetrySnapshot();
  const uint32_t now=micros();
  if(!tel.current_valid || !tel.current_sample_time_us || now-tel.current_sample_time_us>50000 ||
     !tel.speed_sample_time_us ||
     !peak_error_adaptation::speedUsable(tel.speed_rpm,tel.speed_valid,now-tel.speed_sample_time_us) ||
     tel.consecutive_errors || tel.error_raw ||
     tel.requested_current_mA || tel.applied_current_mA ||
     tel.battery_mV<6180 || tel.battery_mV>8500)return pending!=0;
  const float reference=walking_restart_.referenceSpeed(side);
  const float initial=side*walkingCurrentPolarity(Config::ENERGY_CONTROL_AUTONOMOUS_NORMAL_OUTPUT_SIGN,
      Config::MOTOR_DRIVER_POLARITY)*tel.actual_current_mA;
  const float goal=predictCurrentGoalMa(1200,tel.battery_mV*.001f);
  const uint16_t available=walking_restart_.remaining();
  const uint16_t timing=tilt_priority_.timingCap(side,t,available);
  const auto proposal=plan(walking_energy_current_,side,reference,rate,initial,goal,std::min(available,timing));
  if(!proposal.width)return pending!=0;
  auto wheel=wheel_cycle_guard_;
  const uint16_t width=wheel.limit(side,tel.speed_rpm,true,t,proposal.width);
  const float applied_work=excitationAtWidth(walking_energy_current_,side,rate,initial,goal,width);
  if(!width || width>proposal.width || !walkingForeAftAllowsOutput() ||
     !std::isfinite(applied_work) || uint32_t(micros()-sample)>10000)return pending!=0;
  // A settled episode can begin in either detector wait state. Authorize the
  // ordinary directional pulse exactly once and never manufacture a peak.
  const auto saved_state=energy_control_autonomous_half_cycle_state_;
  energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
  auto& a=walking_restart_.attempt(t,side);
  a.width=width;a.angle=angle;a.reference_speed=reference;a.demand_j=proposal.demand_j;
  a.predicted_j=applied_work;
  if(!beginEnergyControlAutonomousPulse(millis(),t,side,width,1200)) {
    energy_control_autonomous_half_cycle_state_=saved_state;
    walking_restart_.failed(); // hardware acknowledgement uncertain: do not retry
    return true;
  }
  a.start=active_pulse_start_test_ms_;
  walking_restart_.accepted(width);
  wheel_cycle_guard_=wheel;
  // The old half cycle ended without an opposite peak; its pending fitted
  // arrival must never be learned from the externally assisted next peak.
  energy_control_autonomous_pending_peak_=false;
  energy_control_autonomous_pending_model_energy_j_=NAN;
  walking_return_pending_=false;work_positive_pending_=work_negative_pending_=false;
  work_walking_pending_=work_walking_low_pending_=false;
  return true;
}

void ExperimentRunner::appendWalkingRestartMetadata(PsramString& json) const {
  json+=",\"walking_restart\":{\"revision\":1,\"max_attempts\":2,\"episode_budget_ms\":80,";
  json+="\"reference\":\"median_last_three_same_direction_centre_speeds_with_opposite_excursion;near_rest_work_extrapolated\",";
  json+="\"columns\":[\"start_ms\",\"side\",\"angle_deg\",\"reference_dps\",\"request_j\",\"predicted_work_j\",\"width_ms\",\"queued\",\"elapsed_ms\",\"opposite_cross_ms\",\"opposite_peak_ms\",\"timeout_ms\"],\"rows\":[";
  for(unsigned i=0;i<walking_restart_.count;++i) {
    const auto& a=walking_restart_.records[i];
    if(i)json+=",";
    json+="["+String(a.start)+","+String(int(a.direction))+","+String(a.angle,4)+","+String(a.reference_speed,4)+",";
    json+=String(a.demand_j,9)+","+String(a.predicted_j,9)+","+String(a.width)+","+(a.queued?"true":"false")+",";
    json+=String(a.elapsed)+","+String(a.cross)+","+String(a.opposite_peak)+","+String(a.timeout)+"]";
  }
  json+="]}";
}
