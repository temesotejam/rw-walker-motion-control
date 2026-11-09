#include "host_v46o/Arduino.h"
#include <fstream>
#include <cassert>
#include <iostream>
#include <vector>
#define private public
#include "../src/psram_logger.h"
#include "../src/foot_observer.h"
#include "../src/immutable_export.h"
#undef private
#include "../src/imu_manager.h"
#include "../src/roller485_manager.h"
ImuManager imu;
Roller485Manager roller;
RunControlWorker run_control;
FootObserver feet;
RollerTelemetry Roller485Manager::telemetrySnapshot() const {return {};}
void ImuManager::appendAcquisitionDiagnostics(PsramString& json) const {json += "{\"host_fixture\":true}";}
// Camera and RTOS are not exercised here; actual serializers/export code are.
CameraOneShotSnapshot OneShotCamera::snapshot() const { return {}; }
bool OneShotCamera::startContinuous() { return false; }
camera_fb_t* OneShotCamera::acquireContinuous(uint32_t) { return nullptr; }
void OneShotCamera::releaseContinuous(camera_fb_t*) {}
void OneShotCamera::stopContinuous() {}
int main(int argc,char** argv){
  assert(argc==2);
  run_control.snapshot_.state_id=5;
  strcpy(run_control.snapshot_.state_name,"ESTOP");
  strcpy(run_control.snapshot_.last_error,"imu_acquisition_overflow_backlog_or_stale");
  run_control.snapshot_.motor_cmd_mA=0;
  run_control.snapshot_.actual_current_mA=-7;
  run_control.snapshot_.heartbeat_us=123456;
  control_work::profile.reset();
  control_work::profile.add(control_work::Stage::LogRow,true,777);
  control_latency::profile.reset();
  for(unsigned i=0;i<70;++i){
    const uint32_t stamp=10000+i*5000;
    control_latency::profile.receive(true,i+1,stamp,stamp-500,stamp+200,{},control_latency::Delivery(stamp+25,stamp+180));
    control_latency::profile.finish(stamp+3000,{});
  }
  // A copied preview must stay paired with its own observation when the next
  // camera frame arrives. No camera hardware or RTOS scheduling is simulated.
  feet.preview_=static_cast<uint8_t*>(ps_malloc(FootObserver::kPreviewBytes));
  feet.preview_mutex_=xSemaphoreCreateMutex();
  std::vector<uint8_t> gray(320*240), frozen(FootObserver::kPreviewBytes);
  for(unsigned y=0;y<240;++y)for(unsigned x=0;x<320;++x)gray[y*320+x]=(x+3*y)%256;
  FootPreviewInfo first{};first.frame.sequence=17;first.frame.frame_valid=true;
  first.left.center_x_px=175;first.frame.zero_reason=FootZeroReason::Collecting;
  first.mekf_attitude.valid=true;first.mekf_attitude.quaternion={.98480775f,.17364818f,0,0};first.mekf_attitude.sample_us=1700000;
  feet.publishPreview(gray.data(),first,false);
  FootPreviewInfo copied{};assert(feet.copyPreview(frozen.data(),copied));
  for(unsigned y=0;y<120;++y)for(unsigned x=0;x<160;++x)assert(frozen[y*160+x]==gray[y*2*320+x*2]);
  auto next=first;next.frame.sequence=18;next.left.center_x_px=35;
  next.mekf_attitude.quaternion={.96592583f,.25881905f,0,0};next.mekf_attitude.sample_us=1800000;
  // Run-time suppression preserves the frozen idle bytes AND their paired
  // timestamp/attitude, and must not dereference an image while suppressed.
  for(uint8_t state : {6,3,7}) {
    auto running=next;running.frame.state_id=state;
    feet.publishPreview(nullptr,running,true);
    assert(feet.copyPreview(frozen.data(),copied));
    assert(copied.frame.sequence==17 && frozen[0]==0);
    assert(copied.mekf_attitude.sample_us==1700000);
  }
  std::fill(gray.begin(),gray.end(),91);feet.publishPreview(gray.data(),next,false);
  assert(copied.frame.sequence==17 && copied.left.center_x_px==175 && frozen[0]==0);
  assert(copied.mekf_attitude.quaternion.x==first.mekf_attitude.quaternion.x && copied.mekf_attitude.sample_us==1700000);
  assert(feet.copyPreview(frozen.data(),copied));
  assert(copied.frame.sequence==18 && copied.left.center_x_px==35 && frozen[0]==91);
  assert(copied.mekf_attitude.quaternion.x==next.mekf_attitude.quaternion.x && copied.mekf_attitude.sample_us==1800000);
  next.frame.frame_valid=false;feet.publishPreview(nullptr,next,false);
  assert(!feet.copyPreview(frozen.data(),copied));
  PsramLogger logger;assert(logger.begin());
  assert(sizeof(PsramLogger)<4096);
  assert(PsramLogger::eventStorageBytes()<256U*1024U);
  upright_reference::Record reference{upright_reference::kMagic,1,3,801,2.2f,.038f,.05f};
  logger.startRun(1,123456,300,100,1000,false,0,800,0,false,NAN,false,0,false,true,65.0f,50,false,true,false,reference);
  logger.recordMeasurementStartAngle(3.2f);
  logger.recordMeasurementStartAngle(9.2f); // Cannot replace the actual start snapshot.
  reference.pitch_deg = 7.7f; // Operator changes after the run cannot mutate its metadata.
  LogSample row{};row.motor_cmd_mA=300;row.roller_actual_current_mA=270;
  row.roll_mekf_abs_cdeg=1234;row.pitch_mekf_abs_cdeg=567;
  row.roller_speed_crpm=12345;
  row.roller_speed_read_start_us=999900;row.roller_speed_sample_time_us=1000000;
  row.roller_speed_read_end_us=1000100;row.roller_speed_sequence=1;row.roller_speed_valid=1;
  row.physical_roll_rate_cdps=6000;row.body_rate_sample_time_us=1000000;
  row.roller_applied_current_mA=300;row.roller_applied_command_sequence=1;
  row.roller_applied_command_time_us=999000;
  row.roller_position_raw_cdeg=INT32_MIN;row.roller_position_valid=1;
  row.roller_position_read_start_us=999800;row.roller_position_sample_time_us=999900;
  row.roller_position_read_end_us=1000000;row.roller_position_sequence=9;
  row.gyro_heading_cdeg=LOG_NAN_I32;
  row.steering_actual_difference_cdeg=LOG_NAN_I16;
  row.steering_desired_difference_cdeg=LOG_NAN_I16;
  row.steering_cycle_yaw_rate_cdps=LOG_NAN_I16;
  row.steering_reason=static_cast<uint8_t>(steering::Reason::Disabled);
  assert(logger.addSample(row));assert(!logger.rwlogDownloadable());
  for(unsigned i=0;i<256;++i){
    PsramLogger::EnergyControlAutonomousPeakEvent p{};p.peak_amplitude_deg=8;
    logger.addEnergyControlAutonomousPeakEvent(p);
    PsramLogger::EnergyControlAutonomousZeroCrossEvent z{};z.zero_cross_time_ms=i*100;
    if(i==0) { z.weak_probe_baseline_ms=145;z.weak_probe_requested_ms=150;z.weak_probe_executed=true; }
    if(i==1) { z.physical_next_peak_side=-1;z.weak_probe_baseline_ms=28;z.weak_probe_requested_ms=33;z.weak_probe_executed=true;
      z.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::NegativeRecoveryStateBit0,true); }
    z.pre_input_measured_current_mA=17;z.pre_input_wheel_speed_rpm=123.5;
    z.unclamped_energy_request_j=.00040325f;z.q_unclamped_mA_s=9.01853f;z.unclamped_inverse_valid=true;
    z.maximum_input_work_j=.00003841f;z.predicted_current_goal_mA=153.f;z.predicted_current_rise_tau_s=.070f;
    if (i % 2) {
      z.negative_probe_batch_index=i;
      z.negative_probe_batch_planned_width_ms=20;
      z.negative_probe_batch_rate_skipped=i%4==1;
    }
    if (i % 3 == 2) {
      z.positive_current_batch_index=i;
      z.positive_current_batch_planned_mA=350;
    }
    logger.addEnergyControlAutonomousZeroCrossEvent(z);
    logger.addTimingProbeEvent({});
  }
  for(unsigned i=0;i<128;++i)logger.addSolverAuditEvent({});
  feet.frames_=static_cast<FootFrame*>(ps_malloc(sizeof(FootFrame)*FootObserver::kCapacity));
  feet.status_.available=true;feet.status_.zero_ready=true;feet.status_.count=FootObserver::kCapacity;
  feet.status_.zero_reason=FootZeroReason::Ready;
  for(unsigned i=0;i<FootObserver::kCapacity;++i){
    FootFrame f{};f.sequence=i;f.run_id=1;f.frame_us=1000000+i*66667;
    f.delivered_us=f.frame_us+30000;f.log_time_us=f.frame_us-123456;
    f.right_valid=f.left_valid=true;f.right_deg=5;f.left_deg=6;
    f.frame_valid=f.timestamp_valid=f.zero_ready=true;
    f.right_scan_y=42;f.left_scan_y=184;f.right_weight=3200;f.left_weight=3100;
    f.right_contrast=200;f.left_contrast=190;
    f.right_reason=f.left_reason=MarkerDetectionReason::Detected;f.right_templates=f.left_templates=17;
    f.right_candidates=1;f.left_candidates=2;f.left_ambiguity=0.4f;f.zero_reason=FootZeroReason::Ready;
    if(i==1){f.right_valid=false;f.right_deg=NAN;f.right_scan_y=NAN;f.right_reason=MarkerDetectionReason::LowContrast;}
    f.control_observed_us=f.delivered_us+125000;
    feet.frames_[i]=f;
  }
  feet.control_observation_attempts_=FootObserver::kCapacity;
  logger.markMeasurementDone();assert(!logger.rwlogDownloadable());
  logger.seal();assert(logger.rwlogDownloadable());
  assert(!logger.addSample(row));logger.addTimingProbeEvent({});assert(logger.timing_probe_event_count_==256);
  ImmutableExport exportFile;assert(exportFile.begin(logger));assert(exportFile.prepare());
  const char* const reserved_metadata = exportFile.metadata_.c_str();
  assert(exportFile.prepare());assert(!exportFile.reset());
  exportFile.build();const auto status=exportFile.status();
  std::cerr<<"metadata bytes="<<exportFile.metadata_.length()<<" phase="<<int(status.phase)<<" error="<<status.error<<"\n";
  assert(status.phase==ImmutableExport::Phase::Ready);
  const std::string metadata(exportFile.metadata_.c_str());
#if defined(FOOT_EARLY_TRIAL) && FOOT_EARLY_TRIAL
#if defined(ANGLE_EARLY_TRIAL) && ANGLE_EARLY_TRIAL
  assert(metadata.find("\"diagnostic_build_id\":\"angle-early-weak-zero-v2\"")!=std::string::npos);
  assert(metadata.find("\"geometry_worker_enabled\":false")!=std::string::npos);
  assert(metadata.find("\"applies_during_startup_ramp\":true")!=std::string::npos);
  assert(metadata.find("\"pulse_width_changes_alongside_timing\":true")!=std::string::npos);
#else
  assert(metadata.find("\"diagnostic_build_id\":\"early-input-geometry-v1\"")!=std::string::npos);
#endif
#else
  assert(metadata.find("\"diagnostic_build_id\"")==std::string::npos);
#endif
  assert(metadata.find("\"stored\":64")!=std::string::npos);
  assert(metadata.find("\"truncated\":true")!=std::string::npos);
  assert(metadata.find("\"long_foot_counterfactual_ms\",\"long_foot_actual_ms\",\"long_foot_effective\"")!=std::string::npos);
  assert(metadata.find(",145,150,true,")!=std::string::npos);
  assert(metadata.find(",28,33,true,")!=std::string::npos);
  assert(exportFile.prepare());assert(exportFile.status().token==std::string(status.token));
  std::ofstream out(argv[1],std::ios::binary);uint8_t bytes[4096];
  for(uint32_t offset=0;offset<status.bytes;){
    const auto n=std::min<uint32_t>(4096,status.bytes-offset);
    assert(exportFile.chunk(status.token,offset,n,bytes));out.write(reinterpret_cast<char*>(bytes),n);offset+=n;
  }
  out.close();assert(!exportFile.chunk("wrong_token",0,4,bytes));
  assert(exportFile.reset());assert(!exportFile.chunk(status.token,0,4,bytes));
  assert(exportFile.metadata_.c_str()==reserved_metadata);
  assert(exportFile.prepare());exportFile.build();
  assert(exportFile.status().phase==ImmutableExport::Phase::Ready);
  assert(exportFile.metadata_.c_str()==reserved_metadata);
  std::cout<<"maximum autonomous events + 768 foot frames: metadata="<<exportFile.header_.metadata_json_size
      <<" bytes, event PSRAM="<<PsramLogger::eventStorageBytes()<<", logger object="<<sizeof(logger)<<" bytes PASS\n";
  PsramLogger extended;
  assert(extended.begin());
  extended.startRun(2, 223456, 300, 100, 1000, false, 0, 800, 0, false,
      NAN, false, 0, false, true, 0.0f, 40, false, true, true, reference, 12345);
  assert(extended.addSample(row));
  extended.markMeasurementDone(); extended.seal();
  ImmutableExport extended_export;
  assert(extended_export.begin(extended));
  assert(extended_export.prepare()); extended_export.build();
  assert(extended_export.status().phase == ImmutableExport::Phase::Ready);
  const std::string extended_metadata = extended_export.metadata_.c_str();
  assert(extended_metadata.find("\"negative_probe_extended\":true") != std::string::npos);
  assert(extended_metadata.find("seeded_fisher_yates_three_event_blocks_v1") != std::string::npos);
  assert(extended_metadata.find("\"autonomous_duration_ms\":30000") != std::string::npos);
  std::cout << "extended negative batch metadata/export PASS\n";

  // A real work-control run mixes fallback/start-up rows and supported work
  // rows. Exercise both schema orders, transitions in both directions, and
  // the full event capacity through the actual immutable RWLOG exporter.
  for (bool work_first : {false, true}) {
    PsramLogger work;
    assert(work.begin());
    work.startRun(3, 323456, 300, 100, 1000, false, 0, 800, 0, false,
        NAN, false, 0, false, true, 0.0f, 40, false, true, false, reference);
    lateral_cg::Record cg;
    cg.magic=lateral_cg::kMagic;cg.version=2;cg.revision=1;cg.upright_revision=1;
    cg.upright_abs_deg=0.f;cg.source=1;cg.shift_mm=work_first ? .5f : -.5f;
    work.recordLateralCg(work_first ? lateral_cg::Record{} : cg);
    cg.shift_mm=1.5f; // The run retains its snapshot, independently of later edits.
    if (work_first) {
      lateral_cg::AdaptiveSnapshot automatic;
      automatic.enabled=true;automatic.initial_mm=.1f;automatic.shift_mm=.5f;
      automatic.common_bias_mj=.02f;automatic.pairs=5;automatic.updates=3;
      work.recordAdaptiveCg(automatic);
      automatic.shift_mm=1.5f;
    }
    assert(work.addSample(row));
    for (unsigned i = 0; i < 256; ++i) {
      PsramLogger::EnergyControlAutonomousZeroCrossEvent z{};
      z.cg_shift_mm=i%2 ? .35f : -.15f;
      z.work_model_weight=i%2 ? .75f : 1.f;
      z.event_index = i;
      z.event_kind = i == 0 ? 1 : 0;
      z.physical_next_peak_side = i % 2 ? -1 : 1;
      z.energy_model_extrapolated = (i % 3 == 0) != work_first;
      z.work_control_branch = z.energy_model_extrapolated ? 0 : (i % 2 ? 4 : 6);
      z.work_prediction_cross_condition = z.work_control_branch==4 && i%4==1;
      z.energy_model_extrapolated = z.energy_model_extrapolated || z.work_prediction_cross_condition;
      z.work_brake_cap_ms = z.work_control_branch==4 ? (z.work_prediction_cross_condition ? 30 : 25) : 0;
      z.work_bias_samples = z.physical_next_peak_side>0 ? 2 : z.work_control_branch==4 ? 3 : 0;
      if(i==251) {
        // Branch 15 records an accepted drive command, not measured motor current.
        z.work_control_branch=15;z.energy_model_extrapolated=true;
        z.work_prediction_cross_condition=true;z.valid=true;z.output_executed=true;
        z.command_current_mA=1200;z.pulse_width_ms=60;
        z.pulse_start_ms=25100;z.pulse_end_ms=25160;
        z.startup_target_deg=4.5f;z.startup_positive_cap_ms=60;
      }
      if(i>=252 && i<=254) {
        // Experimental branches must serialize with no fitted arrival model.
        z.work_control_branch=static_cast<uint8_t>(i-239);
        z.energy_model_extrapolated=true;
        z.work_prediction_cross_condition=true;
      }
      if(i==254) {
        // The same branch also records a coast with no commanded output.
        z.valid=true;z.output_executed=false;
        z.command_current_mA=0;z.pulse_width_ms=0;
        z.startup_target_deg=4.5f;z.startup_positive_cap_ms=60;
      }
      if(i==255) {
        z.work_control_branch=11;z.work_prediction_cross_condition=true;
        z.energy_model_extrapolated=true;z.predicted_loss_j=.00014f;
        z.foot_balance_target_deg=8.2f;z.foot_balance_left_mean=2.5f;
        z.foot_balance_right_mean=.2f;z.target_energy_j=.00076f;
        z.foot_energy_bias=.08f;z.foot_energy_left_px=20.f;z.foot_energy_right_px=2.f;z.foot_energy_valid=true;z.foot_energy_phase=3;z.foot_energy_reason=4;z.low_peak_bias_mj=.08f;z.low_peak_active=true;
        z.low_peak_selected=true;z.low_peak_base_width_ms=80;z.low_peak_base_prediction_mj=.281f;z.low_peak_applied_bias_mj=.08f;
        z.foot_span_left_deg=2.12345f;z.foot_span_right_deg=.23456f;
        z.foot_span_trimmed_left_deg=2.01234f;z.foot_span_trimmed_right_deg=.12345f;
        z.foot_span_start_ms=1000;z.foot_span_end_ms=1800;
        z.foot_span_baseline_ms=80;z.foot_span_requested_ms=75;z.foot_span_state=4;z.foot_span_accepted_cycles=2;
      }
      // Each flag independently cycles on/off across the 256-event export.
      using Flag=PsramLogger::EnergyControlAutonomousZeroCrossEvent;
      const Flag::FootSpanFlag flags[]={Flag::Span_valid,Flag::Span_selected,Flag::Span_startup,
        Flag::Span_left_low,Flag::Span_right_low,Flag::Span_left_both,Flag::Span_right_both,
        Flag::Span_continuous,Flag::Span_left_outlier,Flag::Span_right_outlier,
        Flag::Span_left_edge_unverified,Flag::Span_right_edge_unverified};
      for(unsigned j=0;j<12;++j) {
        z.setFootSpanFlag(flags[j], (i & (1u << (j % 8))) != 0);
        assert(z.footSpanFlag(flags[j]) == ((i & (1u << (j % 8))) != 0));
      }
      // Clearing one bit must never modify the others.
      const uint16_t saved=z.foot_span_flags;
      z.setFootSpanFlag(Flag::Span_selected,false);
      assert((z.foot_span_flags & ~uint16_t(Flag::Span_selected)) == (saved & ~uint16_t(Flag::Span_selected)));
      z.setFootSpanFlag(Flag::Span_selected, (saved & Flag::Span_selected)!=0);
      work.addEnergyControlAutonomousZeroCrossEvent(z);
    }
    work.markMeasurementDone(); work.seal();
    ImmutableExport work_export;
    assert(work_export.begin(work));
    assert(work_export.prepare()); work_export.build();
    const auto result = work_export.status();
    std::cerr << "mixed work/fallback export: first=" << work_first
        << " bytes=" << work_export.metadata_.length()
        << " error=" << result.error << "\n";
    assert(result.phase == ImmutableExport::Phase::Ready);
    const std::string metadata = work_export.metadata_.c_str();
    assert(metadata.find("\"lateral_cg\":{\"valid\":true") != std::string::npos);
    assert(metadata.find(work_first ? "\"shift_mm\":0.500000" : "\"shift_mm\":-0.500000") != std::string::npos);
    assert(metadata.find("\"autonomous_duration_ms\":30000") != std::string::npos);
    assert(metadata.find("CG_reexpressed_side_energy_bias") != std::string::npos);
    if (work_first) {
      assert(metadata.find("\"final_mm\":0.500000") != std::string::npos);
      assert(metadata.find("\"updates\":3") != std::string::npos);
    }
    assert(metadata.find("\"energy_control_autonomous_zero_cross_event_rendered_count\":256") != std::string::npos);
    const std::string path = std::string(argv[1]) + (work_first ? ".work-first.rwlog" : ".fallback-first.rwlog");
    std::ofstream work_out(path, std::ios::binary);
    for (uint32_t offset = 0; offset < result.bytes;) {
      const auto n = std::min<uint32_t>(sizeof(bytes), result.bytes - offset);
      assert(work_export.chunk(result.token, offset, n, bytes));
      work_out.write(reinterpret_cast<char*>(bytes), n); offset += n;
    }
  }
  std::cout << "mixed work/fallback metadata/export PASS\n";
}
