"""Stress real v19/v20 serializers at archive capacity with host PSRAM doubles.

The complete PsramLogger and FootObserver serializers and the production runner
metadata methods execute. Camera, acquisition and RTOS are host substitutes;
host sizes and an 8MiB allocation estimate are not an ESP free-heap guarantee.
"""
import argparse
import configparser
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

import convert_rwlog_to_csv as converter

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature); opening = source.index('{', start)
    tokens = re.finditer(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', source[opening:])
    depth = 0
    for token in tokens:
        if token.group() == '{': depth += 1
        elif token.group() == '}':
            depth -= 1
            if not depth: return source[start:opening + token.end()]
    raise AssertionError('Unclosed serializer: ' + signature)


def unique_object(items):
    keys = [key for key, _ in items]
    assert len(keys) == len(set(keys)), 'Duplicate JSON object keys'
    return dict(items)


def validate_tables(value):
    if isinstance(value, dict):
        if 'fields' in value and 'rows' in value:
            assert len(value['fields']) == len(set(value['fields'])), 'Duplicate compact-table columns'
            assert all(len(row) == len(value['fields']) for row in value['rows']), 'Compact row shape drift'
        for item in value.values(): validate_tables(item)
    elif isinstance(value, list):
        for item in value: validate_tables(item)


FIXTURE = r'''
#include <Arduino.h>
#include <cassert>
#include <fstream>
#include <iostream>
#include "compact_json_table.h"
#define private public
#include "psram_logger.h"
#include "foot_observer.h"
#include "experiment_runner.h"
#include "immutable_export.h"
#undef private
#include "foot_fast_policy.h"
#include "camera_frame_ready.h"
#if defined(CAMERA_FRAME_READY_V21) && CAMERA_FRAME_READY_V21
extern "C" rw_camera_frame_ready_stats_t rw_camera_frame_ready_stats(){return {123,4,5,39000};}
#endif
#include "imu_manager.h"
#include "roller485_manager.h"
ImuManager imu;Roller485Manager roller;RunControlWorker run_control;FootObserver feet;ExperimentRunner runner;
RollerTelemetry Roller485Manager::telemetrySnapshot()const{return {};}
void ImuManager::appendAcquisitionDiagnostics(PsramString& json)const{json+="{\"host_fixture\":true}";}
CameraOneShotSnapshot OneShotCamera::snapshot()const{return {};}
bool OneShotCamera::startContinuous(){return false;}
camera_fb_t* OneShotCamera::acquireContinuous(uint32_t){return nullptr;}
void OneShotCamera::releaseContinuous(camera_fb_t*){}
void OneShotCamera::stopContinuous(){}
__WALKING_SERIALIZER__
__FOOT_SUPPORT_SERIALIZER__
int main(int argc,char** argv){
 assert(argc==2);PsramLogger logger;assert(logger.begin());
 logger.startRun(1,1000000,1200,80,1000,false,0,800,0,false,NAN,false,0,false,true,0.f,40,false,true,false);
 logger.recordMeasurementStartAngle(.512345f);
 run_control.snapshot_.state_id=4;std::strcpy(run_control.snapshot_.state_name,"FINISHED");
 run_control.snapshot_.heartbeat_us=123456789;
 for(unsigned i=0;i<PsramLogger::kMaxEnergyControlAutonomousEvents;++i){
  PsramLogger::EnergyControlAutonomousPeakEvent p{};
  p.peak_index=i+1;p.peak_time_ms=100+i*100;p.physical_peak_side=i%2?-1:1;
  p.peak_amplitude_deg=8.1234567f;p.detector_peak_angle_deg=p.physical_peak_side*p.peak_amplitude_deg;
  p.target_peak_deg=8;p.peak_error_deg=-.1234567f;p.target_energy_j=.000733424f;p.measured_energy_j=.000753112f;
  p.pending_command_matched=true;p.pending_q_command_mA_s=17.12345f;p.model_residual_j=.000123456f;
  logger.addEnergyControlAutonomousPeakEvent(p);
  PsramLogger::EnergyControlAutonomousZeroCrossEvent z{};
  z.event_index=i+1;z.zero_cross_time_ms=150+i*100;z.work_control_branch=24;
  z.valid=z.output_executed=true;z.target_peak_deg=z.startup_target_deg=8;
  z.physical_next_peak_side=i%2?-1:1;z.command_current_mA=1200;z.pulse_width_ms=79;
  z.pre_input_measured_current_mA=-12.34567f;z.pre_input_wheel_speed_rpm=456.123456f;
  z.target_energy_j=.000733424f;z.passive_energy_j=.000334588f;z.kinetic_energy_j=.000123456f;
  z.delta_energy_required_j=.000398836f;z.commanded_energy_deficit_j=.000398836f;
  z.predicted_input_work_j=.000401234f;z.predicted_energy_j=.000735822f;
  z.unclamped_energy_request_j=.000398836f;z.q_unclamped_mA_s=17.12345f;z.unclamped_inverse_valid=true;
  z.maximum_input_work_j=.000499999f;z.predicted_current_goal_mA=347.293823f;z.predicted_current_rise_tau_s=.031652216f;
  z.aligned_initial_current_mA=-12.34567f;z.zero_cross_abs_rate_dps=35.12345f;
  logger.addEnergyControlAutonomousZeroCrossEvent(z);logger.addTimingProbeEvent({});logger.addSolverAuditEvent({});
 }
 assert(runner.walking_energy_audit_.begin(ps_malloc));
 lateral_energy::Geometry geometry;lateral_energy::CurrentTable current;assert(current.prepare(.031652216f));
 for(unsigned i=0;i<192;++i){
  walking_return_energy::Input in;in.side=i%2?-1:1;in.rate_dps=35.12345f;in.body_angle_deg=.12345f;
  in.target_deg=8;in.previous_peak_deg=8.123456f;in.same_side_peak_deg=7.123456f;
  in.initial_current_ma=-12.34567f;in.goal_current_ma=347.293823f;in.return_work_j=.00015f;
  const bool aid=i%3==0;
  const auto d=aid?half_cycle_energy_budget::decideReturnAid(geometry,current,in):walking_return_energy::decide(geometry,current,in);
  assert(d.valid);auto& a=runner.walking_energy_audit_[i];a.decision=d;
  a.time_ms=1000+i*150;a.peak_ms=a.time_ms-200;a.side=in.side;a.trigger_ms=a.time_ms;
  a.requested_ms=a.guarded_ms=d.width_ms;a.executed=true;a.optional=i%3==1;a.rescue=aid;a.return_event=aid;
  a.arrival_bias_j=aid?0.f:.000123456f;a.return_work_j=aid?in.return_work_j:0.f;
  a.return_deceleration_dps2=123.45678f;a.return_stopping_distance_deg=2.123456f;a.return_foot_age_ms=57;
  a.return_evidence=aid?2:0;a.assisted_cycle=aid;a.envelope_before_ms=80;a.envelope_after_ms=0;
  a.output_start_ms=a.time_ms+1;a.start_i0_ma=in.initial_current_ma;a.start_goal_ma=in.goal_current_ma;
  a.start_tau_s=current.tau;a.start_rate_dps=in.rate_dps;
  energy_pulse_completion::Input c;c.side=in.side;c.rate_dps=in.rate_dps;c.initial_current_ma=in.initial_current_ma;
  c.goal_current_ma=in.goal_current_ma;c.tau_s=current.tau;c.available_j=d.available_j;c.request_j=d.request_j;
  c.planned_ms=d.width_ms;c.actual_elapsed_ms=d.width_ms+1;a.completion=energy_pulse_completion::evaluate(c);
  a.output_end_ms=a.output_start_ms+c.actual_elapsed_ms;a.completion_reason=1;
  const auto admission=energy_admission_v19::assess(aid?energy_admission_v19::Scope::ExplicitReturnAid:
      a.optional?energy_admission_v19::Scope::OptionalMain:energy_admission_v19::Scope::OrdinaryZero,d);
  a.energy_admission_checked=true;a.energy_admission_applies=admission.applies;a.energy_admission_allowed=admission.allowed;
  a.energy_unsupported_mask=admission.unsupported_mask;a.energy_admission_reason=static_cast<uint8_t>(admission.reason);
  a.fore_aft_allowed=true;a.fore_aft_reason=0;a.fore_aft_angle_deg=.123456789f;a.fore_aft_rate_dps=1.23456789f;
  ++runner.walking_energy_count_;++runner.walking_energy_total_;
 }
 for(unsigned i=0;i<96;++i){auto& a=runner.foot_support_audit_[i];
  a.time_ms=1000+i*300;a.peak_ms=a.time_ms-200;a.sequence=100000+i;a.frame_ms=a.time_ms-50;
  a.left=3.12345f;a.right=4.12345f;a.confidence=.76543f;a.left_uncertainty=1.23456f;a.right_uncertainty=1.34567f;
  a.body_angle=2.12345f;a.body_rate=35.12345f;a.threshold=4.12345f;a.estimated_zero_ms=12345.6789f;
  a.normal_width=a.requested_width=a.guarded_width=a.normal_guarded_width=a.boosted_guarded_width=79;
  a.next_side=i%2?-1:1;a.executed=a.anticipated_shape_need=a.inward_decelerating=a.local_returning=a.local_turn=true;
  a.prediction_source=2;a.mode=a.candidate_mode=3;a.checked_ms=a.time_ms;a.local_evidence=.76543f;
  a.preview_speed_rpm=456.12345f;a.preview_speed_age_us=12345;a.backend_reason=9;
  ++runner.foot_support_audit_count_;++runner.foot_support_audit_total_;
 }
 feet.frames_=static_cast<FootFrame*>(ps_malloc(sizeof(FootFrame)*FootObserver::kCapacity));assert(feet.frames_);
 feet.status_.available=feet.status_.zero_ready=true;feet.status_.zero_reason=FootZeroReason::Ready;
 feet.status_.count=FootObserver::kCapacity;
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
 feet.status_.row_cache_ready=true;feet.status_.recognition_max_us=9876;feet.status_.row_copy_max_us=1234;
#endif
 for(unsigned i=0;i<FootObserver::kCapacity;++i){
  FootFrame f{};f.sequence=100000+i;f.run_id=1;f.state_id=3;
  f.frame_us=123456789012ULL+i*40000ULL;f.delivered_us=f.frame_us+50000;
  f.log_time_us=5000000+i*40000;f.measurement_time_us=i*40000;f.imu_sample_us=123456789+i*40000;
  f.control_observed_us=f.delivered_us+123456;f.processing_us=12345;
  f.frame_valid=f.timestamp_valid=f.zero_ready=f.right_valid=f.left_valid=true;
  f.right_in_range=f.left_in_range=true;f.right_x=181.45678f;f.left_x=175.12345f;
  f.right_deg=3.123456f;f.left_deg=4.123456f;f.right_scan_y=78;f.left_scan_y=188;
  f.right_weight=12345.6789f;f.left_weight=23456.789f;f.right_contrast=200.12345f;f.left_contrast=190.12345f;
  f.right_reason=f.left_reason=MarkerDetectionReason::Detected;f.right_templates=f.left_templates=17;
  f.right_candidates=f.left_candidates=5;f.right_ambiguity=f.left_ambiguity=.456789f;f.zero_reason=FootZeroReason::Ready;
  auto& p=f.pose_projection;p.run_id=1;p.frame_ms=i*40;p.received_ms=p.frame_ms+50;
  p.reason=foot_phase_prediction::Reason::Ready;p.confidence=.76543f;
  for(unsigned k=0;k<4;++k){p.left[k]=4.123456f+k*.123456f;p.right[k]=3.123456f+k*.123456f;
    p.left_uncertainty[k]=1.23456f;p.right_uncertainty[k]=1.34567f;}
  auto& local=f.local_projection;local.run_id=1;local.frame_ms=p.frame_ms;local.received_ms=p.received_ms;
  local.reason=foot_local_prediction::Reason::Ready;
  for(auto* model:{&local.left_model,&local.right_model}){model->observed=4.123456f;model->trend=-12.3456f;
    model->latest_rate=-13.4567f;model->rate_difference=1.11111f;model->span=3.12345f;
    model->net_change=-2.12345f;model->phase=foot_local_prediction::Phase::Returning;model->qualified=true;}
  f.pose_compute_us=123;f.local_compute_us=234;
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
  f.released_us=f.delivered_us+1234;f.ready_us=f.delivered_us+12345;
  f.capture_wait_us=2345;f.row_copy_us=1234;f.recognition_us=9876;f.row_cache_used=true;
#endif
  feet.frames_[i]=f;
 }
 feet.control_observation_attempts_=FootObserver::kCapacity;
 logger.markMeasurementDone();logger.seal();PsramString json;assert(json.reserve(PsramLogger::kMetadataJsonReserveBytes));
 const bool built=logger.buildMetadataJson(json);
 // Exercise the production export callback throughout real maximum-capacity
 // serializers. Simulated CPU time is not a board performance measurement.
 ImmutableExport export_job;export_job.logger_=&logger;
 export_job.metadata_yield_us_=host_us=UINT32_MAX-5000;
 PsramString cooperative;assert(cooperative.reserve(PsramLogger::kMetadataJsonReserveBytes));
 cooperative.setProgress([](void* context,size_t bytes){
   auto* job=static_cast<ImmutableExport*>(context);
   host_us+=1000;const auto before=host_us;
   ImmutableExport::metadataProgress(job,bytes);
   assert(static_cast<uint32_t>(host_us-before)==0 || static_cast<uint32_t>(host_us-before)==1000);
   assert(static_cast<uint32_t>(host_us-job->metadata_yield_us_)<2000);
 },&export_job);
 assert(logger.buildMetadataJson(cooperative));cooperative.setProgress(nullptr);
 assert(export_job.status().metadata_bytes>json.length()-4096);
 assert(std::string(json.c_str())==cooperative.c_str());
 // Run the actual worker through immutable publication and CRC validation.
 assert(export_job.reserveMetadata());export_job.build();
 const auto exported=export_job.status();
 assert(exported.phase==ImmutableExport::Phase::Ready && exported.metadata_bytes==json.length());
 assert(export_job.metadata_.progress_==nullptr);
 std::vector<uint8_t> binary(exported.bytes);
 for(uint32_t offset=0;offset<exported.bytes;){
   const uint32_t n=std::min<uint32_t>(4096,exported.bytes-offset);
   assert(export_job.chunk(exported.token,offset,n,binary.data()+offset));offset+=n;
 }
 assert(export_protocol::crc32(0,binary.data(),binary.size()-4)==exported.crc);
 ImmutableExport failed;failed.logger_=&logger;assert(failed.metadata_.reserve(128));failed.build();
 assert(failed.status().phase==ImmutableExport::Phase::Error && failed.metadata_.progress_==nullptr);
 assert(!failed.chunk("",0,1,binary.data()));
 std::ofstream output(argv[1],std::ios::binary);output.write(json.c_str(),json.length());output.close();
 const size_t samples=Config::LOG_BUFFER_BYTES/sizeof(LogSample)*sizeof(LogSample);
 const size_t core=samples+PsramLogger::eventStorageBytes()+sizeof(FootFrame)*FootObserver::kCapacity+
   sizeof(ExperimentRunner::WalkingEnergyAudit)*192+PsramLogger::kMetadataJsonReserveBytes+1;
 const size_t camera=320*240*(
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
   foot_fast_policy::kCameraFrameBuffers
#else
   1
#endif
 );
 std::cout<<"{\"built\":"<<(built?"true":"false")<<",\"metadata_bytes\":"<<json.length()
   <<",\"event_storage_bytes\":"<<PsramLogger::eventStorageBytes()<<",\"frame_bytes\":"<<sizeof(FootFrame)
   <<",\"foot_capacity\":"<<FootObserver::kCapacity<<",\"energy_row_bytes\":"<<sizeof(ExperimentRunner::WalkingEnergyAudit)
   <<",\"energy_capacity\":192,\"log_sample_bytes\":"<<sizeof(LogSample)<<",\"log_allocated_bytes\":"<<samples
   <<",\"metadata_reserve_bytes\":"<<PsramLogger::kMetadataJsonReserveBytes
   <<",\"core_psram_estimate_bytes\":"<<core<<",\"camera_frame_estimate_bytes\":"<<camera
   <<",\"preview_bytes\":"<<FootObserver::kPreviewBytes
   <<",\"idle_thumbnail_staging_psram_bytes\":"<<
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
   FootObserver::kPreviewBytes
#else
   0
#endif
   <<",\"row_cache_internal_bytes\":"<<
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
   foot_image_rows::kRequiredBytes
#else
   0
#endif
   <<",\"logger_object_bytes\":"<<sizeof(PsramLogger)<<",\"runner_object_bytes\":"<<sizeof(ExperimentRunner)<<"}\n";
}
'''


def flags(v20):
    cfg = configparser.ConfigParser(interpolation=None); cfg.read(ROOT / 'platformio.ini', encoding='utf8')
    def expand(section):
        return re.sub(r'\$\{([^}]+)\.build_flags\}', lambda m: expand(m[1]), cfg[section]['build_flags'])
    return [f for f in expand('env:atoms3cam-foot-fast' if v20 else 'env:atoms3cam-energy-admission').split() if f.startswith('-D')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, help='Optional JSON report destination')
    parser.add_argument('--revisions',type=int,nargs='+',choices=[19,20,21,22,23,24],default=[19,20,21,22,23,24])
    args = parser.parse_args()
    cpp = (ROOT / 'src/experiment_runner.cpp').read_text(encoding='utf8')
    inc = (ROOT / 'src/walking_energy_runner.inc').read_text(encoding='utf8')
    fixture = FIXTURE.replace('__WALKING_SERIALIZER__', function(inc, 'void ExperimentRunner::appendWalkingEnergyMetadata('))
    fixture = fixture.replace('__FOOT_SUPPORT_SERIALIZER__', function(cpp, 'void ExperimentRunner::appendFootSupportMetadata('))
    sources = ['src/immutable_export.cpp', 'src/psram_logger.cpp', 'src/foot_observer.cpp', 'src/foot_angle_estimator.cpp',
               'src/white_marker_tracker.cpp', 'src/control_work_profile.cpp', 'src/control_latency.cpp', 'src/mekf6.cpp',
               'tools/fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_Madgwick.cpp']
    reports = []
    with tempfile.TemporaryDirectory(prefix='foot_fast_metadata_') as folder:
        temp = Path(folder); (temp / 'fixture.cpp').write_text(fixture, encoding='utf8')
        for revision in args.revisions: # 23=v22-fix1; 24=v23 capture fixture.
            v20 = revision >= 20
            additions = ['-DEXPORT_RETRY_FIX=1', '-DEXPORT_COOPERATIVE_FIX=1', '-DCAMERA_FRAME_READY_V21=1'] if revision >= 21 else []
            if revision >= 22: additions += ['-DFOOT_SPEED_V22=1']
            if revision >= 23: additions += ['-DFOOT_SPEED_FIX1=1']
            if revision == 24: additions += ['-DFOOT_CAPTURE_V23=1']
            exe = temp / ('v'+str(revision)+'.exe'); destination = exe.with_suffix('.json')
            subprocess.run(['g++', '-std=c++17', '-O0', '-Wall', '-Wextra', '-Werror', '-Wno-format',
                            '-DMALLOC_CAP_INTERNAL=4',
                            '-ffunction-sections', '-fdata-sections', '-I' + str(ROOT / 'tools/host_v46o'),
                            '-I' + str(ROOT / 'src')] + flags(v20) + additions + [str(temp / 'fixture.cpp')] +
                           [str(ROOT / s) for s in sources] + ['-Wl,--gc-sections', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe), str(destination)], text=True, capture_output=True, check=True)
            size = json.loads(result.stdout); blob = destination.read_bytes()
            metadata = json.loads(blob, object_pairs_hook=unique_object,
                                  parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
            validate_tables(metadata)
            expanded = converter.expand_tables(metadata)
            size.update(version=revision, hard_budget_bytes=metadata['metadata_json_budget_bytes'],
                        foot_rows=len(expanded['foot_frames']), energy_rows=len(expanded['walking_energy']['records']))
            size['non_driver_psram_remaining_estimate_bytes'] = (8*1024*1024 - size['core_psram_estimate_bytes'] -
                size['camera_frame_estimate_bytes'] - size['preview_bytes'] - size['idle_thumbnail_staging_psram_bytes'])
            for key in ['foot_pose_prediction', 'foot_local_prediction', 'walking_energy', 'foot_support_assist']:
                table = expanded[key]
                assert table['stored'] == len(table['records']) and table['total'] >= table['stored']
                assert table['truncated'] == (table['stored'] < table['total'])
                size[key + '_counts'] = {k: table[k] for k in ['stored', 'total', 'truncated']}
            reports.append(size); print(json.dumps(size, indent=2))
            assert size['built'], 'Actual full metadata serializer failed its budget'
            assert size['metadata_bytes'] == len(blob) == metadata['metadata_json_final_bytes']
            assert metadata['metadata_json_reserve_bytes'] == size['metadata_reserve_bytes']
            assert size['metadata_bytes'] <= size['hard_budget_bytes']
            assert metadata['diagnostic_build_id'] == {19:'walking-energy-admission-v19',20:'walking-foot-fast-v20',21:'walking-foot-frame-ready-v21',22:'walking-foot-speed-v22',23:'walking-foot-speed-v22-fix1',24:'walking-foot-capture-v23'}[revision]
            assert metadata['energy_model'] == 'walking_energy_admission_v19'
            assert expanded['walking_energy']['revision'] == 'walking_energy_admission_v19' and expanded['walking_energy']['branch'] == 24
            assert size['foot_rows'] == size['foot_capacity'] == (1280 if v20 else 768)
            assert size['hard_budget_bytes'] == (1024*1024 if v20 else 640*1024)
            assert size['metadata_reserve_bytes'] == (1152*1024 if v20 else 768*1024)
            if v20:
                assert size['energy_rows'] == 192, 'V20 foot export must leave room for all energy rows'
                assert expanded['foot_support_assist']['stored'] == 96, 'V20 must retain support audit capacity'
                for key in ['ready_us', 'released_us', 'capture_wait_us', 'row_copy_us', 'recognition_us', 'row_cache_used']:
                    assert key in expanded['foot_frames'][0], key
                assert expanded['foot_frames'][-1]['sequence'] == 101279
                assert expanded['foot_frames'][-1]['control_observed_us'] == 123456789012 + 1279*40000 + 50000 + 123456
                assert expanded['foot_frames'][-1]['row_cache_used'] is True
                observation = expanded['foot_observation']
                assert observation['target_fps'] == (30 if revision >= 22 else 25) and observation['capacity'] == 1280
                assert observation['row_cache_ready'] is True and observation['row_cache_internal_bytes'] == 21120
                assert observation['performance_validated_on_board'] is False and observation['all_frame_control_receipts_exported'] is True
                assert size['non_driver_psram_remaining_estimate_bytes'] > 0
            if revision >= 21:
                assert observation['frame_publication'] == 'complete_qvga_y8_dma_before_vsync_v21'
                assert [observation[k] for k in ['dma_early_frames','dma_vsync_frames','dma_queue_drops','dma_last_full_age_us']] == [123,4,5,39000]
            if revision >= 22:
                assert observation['sensor_hblank']==(128 if revision>=23 else 64) and observation['worker_min_period_us']==33333
                assert observation['contrast_reuse']=='exact_integer_vertical_rolling_v22'
            if revision >= 23:
                assert observation['sensor_speed_profile']==('gc0308_div7_hb128_sdk_aec_v23' if revision==24 else 'gc0308_hb128_sdk_aec_v22_fix1')
                assert observation['sensor_period_estimate_ms']==(35.96 if revision==24 else 41.1)
                if revision==24:
                    assert observation['sensor_xclk_nominal_hz']==22857142
                    assert [observation[k] for k in ['sensor_xclk_divider','sensor_xclk_fraction_num','sensor_xclk_fraction_den']]==[7,0,0]
                assert observation['sensor_aec_modified'] is False
                assert observation['sensor_flicker_step_rows']==120 and observation['sensor_max_exposure_level_rows']==480
                assert observation['sensor_profile_applied'] is False and observation['sensor_profile_stage']=='not_attempted'
            csv_dir = temp / ('v'+str(revision)+'_csv'); csv_dir.mkdir()
            assert converter.write_foot_frames(expanded, csv_dir) == size['foot_rows']
            assert converter.write_walking_energy_diagnostics(expanded, csv_dir) == size['energy_rows']
            with (csv_dir / 'foot_angles.csv').open(encoding='utf8', newline='') as stream:
                foot_csv = list(csv.DictReader(stream))
            assert len(foot_csv) == size['foot_rows']
            assert ('ready_us' in foot_csv[0]) == v20
            if v20:
                assert int(foot_csv[-1]['ready_us']) == 123456789012 + 1279*40000 + 50000 + 12345
                assert foot_csv[-1]['row_cache_used'] == 'True'
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps({'host_only': True, 'sizes': reports,
                'limits': 'Host ABI sizes and expected explicit PSRAM buffers; camera driver, allocator fragmentation and other allocations require real-board verification.',
                'source_sha256': {s: hashlib.sha256((ROOT / s).read_bytes()).hexdigest() for s in sources +
                    ['src/psram_logger.h', 'src/foot_observer.h', 'src/foot_fast_policy.h',
                     'src/walking_energy_runner.inc', 'src/experiment_runner.cpp']}}, indent=2) + '\n', encoding='utf8')
    print('real foot + energy metadata capacity, budgets, schema/revisions and host PSRAM estimate PASS',args.revisions)


if __name__ == '__main__':
    main()
