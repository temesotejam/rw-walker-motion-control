#pragma once
#include <math.h>
#include "psram_string.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include "imu_poll_timer.h"
#include "imu_acquisition_audit.h"
#include "imu_startup_boundary.h"
#include "imu_poll_profile.h"
#include "realtime_code.h"

struct ImuReading {
  bool imu_ok = false;
  // Legacy API fields; ExperimentRunner owns the attitude estimators.
  float pitch_deg = 0.0f;
  float pitch_fixed_beta_deg = 0.0f;
  float pitch_madgwick_beta1_deg = 0.0f;
  float pitch_step_beta_deg = 0.0f;
  float pitch_smooth_beta_deg = 0.0f;
  float pitch_dynamic_beta_deg = 0.0f;
  float pitch_accel_only_deg = 0.0f;
  float pitch_rate_dps = 0.0f;
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
  float acc_norm_g = 0.0f;
  float acc_norm_error_g = 0.0f;
  float beta_fixed = 0.0f;
  float beta_step = 0.0f;
  float beta_target = 0.0f;
  float beta_smooth = 0.0f;
  float beta_tau_s = 0.0f;
  float beta_dynamic = 0.0f;
  uint8_t beta_mode = 0;
  uint8_t beta_update_mode = 0;
  uint16_t time_since_last_pulse_ms = 65535;
  bool accel_fresh = false;
  bool gyro_fresh = false;
  uint8_t sensor_mask = 0;
  bool rate_config_ok = false;
  uint8_t bmi270_acc_conf = 0;
  uint8_t bmi270_gyr_conf = 0;
  uint32_t accel_sequence = 0;
  uint32_t gyro_sequence = 0;
  uint32_t accel_update_dt_us = 0;
  uint32_t gyro_update_dt_us = 0;
  uint32_t last_accel_update_us = 0;
  uint32_t last_gyro_update_us = 0;
  uint32_t acquisition_poll_start_us = 0;
  uint32_t queue_submit_us = 0;  // Immediately before the successful queue-send attempt.
  uint32_t update_dt_us = 0;
  uint32_t last_update_us = 0;
  uint32_t last_update_ms = 0;
  uint32_t error_count = 0;
};

class ImuManager {
 public:
  bool begin();
  void RW_SPEED_CODE update();  // Consume only. Never performs sensor I/O.
  void setAcquisitionContext(bool sequential, bool measurement, uint8_t state_id = 0);
  bool acquisitionHealthy() const;
  String startupDiagnosticsJson() const;
  void setStartupGuideState(const char* reason, bool confirmed, uint32_t hold_ms);
  String acquisitionDiagnosticsJson() const;
  void appendAcquisitionDiagnostics(PsramString& json) const;
  void zeroPitch();
  void setDynamicBetaContext(bool pulse_active, uint32_t time_since_last_pulse_ms, bool pre_start_stabilize = false);
  void forceSmoothBeta(float beta, uint8_t update_mode);

  const ImuReading& reading() const { return reading_; }
  bool ok() const;
  bool stale(uint32_t now_ms) const;
  const char* lastError() const { return last_error_; }

 private:
  template<class Output> void appendDiagnostics(Output& json) const;
  template<class Output> void appendPollProfile(Output& json) const;
  struct StartupView {
    const char* reason = "not_started";
    bool confirmed = false;
    uint32_t hold_ms = 0, age_us = UINT32_MAX, sequence = 0;
    float direction = NAN, norm = NAN, gyro = NAN;
  };
  StartupView startup_view_;
  static constexpr uint32_t kQueueLength = 32;
  static constexpr uint32_t kMaximumDeliveryAgeUs = 10000;
  static constexpr uint8_t kReaderCore = 1;
  static constexpr uint8_t kReaderPriority = 6;
  static bool IRAM_ATTR timerCallback(void* arg);
  static void taskEntry(void* arg);
  bool initializeSensorAttempt();
  bool startAcquisition();
  void RW_SPEED_CODE acquisitionLoop();
  void RW_SPEED_CODE captureSensor();
  void RW_SPEED_CODE recordPollProfile(const ImuPollObservation& observation);
  String pollProfileJson() const;
  void RW_SPEED_CODE publishSample();
  void latchFault(const char* reason, uint32_t sample_us = 0,
                  uint32_t age_us = 0, uint32_t depth = 0);

  // After begin(), the producer exclusively owns capture_ and M5.Imu.
  // The permanent control task exclusively owns reading_, beta context and last_error_.
  ImuReading reading_;
  ImuReading capture_;
  bool imu_present_ = false;
  bool started_ = false;
  uint32_t prev_gyro_update_us_ = 0;
  uint32_t prev_accel_update_us_ = 0;
  bool beta_context_pulse_active_ = false;
  uint32_t beta_context_time_since_last_pulse_ms_ = 65535;
  bool beta_context_pre_start_stabilize_ = false;
  const char* last_error_ = "not_initialized";

  TaskHandle_t acquisition_task_ = nullptr;
  ImuPollTimer acquisition_timer_;
  QueueHandle_t sample_queue_ = nullptr;
  StaticQueue_t queue_storage_;
  alignas(4) uint8_t queue_bytes_[kQueueLength * sizeof(ImuReading)];
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  bool sequential_ = false;
  ImuStartupBoundary boundary_;  // Consumer-owned; never resets a live-run queue.
  uint32_t latest_capture_sequence_ = 0;
  uint32_t boundary_producer_discards_ = 0;
  uint8_t context_state_ = 0;
  uint32_t start_sync_deliveries_ = 0, start_sync_age_max_us_ = 0;
  uint32_t init_attempts_ = 0, init_valid_accel_ = 0, init_valid_gyro_ = 0;
  uint8_t init_internal_status_ = 0, init_power_ctrl_ = 0;
  const char* init_last_failure_ = "";
  const char* startup_guide_reason_ = "initializing";
  bool startup_guide_confirmed_ = false;
  uint32_t startup_guide_hold_ms_ = 0;
  struct FaultSnapshot {
    uint32_t time_us = 0, sample_us = 0, age_us = 0, queue_depth = 0;
    uint32_t latest_sequence = 0;
    uint8_t state_id = 0;
  } fault_snapshot_;

  bool fault_ = false;  // Latched; a fault requires a reboot, never automatic re-arm.
  const char* fault_reason_ = "";
  uint32_t latest_capture_ms_ = 0;
  uint32_t latest_capture_us_ = 0;
  uint32_t total_captured_ = 0;
  int reader_core_ = -1, consumer_core_ = -1;
  uint32_t reader_priority_ = 0, consumer_priority_ = 0;
  int internal_i2c_port_ = -1, internal_sda_ = -1, internal_scl_ = -1;
  struct NotifyStamp {
    bool seen = false;
    uint32_t time_us = 0, gap_us = 0, sequence = 0;
    int core = -1;
  };
  mutable portMUX_TYPE notify_mux_ = portMUX_INITIALIZER_UNLOCKED;
  NotifyStamp notify_stamp_;  // Timer writes; reader takes a small coherent copy.
  ImuPollObservation poll_observation_;  // Reader only, also used before task startup.
  ImuPollProfile poll_profile_;  // Reader writes ONLY during measurement.
  uint32_t previous_poll_start_us_ = 0, previous_poll_total_us_ = 0;
  uint32_t previous_yield_us_ = 0;
  bool have_previous_poll_ = false;
  ImuAcquisitionAudit audit_;
  mutable ImuAcquisitionAudit audit_snapshot_;  // Avoid a large HTTP-task stack object.
};
