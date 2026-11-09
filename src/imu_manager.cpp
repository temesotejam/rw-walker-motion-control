#include "imu_manager.h"
#include "runtime_diagnostics.h"
#include <math.h>
#include <M5Unified.h>
#include "config.h"
#include "upright_pose_guide.h"
#include "bmi270_timing_reader.h"
#include "imu_i2c_transport.h"
#include "control_latency.h"
#include "realtime_code.h"


namespace {
// Identical V46p expressions, relocated, not approximated. Run only at startup
// or when the consumer sees a new accel sequence (normally 200Hz, not 400Hz).
void RW_SPEED_CODE updateDerivedAccel(ImuReading& r) {
  r.acc_norm_g = sqrtf(r.ax_g*r.ax_g + r.ay_g*r.ay_g + r.az_g*r.az_g);
  r.acc_norm_error_g = r.acc_norm_g - 1.0f;
  r.pitch_accel_only_deg = Config::PITCH_SIGN * atan2f(-r.ax_g,
      sqrtf(r.ay_g*r.ay_g + r.az_g*r.az_g)) * 57.2957795f;
}
}

static_assert(configTICK_RATE_HZ == 1000, "V46p requires one-millisecond RTOS ticks");

bool ImuManager::begin() {
  // Startup retries ONLY, before a reader/task is created. Runtime faults are
  // never cleared by begin(), Clear log, or another Start request.
  if (started_ || fault_) return ok();
  delay(100);  // Allow the supply/IMU to settle after a cold battery connection.
  for (uint32_t attempt = 0; attempt < 5; ++attempt) {
    ++init_attempts_;
    reading_ = ImuReading{};
    capture_ = ImuReading{};
    prev_gyro_update_us_ = prev_accel_update_us_ = 0;
    init_valid_accel_ = init_valid_gyro_ = 0;
    init_internal_status_ = init_power_ctrl_ = 0;
    if (initializeSensorAttempt()) return startAcquisition();
    init_last_failure_ = last_error_;
    reading_.imu_ok = false;
    reading_.rate_config_ok = false;
    if (attempt < 4) delay(200);
  }
  return false;
}

bool ImuManager::initializeSensorAttempt() {
  // Roller uses Arduino Wire controller 0. Do not start with a shared controller.
  internal_i2c_port_ = static_cast<int>(M5.In_I2C.getPort());
  internal_sda_ = M5.In_I2C.getSDA();
  internal_scl_ = M5.In_I2C.getSCL();
  if (internal_i2c_port_ != 1) {
    last_error_ = "imu_internal_i2c_must_be_separate_from_roller_port0";
    return false;
  }
  imu_present_ = M5.Imu.begin(&M5.In_I2C, M5.getBoard());
  if (!imu_present_) {
    last_error_ = "imu_init_failed";
    return false;
  }
  if (M5.Imu.getType() != m5::imu_bmi270) {
    last_error_ = "unexpected_imu_type_not_bmi270";
    return false;
  }
  // V46v timing-only change: BMI270 supports Fast-mode Plus up to 1 MHz.
  // Keep the same internal bus, axes, ODR and estimator path; only shorten transfers.
  M5.Imu.setClock(Config::BMI270_I2C_HZ);
  auto* dev = M5.Imu.getImuInstancePtr(0);
  if (!dev) { last_error_ = "bmi270_instance_missing"; return false; }
  // Library begin() alone is insufficient: also check BMI270 initialization
  // status and power enable bits, then require an actual accel/gyro stream.
  const uint32_t status_start_ms = millis();
  do {
    init_internal_status_ = dev->readRegister8(0x21);
    if ((init_internal_status_ & 0x0Fu) == 1u) break;
    delay(5);
  } while (static_cast<uint32_t>(millis() - status_start_ms) < 200);
  init_power_ctrl_ = dev->readRegister8(0x7D);
  if ((init_internal_status_ & 0x0Fu) != 1u || (init_power_ctrl_ & 0x06u) != 0x06u) {
    last_error_ = "bmi270_startup_status_or_power_invalid";
    return false;
  }
  constexpr uint8_t kAccConf = 0x40, kGyrConf = 0x42;
  const uint8_t acc0 = dev->readRegister8(kAccConf);
  const uint8_t gyr0 = dev->readRegister8(kGyrConf);
  const uint8_t acc_target = static_cast<uint8_t>((acc0 & 0xF0u) | Config::BMI270_ACCEL_ODR_CODE);
  const uint8_t gyr_target = static_cast<uint8_t>((gyr0 & 0xF0u) | Config::BMI270_GYRO_ODR_CODE);
  const bool write_ok = dev->writeRegister8(kAccConf, acc_target) && dev->writeRegister8(kGyrConf, gyr_target);
  delay(2);
  reading_.bmi270_acc_conf = dev->readRegister8(kAccConf);
  reading_.bmi270_gyr_conf = dev->readRegister8(kGyrConf);
  reading_.rate_config_ok = write_ok &&
      ((reading_.bmi270_acc_conf & 0x0Fu) == Config::BMI270_ACCEL_ODR_CODE) &&
      ((reading_.bmi270_gyr_conf & 0x0Fu) == Config::BMI270_GYRO_ODR_CODE);
  if (!reading_.rate_config_ok) { last_error_ = "bmi270_odr_config_failed"; return false; }
  reading_.imu_ok = true;
  reading_.last_update_ms = millis();
  capture_ = reading_;
  const uint32_t warmup_start_ms = millis();
  while (static_cast<uint32_t>(millis() - warmup_start_ms) < 400) {
    const uint32_t a_seq = capture_.accel_sequence, g_seq = capture_.gyro_sequence;
    captureSensor();  // No task/queue yet: same sensor-owner code, synchronous startup only.
    if (capture_.accel_sequence != a_seq) updateDerivedAccel(capture_);
    if (capture_.accel_sequence != a_seq && isfinite(capture_.acc_norm_g) &&
        capture_.acc_norm_g >= UprightPoseGuide::UPRIGHT_MIN_ACCEL_NORM_G &&
        capture_.acc_norm_g <= UprightPoseGuide::UPRIGHT_MAX_ACCEL_NORM_G) ++init_valid_accel_;
    if (capture_.gyro_sequence != g_seq) ++init_valid_gyro_;
    if (init_valid_accel_ >= 8 && init_valid_gyro_ >= 16) {
      reading_ = capture_;
      last_error_ = "";
      return true;
    }
    delay(1);
  }
  last_error_ = "bmi270_startup_stream_not_valid";
  return false;
}

bool ImuManager::startAcquisition() {
  // No task can access the internal bus yet. BMI configuration/conversion stay
  // in M5Unified; only getImuRawData's register transport changes after boot.
  auto* device = M5.Imu.getImuInstancePtr(0);
  if (!device || !M5.In_I2C.release() ||
      !imu_i2c::begin(internal_i2c_port_, internal_sda_, internal_scl_,
                      device->getAddress(), Config::BMI270_I2C_HZ)) {
    last_error_ = "imu_interrupt_transport_init_failed";
    latchFault(last_error_); return false;
  }
  bmi270_timing::transport() = &imu_i2c::read;
  sample_queue_ = xQueueCreateStatic(kQueueLength, sizeof(ImuReading), queue_bytes_, &queue_storage_);
  if (!sample_queue_) { last_error_ = "imu_queue_create_failed"; return false; }
  if (xTaskCreatePinnedToCore(&ImuManager::taskEntry, "bmi270_reader", 4096,
          this, kReaderPriority, &acquisition_task_, kReaderCore) != pdPASS) {
    last_error_ = "imu_task_create_failed";
    return false;
  }
  if (!acquisition_timer_.begin(&ImuManager::timerCallback, this, Config::IMU_POLL_PERIOD_US)) {
    // begin() quiesces/frees a partially installed IRQ before returning failure.
    vTaskDelete(acquisition_task_);
    acquisition_task_ = nullptr;
    last_error_ = "imu_timer_start_failed";
    return false;
  }
  started_ = true;
  return true;
}

bool ImuManager::timerCallback(void* arg) {
  // Core1 hardware ISR: stamp and notify only. IDF clears/rearms the alarm and
  // performs portYIELD_FROM_ISR after this callback returns its wake flag.
  auto* self = static_cast<ImuManager*>(arg);
  const uint32_t stamp = static_cast<uint32_t>(esp_timer_get_time());
  portENTER_CRITICAL_ISR(&self->notify_mux_);
  if (self->notify_stamp_.seen) self->notify_stamp_.gap_us = stamp - self->notify_stamp_.time_us;
  self->notify_stamp_.seen = true;
  self->notify_stamp_.time_us = stamp;
  ++self->notify_stamp_.sequence;
  self->notify_stamp_.core = xPortGetCoreID();
  portEXIT_CRITICAL_ISR(&self->notify_mux_);
  // Do not notify while holding a spinlock. No I2C, math or JSON in callback.
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(self->acquisition_task_, &woken);
  return woken == pdTRUE;
}
void ImuManager::taskEntry(void* arg) {
  static_cast<ImuManager*>(arg)->acquisitionLoop();
}
void ImuManager::acquisitionLoop() {
  const int core = xPortGetCoreID();
  const uint32_t priority = uxTaskPriorityGet(nullptr);
  portENTER_CRITICAL(&mux_);
  reader_core_ = core;
  reader_priority_ = priority;
  portEXIT_CRITICAL(&mux_);
  for (;;) {
    RuntimeDiag::phase(RuntimeDiag::Lane::Imu, RuntimeDiag::Phase::Wait);
    const uint32_t wakes = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    RuntimeDiag::phase(RuntimeDiag::Lane::Imu, RuntimeDiag::Phase::ImuRead);
    NotifyStamp notification;
    portENTER_CRITICAL(&notify_mux_);
    notification = notify_stamp_;
    portEXIT_CRITICAL(&notify_mux_);
    // Clock AFTER the snapshot: a callback on the other core cannot cause
    // unsigned underflow. Coalescing means this is latest-notification age.
    control_latency::pollBegin();
    const uint32_t t0 = micros();
    poll_observation_ = ImuPollObservation{};
    poll_observation_.start_us = t0;
    poll_observation_.wakes = wakes;
    poll_observation_.has_notify = notification.seen;
    poll_observation_.notify_age_us = notification.seen ? t0 - notification.time_us : 0;
    poll_observation_.callback_gap_us = notification.gap_us;
    poll_observation_.callback_sequence = notification.sequence;
    poll_observation_.period_us = have_previous_poll_ ? t0 - previous_poll_start_us_ : 0;
    poll_observation_.previous_total_us = previous_poll_total_us_;
    poll_observation_.previous_yield_us = previous_yield_us_;
    previous_poll_start_us_ = t0;
    have_previous_poll_ = true;
    captureSensor();
    const uint32_t elapsed = static_cast<uint32_t>(micros() - t0);
    poll_observation_.total_us = elapsed;
    poll_observation_.forced_yield = elapsed >= Config::IMU_POLL_PERIOD_US;
    previous_poll_total_us_ = elapsed;
    portENTER_CRITICAL(&mux_);
    audit_.poll(elapsed, wakes);
    const bool allow_stack_scan = !sequential_;
    portEXIT_CRITICAL(&mux_);
    RuntimeDiag::phase(RuntimeDiag::Lane::Imu, RuntimeDiag::Phase::ImuAudit);
    recordPollProfile(poll_observation_);
    RuntimeDiag::beat(RuntimeDiag::Lane::Imu, elapsed, allow_stack_scan);
    control_latency::pollEnd();
    previous_yield_us_ = 0;
    // Keep the established overrun wait. Removing it could starve control/STOP.
    const uint32_t yield_start = micros();
    if (elapsed >= Config::IMU_POLL_PERIOD_US) vTaskDelay(1);
    if (elapsed >= Config::IMU_POLL_PERIOD_US) {
      previous_yield_us_ = static_cast<uint32_t>(micros() - yield_start);
      portENTER_CRITICAL(&mux_);
      const bool same_measurement = audit_.active && audit_.epoch_us == poll_profile_.epoch_us;
      portEXIT_CRITICAL(&mux_);
      if (same_measurement) poll_profile_.stage[ImuPollProfile::YIELD].add(previous_yield_us_);
    }
  }
}

void ImuManager::recordPollProfile(const ImuPollObservation& observation) {
  // Same-core writer priority6 exceeds both possible context owners (4/2).
  // The profile is never modified when measurement is inactive, so stopped
  // HTTP export needs neither a 20KB stack copy nor a long critical section.
  portENTER_CRITICAL(&mux_);
  const bool active = audit_.active;
  const uint32_t epoch = audit_.epoch_us;
  portEXIT_CRITICAL(&mux_);
  if (!active) return;
  const uint32_t t0 = micros();
  if (!poll_profile_.initialized || poll_profile_.epoch_us != epoch) poll_profile_.start(epoch);
  poll_profile_.record(observation);
  ImuPollProfile::maximum(poll_profile_.record_overhead_max_us,
                         static_cast<uint32_t>(micros() - t0));
}

void ImuManager::captureSensor() {
  capture_.accel_fresh = false;
  capture_.gyro_fresh = false;
  capture_.sensor_mask = 0;
  const uint32_t now_us = micros();
  const auto mask = M5.Imu.update();
  const uint32_t update_done_us = micros();
  const auto driver = bmi270_timing::lastRead();
  poll_observation_.driver_called = driver.called;
  poll_observation_.driver_status_us = driver.status_us;
  poll_observation_.driver_data_us = driver.data_us;
  poll_observation_.driver_data_bytes = driver.data_bytes;
  poll_observation_.driver_failures = driver.failures;
  poll_observation_.update_us = static_cast<uint32_t>(update_done_us - now_us);
  const uint8_t bits = static_cast<uint8_t>(mask);
  poll_observation_.mask = bits;
  if (bits == 0) return;  // No fresh value is not itself a read error.
  const uint32_t convert_start_us = micros();
  const auto d = M5.Imu.getImuData();
  const uint32_t convert_done_us = micros();
  poll_observation_.convert_us = static_cast<uint32_t>(convert_done_us - convert_start_us);
  // M5Unified's host acquisition timestamp, NOT the BMI270 hardware sensor clock.
  const uint32_t sample_us = d.usec ? d.usec : now_us;
  const bool accel_new = bits & static_cast<uint8_t>(m5::IMU_Class::sensor_mask_accel);
  const bool gyro_new = bits & static_cast<uint8_t>(m5::IMU_Class::sensor_mask_gyro);
  if ((accel_new && (!isfinite(d.accel.x) || !isfinite(d.accel.y) || !isfinite(d.accel.z))) ||
      (gyro_new && (!isfinite(d.gyro.x) || !isfinite(d.gyro.y) || !isfinite(d.gyro.z)))) {
    if (started_) latchFault("imu_nonfinite_sample", sample_us);
    else last_error_ = "bmi270_startup_nonfinite_sample";
    return;
  }
  capture_.sensor_mask = bits;
  capture_.accel_fresh = accel_new;
  capture_.gyro_fresh = gyro_new;
  if (accel_new) {
    capture_.ax_g = d.accel.x; capture_.ay_g = d.accel.y; capture_.az_g = d.accel.z;
    // Derived accel norm/angle are evaluated by the consumer, not the reader.
    capture_.accel_update_dt_us = prev_accel_update_us_ ? sample_us - prev_accel_update_us_ : 1000000UL / Config::BMI270_ACCEL_ODR_HZ;
    prev_accel_update_us_ = sample_us;
    capture_.last_accel_update_us = sample_us;
    ++capture_.accel_sequence;
  }
  if (gyro_new) {
    capture_.gx_dps = d.gyro.x; capture_.gy_dps = d.gyro.y; capture_.gz_dps = d.gyro.z;
    capture_.pitch_rate_dps = Config::GYRO_PITCH_RATE_SIGN * capture_.gy_dps;
    capture_.gyro_update_dt_us = prev_gyro_update_us_ ? sample_us - prev_gyro_update_us_ : 1000000UL / Config::BMI270_GYRO_ODR_HZ;
    prev_gyro_update_us_ = sample_us;
    capture_.last_gyro_update_us = sample_us;
    capture_.acquisition_poll_start_us = poll_observation_.start_us;
    ++capture_.gyro_sequence;
    capture_.update_dt_us = capture_.gyro_update_dt_us;
    capture_.last_update_us = sample_us;
    capture_.last_update_ms = millis();
    capture_.imu_ok = true;
    poll_observation_.fresh_gyro = true;
    poll_observation_.sample_us = sample_us;
    poll_observation_.sequence = capture_.gyro_sequence;
    poll_observation_.dt_us = capture_.gyro_update_dt_us;
    const uint32_t publish_start_us = micros();
    poll_observation_.pack_us = static_cast<uint32_t>(publish_start_us - convert_done_us);
    publishSample();
    poll_observation_.publish_us = static_cast<uint32_t>(micros() - publish_start_us);
  } else {
    poll_observation_.pack_us = static_cast<uint32_t>(micros() - convert_done_us);
  }
}

void ImuManager::latchFault(const char* reason, uint32_t sample_us,
                            uint32_t age_us, uint32_t depth) {
  const uint32_t now_us = micros();
  portENTER_CRITICAL(&mux_);
  if (!fault_) {
    fault_reason_ = reason;
    fault_snapshot_.time_us = now_us;
    fault_snapshot_.sample_us = sample_us;
    fault_snapshot_.age_us = age_us;
    fault_snapshot_.queue_depth = depth;
    fault_snapshot_.latest_sequence = latest_capture_sequence_;
    fault_snapshot_.state_id = context_state_;
  }
  fault_ = true;
  portEXIT_CRITICAL(&mux_);
}
void ImuManager::publishSample() {
  bool sequential;
  uint32_t cutoff;
  portENTER_CRITICAL(&mux_);
  sequential = sequential_;
  cutoff = boundary_.cutoff;
  latest_capture_sequence_ = capture_.gyro_sequence;
  latest_capture_us_ = capture_.last_gyro_update_us;
  latest_capture_ms_ = capture_.last_update_ms;
  ++total_captured_;
  audit_.sample(capture_.last_gyro_update_us, capture_.gyro_update_dt_us, capture_.gyro_sequence);
  portEXIT_CRITICAL(&mux_);
  if (!sample_queue_) return;  // Startup stream validation, before task creation.
  auto send = [&]() RW_SPEED_CODE {
    capture_.queue_submit_us = micros();
    return xQueueSend(sample_queue_, &capture_, 0);
  };
  if (send() != pdTRUE) {
    if (sequential) {
      // The high-priority reader can wake before the consumer removes the
      // pre-start history. Evict ONE pre-boundary idle item, never a live item.
      // Both owners are pinned to Core 1; this priority-6 producer cannot be
      // preempted by the priority-4 consumer between peek and receive.
      ImuReading oldest;
      bool inserted = false;
      if (xQueuePeek(sample_queue_, &oldest, 0) == pdTRUE &&
          static_cast<int32_t>(oldest.gyro_sequence - cutoff) <= 0 &&
          xQueueReceive(sample_queue_, &oldest, 0) == pdTRUE) {
        inserted = send() == pdTRUE;
        portENTER_CRITICAL(&mux_);
        ++boundary_producer_discards_;
        portEXIT_CRITICAL(&mux_);
      }
      if (!inserted) {
        portENTER_CRITICAL(&mux_);
        audit_.drop();
        portEXIT_CRITICAL(&mux_);
        latchFault("imu_acquisition_queue_overflow", capture_.last_gyro_update_us,
                   static_cast<uint32_t>(micros() - capture_.last_gyro_update_us), kQueueLength);
      }
    } else {
      // While idle only, prefer the most recent sample (e.g. during a download).
      ImuReading discarded;
      xQueueReceive(sample_queue_, &discarded, 0);
      send();
    }
  }
  const uint32_t depth = uxQueueMessagesWaiting(sample_queue_);
  portENTER_CRITICAL(&mux_);
  if (audit_.active && depth > audit_.queue_high_water) audit_.queue_high_water = depth;
  portEXIT_CRITICAL(&mux_);
}

void ImuManager::setAcquisitionContext(bool sequential, bool measurement, uint8_t state_id) {
  // Called on the consumer thread after the start HTTP response has returned.
  // Snapshot a sequence boundary once. Discard ONLY pre-boundary idle history;
  // keep every later sample in order, including all measurement samples.
  portENTER_CRITICAL(&mux_);
  const uint32_t now_us = micros();
  if (sequential && !sequential_) {
    boundary_.enter(now_us, latest_capture_sequence_);
    boundary_producer_discards_ = 0;
    start_sync_deliveries_ = start_sync_age_max_us_ = 0;
  }
  context_state_ = state_id;
  sequential_ = sequential;
  if (measurement && !audit_.active) audit_.start(now_us);
  if (!measurement && audit_.active) audit_.finish(now_us);
  portEXIT_CRITICAL(&mux_);
}
void ImuManager::update() {
  // Permanent control owner only. It is the ONLY writer of reading_.
  control_latency::profile.current = control_latency::Row{};
  reading_.accel_fresh = false;
  reading_.gyro_fresh = false;
  reading_.sensor_mask = 0;
  if (!started_ || !sample_queue_) { reading_.imu_ok = false; return; }
  bool sequential, fault;
  const char* reason;
  portENTER_CRITICAL(&mux_);
  sequential = sequential_;
  fault = fault_;
  reason = fault_reason_;
  portEXIT_CRITICAL(&mux_);
  if (fault) { reading_.imu_ok = false; last_error_ = reason; return; }
  ImuReading next;
  const uint32_t depth = uxQueueMessagesWaiting(sample_queue_);
  // Empty queue: block for at most one tick, not a priority-4 busy loop.
  // A published sample wakes us immediately; timed motor/HTTP service remains bounded.
  if (xQueueReceive(sample_queue_, &next, 1) != pdTRUE) return;
  if (sequential) {
    for (uint32_t i = 0; !boundary_.accepts(next.gyro_sequence); ++i) {
      ++boundary_.discarded_idle_samples;
      if (i + 1 >= kQueueLength || xQueueReceive(sample_queue_, &next, 0) != pdTRUE) return;
    }
  } else {
    // Bounded drain, never an unbounded loop racing the producer.
    ImuReading newer;
    for (uint32_t i = 1; i < kQueueLength && xQueueReceive(sample_queue_, &newer, 0) == pdTRUE; ++i)
      next = newer;
  }
  const uint32_t queue_receive_us = micros();
  const uint32_t age_us = static_cast<uint32_t>(queue_receive_us - next.last_gyro_update_us);
  const int core = xPortGetCoreID();
  const uint32_t priority = uxTaskPriorityGet(nullptr);
  portENTER_CRITICAL(&mux_);
  consumer_core_ = core;
  consumer_priority_ = priority;
  audit_.delivery(next.last_gyro_update_us, next.gyro_sequence, age_us, depth);
  if (sequential && context_state_ == 6) {  // START_SYNC; separate from 30 s run statistics.
    ++start_sync_deliveries_;
    if (age_us > start_sync_age_max_us_) start_sync_age_max_us_ = age_us;
  }
  portEXIT_CRITICAL(&mux_);
  if (sequential && age_us > kMaximumDeliveryAgeUs) {
    latchFault("imu_delivery_backlog_over_10ms", next.last_gyro_update_us, age_us, depth);
    reading_.imu_ok = false;
    last_error_ = "imu_delivery_backlog_over_10ms";
    return;
  }
  control_latency::progress();
  uint32_t receive_time;
  const auto receive_activity = control_latency::activity(&receive_time);
  control_latency::profile.receive(audit_.active, next.gyro_sequence,
      next.last_gyro_update_us, next.acquisition_poll_start_us, receive_time, receive_activity,
      control_latency::Delivery(next.queue_submit_us, queue_receive_us));
  const uint32_t previous_accel_sequence = reading_.accel_sequence;
  // Preserve the same calibrated raw values and formulas. Only relocate work;
  // cache across gyro-only deliveries so this runs once per new accel sample.
  if (next.accel_sequence != previous_accel_sequence) {
    updateDerivedAccel(next);
  } else {
    next.acc_norm_g = reading_.acc_norm_g;
    next.acc_norm_error_g = reading_.acc_norm_error_g;
    next.pitch_accel_only_deg = reading_.pitch_accel_only_deg;
  }
  control_latency::mark(control_latency::Derived);
  reading_ = next;
  // Accel may arrive on a poll before the next gyro publishes the combined row.
  reading_.accel_fresh = reading_.accel_sequence != previous_accel_sequence;
  reading_.time_since_last_pulse_ms = static_cast<uint16_t>(min<uint32_t>(65535, beta_context_time_since_last_pulse_ms_));
  last_error_ = "";
}

bool ImuManager::acquisitionHealthy() const {
  portENTER_CRITICAL(&mux_);
  const bool healthy = !fault_;
  portEXIT_CRITICAL(&mux_);
  return started_ && healthy;
}
bool ImuManager::ok() const {
  return imu_present_ && reading_.imu_ok && reading_.rate_config_ok && acquisitionHealthy();
}
bool ImuManager::stale(uint32_t now_ms) const {
  (void)now_ms;
  portENTER_CRITICAL(&mux_);
  const uint32_t stamp = latest_capture_us_, stamp_ms = latest_capture_ms_;
  portEXIT_CRITICAL(&mux_);
  // The reader can preempt between the caller's millis() and this snapshot.
  // Read the comparison clock AFTER the snapshot, avoiding unsigned underflow
  // from a perfectly fresh sample. The existing stale limit is unchanged.
  const uint32_t check_now_ms = millis();
  return !acquisitionHealthy() || stamp == 0 ||
      static_cast<uint32_t>(check_now_ms - stamp_ms) > Config::IMU_STALE_LIMIT_MS;
}
void ImuManager::zeroPitch() { }
void ImuManager::setDynamicBetaContext(bool pulse_active, uint32_t since_ms, bool pre_start) {
  beta_context_pulse_active_ = pulse_active;
  beta_context_time_since_last_pulse_ms_ = since_ms;
  beta_context_pre_start_stabilize_ = pre_start;
}
void ImuManager::forceSmoothBeta(float beta, uint8_t mode) {
  reading_.beta_target = beta; reading_.beta_smooth = beta;
  reading_.beta_dynamic = beta; reading_.beta_update_mode = mode;
}

String ImuManager::acquisitionDiagnosticsJson() const {
  String json; json.reserve(10000); appendDiagnostics(json); return json;
}
void ImuManager::appendAcquisitionDiagnostics(PsramString& json) const { appendDiagnostics(json); }
template<class Output> void ImuManager::appendDiagnostics(Output& json) const {
  bool fault;
  const char* reason;
  int reader_core, consumer_core;
  uint32_t reader_priority, consumer_priority, total;
  FaultSnapshot first_fault;
  // Fixed-size snapshot. No String construction, sensor I/O or queue operations in the lock.
  portENTER_CRITICAL(&mux_);
  audit_snapshot_ = audit_;
  fault = fault_; reason = fault_reason_;
  first_fault = fault_snapshot_;
  reader_core = reader_core_; consumer_core = consumer_core_;
  reader_priority = reader_priority_; consumer_priority = consumer_priority_;
  total = total_captured_;
  portEXIT_CRITICAL(&mux_);
  const auto& a = audit_snapshot_;
  json += "{\"revision\":\"core1_hw_timer_acquisition_04724\",\"firmware_version\":\"" RUNTIME_VERSION "\"";
  json += ",\"timestamp_semantics\":\"M5Unified_host_acquisition_not_sensor_clock\"";
  json += ",\"motor_controller\":\"unchanged_V46l_legacy_V7\"";
  json += ",\"reader_core\":" + String(reader_core) + ",\"reader_priority\":" + String(reader_priority);
  json += ",\"consumer_core\":" + String(consumer_core) + ",\"consumer_priority\":" + String(consumer_priority);
  json += ",\"transport\":\"idf_interrupt_i2c1_04719\"";
  NotifyStamp notification;
  portENTER_CRITICAL(&notify_mux_); notification = notify_stamp_; portEXIT_CRITICAL(&notify_mux_);
  json += ",\"poll_trigger\":{\"type\":\"core1_hardware_timer_isr_04724\",\"group\":1,\"timer\":0,\"period_us\":1000";
  json += ",\"allocation_core\":" + String(acquisition_timer_.ownerCore());
  json += ",\"observed_isr_core\":" + String(notification.core);
  json += ",\"last_error\":" + String(acquisition_timer_.lastError()) + "}";
  json += ",\"internal_i2c_port\":" + String(internal_i2c_port_);
  json += ",\"internal_sda\":" + String(internal_sda_) + ",\"internal_scl\":" + String(internal_scl_);
  json += ",\"roller_i2c_port\":0,\"queue_capacity\":32,\"delivery_age_limit_us\":10000";
  json += ",\"consumer_empty_wait_ticks\":1,\"reader_overrun_yield_ticks\":1";
  json += ",\"started\":" + String(started_ ? "true" : "false");
  json += ",\"fault\":" + String(fault ? "true" : "false") + ",\"fault_reason\":\"" + String(reason) + "\"";
  json += ",\"startup\":" + startupDiagnosticsJson();
  json += ",\"sequence_epoch_us\":" + String(boundary_.epoch_us);
  json += ",\"sequence_cutoff\":" + String(boundary_.cutoff);
  json += ",\"discarded_idle_samples\":" + String(boundary_.discarded_idle_samples);
  json += ",\"producer_discarded_idle_samples\":" + String(boundary_producer_discards_);
  json += ",\"start_sync_deliveries\":" + String(start_sync_deliveries_);
  json += ",\"start_sync_age_max_us\":" + String(start_sync_age_max_us_);
  json += ",\"first_fault\":{\"time_us\":" + String(first_fault.time_us);
  json += ",\"sample_us\":" + String(first_fault.sample_us);
  json += ",\"age_us\":" + String(first_fault.age_us);
  json += ",\"queue_depth\":" + String(first_fault.queue_depth);
  json += ",\"latest_sequence\":" + String(first_fault.latest_sequence);
  json += ",\"state_id\":" + String(first_fault.state_id) + "}";
  json += ",\"total_captured_since_boot\":" + String(total);
  json += ",\"measurement_finished\":" + String(a.finished ? "true" : "false");
  json += ",\"epoch_us\":" + String(a.epoch_us) + ",\"duration_us\":" + String(a.duration_us);
  json += ",\"captured\":" + String(a.samples) + ",\"delivered\":" + String(a.delivered);
  json += ",\"polls\":" + String(a.polls) + ",\"coalesced_wakes\":" + String(a.coalesced_wakes);
  json += ",\"dt_mean_us\":" + String(a.samples ? static_cast<double>(a.dt_sum_us) / a.samples : 0.0, 3);
  json += ",\"dt_max_us\":" + String(a.dt_max_us) + ",\"poll_max_us\":" + String(a.poll_max_us);
  json += ",\"delivery_age_mean_us\":" + String(a.delivered ? static_cast<double>(a.age_sum_us) / a.delivered : 0.0, 3);
  json += ",\"delivery_age_max_us\":" + String(a.age_max_us);
  json += ",\"over_4ms\":" + String(a.over_4ms) + ",\"over_5ms\":" + String(a.over_5ms);
  json += ",\"over_10ms\":" + String(a.over_10ms) + ",\"queue_drops\":" + String(a.queue_drops);
  json += ",\"queue_high_water\":" + String(a.queue_high_water);
  json += ",\"delivery_sequence_gaps\":" + String(a.delivery_sequence_gaps);
  json += ",\"histogram_bin_us\":250,\"histogram_last_bin_lower_us\":10000";
  auto array = [&json](const char* key, const uint32_t* data, unsigned count) {
    json += ",\"" + String(key) + "\":[";
    for (unsigned i = 0; i < count; ++i) { if (i) json += ","; json += String(data[i]); }
    json += "]";
  };
  array("acquisition_dt_histogram", a.dt_bins, a.kBins);
  array("delivery_age_histogram", a.age_bins, a.kBins);
  array("samples_per_second", a.samples_per_second, a.kSeconds);
  array("max_dt_per_second_us", a.max_dt_per_second, a.kSeconds);
  array("max_delivery_age_per_second_us", a.max_age_per_second, a.kSeconds);
  json += ",\"long_gap_detail_overflow\":" + String(a.gap_overflow) + ",\"long_gaps\":[";
  for (uint32_t i = 0; i < a.gap_count; ++i) {
    if (i) json += ",";
    json += "{\"time_us\":" + String(a.gaps[i].time_us);
    json += ",\"dt_us\":" + String(a.gaps[i].dt_us) + ",\"sequence\":" + String(a.gaps[i].sequence) + "}";
  }
  json += "],\"v46q_poll_profile\":";
  appendPollProfile(json);
  json += "}";
}

void ImuManager::setStartupGuideState(const char* reason, bool confirmed, uint32_t hold_ms) {
  // Called only by the permanent IMU consumer. The export worker reads this
  // bounded POD view, never the live reading_ or its mutable startup fields.
  StartupView view;
  view.reason = reason; view.confirmed = confirmed; view.hold_ms = hold_ms;
  const auto& r = reading_;
  view.age_us = r.last_gyro_update_us ? static_cast<uint32_t>(micros() - r.last_gyro_update_us) : UINT32_MAX;
  view.sequence = r.gyro_sequence;
  view.direction = UprightPoseGuide::directionErrorDeg(r);
  view.norm = UprightPoseGuide::accelNormG(r); view.gyro = UprightPoseGuide::gyroNormDps(r);
  portENTER_CRITICAL(&mux_); startup_view_ = view; portEXIT_CRITICAL(&mux_);
}

String ImuManager::startupDiagnosticsJson() const {
  portENTER_CRITICAL(&mux_); const auto view = startup_view_; portEXIT_CRITICAL(&mux_);
  String s;
  s.reserve(704);
  s = "{\"init_attempts\":" + String(init_attempts_);
  s += ",\"init_last_failure\":\"" + String(init_last_failure_) + "\"";
  s += ",\"init_internal_status\":" + String(init_internal_status_);
  s += ",\"init_power_ctrl\":" + String(init_power_ctrl_);
  s += ",\"init_valid_accel\":" + String(init_valid_accel_);
  s += ",\"init_valid_gyro\":" + String(init_valid_gyro_);
  s += ",\"imu_error\":\"" + String(last_error_) + "\"";
  s += ",\"guide_reason\":\"" + String(view.reason) + "\"";
  s += ",\"upright_confirmed\":" + String(view.confirmed ? "true" : "false");
  s += ",\"stable_hold_ms\":" + String(view.hold_ms);
  s += ",\"sample_age_us\":" + String(view.age_us);
  s += ",\"gyro_sequence\":" + String(view.sequence);
  s += ",\"direction_error_deg\":" + String(view.direction, 3);
  s += ",\"accel_norm_g\":" + String(view.norm, 4);
  s += ",\"gyro_norm_dps\":" + String(view.gyro, 3) + "}";
  s.replace(":nan", ":null"); s.replace(":inf", ":null"); s.replace(":-inf", ":null");
  return s;
}

String ImuManager::pollProfileJson() const {
  String s; s.reserve(48000); appendPollProfile(s); return s;
}
template<class Output> void ImuManager::appendPollProfile(Output& s) const {
  portENTER_CRITICAL(&mux_);
  const bool active = audit_.active;
  portEXIT_CRITICAL(&mux_);
  // This large profile has one high-priority same-core writer. During a live
  // measurement refuse export; do not block the writer with a serialization lock.
  if (active) { s += "{\"available\":false,\"reason\":\"measurement_active\"}"; return; }
  const auto& p = poll_profile_;
  if (!p.initialized) { s += "{\"available\":false,\"reason\":\"no_measurement_polls\"}"; return; }
  s += "{\"available\":true,\"revision\":\"v46q_lightweight_acquisition_20260914\"";
  s += ",\"semantics\":\"host_wall_times;update_includes_driver;notify_age_from_latest_callback;not_ISR_latency\"";
  s += ",\"derived_accel_location\":\"consumer_new_accel_only_and_synchronous_startup\"";
  s += ",\"epoch_us\":" + String(p.epoch_us);
  s += ",\"polls\":" + String(p.polls) + ",\"fresh_gyro_polls\":" + String(p.gyro);
  s += ",\"no_data_polls\":" + String(p.no_data);
  s += ",\"coalesced_wakes\":" + String(p.coalesced_wakes);
  s += ",\"forced_yields\":" + String(p.forced_yields);
  s += ",\"long_gaps_over_4ms\":" + String(p.long_gaps);
  s += ",\"outside_detail_window\":" + String(p.outside_buckets);
  s += ",\"record_overhead_max_us\":" + String(p.record_overhead_max_us);
  s += ",\"v46u_timing\":{\"driver_calls\":" + String(p.driver_calls);
  s += ",\"driver_failures\":" + String(p.driver_failures);
  s += ",\"data_bytes\":" + String(p.driver_bytes);
  s += ",\"status_read_max_us\":" + String(p.driver_status.maximum);
  s += ",\"data_read_max_us\":" + String(p.driver_data.maximum);
  s += ",\"poll_work_budget_us\":1000,\"poll_work_count\":" + String(p.poll_work_deadline.count);
  s += ",\"poll_work_over_budget\":" + String(p.poll_work_deadline.over);
  s += ",\"host_gyro_interval_budget_us\":2500,\"host_gyro_interval_count\":" + String(p.gyro_interval_deadline.count);
  s += ",\"host_gyro_interval_over_budget\":" + String(p.gyro_interval_deadline.over);
  s += ",\"host_gyro_interval_max_us\":" + String(p.gyro_interval_deadline.maximum);
  s += ",\"sensor_clock_verified\":false,\"policy\":\"strict_greater_than;host_polling_not_FIFO;no_tolerance_hidden\"}";
  static const char* names[ImuPollProfile::STAGES] = {
    "latest_notify_age", "observed_callback_gap", "poll_start_interval", "update_api",
    "convert_api", "validate_pack", "publish_queue", "poll_total", "overrun_yield"};
  s += ",\"stages\":{";
  for (unsigned i = 0; i < ImuPollProfile::STAGES; ++i) {
    const auto& v = p.stage[i];
    if (i) s += ",";
    s += "\"" + String(names[i]) + "\":{\"count\":" + String(v.count);
    s += ",\"mean_us\":" + String(v.count ? static_cast<double>(v.sum_us) / v.count : 0.0, 3);
    s += ",\"max_us\":" + String(v.max_us) + "}";
  }
  s += "},\"second_columns\":[\"second\",\"polls\",\"gyro\",\"no_data\",\"forced_yields\",\"notify_max_us\",\"update_max_us\",\"convert_max_us\",\"total_max_us\",\"period_max_us\",\"long_gaps\"]";
  s += ",\"seconds\":[";
  bool comma = false;
  for (unsigned i = 0; i < ImuPollProfile::kSeconds; ++i) {
    const auto& v = p.seconds[i];
    if (!v.polls) continue;
    if (comma) s += ",";
    comma = true;
    s += "[" + String(i) + "," + String(v.polls) + "," + String(v.gyro);
    s += "," + String(v.no_data) + "," + String(v.forced_yields);
    s += "," + String(v.notify_max) + "," + String(v.update_max) + "," + String(v.convert_max);
    s += "," + String(v.total_max) + "," + String(v.period_max) + "," + String(v.long_gaps) + "]";
  }
  s += "],\"detail_policy\":\"worst_gap_per_100ms_bucket_not_all_gaps;full_counts_above\"";
  s += ",\"bucket_us\":100000,\"detail_window_us\":32000000";
  s += ",\"gap_columns\":[\"time_us\",\"dt_us\",\"sequence\",\"latest_notify_age_us\",\"callback_gap_us\",\"poll_period_us\",\"update_us\",\"convert_us\",\"pack_us\",\"publish_us\",\"total_us\",\"previous_total_us\",\"previous_yield_us\",\"wakes\"]";
  s += ",\"worst_gap_by_100ms\":[";
  comma = false;
  for (const auto& g : p.worst_gap) {
    if (!g.dt_us) continue;
    if (comma) s += ",";
    comma = true;
    s += "[" + String(g.time_us) + "," + String(g.dt_us) + "," + String(g.sequence);
    s += "," + String(g.notify_age_us) + "," + String(g.callback_gap_us) + "," + String(g.period_us);
    s += "," + String(g.update_us) + "," + String(g.convert_us) + "," + String(g.pack_us);
    s += "," + String(g.publish_us) + "," + String(g.total_us);
    s += "," + String(g.previous_total_us) + "," + String(g.previous_yield_us) + "," + String(g.wakes) + "]";
  }
  s += "]}";
}

