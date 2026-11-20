#include "support_transfer_audit.h"
#if SUPPORT_TRANSFER_V39
#include "foot_fast_policy.h"
#include <cstdio>

void SupportTransferAudit::begin(float tau) {
  if (!current_.prepare(tau) || !rows_.begin(ps_malloc)) return;
  // Core 1 owns the IMU and real-time control. On Core 0 both the camera and
  // network tasks can preempt this optional observer.
  if (xTaskCreatePinnedToCore(taskEntry, "transfer_shadow", 6144, this,
                             tskIDLE_PRIORITY, &worker_, 0) != pdPASS) worker_ = nullptr;
}

void SupportTransferAudit::startRun() {
  portENTER_CRITICAL(&mux_);
  ++generation_;
  active_ = true;
  pending_ = false;
  stats_ = {};
  portEXIT_CRITICAL(&mux_);
}

void SupportTransferAudit::seal() {
  portENTER_CRITICAL(&mux_);
  active_ = pending_ = false;  // In-flight work may finish but cannot enter the sealed archive.
  portEXIT_CRITICAL(&mux_);
}

void SupportTransferAudit::submit(const Sample& sample) {
  portENTER_CRITICAL(&mux_);
  if (active_) {
    ++stats_.submitted;
    if (worker_) {
      if (pending_) ++stats_.replaced;
      pending_sample_ = sample;
      pending_ = true;
    } else ++stats_.unavailable;
  }
  portEXIT_CRITICAL(&mux_);
  if (worker_) xTaskNotifyGive(worker_);
}

void SupportTransferAudit::taskEntry(void* self) { static_cast<SupportTransferAudit*>(self)->work(); }

void SupportTransferAudit::work() {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Sample sample;
    uint32_t generation;
    portENTER_CRITICAL(&mux_);
    const bool have_sample = active_ && pending_;
    if (have_sample) {
      sample = pending_sample_;
      generation = generation_;
      pending_ = false;
    }
    portEXIT_CRITICAL(&mux_);
    if (!have_sample) continue;

    const uint32_t start = micros();
    support_transfer_plan::Decision decision;
    // Zero remaining on-time still permits a meaningful passive-reachability prediction.
    if (!(sample.gate_mask & ~NoBudget)) decision = support_transfer_plan::select(current_, sample.input);
    const uint32_t end = micros();
    const Row row{sample, decision, end - sample.captured_us, end - start};
    portENTER_CRITICAL(&mux_);
    if (active_ && generation == generation_) {
      ++stats_.completed;
      if (row.solve_us > stats_.max_solve_us) stats_.max_solve_us = row.solve_us;
      if (row.ready_after_us > stats_.max_ready_after_us) stats_.max_ready_after_us = row.ready_after_us;
      if (stats_.stored < capacity) rows_[stats_.stored++] = row;
    }
    portEXIT_CRITICAL(&mux_);
  }
}

void SupportTransferAudit::append(PsramString& json) const {
  // Export owns a sealed run. No writer can alter these rows until the next start.
  portENTER_CRITICAL(&mux_);
  const Stats stats = stats_;
  portEXIT_CRITICAL(&mux_);
  json += ",\"support_transfer_shadow\":{\"revision\":1,\"output_enabled\":false";
  json += ",\"worker_ready\":" + String(worker_ ? "true" : "false");
  json += ",\"submitted\":" + String(stats.submitted) + ",\"replaced\":" + String(stats.replaced);
  json += ",\"unavailable\":" + String(stats.unavailable) + ",\"completed\":" + String(stats.completed);
  json += ",\"abandoned\":" + String(stats.submitted - stats.completed - stats.replaced - stats.unavailable);
  json += ",\"stored\":" + String(stats.stored) + ",\"max_ready_after_us\":" + String(stats.max_ready_after_us);
  json += ",\"max_solve_us\":" + String(stats.max_solve_us) + ",\"rise_tau_s\":" + String(current_.tau, 7);
  json += ",\"reason_names\":[\"invalid\",\"outside_approach\",\"coast\",\"wait\",\"drive\",\"insufficient\"]";
  json += ",\"gate_bits\":[\"stale_imu\",\"stale_packet\",\"invalid_foot\",\"missing_acceleration\","
          "\"fore_aft\",\"current\",\"wheel_speed\",\"battery\",\"no_budget\",\"pulse_active\"]";
  json += ",\"baseline_bits\":[\"pulse_active\",\"pulse_applied_in_half_cycle\",\"baseline_crossing_consumed\",\"return_aid_accepted\"]";
  json += ",\"columns\":[\"time_ms\",\"peak_ms\",\"packet\",\"side\",\"lateral_deg\",\"rate_dps\","
          "\"accel_dps2\",\"fore_aft_deg\",\"left_deg\",\"right_deg\",\"left_rate_dps\",\"right_rate_dps\","
          "\"left_age_ms\",\"right_age_ms\",\"i0_ma\",\"goal_ma\",\"cap_ms\",\"reason\",\"delay_ms\","
          "\"width_ms\",\"transfer_ms\",\"transfer_deg\",\"coast_margin_j\",\"margin_j\",\"work_j\","
          "\"work_supported\",\"ready_after_us\",\"solve_us\",\"gate_mask\",\"baseline_flags\","
          "\"raw_vbat_mv\",\"model_vbat_mv\",\"model_vbat_status\",\"capture_us32\"],\"rows\":[";
  // Respect v38's existing metadata ceiling and the final file-size footer.
  // Under pressure spread the surviving rows across the run, with explicit counts.
  const size_t available = json.length() + 128 < foot_fast_policy::kMetadataHardBudgetBytes
                         ? foot_fast_policy::kMetadataHardBudgetBytes - json.length() - 128 : 0;
  size_t written = 0;
  uint32_t exported = 0;
  for (uint32_t i = 0; i < stats.stored; ++i) {
    const auto& r = rows_[i]; const auto& s = r.sample;
    const auto& in = s.input; const auto& d = r.decision;
    // Each finite %.7g or 32-bit integer needs at most 15 bytes including its comma.
    char row[544];
    unsigned pos = 0;
    const auto integer = [&](long n) { pos += std::sprintf(row + pos, "%ld,", n); };
    const auto number = [&](float n) {
      pos += std::isfinite(n) ? std::sprintf(row + pos, "%.7g,", static_cast<double>(n))
                              : std::sprintf(row + pos, "null,");
    };
    if (exported) row[pos++] = ',';
    row[pos++] = '[';
    integer(s.time_ms); integer(s.peak_ms); integer(s.packet); integer(in.side);
    number(in.angle_deg); number(in.rate_dps); number(in.accel_dps2); number(in.fore_aft_deg);
    number(in.left_deg); number(in.right_deg); number(in.left_rate_dps); number(in.right_rate_dps);
    integer(s.left_age_ms); integer(s.right_age_ms); number(in.initial_current_ma); number(in.goal_current_ma);
    integer(in.cap_ms); integer(static_cast<int>(d.reason)); integer(d.delay_ms); integer(d.width_ms);
    number(d.transfer_ms); number(d.transfer_deg); number(d.coast_margin_j); number(d.margin_j); number(d.work_j);
    integer(d.work_supported); integer(r.ready_after_us); integer(r.solve_us); integer(s.gate_mask); integer(s.baseline_flags);
    integer(s.raw_vbat_mv); integer(s.model_vbat_mv); integer(s.model_vbat_status);
    pos += std::sprintf(row + pos, "%lu]", static_cast<unsigned long>(s.captured_us));
    const size_t quota = uint64_t(available) * (i + 1) / stats.stored;
    if (written + pos > available || (exported && written + pos > quota)) continue;
    json.append(row, pos);
    written += pos;
    ++exported;
  }
  json += "],\"exported\":" + String(exported) + "}";
}
#endif
