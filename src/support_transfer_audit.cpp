#include "support_transfer_audit.h"
#if SUPPORT_TRANSFER_V39
#include "foot_fast_policy.h"
#include <algorithm>
#include <cinttypes>
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

namespace {
constexpr char kSchema[] =
          ",\"reason_names\":[\"invalid\",\"outside_approach\",\"coast\",\"wait\",\"drive\",\"insufficient\"]"
          ",\"gate_bits\":[\"stale_imu\",\"stale_packet\",\"invalid_foot\",\"missing_acceleration\","
          "\"fore_aft\",\"current\",\"wheel_speed\",\"battery\",\"no_budget\",\"pulse_active\"]"
          ",\"baseline_bits\":[\"pulse_active\",\"pulse_applied_in_half_cycle\",\"baseline_crossing_consumed\",\"return_aid_accepted\"]"
          ",\"columns\":[\"time_ms\",\"peak_ms\",\"packet\",\"side\",\"lateral_deg\",\"rate_dps\","
          "\"accel_dps2\",\"fore_aft_deg\",\"left_deg\",\"right_deg\",\"left_rate_dps\",\"right_rate_dps\","
          "\"left_age_ms\",\"right_age_ms\",\"i0_ma\",\"goal_ma\",\"cap_ms\",\"reason\",\"delay_ms\","
          "\"width_ms\",\"transfer_ms\",\"transfer_deg\",\"coast_margin_j\",\"margin_j\",\"work_j\","
          "\"work_supported\",\"ready_after_us\",\"solve_us\",\"gate_mask\",\"baseline_flags\","
          "\"raw_vbat_mv\",\"model_vbat_mv\",\"model_vbat_status\",\"capture_us32\"],\"rows\":[";
constexpr size_t kFooterBytes = sizeof(",\"metadata_json_final_bytes\":4294967295}") - 1;
constexpr size_t kClosingBytes = sizeof("],\"exported\":4294967295}") - 1;
// Even 34 single-digit values require two brackets and 33 commas.
constexpr size_t kMinRowBytes = 69;

class RowText {
 public:
  char bytes[544];
  size_t length = 0;
  void start() { bytes[0] = '['; length = 1; }
  void integer(uint32_t value) { field("%" PRIu32 ",", value); }
  void side(int value) { field("%d,", value); }
  void number(float value) {
    if (std::isfinite(value)) field("%.7g,", static_cast<double>(value));
    else field("%s,", "null");
  }
  void finish() { if (length) bytes[length - 1] = ']'; }
 private:
  template<typename T> void field(const char* format, T value) {
    if (!length) return;
    const size_t room = sizeof(bytes) - length;
    const int n = std::snprintf(bytes + length, room, format, value);
    length = (n >= 0 && static_cast<size_t>(n) < room) ? length + n : 0;
  }
};
} // namespace

void SupportTransferAudit::append(PsramString& json) const {
  // Export owns a sealed run. No writer can alter these rows until the next start.
  portENTER_CRITICAL(&mux_);
  const Stats stats = stats_;
  portEXIT_CRITICAL(&mux_);
  json.checkpoint();
  const size_t section_end = foot_fast_policy::kMetadataHardBudgetBytes - kFooterBytes;
  if (!json.ok() || json.length() >= section_end) return;
  const size_t room = section_end - json.length();
  char header[448];
  const int h = std::snprintf(header, sizeof(header),
      ",\"support_transfer_shadow\":{\"revision\":1,\"output_enabled\":false,\"worker_ready\":%s"
      ",\"submitted\":%" PRIu32 ",\"replaced\":%" PRIu32 ",\"unavailable\":%" PRIu32
      ",\"completed\":%" PRIu32 ",\"abandoned\":%" PRIu32 ",\"stored\":%" PRIu32
      ",\"max_ready_after_us\":%" PRIu32 ",\"max_solve_us\":%" PRIu32 ",\"rise_tau_s\":%.7f",
      worker_ ? "true" : "false", stats.submitted, stats.replaced, stats.unavailable, stats.completed,
      stats.submitted - stats.completed - stats.replaced - stats.unavailable,
      stats.stored, stats.max_ready_after_us, stats.max_solve_us, static_cast<double>(current_.tau));
  if (h < 0 || static_cast<size_t>(h) >= sizeof(header) ||
      static_cast<size_t>(h) + sizeof(kSchema) - 1 + kClosingBytes > room) {
    // The optional schema itself must never invalidate the sealed sensor and motor data.
    const int n = std::snprintf(header, sizeof(header),
        ",\"support_transfer_shadow_omitted\":{\"stored\":%" PRIu32 ",\"exported\":0}", stats.stored);
    if (n > 0 && static_cast<size_t>(n) < sizeof(header) && static_cast<size_t>(n) <= room)
      json.append(header, n);
    return;
  }
  json.append(header, h);
  json.append(kSchema, sizeof(kSchema) - 1);
  const size_t available = section_end - json.length() - kClosingBytes;
  const auto encode = [&](uint32_t index, RowText& row) {
    json.checkpoint(); // Also service the export worker if the row is subsequently omitted.
    const auto& r = rows_[index]; const auto& s = r.sample;
    const auto& in = s.input; const auto& d = r.decision;
    row.start();
    row.integer(s.time_ms); row.integer(s.peak_ms); row.integer(s.packet); row.side(in.side);
    row.number(in.angle_deg); row.number(in.rate_dps); row.number(in.accel_dps2); row.number(in.fore_aft_deg);
    row.number(in.left_deg); row.number(in.right_deg); row.number(in.left_rate_dps); row.number(in.right_rate_dps);
    row.integer(s.left_age_ms); row.integer(s.right_age_ms); row.number(in.initial_current_ma); row.number(in.goal_current_ma);
    row.integer(in.cap_ms); row.integer(static_cast<unsigned>(d.reason)); row.integer(d.delay_ms); row.integer(d.width_ms);
    row.number(d.transfer_ms); row.number(d.transfer_deg); row.number(d.coast_margin_j); row.number(d.margin_j); row.number(d.work_j);
    row.integer(d.work_supported); row.integer(r.ready_after_us); row.integer(r.solve_us); row.integer(s.gate_mask); row.integer(s.baseline_flags);
    row.integer(s.raw_vbat_mv); row.integer(s.model_vbat_mv); row.integer(s.model_vbat_status); row.integer(s.captured_us);
    row.finish();
  };
  uint32_t exported = 0;
  if (stats.stored && available >= kMinRowBytes) {
    RowText tail;
    encode(stats.stored - 1, tail);
    const bool keep_tail = tail.length && tail.length <= available;
    const size_t before_tail = keep_tail ? available - tail.length : available;
    const size_t budget = before_tail - (keep_tail && before_tail ? 1 : 0);
    const uint32_t earlier = stats.stored - 1;
    // Jump directly to evenly spaced candidates. Even the shortest possible rows
    // could not fill more slots than this in the available bytes.
    const uint32_t candidates = std::min<uint32_t>(earlier, budget / kMinRowBytes);
    size_t used = 0;
    RowText row;
    for (uint32_t j = 0; j < candidates; ++j) {
      encode(j * earlier / candidates, row);
      const size_t next = used + (exported ? 1 : 0) + row.length;
      const size_t quota = uint64_t(budget) * (j + 1) / candidates;
      if (!row.length || next > budget || (exported && next > quota)) continue;
      if (exported) json += ",";
      json.append(row.bytes, row.length);
      ++exported;
      used = next;
    }
    if (keep_tail) {
      if (exported) json += ",";
      json.append(tail.bytes, tail.length);
      ++exported;
    }
  }
  const int closing = std::snprintf(header, sizeof(header), "],\"exported\":%" PRIu32 "}", exported);
  if (closing > 0 && static_cast<size_t>(closing) <= kClosingBytes) json.append(header, closing);
  else json.fail();
}
#endif
