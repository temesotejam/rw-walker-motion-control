#include "periodic_foot_stream.h"
#if defined(FOOT_PERIODIC_V38) && FOOT_PERIODIC_V38
#include "foot_calibration.h"
PeriodicFootStream periodicFeet;
static String angleJson(float angle) { return isfinite(angle) ? String(angle,3) : String("null"); }
void PeriodicFootStream::begin() { rows_ = static_cast<Row*>(ps_malloc(capacity*sizeof(Row))); }
void PeriodicFootStream::startRun(uint16_t id) {
  run_id_ = id; stored_ = 0; run_final_ = {};
}
void PeriodicFootStream::enter(bool walking,uint64_t epoch,uint64_t now) {
  if (initialized_ && walking == active_ && (!walking || epoch == epoch_)) return;
  initialized_ = true; active_ = walking; epoch_ = walking ? epoch : now;
  estimator_.start(epoch_);
}
void PeriodicFootStream::observe(const FootFrame& f,uint64_t received) {
  if (active_ && (f.run_id != run_id_ || f.state_id != 3)) return;
  estimator_.observe(f.sequence,f.frame_us,received,f.timestamp_valid && f.zero_ready,
      f.left_valid,foot_control_range::allows(f.left_in_range,f.left_x,false),f.left_deg,
      f.right_valid,foot_control_range::allows(f.right_in_range,f.right_x,true),f.right_deg);
}
bool PeriodicFootStream::service(uint64_t now) {
  if (!estimator_.publish(now)) return false;
  const auto& p = estimator_.packet;
  portENTER_CRITICAL(&mux_); live_ = p; portEXIT_CRITICAL(&mux_);
  if (!active_) return false;
  run_final_ = p;
  if (rows_ && stored_ < capacity) {
    rows_[stored_++] = {uint32_t(p.scheduled_us-epoch_),uint32_t(now-epoch_),p.left.sequence,p.right.sequence,
                        p.left.estimated,p.right.estimated};
  }
  return true;
}
periodic_foot::Packet PeriodicFootStream::current() const {
  portENTER_CRITICAL(&mux_); auto copy = live_; portEXIT_CRITICAL(&mux_); return copy;
}
String PeriodicFootStream::liveJson() const {
  const auto p = current();
  auto side = [&](const periodic_foot::Side& s) {
    return String("{\"sequence\":")+s.sequence+",\"measured_deg\":"+angleJson(s.measured)+
        ",\"estimated_deg\":"+angleJson(s.estimated)+",\"age_us\":"+
        (s.image_us ? String(uint32_t(p.published_us-s.image_us)) : String("null"))+"}";
  };
  return String("{\"period_us\":25000,\"count\":")+p.count+",\"missed\":"+p.missed+
      ",\"published_hz\":"+angleJson(p.published_us>p.epoch_us ? p.count*1e6/(p.published_us-p.epoch_us) : NAN)+
      ",\"packet_age_ms\":"+String(p.count ? (esp_timer_get_time()-p.published_us)/1000.f : -1.f,1)+
      ",\"both_valid\":"+p.both_valid+",\"max_late_us\":"+p.max_late_us+
      ",\"left\":"+side(p.left)+",\"right\":"+side(p.right)+"}";
}
void PeriodicFootStream::appendRun(PsramString& json) const {
  json += "\"foot_periodic\":{\"period_us\":25000,\"max_age_us\":75000,\"count\":"+String(run_final_.count)+
      ",\"missed\":"+String(run_final_.missed)+",\"stored\":"+String(stored_)+
      ",\"both_valid\":"+String(run_final_.both_valid)+",\"max_late_us\":"+String(run_final_.max_late_us)+
      ",\"columns\":[\"slot_us\",\"publish_us\",\"left_sequence\",\"right_sequence\",\"left_est_deg\",\"right_est_deg\"],\"samples\":[";
  for (unsigned i=0;i<stored_;++i) {
    const auto& r=rows_[i];
    if (i) json += ",";
    json += "["+String(r.slot)+","+String(r.time)+","+String(r.left_seq)+","+String(r.right_seq)+","+
            angleJson(r.left)+","+angleJson(r.right)+"]";
  }
  json += "]},";
}
#endif
