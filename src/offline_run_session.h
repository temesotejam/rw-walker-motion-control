#pragma once
#include <stdint.h>

// HTTP lifecycle only; Wi-Fi/AP stays up throughout every phase. Host callbacks
// never reset a logger, calibration, exporter or controller. All calls execute
// in the low-priority HTTP owner. Legacy class/wire names retain compatibility.
class OfflineRunSession {
 public:
  enum class Phase { Online, Draining, Starting, Running, Cancelling, Restoring };
  enum class StartResult { Pending, Started, Rejected };
  static constexpr uint32_t kDisplayWaitMs = 30000;
  bool queue(uint32_t now) {
    if (busy()) return false;
    error_ = ""; queued_ms_ = since_ms_ = now; phase_ = Phase::Draining;
    attempted_restore_ = false; return true;
  }
  bool busy() const { return phase_ != Phase::Online; }
  bool serving() const { return phase_ == Phase::Online || phase_ == Phase::Draining; }
  Phase phase() const { return phase_; }
  const char* error() const { return error_; }
  const char* name() const {
    switch (phase_) {
      case Phase::Online: return "online";
      case Phase::Draining: return "preparing";
      case Phase::Starting: return "starting_run";
      case Phase::Running: return "http_paused_run";
      case Phase::Cancelling: return "cancelling_start";
      default: return "restoring_http";
    }
  }
  uint32_t displayWaitMs(uint32_t now) const {
    const uint32_t elapsed = now - queued_ms_;
    return elapsed < kDisplayWaitMs ? kDisplayWaitMs - elapsed : 0;
  }
  void cancel(uint32_t now) { if (busy()) { phase_ = Phase::Cancelling; since_ms_ = now; } }
  template<class Host> void update(uint32_t now, Host& h) {
    switch (phase_) {
      case Phase::Online: break;
      case Phase::Draining:
        // Allow the START response to drain before closing its TCP connection.
        if (now - since_ms_ < 350) break;
        // Close the client/listener before submitting START. There is no AP
        // stop event to wait for: association, DHCP and radio remain intact.
        if (!h.stopServer()) restore(now, "http_stop_failed");
        else if (!h.queueStart()) restore(now, "start_queue_failed");
        else { phase_ = Phase::Starting; since_ms_ = now; }
        break;
      case Phase::Starting: {
        const auto result = h.startResult();
        if (result == StartResult::Rejected) restore(now, "start_rejected");
        else if (result == StartResult::Started) {
          // A short failed run may already be sealed before this observer runs.
          if (h.runActive()) phase_ = Phase::Running;
          else restore(now, "");
        } else if (now - since_ms_ >= 5000) {
          error_ = "start_timeout"; phase_ = Phase::Cancelling;
        }
        break;
      }
      case Phase::Running:
        // active becomes false only AFTER control seals logs and foot frames.
        if (!h.runActive()) restore(now, "");
        break;
      case Phase::Cancelling:
        h.cancelStart();
        if (h.startResult() != StartResult::Pending && !h.runActive()) restore(now, error_);
        break;
      case Phase::Restoring:
        if (!attempted_restore_ || now - since_ms_ >= 1000) {
          attempted_restore_ = true; since_ms_ = now;
          if (h.restoreTransport()) { phase_ = Phase::Online; h.transportReady(); }
        }
        break;
    }
  }
 private:
  void restore(uint32_t now, const char* error) {
    error_ = error; phase_ = Phase::Restoring; since_ms_ = now; attempted_restore_ = false;
  }
  Phase phase_ = Phase::Online;
  uint32_t queued_ms_ = 0, since_ms_ = 0;
  bool attempted_restore_ = false;
  const char* error_ = "";
};
