#pragma once
#include <stdint.h>
#include <string.h>

// Owned by the HTTP task. Export progress and Wi-Fi reachability have
// separate clocks: a stalled range can still have a working Wi-Fi link.
class ExportTransportRecovery {
 public:
  static constexpr uint32_t kQuietMs = 35000;
  static constexpr uint32_t kTransferStallMs = 20000;
  enum class Repair { None, HttpListener, WifiRadio };

  void startRun() {
    state_ = State::Idle;
    priority_requested_ = false;
    radio_used_ = false;
    listener_attempted_ = false;
    listener_recoveries_ = 0;
  }
  void sealed(uint32_t now) {
    if (state_ != State::Idle) return;
    state_ = State::Sealed;
    last_activity_ms_ = now;
  }
  bool awaitingDownload() const {
    return (state_ == State::Sealed && !radio_used_) ||
           (state_ == State::Downloading && (!listener_attempted_ || !radio_used_));
  }
  bool sealedAfterRadioRepair() const { return state_ == State::Sealed && radio_used_; }
  uint32_t listenerRecoveryCount() const { return listener_recoveries_; }
  // Only an export client requests transfer priority. Ordinary status polls,
  // metadata building and radio recovery cannot extend a departed client's lease.
  void exportContact(uint32_t now) {
    if (state_ != State::Downloading) return;
    last_export_request_ms_ = now;
    priority_requested_ = true;
  }
  bool priorityRequested(uint32_t now) {
    if (state_ != State::Downloading || now - last_export_request_ms_ >= kQuietMs)
      priority_requested_ = false;
    return priority_requested_;
  }
  void onRequest(uint32_t now) {
    if (state_ == State::Sealed) last_activity_ms_ = now;
    if (state_ == State::Downloading) last_request_ms_ = now;
  }
  void downloadRequested(uint32_t now, uint32_t client) {
    if (state_ == State::Idle || (state_ == State::Paused && client_ == client)) return;
    if (state_ == State::Downloading && client_ == client) {
      exportContact(now);
      return;
    }
    state_ = State::Downloading;
    client_ = client;
    build_bytes_ = 0;
    chunk_offset_ = UINT32_MAX;
    sent_end_ = 0;
    last_request_ms_ = now;
    exportContact(now);
    progress(now);
  }
  bool pauseDownload(uint32_t client) {
    if (!client || client != client_ || (state_ != State::Downloading && state_ != State::Paused)) return false;
    state_ = State::Paused;
    return true;
  }
  void buildProgress(uint32_t now, uint32_t bytes) {
    if (state_ != State::Downloading || bytes <= build_bytes_) return;
    build_bytes_ = bytes;
    progress(now);
  }
  void chunkRequested(uint32_t now, uint32_t offset) {
    // The next distinct, validated offset is evidence of client progress,
    // including a resume from a different cache. Repeating one offset is not.
    if (state_ != State::Downloading) return;
    exportContact(now);
    last_request_ms_ = now;
    if (offset == chunk_offset_) return;
    chunk_offset_ = offset;
    progress(now);
  }
  // Grant time for the browser to verify a newly served range. Re-sending
  // the same range does not renew the transfer progress deadline.
  void chunkServed(uint32_t now, uint32_t end) {
    if (state_ == State::Downloading && end > sent_end_) {
      sent_end_ = end;
      progress(now);
    }
  }

  // The caller verifies the browser's whole-file acknowledgement against the
  // current immutable export. Serving any particular chunk does not prove this.
  static bool matchesCompletion(bool ready, const char* expected_token, uint32_t expected_crc,
                                uint16_t expected_run, const char* token, uint32_t crc, uint32_t run) {
    return ready && expected_run == run && expected_crc == crc && expected_token && token &&
           strlen(expected_token) == 16 && strcmp(expected_token, token) == 0;
  }
  void downloadComplete() {
    if (state_ != State::Idle) state_ = State::Complete;
  }

  Repair requestRepair(uint32_t now, bool stopped, bool manual) {
    if (!stopped) return Repair::None;
    if (!manual) {
      if (!awaitingDownload()) return Repair::None;
      if (state_ == State::Downloading && !listener_attempted_) {
        if (now - last_activity_ms_ < kTransferStallMs) return Repair::None;
        listener_attempted_ = true;
        ++listener_recoveries_;
        last_activity_ms_ = now;
        return Repair::HttpListener;
      }
      if (now - last_activity_ms_ < kQuietMs) return Repair::None;
      // A status request or same-offset retry proves that the browser can reach
      // the AP, even if the body transfer failed. Keep its Wi-Fi association.
      if (state_ == State::Downloading && now - last_request_ms_ < kQuietMs)
        return Repair::None;
    }
    if (have_repaired_ && now - last_repair_ms_ < kQuietMs) return Repair::None;
    have_repaired_ = true;
    last_repair_ms_ = now;
    if (state_ != State::Idle) radio_used_ = true;
    return Repair::WifiRadio;
  }

 private:
  enum class State { Idle, Sealed, Downloading, Paused, Complete };
  void progress(uint32_t now) {
    last_activity_ms_ = now;
    listener_attempted_ = false;
  }
  State state_ = State::Idle;
  // Only starting/clearing a measurement renews its automatic radio budget.
  bool radio_used_ = false;
  bool listener_attempted_ = false;
  bool priority_requested_ = false;
  uint32_t last_export_request_ms_ = 0;
  uint32_t listener_recoveries_ = 0;
  bool have_repaired_ = false;
  uint32_t last_activity_ms_ = 0, last_repair_ms_ = 0, last_request_ms_ = 0;
  uint32_t build_bytes_ = 0, chunk_offset_ = UINT32_MAX, sent_end_ = 0, client_ = 0;
};

// Track one observed HTTP outage. A further failed response does not
// postpone the grace period or grant another attempt after a repair.
class HttpStallRecovery {
 public:
  void clear() { state_ = State::Clear; }
  void stalled(uint32_t now) {
    if (state_ == State::Clear) { state_ = State::Waiting; since_ms_ = now; }
  }
  bool due(uint32_t now) const { return state_ == State::Waiting && now - since_ms_ >= 35000; }
  void repairIssued() { if (state_ == State::Waiting) state_ = State::Repaired; }
 private:
  enum class State { Clear, Waiting, Repaired };
  State state_ = State::Clear;
  uint32_t since_ms_ = 0;
};

// HTTP-owner state. Only the radio, its address/DHCP service and the TCP listener
// participate; repeated AP or listener startup failures retry on a later tick.
class WifiTransportRestart {
 public:
  bool active() const { return phase_ != Phase::Idle; }
  uint32_t count() const { return count_; }
  void request() { if (!active()) phase_ = Phase::Stop; }
  // Startup runs before any walking command can be served. Retry only the
  // missing stage; a failed bind must leave an already available AP online.
  void afterStartup(uint32_t now, bool ap_ready, bool listening) {
    if (ap_ready && listening) return;
    phase_ = ap_ready ? Phase::Listen : Phase::StartAp;
    last_attempt_ = now;
    wait_ms_ = 1000;
    stopped_ = false;
  }

  template<class Host> void update(uint32_t now, Host& host) {
    if (phase_ == Phase::Stop) {
      host.closeHttpTransport();
      stopped_ = host.stopWifiRadio();
      phase_ = Phase::StartAp;
      last_attempt_ = now;
      wait_ms_ = 250;
      return;
    }
    if (!active() || now - last_attempt_ < wait_ms_) return;
    last_attempt_ = now;
    wait_ms_ = 1000;
    if (phase_ == Phase::StartAp) {
      if (!host.startWifiAp()) return;
      phase_ = Phase::Listen;
    }
    if (!host.startHttpListener()) return;
    if (stopped_) ++count_;
    phase_ = Phase::Idle;
  }

 private:
  enum class Phase { Idle, Stop, StartAp, Listen };
  Phase phase_ = Phase::Idle;
  uint32_t last_attempt_ = 0, wait_ms_ = 0, count_ = 0;
  bool stopped_ = false;
};
