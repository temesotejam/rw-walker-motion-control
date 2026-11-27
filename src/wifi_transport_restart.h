#pragma once
#include <stdint.h>

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
