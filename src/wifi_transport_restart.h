#pragma once
#include <stdint.h>

// HTTP-owner state. Only the radio, its address/DHCP service and the TCP listener
// participate; repeated AP or listener startup failures retry on a later tick.
class WifiTransportRestart {
 public:
  bool active() const { return phase_ != Phase::Idle; }
  uint32_t count() const { return count_; }
  void request() { if (!active()) phase_ = Phase::Stop; }

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
