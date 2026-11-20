#pragma once
#include <stdint.h>
#include <string.h>

// Owned by the HTTP task. A sealed measurement gets at most one automatic
// radio repair, even if its browser has simply been closed.
class ExportTransportRecovery {
 public:
  static constexpr uint32_t kQuietMs = 35000;

  void startRun() { state_ = State::Idle; }
  void sealed(uint32_t now) {
    if (state_ != State::Idle) return;  // The explicit prepare route is idempotent.
    state_ = State::Waiting;
    last_activity_ms_ = now;
  }
  bool awaitingDownload() const { return state_ == State::Waiting; }
  void onRequest(uint32_t now) {
    if (awaitingDownload()) last_activity_ms_ = now;
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

  bool requestRepair(uint32_t now, bool stopped, bool manual) {
    if (!stopped) return false;
    if (!manual && (!awaitingDownload() || now - last_activity_ms_ < kQuietMs)) return false;
    if (have_repaired_ && now - last_repair_ms_ < kQuietMs) return false;
    have_repaired_ = true;
    last_repair_ms_ = now;
    if (awaitingDownload()) state_ = State::RepairAttempted;
    return true;
  }

 private:
  enum class State { Idle, Waiting, RepairAttempted, Complete };
  State state_ = State::Idle;
  bool have_repaired_ = false;
  uint32_t last_activity_ms_ = 0, last_repair_ms_ = 0;
};
