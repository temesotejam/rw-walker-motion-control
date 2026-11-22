#pragma once
#include <stdint.h>
#include <string.h>

// HTTP-task owned. Browsing is sufficient liveness until a download is
// requested. During a download only progress on that export extends its grace.
class ExportTransportRecovery {
 public:
  static constexpr uint32_t kQuietMs = 35000;
  static constexpr uint32_t kTransferStallMs = 20000;

  void startRun() { state_ = State::Idle; }
  void sealed(uint32_t now) {
    if (state_ != State::Idle) return;
    state_ = State::Sealed;
    repairs_ = 0;
    last_activity_ms_ = now;
  }
  bool awaitingDownload() const {
    return (state_ == State::Sealed && repairs_ == 0) ||
           (state_ == State::Downloading && repairs_ < kMaxTransferRepairs);
  }
  void onRequest(uint32_t now) {
    if (state_ == State::Sealed) last_activity_ms_ = now;
  }
  void downloadRequested(uint32_t now, uint32_t client) {
    if (state_ == State::Idle ||
        ((state_ == State::Downloading || state_ == State::Paused) && client_ == client)) return;
    state_ = State::Downloading;
    client_ = client;
    build_bytes_ = 0;
    chunk_offset_ = UINT32_MAX;
    sent_end_ = 0;
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
    if (state_ != State::Downloading || offset == chunk_offset_) return;
    chunk_offset_ = offset;
    progress(now);
  }
  // Grant time for the browser to verify a newly served range. Re-sending
  // the same range cannot postpone repair indefinitely and is not an ACK.
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

  bool requestRepair(uint32_t now, bool stopped, bool manual) {
    if (!stopped) return false;
    const uint32_t grace = state_ == State::Downloading ? kTransferStallMs : kQuietMs;
    if (!manual && (!awaitingDownload() || now - last_activity_ms_ < grace)) return false;
    if (have_repaired_ && now - last_repair_ms_ < kQuietMs) return false;
    have_repaired_ = true;
    last_repair_ms_ = now;
    if (awaitingDownload()) ++repairs_;
    return true;
  }

 private:
  // At 20 s and 55 s both attempts fall within the browser's 70 s retry window.
  static constexpr uint8_t kMaxTransferRepairs = 2;
  enum class State { Idle, Sealed, Downloading, Paused, Complete };
  void progress(uint32_t now) {
    last_activity_ms_ = now;
    repairs_ = 0;
  }
  State state_ = State::Idle;
  uint8_t repairs_ = 0;
  bool have_repaired_ = false;
  uint32_t last_activity_ms_ = 0, last_repair_ms_ = 0;
  uint32_t build_bytes_ = 0, chunk_offset_ = UINT32_MAX, sent_end_ = 0, client_ = 0;
};
