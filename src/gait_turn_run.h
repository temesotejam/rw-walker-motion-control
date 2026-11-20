#pragma once
#if WALKING_TURN_OBSERVER
#include "gait_turn_observer.h"
#include "psram_archive.h"

class PsramString;

// Control-core owner. Sealed export reads the archive after measurement stops.
class GaitTurnRun {
 public:
  void begin(void* (*allocate)(size_t)) { rows_.begin(allocate); }
  void start(bool enabled) {
    observer_.reset();
    status_ = {};
    status_.enabled = enabled;
    total_ = stored_ = 0;
  }
  void observe(const periodic_foot::Packet& packet) {
    if (status_.enabled) observer_.observe(packet);
  }
  void peak(int side, uint32_t confirmed_ms, float yaw_deg);
  const gait_turn::Status& status() const { return status_; }
  void append(PsramString& json) const;

 private:
  // Thirty seconds at the 125 ms minimum ZERO-to-peak spacing gives at
  // most 120 alternating full cycles. Irregular/repeated peaks also count.
  static constexpr uint16_t capacity = 128;
  PsramArchive<gait_turn::Result, capacity> rows_;
  gait_turn::Observer observer_;
  gait_turn::Status status_;
  uint16_t stored_ = 0;
  uint32_t total_ = 0;
};
#endif
