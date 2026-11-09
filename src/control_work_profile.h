#pragma once
#include <Arduino.h>
#include <stdint.h>

// Written only by the permanent controller; exported only after its run stops.
// No locks, allocation, floating point, or I/O in the measurement path.
namespace control_work {
enum class Stage : uint8_t {
  Owner, Service, ImuDelivery, PoseGuide, Filter, AngleDisplay, CurrentRoll,
  Motion, LogRow, Snapshot, Publish, MekfPredict, MekfAccel, MekfAttitude,
  Madgwick, LogEncode, LogStore, Count
};
struct Stat {
  uint32_t count = 0, max_us = 0;
  uint64_t sum_us = 0;
  void add(uint32_t us) { ++count; sum_us += us; if (us > max_us) max_us = us; }
};
struct Profile {
  Stat stages[2][static_cast<uint8_t>(Stage::Count)]{};
  void reset() { for (auto& group : stages) for (auto& s : group) s = Stat{}; }
  void add(Stage stage, bool pulse, uint32_t us) {
    stages[pulse ? 1 : 0][static_cast<uint8_t>(stage)].add(us);
  }
  String json() const;
};
extern Profile profile;
class Scope {
 public:
  Scope(Stage stage, bool enabled, bool pulse)
      : stage_(stage), enabled_(enabled), pulse_(pulse), start_(enabled ? micros() : 0) {}
  ~Scope() { if (enabled_) profile.add(stage_, pulse_, static_cast<uint32_t>(micros() - start_)); }
  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;
 private:
  Stage stage_;
  bool enabled_, pulse_;
  uint32_t start_;
};
}
