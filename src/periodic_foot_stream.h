#pragma once
#if defined(FOOT_PERIODIC_V38) && FOOT_PERIODIC_V38
#include "foot_observer.h"
#include "periodic_foot_state.h"

class PeriodicFootStream {
 public:
  void begin();
  void startRun(uint16_t id);
  void enter(bool walking,uint64_t epoch,uint64_t now);
  void observe(const FootFrame& frame,uint64_t received);
  bool service(uint64_t now);
  periodic_foot::Packet current() const;
  String liveJson() const;
  void appendRun(PsramString& json) const;  // called after sealing
 private:
  struct Row { uint32_t slot, time, left_seq, right_seq; float left,right; };
  static constexpr unsigned capacity = 1201;
  Row* rows_ = nullptr;
  unsigned stored_ = 0;
  uint16_t run_id_ = 0;
  uint64_t epoch_ = 0;
  bool active_ = false, initialized_ = false;
  periodic_foot::Estimator estimator_;
  periodic_foot::Packet run_final_, live_;
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
extern PeriodicFootStream periodicFeet;
#endif
