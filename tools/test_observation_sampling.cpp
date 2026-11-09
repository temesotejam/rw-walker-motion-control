#include "host_v46o/Arduino.h"
#include <cassert>
#include <iostream>
#define private public
#include "../src/foot_observer.h"
#undef private
CameraOneShotSnapshot OneShotCamera::snapshot() const { return {}; }
bool OneShotCamera::startContinuous() { return false; }
camera_fb_t* OneShotCamera::acquireContinuous(uint32_t) { return nullptr; }
void OneShotCamera::releaseContinuous(camera_fb_t*) {}
void OneShotCamera::stopContinuous() {}
int main() {
  FootObserver feet;
  feet.frames_=static_cast<FootFrame*>(ps_malloc(sizeof(FootFrame)*FootObserver::kCapacity));
  feet.status_.count=FootObserver::kCapacity; feet.status_.recording=true; feet.recording_run_id_=7;
  for(unsigned i=0;i<FootObserver::kCapacity;i++) {
    FootFrame f{}; f.sequence=i+1; f.run_id=7; f.frame_us=1000000+i*50000;
    f.delivered_us=f.frame_us+30000; feet.frames_[i]=f;
    feet.status_.count=i+1; feet.recordControlObservation(7,f.sequence,f.delivered_us+12000);
  }
  // New camera frame wins the race after snapshot: do not attach old time
  // to the last frame, count the unmatched attempt, preserve last frame time.
  const uint64_t last=feet.frames_[767].control_observed_us;
  feet.recordControlObservation(7,767,feet.frames_[767].delivered_us+20000);
  assert(feet.control_observation_attempts_==769 && feet.control_observation_unmatched_==1);
  assert(feet.frames_[767].control_observed_us==last);
  feet.recordControlObservation(8,768,feet.frames_[767].delivered_us+20000);
  assert(feet.control_observation_attempts_==769);
  feet.status_.recording=false;
  PsramString json; assert(json.reserve(655360)); feet.appendMetadata(json); assert(json.ok());
  std::cout << "{" << json.c_str() << "}";
}
