#pragma once

#include <Arduino.h>

struct CameraTaskPriorityPatchSnapshot {
  bool observed = false;
  uint8_t original_priority = 0;
  uint8_t effective_priority = 0;
  int8_t core = -1;
  bool created = false;
  bool stack_observed = false;
  uint32_t original_stack_bytes = 0;
  uint32_t effective_stack_bytes = 0;
  uint32_t minimum_free_stack_bytes = 0;
};

CameraTaskPriorityPatchSnapshot cameraTaskPriorityPatchSnapshot();
// Camera lifecycle owner only: after successful init and before deinit. The
// current runtime hands ownership from setup to FootObserver, with no parallel
// init/deinit. Never call these from HTTP or the independent USB observer.
void cameraTaskPriorityPatchSampleStack();
void cameraTaskPriorityPatchForgetTask();
