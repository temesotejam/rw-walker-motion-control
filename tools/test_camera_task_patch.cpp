#include <cassert>
#include <cstdio>
#include <freertos/task.h>
#include "camera_task_priority_patch.h"
#include "runtime_diagnostics.h"

namespace {
struct Call {
  TaskFunction_t function;
  const char* name;
  uint32_t stack;
  void* argument;
  UBaseType_t priority;
  TaskHandle_t* output;
  BaseType_t core;
} last{};
BaseType_t create_result = pdPASS;
int task_storage = 0, argument_storage = 0;
const TaskHandle_t task_handle = &task_storage;
void task(void*) {}
}
extern "C" BaseType_t __real_xTaskCreatePinnedToCore(
    TaskFunction_t function, const char* name, uint32_t stack, void* argument,
    UBaseType_t priority, TaskHandle_t* output, BaseType_t core) {
  last = {function, name, stack, argument, priority, output, core};
  if (create_result == pdPASS && output) *output = task_handle;
  return create_result;
}
extern "C" BaseType_t __wrap_xTaskCreatePinnedToCore(
    TaskFunction_t, const char*, uint32_t, void*, UBaseType_t,
    TaskHandle_t*, BaseType_t);

int main() {
  // Exercise the actual compiled wrapper, not a duplicated policy function.
  TaskHandle_t handle = nullptr;
  auto create = [&](const char* name, uint32_t stack, TaskHandle_t* out) {
    return __wrap_xTaskCreatePinnedToCore(task, name, stack, &argument_storage,
                                        23, out, 1);
  };
  assert(!cameraTaskPriorityPatchSnapshot().observed);
  assert(create("cam_task", 2048, &handle) == pdPASS);
  assert(handle == task_handle && last.stack == 8192);
  assert(last.priority == 3 && last.core == 0);
  assert(last.function == task && last.argument == &argument_storage);
  auto s = cameraTaskPriorityPatchSnapshot();
  assert(s.observed && s.created && !s.stack_observed);
  assert(s.original_stack_bytes == 2048 && s.effective_stack_bytes == 8192);
  assert(s.original_priority == 23 && s.effective_priority == 3 && s.core == 0);

  host_us = 1000000;
  cameraTaskPriorityPatchSampleStack();
  assert(host_stack_scans == 0); // Default OFF does not scan the camera driver.
  RuntimeDiag::setEnabled(true);
  RuntimeDiag::setRunActive(true);
  cameraTaskPriorityPatchSampleStack();
  assert(host_stack_scans == 0); // Even opt-in diagnostics defer run-time scans.
  RuntimeDiag::setRunActive(false);
  cameraTaskPriorityPatchSampleStack();
  assert(host_stack_scans == 1 && host_stack_scanned_task == task_handle);
  s = cameraTaskPriorityPatchSnapshot();
  assert(s.stack_observed && s.minimum_free_stack_bytes == 5000);
  cameraTaskPriorityPatchSampleStack();
  assert(host_stack_scans == 1);
  host_us += 1000000; host_stack_free = 4200;
  cameraTaskPriorityPatchSampleStack();
  assert(host_stack_scans == 2);
  assert(cameraTaskPriorityPatchSnapshot().minimum_free_stack_bytes == 4200);
  cameraTaskPriorityPatchForgetTask();
  host_us += 1000000;
  cameraTaskPriorityPatchSampleStack();
  assert(host_stack_scans == 2); // A deleted task must never be scanned.

  // Preserve a larger future SDK stack; accept callers not requesting a handle.
  assert(create("cam_task", 12288, nullptr) == pdPASS);
  assert(last.stack == 12288 && last.output != nullptr);
  cameraTaskPriorityPatchSampleStack();
  assert(host_stack_scans == 3);
  cameraTaskPriorityPatchForgetTask();

  create_result = -1;
  handle = task_handle;
  assert(create("cam_task", 2048, &handle) == -1);
  assert(handle == nullptr);
  s = cameraTaskPriorityPatchSnapshot();
  assert(s.observed && !s.created && !s.stack_observed);
  host_us += 1000000;
  cameraTaskPriorityPatchSampleStack();
  assert(host_stack_scans == 3); // Allocation failure cannot leave a stale handle.

  create_result = pdPASS;
  for (const char* name : {"imu_reader", "foot_observer", "cam_task_extra", "CAM_TASK", static_cast<const char*>(nullptr)}) {
    assert(create(name, 4096, &handle) == pdPASS);
    assert(last.name == name && last.stack == 4096 && last.priority == 23 && last.core == 1);
    assert(last.output == &handle && handle == task_handle);
    assert(last.function == task && last.argument == &argument_storage);
  }
  assert(create(nullptr, 6144, nullptr) == pdPASS && last.output == nullptr);
  assert(!cameraTaskPriorityPatchSnapshot().created);
  std::puts("PASS: camera RTOS stack allocation, task isolation, failure propagation and safe stack sampling");
}
