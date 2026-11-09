#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace camera_lifecycle {
using Work = bool (*)(void*);
struct Result { bool invoked; bool ok; const char* error; };
struct Call { Work work; void* context; SemaphoreHandle_t done; bool ok; };
inline void entry(void* arg) {
  auto* call = static_cast<Call*>(arg);
  call->ok = call->work(call->context);
  // The caller owns the call/semaphore storage. Never touch it after signalling.
  const SemaphoreHandle_t done = call->done;
  xSemaphoreGive(done);
  vTaskDelete(nullptr);
}
inline Result run(Work work, void* context) {
  if (xPortGetCoreID() == 0) return {true, work(context), nullptr};
  StaticSemaphore_t storage{};
  const auto done = xSemaphoreCreateBinaryStatic(&storage);
  if (!done) return {false, false, "camera_lifecycle_semaphore_failed"};
  Call call{work, context, done, false};
  if (xTaskCreatePinnedToCore(entry, "camera_lifecycle", 8192, &call, 2, nullptr, 0) != pdPASS) {
    vSemaphoreDelete(done);
    return {false, false, "camera_lifecycle_task_failed"};
  }
  // Startup waits for sensor setup AND temporary I2C0 release before Roller
  // can take ownership. Do not time out and let a worker outlive this storage.
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return {true, call.ok, nullptr};
}
}
