#include "camera_task_priority_patch.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "config.h"
#include "runtime_diagnostics.h"

namespace {

constexpr UBaseType_t kCameraInternalTaskPriority = 3;
// ESP-IDF uses BYTES here. Its precompiled camera requests only 2048 bytes,
// insufficient for FB-SIZE -> esp_log_write -> newlib -> UART error reporting.
// Changing an application CONFIG_CAMERA_TASK_STACK_SIZE macro cannot rebuild
// that library: change the actual RTOS allocation at this existing link hook.
constexpr uint32_t kCameraInternalTaskStackBytes = 8192;
static_assert(kCameraInternalTaskPriority < Config::ROLLER_IO_TASK_PRIORITY,
              "Camera internal task must stay below Roller485");

portMUX_TYPE g_patch_mux = portMUX_INITIALIZER_UNLOCKED;
CameraTaskPriorityPatchSnapshot g_patch_snapshot;
TaskHandle_t g_camera_task = nullptr;
uint32_t g_stack_sample_ms = 0;

}  // namespace

extern "C" BaseType_t __real_xTaskCreatePinnedToCore(
    TaskFunction_t pvTaskCode,
    const char* const pcName,
    const uint32_t usStackDepth,
    void* const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t* const pvCreatedTask,
    const BaseType_t xCoreID);

extern "C" BaseType_t __wrap_xTaskCreatePinnedToCore(
    TaskFunction_t pvTaskCode,
    const char* const pcName,
    const uint32_t usStackDepth,
    void* const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t* const pvCreatedTask,
    const BaseType_t xCoreID) {
  UBaseType_t effective_priority = uxPriority;
  BaseType_t effective_core = xCoreID;
  uint32_t effective_stack = usStackDepth;
  const bool camera_task = pcName && strcmp(pcName, "cam_task") == 0;

  if (camera_task) {
    effective_priority = kCameraInternalTaskPriority;
    effective_core = 0;
    if (effective_stack < kCameraInternalTaskStackBytes)
      effective_stack = kCameraInternalTaskStackBytes;
    portENTER_CRITICAL(&g_patch_mux);
    g_patch_snapshot = CameraTaskPriorityPatchSnapshot{};
    g_patch_snapshot.observed = true;
    g_patch_snapshot.original_priority =
        static_cast<uint8_t>(uxPriority > 255 ? 255 : uxPriority);
    g_patch_snapshot.effective_priority =
        static_cast<uint8_t>(effective_priority);
    g_patch_snapshot.core = static_cast<int8_t>(effective_core);
    g_patch_snapshot.original_stack_bytes = usStackDepth;
    g_patch_snapshot.effective_stack_bytes = effective_stack;
    portEXIT_CRITICAL(&g_patch_mux);
    g_camera_task = nullptr;
    g_stack_sample_ms = 0;
  }

  TaskHandle_t created_task = nullptr;
  const BaseType_t result = __real_xTaskCreatePinnedToCore(
      pvTaskCode, pcName, effective_stack, pvParameters,
      effective_priority, camera_task ? &created_task : pvCreatedTask, effective_core);
  if (camera_task) {
    if (pvCreatedTask) *pvCreatedTask = created_task;
    if (result == pdPASS) g_camera_task = created_task;
    portENTER_CRITICAL(&g_patch_mux);
    g_patch_snapshot.created = result == pdPASS && created_task != nullptr;
    portEXIT_CRITICAL(&g_patch_mux);
    RuntimeDiag::cameraDriverTask(usStackDepth, effective_stack, g_camera_task != nullptr);
  }
  return result;
}

CameraTaskPriorityPatchSnapshot cameraTaskPriorityPatchSnapshot() {
  portENTER_CRITICAL(&g_patch_mux);
  const CameraTaskPriorityPatchSnapshot copy = g_patch_snapshot;
  portEXIT_CRITICAL(&g_patch_mux);
  return copy;
}

void cameraTaskPriorityPatchSampleStack() {
  if (!g_camera_task || !RuntimeDiag::heavyAllowed()) return;
  const uint32_t now = millis();
  if (g_stack_sample_ms && now - g_stack_sample_ms < 1000) return;
  // The lifecycle owner guarantees this task is alive throughout this scan.
  // Scan outside the snapshot lock, cache results for all other readers.
  const uint32_t free_bytes = uxTaskGetStackHighWaterMark(g_camera_task);
  portENTER_CRITICAL(&g_patch_mux);
  g_patch_snapshot.stack_observed = true;
  g_patch_snapshot.minimum_free_stack_bytes = free_bytes;
  portEXIT_CRITICAL(&g_patch_mux);
  g_stack_sample_ms = now;
  RuntimeDiag::cameraDriverStack(free_bytes);
}

void cameraTaskPriorityPatchForgetTask() {
  // Clear BEFORE the driver deletes the task; never scan a stale RTOS handle.
  g_camera_task = nullptr;
  RuntimeDiag::cameraDriverStopped();
}
