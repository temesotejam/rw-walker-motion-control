#pragma once
namespace work_start_policy {
inline bool referenceAllowed(bool absolute_work, bool saved_reference_valid) {
  return absolute_work || saved_reference_valid;
}
inline bool ready(bool controller_ready, bool fresh, bool imu_ok, bool motor_ok, bool exporter_ready) {
  return controller_ready && fresh && imu_ok && motor_ok && exporter_ready;
}
}
