#include "mekf_attitude_diagnostics.h"

// A single non-inline definition keeps the Xtensa literal pool with this
// IRAM routine. A COMDAT inline body can leave its literals in flash.
MekfAttitudeSnapshot captureMekfAttitude(const mekf6::Mekf6& filter,
                                        bool initialized, uint32_t sample_us,
                                        const mekf6::Vec3& accel_g,
                                        const mekf6::Vec3& gyro_rad_s,
                                        uint32_t accel_sample_us) {
  MekfAttitudeSnapshot out;
  if (!initialized) return out;
  out.quaternion = filter.quaternion();
  out.pitch_deg = mekf6::Mekf6::pitchDegFromQuaternion(out.quaternion);
  out.sample_us = sample_us;
  out.accel_g = accel_g; out.gyro_rad_s = gyro_rad_s;
  out.accel_sample_us = accel_sample_us;
  out.bias_rad_s = filter.gyroBiasRadS();
  out.accel_update = filter.diagnostics();
  const auto& q = out.quaternion;
  out.valid = isfinite(out.pitch_deg) &&
      isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
  return out;
}
