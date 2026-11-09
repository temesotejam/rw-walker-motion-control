#pragma once
#include "realtime_code.h"
#include <stddef.h>
#include <stdint.h>

// Boot hands the dedicated I2C1 controller to this driver exactly once, before
// any reader task exists. No M5GFX transactions are allowed after that handoff.
namespace imu_i2c {
bool begin(int port, int sda, int scl, uint8_t address, uint32_t frequency);
bool RW_SPEED_CODE read(uint8_t reg, uint8_t* dst, size_t size);
bool ready();
int lastError();
}
