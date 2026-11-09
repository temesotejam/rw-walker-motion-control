#include "imu_i2c_transport.h"
#include "control_latency.h"
#include <Arduino.h>
#include <driver/i2c.h>
#include <esp_intr_alloc.h>

namespace imu_i2c {
namespace {
bool installed = false;
uint8_t sensor_address = 0;
int error = 0;
constexpr i2c_port_t port = I2C_NUM_1;
}
bool ready() { return installed; }
int lastError() { return error; }
bool read(uint8_t reg, uint8_t* dst, size_t size) {
  if (!installed || !dst || size == 0 || size > 12) return false;
  control_latency::ioBegin();
  // IDF 4.4 uses a stack command link, a completion ISR and xQueueReceive.
  // The reader can BLOCK here; the priority-4 controller can then execute.
  // The tick argument is NOT a hard deadline: IDF's event watchdog may wait
  // longer on a broken bus. The independent 10 ms stale-data STOP remains.
  const esp_err_t result = i2c_master_write_read_device(
      port, sensor_address, &reg, 1, dst, size, pdMS_TO_TICKS(2));
  control_latency::ioEnd(result == ESP_OK);
  error = result;
  return result == ESP_OK;
}
bool begin(int requested_port, int sda, int scl, uint8_t address, uint32_t frequency) {
  if (installed || requested_port != 1 || sda != 45 || scl != 0 ||
      (address != 0x68 && address != 0x69) || frequency != 1000000) {
    error = ESP_ERR_INVALID_ARG; return false;
  }
  i2c_config_t config{};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = sda; config.scl_io_num = scl;
  config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  config.master.clk_speed = frequency;
  error = i2c_param_config(port, &config);
  if (error != ESP_OK) return false;
  // Internal stack buffers and the IDF driver's internal allocations are
  // compatible with an IRAM interrupt. Roller/camera continue to use I2C0.
  error = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_IRAM);
  if (error != ESP_OK) return false;
  installed = true; sensor_address = address;
  uint8_t chip = 0, state = 0, power = 0;
  const bool ok = read(0x00, &chip, 1) && chip == 0x24 &&
      read(0x21, &state, 1) && (state & 0x0f) == 1 &&
      read(0x7d, &power, 1) && (power & 0x06) == 0x06;
  if (!ok) {
    if (error == ESP_OK) error = ESP_ERR_INVALID_RESPONSE;
    i2c_driver_delete(port); installed = false;
  }
  return ok;
}
}
