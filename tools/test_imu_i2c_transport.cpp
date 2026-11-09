#include <cassert>
#include <iostream>
#include <driver/i2c.h>
#include <esp_intr_alloc.h>
#include "imu_i2c_transport.h"
#include "control_latency.h"
int main() {
  using namespace imu_i2c; using namespace idf_stub;
  uint8_t value=0;
  assert(!read(3,&value,1));
  assert(!begin(0,45,0,0x68,1000000) && installs==0);
  config_error=-8; assert(!begin(1,45,0,0x68,1000000) && installs==0); config_error=0;
  install_error=-9; assert(!begin(1,45,0,0x68,1000000) && !ready()); install_error=0;
  chip=0; assert(!begin(1,45,0,0x68,1000000) && !ready() && deletes==1); chip=0x24;
  power=0; assert(!begin(1,45,0,0x68,1000000) && !ready() && deletes==2); power=6;
  assert(begin(1,45,0,0x68,1000000) && ready());
  assert(config.master.clk_speed==1000000 && config.sda_io_num==45 && config.scl_io_num==0);
  assert(interrupt_flags==ESP_INTR_FLAG_IRAM);
  assert(!begin(1,45,0,0x68,1000000));
  control_latency::profile.active=true;
  while_waiting=control_latency::progress;
  assert(read(3,&value,1));
  assert(control_latency::profile.io_calls_with_control_progress==1);
  read_error=-7; assert(!read(3,&value,1) && lastError()==-7 && ready());
  assert(control_latency::profile.io_failures==1); // No silent old-driver fallback.
  const auto before=reads; assert(!read(3,&value,13) && reads==before);
  std::cout << "Actual transport: port ownership, probe, cleanup, error propagation and progress counters PASS (mock IDF; not hardware)\n";
}
