#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <esp_err.h>
using i2c_port_t = int;
constexpr int I2C_NUM_1=1, I2C_MODE_MASTER=1, GPIO_PULLUP_ENABLE=1;
struct i2c_config_t { int mode=0,sda_io_num=0,scl_io_num=0,sda_pullup_en=0,scl_pullup_en=0; struct {uint32_t clk_speed=0;} master; };
namespace idf_stub {
inline int config_error=0, install_error=0, read_error=0, installs=0, deletes=0, reads=0;
inline uint8_t chip=0x24, state=1, power=6;
inline i2c_config_t config;
inline int interrupt_flags=0;
inline void (*while_waiting)()=nullptr;
}
inline int i2c_param_config(int p,const i2c_config_t* c) { if(p!=1) abort(); idf_stub::config=*c; return idf_stub::config_error; }
inline int i2c_driver_install(int p,int m,int rx,int tx,int flags) {
  if(p!=1 || m!=1 || rx || tx) abort();
  ++idf_stub::installs; idf_stub::interrupt_flags=flags; return idf_stub::install_error;
}
inline int i2c_driver_delete(int p) { if(p!=1) abort(); ++idf_stub::deletes; return 0; }
inline int i2c_master_write_read_device(int p,uint8_t address,const uint8_t* reg,size_t nw,uint8_t* dst,size_t nr,TickType_t ticks) {
  if(p!=1 || address!=0x68 || nw!=1 || nr>12 || ticks!=2) abort();
  ++idf_stub::reads;
  if (idf_stub::while_waiting) idf_stub::while_waiting();
  host_us += 50;
  if(idf_stub::read_error) return idf_stub::read_error;
  *dst=*reg==0? idf_stub::chip : *reg==0x21? idf_stub::state : *reg==0x7d? idf_stub::power : 0;
  return 0;
}
