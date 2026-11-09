#pragma once
#include <Arduino.h>
namespace m5 {
constexpr int imu_bmi270=1;
struct IMU_Class {enum {sensor_mask_accel=1,sensor_mask_gyro=2};};
}
struct HostBus { bool release_ok=true; bool release() const { return release_ok; } int getPort()const{return 1;} int getSDA()const{return 45;}int getSCL()const{return 0;}};
struct HostDevice {
  uint8_t getAddress() const { return 0x68; }
  uint8_t acc=0xa8,gyr=0xe9,status=1,power=0x0e;
  uint8_t readRegister8(uint8_t a){switch(a){case 0x21:return status;case 0x7d:return power;case 0x40:return acc;case 0x42:return gyr;default:return 0;}}
  bool readRegister(uint8_t a,uint8_t* p,size_t n){for(size_t i=0;i<n;++i)p[i]=readRegister8(a+i);return true;}
  bool writeRegister8(uint8_t a,uint8_t v){if(a==0x40)acc=v;if(a==0x42)gyr=v;return true;}
};
struct HostImu {
  struct Axis{float x=0,y=0,z=0;};
  struct Data{Axis accel{0.021626f,0.033568f,-0.999202f},gyro{0.1f,0.1f,0.1f};uint32_t usec=0;} data;
  HostDevice dev;
  unsigned begin_calls=0,fail_begins=0,no_stream_attempts=0;
  uint32_t last_a=0,last_g=0;
  uint32_t update_cost_us=0,convert_cost_us=0,clock_hz=400000;
  bool begin(HostBus* b,int board){if(!b||board!=7)abort();++begin_calls;last_a=last_g=0;return begin_calls>fail_begins;}
  int getType(){return m5::imu_bmi270;}
  void setClock(uint32_t hz){clock_hz=hz;}
  HostDevice* getImuInstancePtr(unsigned){return &dev;}
  uint8_t update(){host_us+=update_cost_us;if(begin_calls<=no_stream_attempts)return 0;uint8_t b=0;if(host_us/5000!=last_a){last_a=host_us/5000;b|=1;}if(host_us/2500!=last_g){last_g=host_us/2500;b|=2;}data.usec=host_us;return b;}
  Data getImuData(){host_us+=convert_cost_us;return data;}
};
struct HostM5{HostBus In_I2C;HostImu Imu;int getBoard(){return 7;}};
inline HostM5 M5;
