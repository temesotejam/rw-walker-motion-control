#pragma once
#include "FreeRTOS.h"
using SemaphoreHandle_t=void*;
struct StaticSemaphore_t { bool available=false; };
inline bool host_semaphore_fail=false;
inline uint32_t host_semaphores_deleted=0;
inline SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t* s){return host_semaphore_fail?nullptr:s;}
inline SemaphoreHandle_t xSemaphoreCreateMutex(){return reinterpret_cast<void*>(1);}
inline int xSemaphoreTake(SemaphoreHandle_t h, unsigned){
  if(h==reinterpret_cast<void*>(1))return pdTRUE;
  auto* s=static_cast<StaticSemaphore_t*>(h);const bool ready=s->available;s->available=false;return ready?pdTRUE:pdFALSE;
}
inline int xSemaphoreGive(SemaphoreHandle_t h){if(h!=reinterpret_cast<void*>(1))static_cast<StaticSemaphore_t*>(h)->available=true;return pdTRUE;}
inline void vSemaphoreDelete(SemaphoreHandle_t){++host_semaphores_deleted;}
