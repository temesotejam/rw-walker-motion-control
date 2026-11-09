#pragma once
#include "FreeRTOS.h"
using TaskHandle_t = void*;
using TaskFunction_t = void (*)(void*);
inline uint32_t host_stack_scans = 0, host_stack_free = 5000;
inline TaskHandle_t host_stack_scanned_task = nullptr;
inline UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task) {
  ++host_stack_scans; host_stack_scanned_task = task; return host_stack_free;
}
inline uint32_t host_tasks_created=0;
inline int host_core=1;
inline BaseType_t host_task_result=pdPASS;
inline void (*host_task_hook)(TaskFunction_t, void*, uint32_t, uint32_t, uint32_t)=nullptr;
inline uint32_t host_tasks_deleted=0, host_isr_notifications=0;
inline BaseType_t host_isr_wakes=pdTRUE;
inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn,const char*,uint32_t stack,void* arg,uint32_t priority,TaskHandle_t* h,uint32_t core){
  ++host_tasks_created;
  if (host_task_result!=pdPASS) return host_task_result;
  if(h) *h=reinterpret_cast<void*>(1);
  if(host_task_hook) host_task_hook(fn,arg,stack,priority,core);
  return pdPASS;
}
inline void vTaskDelete(TaskHandle_t) {++host_tasks_deleted;}
inline void xTaskNotifyGive(TaskHandle_t) {}
inline void vTaskNotifyGiveFromISR(TaskHandle_t task,BaseType_t* woken) {if(!task) abort();++host_isr_notifications;*woken=host_isr_wakes;}
inline uint32_t ulTaskNotifyTake(BaseType_t,TickType_t) {return 1;}
inline void vTaskDelay(TickType_t n) { delay(n); }
inline int xPortGetCoreID() {return host_core;}
inline UBaseType_t uxTaskPriorityGet(TaskHandle_t) {return 2;}
