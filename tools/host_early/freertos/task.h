#pragma once
using TaskHandle_t=void*;
int xTaskCreatePinnedToCore(void(*)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t*,int);
void vTaskDelay(unsigned);
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t);
