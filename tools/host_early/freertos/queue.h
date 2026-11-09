#pragma once
#include <cstddef>
using QueueHandle_t=void*;
QueueHandle_t xQueueCreate(unsigned,std::size_t);
int xQueueSend(QueueHandle_t,const void*,unsigned);
int xQueueReceive(QueueHandle_t,void*,unsigned);
int xQueueOverwrite(QueueHandle_t,const void*);
void vQueueDelete(QueueHandle_t);
