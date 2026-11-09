#pragma once
#include "FreeRTOS.h"
struct StaticQueue_t {size_t capacity=0,size=0;std::deque<std::vector<uint8_t>> items;};
using QueueHandle_t = StaticQueue_t*;
inline QueueHandle_t xQueueCreateStatic(size_t n,size_t size,uint8_t*,StaticQueue_t* q){q->capacity=n;q->size=size;q->items.clear();return q;}
inline BaseType_t xQueueSend(QueueHandle_t q,const void* p,TickType_t){if(q->items.size()>=q->capacity)return pdFALSE;const auto* b=static_cast<const uint8_t*>(p);q->items.emplace_back(b,b+q->size);return pdTRUE;}
inline BaseType_t xQueueReceive(QueueHandle_t q,void* p,TickType_t wait){if(q->items.empty()){delay(wait);return pdFALSE;}memcpy(p,q->items.front().data(),q->size);q->items.pop_front();return pdTRUE;}
inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q){return q->items.size();}

inline BaseType_t xQueuePeek(QueueHandle_t q,void* p,TickType_t){if(q->items.empty())return pdFALSE;memcpy(p,q->items.front().data(),q->size);return pdTRUE;}
