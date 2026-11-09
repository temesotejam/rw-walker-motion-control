#pragma once
#include <Arduino.h>
using BaseType_t = int;
using UBaseType_t = unsigned;
using TickType_t = unsigned;
using portMUX_TYPE = int;
constexpr int pdTRUE=1, pdFALSE=0, pdPASS=1;
constexpr uint32_t portMAX_DELAY=UINT32_MAX;
#define configTICK_RATE_HZ 1000
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define portENTER_CRITICAL_ISR(x) portENTER_CRITICAL(x)
#define portEXIT_CRITICAL_ISR(x) portEXIT_CRITICAL(x)
#define pdMS_TO_TICKS(x) (x)
