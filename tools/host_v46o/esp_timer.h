#pragma once
#include <Arduino.h>
#include "esp_err.h"
using esp_timer_handle_t=void*;
constexpr int ESP_TIMER_TASK=0;
struct esp_timer_create_args_t {void(*callback)(void*)=nullptr;void* arg=nullptr;int dispatch_method=0;const char* name=nullptr;bool skip_unhandled_events=false;};
inline int esp_timer_create(const esp_timer_create_args_t*,esp_timer_handle_t* t){*t=reinterpret_cast<void*>(1);return ESP_OK;}
inline int esp_timer_start_periodic(esp_timer_handle_t,uint32_t){return ESP_OK;}
inline int esp_timer_delete(esp_timer_handle_t){return ESP_OK;}
inline int64_t esp_timer_get_time(){return host_us;}
