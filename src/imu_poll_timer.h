#pragma once
#include <Arduino.h>
#include <driver/timer.h>
#include <esp_intr_alloc.h>
#include <freertos/task.h>

// Dedicated TIMG1/T0 (the watchdog in this group is a separate peripheral).
// Only startup/failure cleanup touches this driver. HTTP/run transitions do not.
class ImuPollTimer {
 public:
  static constexpr int kCore = 1;
  bool begin(timer_isr_t callback, void* context, uint32_t period_us) {
    if (initialized_ || xPortGetCoreID() != kCore || !callback || period_us != 1000) {
      error_ = ESP_ERR_INVALID_ARG; return false;
    }
    timer_config_t config{};
    config.alarm_en = TIMER_ALARM_EN;
    config.counter_en = TIMER_PAUSE;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_dir = TIMER_COUNT_UP;
    config.auto_reload = TIMER_AUTORELOAD_EN;
    config.divider = 80;  // Fixed 80 MHz APB / 80 = one microsecond.
#if SOC_TIMER_GROUP_SUPPORT_XTAL
    config.clk_src = TIMER_SRC_CLK_APB;
#endif
    error_ = timer_init(TIMER_GROUP_1, TIMER_0, &config);
    if (error_ != ESP_OK) return false;
    initialized_ = true;
    owner_core_ = xPortGetCoreID();
    error_ = timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
    if (error_ == ESP_OK) error_ = timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, period_us);
    if (error_ == ESP_OK) {
      error_ = timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, callback, context,
                                    ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1);
      callback_installed_ = error_ == ESP_OK;
    }
    if (error_ == ESP_OK) error_ = timer_start(TIMER_GROUP_1, TIMER_0);
    if (error_ != ESP_OK) {
      // Quiesce/free the interrupt before its task/context can be destroyed.
      timer_pause(TIMER_GROUP_1, TIMER_0);
      if (callback_installed_) timer_isr_callback_remove(TIMER_GROUP_1, TIMER_0);
      timer_deinit(TIMER_GROUP_1, TIMER_0);
      initialized_ = callback_installed_ = false;
      return false;
    }
    return true;
  }
  int ownerCore() const { return owner_core_; }
  int lastError() const { return error_; }
 private:
  bool initialized_ = false, callback_installed_ = false;
  int owner_core_ = -1, error_ = ESP_OK;
};
