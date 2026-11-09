#pragma once
#include <Arduino.h>
#include <esp_err.h>
#include <cassert>
#define SOC_TIMER_GROUP_SUPPORT_XTAL 1
using timer_isr_t=bool (*)(void*);
constexpr int TIMER_GROUP_1=1,TIMER_0=0,TIMER_ALARM_EN=1,TIMER_PAUSE=0,TIMER_INTR_LEVEL=1;
constexpr int TIMER_COUNT_UP=1,TIMER_AUTORELOAD_EN=1,TIMER_SRC_CLK_APB=0;
struct timer_config_t {int alarm_en=0,counter_en=0,intr_type=0,counter_dir=0,auto_reload=0,divider=0,clk_src=0;};
namespace timer_stub {
inline std::vector<std::string> calls;
inline std::string fail;
inline timer_config_t config;
inline uint64_t counter=0,alarm=0;
inline int flags=0;
inline timer_isr_t callback=nullptr;
inline void* context=nullptr;
inline bool initialized=false,registered=false,started=false;
inline int step(int group,int timer,const char* name){assert(group==1&&timer==0);calls.emplace_back(name);return fail==name?ESP_FAIL:ESP_OK;}
inline void reset(){calls.clear();fail.clear();callback=nullptr;context=nullptr;initialized=registered=started=false;}
}
inline int timer_init(int g,int t,const timer_config_t* c){auto r=timer_stub::step(g,t,"init");timer_stub::config=*c;if(!r)timer_stub::initialized=true;return r;}
inline int timer_set_counter_value(int g,int t,uint64_t n){timer_stub::counter=n;return timer_stub::step(g,t,"counter");}
inline int timer_set_alarm_value(int g,int t,uint64_t n){timer_stub::alarm=n;return timer_stub::step(g,t,"alarm");}
inline int timer_isr_callback_add(int g,int t,timer_isr_t cb,void* arg,int flags){auto r=timer_stub::step(g,t,"add");timer_stub::flags=flags;if(!r){timer_stub::registered=true;timer_stub::callback=cb;timer_stub::context=arg;}return r;}
inline int timer_start(int g,int t){auto r=timer_stub::step(g,t,"start");if(!r)timer_stub::started=true;return r;}
inline int timer_pause(int g,int t){timer_stub::started=false;return timer_stub::step(g,t,"pause");}
inline int timer_isr_callback_remove(int g,int t){timer_stub::registered=false;timer_stub::callback=nullptr;timer_stub::context=nullptr;return timer_stub::step(g,t,"remove");}
inline int timer_deinit(int g,int t){timer_stub::initialized=false;return timer_stub::step(g,t,"deinit");}
