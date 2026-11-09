#include "foot_geometry_async.h"
#include <cassert>
#include <cstdint>
static int created=0,deleted=0,fail_at=0,task_ok=0;
QueueHandle_t xQueueCreate(unsigned,std::size_t) {
  ++created;if(created==fail_at)return nullptr;
  return reinterpret_cast<void*>(static_cast<uintptr_t>(created));
}
void vQueueDelete(QueueHandle_t p){assert(p);++deleted;}
int xQueueSend(QueueHandle_t,const void*,unsigned){return 1;}
int xQueueReceive(QueueHandle_t,void*,unsigned){return 0;}
int xQueueOverwrite(QueueHandle_t,const void*){return 1;}
int xTaskCreatePinnedToCore(void(*)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t*,int){return task_ok;}
void vTaskDelay(unsigned){}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t){return 123;}
int64_t esp_timer_get_time(){return 0;}
int main(){
  foot_geometry_worker::Async a;
  fail_at=2;assert(!a.begin());assert(created==3 && deleted==2);
  fail_at=0;assert(!a.begin());assert(created==6 && deleted==5);
  assert(!a.begin());assert(created==9 && deleted==8);
}
