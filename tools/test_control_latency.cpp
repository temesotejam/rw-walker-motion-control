#include <cassert>
#include <iostream>
#include "control_latency.h"
int main() {
  using namespace control_latency;
  Span span; span.begin(UINT32_MAX-20); assert(span.at(29)==50);
  span.end(39); assert(span.at(99)==60);
  Profile p;
  p.receive(true,1,1000,600,1100,{100,60,8});
  p.mark(Derived,1130,{100,60,8});
  p.mark(Mekf,1500,{300,230,9});
  p.mark(Ready,1600,{400,310,9});
  p.mark(Decision,1650,{400,310,9});
  p.finish(1800,{400,310,9});
  assert(p.worst[0].poll_to_sample_us==400);
  assert(p.worst[0].age[Ready]==600 && p.worst[0].age[Done]==800);
  assert(p.worst[0].poll_overlap_to_ready_us==300 && p.worst[0].io_overlap_to_ready_us==250);
  assert(p.age[Decision].count==1 && p.worst[0].valid==63);
  p.receive(true,2,2000,1600,2050,{400,310,9});
  p.mark(Ready,2100,{400,310,9}); p.finish(2200,{400,310,9});
  assert(p.worst[0].sequence==1 && p.age[Done].count==2 && p.age[Decision].count==1);
  PsramString out; assert(out.reserve(12000)); p.appendJson(out); assert(out.ok());
  assert(std::string(out.c_str()).find("not_cpu")!=std::string::npos);
  p.reset(); assert(!p.epoch && !p.worst[0].sequence && !p.age[Done].count);
  p.receive(false,3,2000,1600,2050,{}); p.finish(2200,{}); assert(!p.age[Done].count);
  // Strict boundary: 2500 us passes; retain repeated violations in one bucket.
  p.receive(true,10,10000,9700,10100,{},Delivery(10020,10070));p.finish(12500,{});
  assert(!p.overrun_total && p.queue_submit_age.max_us==20);
  assert(p.queue_receive_age.max_us==70 && p.submit_to_receive.max_us==50 && p.receive_to_mark.max_us==30);
  for(unsigned i=0;i<70;++i) {
    const uint32_t stamp=20000+i*4000;
    p.receive(true,11+i,stamp,stamp-400,stamp+200,{},Delivery(stamp+25,stamp+180));
    p.finish(stamp+2501+i,{});
  }
  assert(p.overrun_total==70&&p.overrun_stored==64&&p.overrun_overflow==6);
  for(unsigned i=0;i<64;++i) {
    assert(p.overruns[i].sequence==11+i&&p.overruns[i].age[Done]==2501+i);
    assert(p.overruns[i].delivery_valid&&p.overruns[i].queue_submit_age_us==25&&p.overruns[i].queue_receive_age_us==180);
  }
  assert(p.worst[0].sequence==80); // Bucket summary and full overrun detail are independent.
  PsramString detail;assert(detail.reserve(32000));p.appendJson(detail);assert(detail.ok());
  const std::string json=detail.c_str();
  assert(json.find("\"total\":70,\"stored\":64,\"overflow\":6")!=std::string::npos);
  p.reset();assert(!p.overrun_total&&!p.overrun_stored&&!p.overrun_overflow&&!p.queue_submit_age.count);
  p.receive(true,100,UINT32_MAX-20,UINT32_MAX-50,50,{},Delivery(UINT32_MAX-10,29));
  p.finish(2600,{});
  assert(p.overruns[0].queue_submit_age_us==10&&p.overruns[0].queue_receive_age_us==50);
  assert(p.overruns[0].age[Done]==2621&&p.submit_to_receive.max_us==40);
  std::cout << "Latency: clock wrap, delivery stages, strict deadline, same-bucket overruns, bounded detail/overflow, reset and idle exclusion PASS\n";
}
