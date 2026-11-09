#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;
static uint16_t tick(Controller& x,uint32_t start,float l,float r,float peak=5.f,float target=1.f){
  const auto w=x.limit(start+50,30,1200,1,60,true,target);x.confirm(start+50,true,1200,1,w);
  Cycle c;c.start_ms=start;c.end_ms=start+100;c.positive_ms=start+50;
  c.left=l;c.right=r;c.negative_peak=peak;c.target_mj=target;
  c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 80u;
  c.positive_width=w;c.positive_ma=1200;c.positive_sign=1;
  c.complete=c.negative_known=c.positive_executed=true;x.cycle(c);return w;
}
int main(){
  static_assert(foot_trend::Window::kCycles==4,"production window must be four cycles");
  Controller x;x.preferHighCurrentFirst();
  assert(tick(x,100,1,3)==30);assert(tick(x,200,3,1)==30);assert(tick(x,300,3,1)==30);
  assert(x.offset()==0);assert(tick(x,400,3,1)==30);assert(x.offset()==5);
  assert(tick(x,500,2.4f,1.8f)==35);
  for(uint32_t t=600;t<=800;t+=100){assert(tick(x,t,2.4f,1.8f)==35);assert(x.offset()==5);}
  assert(tick(x,900,2.4f,1.8f)==35);assert(x.offset()==10);
  // Long-term opposite-signed cycles cancel as an average instead of being
  // counted as four large same-sided errors.
  Controller alternating;alternating.preferHighCurrentFirst();
  for(uint32_t t=100;t<=400;t+=100)tick(alternating,t,3,1);
  tick(alternating,500,3,1);
  tick(alternating,600,3,1);tick(alternating,700,1,3);
  tick(alternating,800,3,1);tick(alternating,900,1,3);
  assert(alternating.offset()==5);
  Controller overlap;overlap.preferHighCurrentFirst();
  for(uint32_t t=100;t<=400;t+=100)tick(overlap,t,3,1);
  assert(tick(overlap,500,3,1,2)==35);assert(overlap.offset()==0);
  Controller loss;loss.preferHighCurrentFirst();
  for(uint32_t t=100;t<=400;t+=100)tick(loss,t,3,1);
  assert(tick(loss,500,3,1)==35);
  assert(tick(loss,600,3,1,2)==35);assert(loss.offset()==0);
  assert(loss.limit(750,30,1200,1,60,true,1)==30);
  // A gap after partial response evidence must abort the experiment instead
  // of joining observations from before and after the missing cycle.
  Controller gap;gap.preferHighCurrentFirst();
  for(uint32_t t=100;t<=400;t+=100)tick(gap,t,3,1);
  tick(gap,500,2.5f,1.2f);tick(gap,600,2.5f,1.2f);tick(gap,700,2.5f,1.2f);
  assert(gap.offset()==5);tick(gap,900,2.5f,1.2f);assert(gap.offset()==0);
  assert(gap.limit(1050,30,1200,1,60,true,1)==30);
  Controller changing;changing.preferHighCurrentFirst();
  tick(changing,100,3,1,5,1);tick(changing,200,3,1,5,1);
  tick(changing,300,3,1,5,1.06f);tick(changing,400,3,1,5,1.06f);
  assert(changing.offset()==5); // older different target kept only in trend

  std::cout<<"production foot feedback: four-cycle trend, delayed evaluation and immediate loss guard PASS\n";
}
