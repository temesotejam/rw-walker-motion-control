#include "../src/periodic_foot_state.h"
#include <cassert>
#include <iostream>

int main() {
  using periodic_foot::Estimator;
  Estimator check;
  check.start(100000);
  for (unsigned i=1;i<=3;++i)
    check.observe(i,100000+i*20000,125000+i*20000,true,true,true,float(i),true,true,0);
  assert(check.publish(190000));
  assert(check.packet.missed==2 && check.packet.left.sequence==3);
  assert(check.packet.left.estimated>3 && check.packet.left.estimated<=4.25f);
  check.observe(4,180000,201000,true,false,false,0,true,true,1);
  assert(check.publish(205000) && check.packet.left.sequence==3 && check.packet.right.sequence==4);
  assert(check.publish(240000) && !std::isfinite(check.packet.left.estimated));
  check.start(300000);
  assert(check.publish(325000) && !check.packet.left.sequence && !check.packet.missed);
  // Replay receipt and deadline events in chronological order.
  Estimator replay;
  uint64_t epoch;
  if (!(std::cin>>epoch)) return 0;
  replay.start(epoch);
  char event;
  uint64_t t;
  while (std::cin>>event>>t) {
    if (event=='P') {
      assert(replay.publish(t));
      const auto& p=replay.packet;
      std::cout<<p.count<<' '<<p.missed<<' '<<p.both_valid<<' '<<p.left.estimated<<' '<<p.right.estimated<<'\n';
    } else {
      uint32_t seq; uint64_t image;
      bool ok,lv,la,rv,ra; float l,r;
      std::cin>>seq>>image>>ok>>lv>>la>>l>>rv>>ra>>r;
      replay.observe(seq,image,t,ok,lv,la,l,rv,ra,r);
    }
  }
}
