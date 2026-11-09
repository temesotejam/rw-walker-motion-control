#include "early_motion_gate.h"
#include <cassert>
#include <limits>
int main() {
 using namespace early_motion_gate;
 Gate g;
 assert(!g.offer(1,900,1000,1020,std::numeric_limits<float>::quiet_NaN(),1030));
 assert(!g.offer(1,900,1000,1040,.5,1030));
 assert(!g.offer(1,900,1000,1020,1.1,1030));
 assert(g.offer(1,900,1000,1020,1.f,1030));
 assert(!g.fire(1,900,1030,1.1,-10));
 assert(!g.fire(1,900,1030,.5,10));
 assert(!g.fire(1,900,1030,.1,-20)); // lead estimate 5ms
 assert(!g.fire(1,900,1030,.4f,-20.f)); // exact 20ms bound, conservative
 assert(!g.fire(1,900,1030,std::nextafter(.4f,0.f),-20.f));
 assert(g.fire(1,900,1030,.401f,-20.f));
 assert(g.fire(1,900,1030,.5,-10));
 assert(!g.fire(1,901,1030,.5,-10)); // peak changed
 assert(g.offer(1,900,1000,1020,-.5,1030));
 assert(g.fire(1,900,1030,-.5,10)); // reflected side
 assert(!g.fire(2,900,1030,-.5,10)); // run changed
 assert(g.offer(1,900,1000,1020,-.5,1030));
 assert(!g.fire(1,900,1121,-.5,10)); // frame expired
 assert(g.offer(1,900,1000,1020,.5,1030));
 assert(!g.fire(1,900,1030,-.1,-10)); // already crossed ZERO
 assert(!g.valid);
}
