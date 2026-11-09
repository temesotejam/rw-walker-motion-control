#include "low_peak_energy.h"
#include <cassert>
#include <cmath>
#include <iostream>
using low_peak_energy::Feedback;
static void cycle(Feedback& f,unsigned t,uint16_t width=80,float left=20,float right=2,float neg=4) {
  f.negative(t,neg,true);f.command(t+200,1,true,400,width,5,26,10,7900);
  f.positive(t+400,.48f,.38f,0.f,.40f,5.f,true,true,left,right);
}
int main(){
 Feedback f;for(unsigned i=0;i<15;i++)cycle(f,1000+i*800);
 assert(f.active() && f.bias()>.09f && f.bias()<=.12f);assert(f.eligibleForCorrection(12700,80,.4f,true,20,2));
 assert(!f.eligibleForCorrection(12700,80,.4f,false,20,2));
 assert(!f.eligibleForCorrection(12700,80,.45f,true,20,2));
 auto before=f.bias();f.command(12900,1,false,0,0,5,26,10,7900);
 assert(!f.eligibleForCorrection(13000,80,.4f,true,20,2) && f.bias()<before);
 assert(!f.recover(13000,80));
 f.reset();for(unsigned i=0;i<4;i++)cycle(f,1000+i*800);
 assert(f.bias()>0);for(unsigned i=0;i<10;i++)cycle(f,5000+i*800,75,20,2);
 assert(!f.active()); // no foot improvement eventually releases trial
 assert(!f.eligibleForCorrection(12500,80,.4f,true,20,2));
 assert(f.recover(12500,80));
 f.reset();for(unsigned i=0;i<4;i++)cycle(f,1000+i*800);
 cycle(f,4500,75);
 for(unsigned i=0;i<2;i++)f.negative(5000+i*800,2.9f,true);
 assert(!f.active());
 f.reset();for(unsigned i=0;i<4;i++)cycle(f,1000+i*800);
 f.negative(5000,4.f,true);f.command(5200,1,true,400,80,5,26,10,7900);
 f.positive(5400,.57f,.38f,0.f,.40f,5.f,true,true,20,2); // inconsistent residual invalidates
 assert(f.bias()==0 && !f.eligibleForCorrection(5500,80,.4f,true,20,2));
 f.reset();
 for(unsigned i=0;i<15;i++) {
   const unsigned t=1000+i*800;f.negative(t,4.f,true);f.command(t+200,1,true,400,80,5,26,10,7900);
   // Prediction includes an existing bridge offset and our .04mJ offset.
   f.positive(t+400,.50f,.45f,.04f,.40f,5.f,true,true,20,2);
 }
 assert(f.bias()>.089f && f.bias()<.091f); // .50-(.45-.04), no bridge double count
 f.reset();for(unsigned i=0;i<10;i++)cycle(f,1000+i*800,80,2,20);
 assert(f.bias()==0 && !f.active());
 lateral_energy::Geometry g;wheel_work_energy_control::State state{26,10,7900,true,4};state.absolute_reference=true;state.target_deg=8;
 lateral_energy::CurrentTable table;assert(table.prepare(.06f));
 auto c=low_peak_energy::choose(g,state,table,350,0,.38f,0,.12f,80,80);
 assert(c.valid && c.current_ma==400 && c.width_ms>=75 && c.width_ms<=80);
 assert(!low_peak_energy::choose(g,state,table,350,0,.38f,0,.12f,60,80).valid);
 std::cout<<"Low-peak matched bias: slew, width window, zero/safety, weak opposite side and unchanged-foot rollback PASS\n";
}
