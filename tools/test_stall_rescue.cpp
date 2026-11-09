#include "stall_rescue.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using stall_rescue::Detector;
int main(){
  Detector d;
  // Measured near-stall episode from run 307, held inward and confirmed.
  assert(!d.observe(17168,17163,16796,8.91235f,3.14f,3.11f,-7.10f,-5.69f));
  assert(!d.observe(17173,17168,16796,8.91235f,3.11f,3.08f,-5.69f,-5.51f));
  assert(d.observe(17178,17173,16796,8.91235f,3.08f,3.03f,-5.51f,-5.45f));
  // Run 46277170 returned to +0.54 degrees and reversed without a center
  // crossing. These are actual logged, downsampled angle/rate observations;
  // triggering a candidate here does not predict the effect of a pulse.
  d.reset();
  assert(!d.observe(19564,19558,19182,7.90378f,.77f,.74f,-7.22f,-5.63f));
  assert(!d.observe(19569,19564,19182,7.90378f,.74f,.70f,-5.63f,-5.75f));
  assert(d.observe(19574,19569,19182,7.90378f,.70f,.67f,-5.75f,-4.35f));
  // Do not use rescue below the lower bound or on/after center crossing.
  d.reset();
  assert(!d.observe(19593,19586,19182,7.9f,.40f,.34f,-6.1f,-5.4f));
  assert(!d.observe(19598,19593,19182,7.9f,.34f,.00f,-2.4f,-1.7f));
  assert(!d.observe(19604,19598,19182,7.9f,.00f,-.05f,-1.7f,-.7f));
  // One-sample noise, true reversal, stale data, and motion away from centre
  // never cause a rescue. A fresh trigger transition must be observed again.
  assert(!d.observe(17207,17204,16796,8.91f,2.93f,2.91f,-6.4f,-5.8f));
  assert(!d.observe(17210,17207,16796,8.91f,2.91f,2.90f,-2.8f,+.2f));
  assert(!d.observe(17213,17210,16796,8.91f,2.90f,2.89f,-2.5f,-2.0f));
  assert(!d.observe(17250,17210,16796,8.91f,2.90f,2.89f,-3.2f,-2.0f));
  assert(!d.observe(17260,17257,16796,8.91f,2.80f,2.90f,-3.2f,-2.0f));
  assert(!d.observe(17270,17267,16796,8.91f,2.93f,2.91f,-3.2f,NAN));
  assert(!d.observe(16900,16897,16796,8.91f,3.1f,3.0f,-3.2f,-2.0f));
  assert(stall_rescue::kMaxPulseMs==15);
  std::puts("inward stall rescue: confirmed low speed, stale/reversal rejection PASS");
}
