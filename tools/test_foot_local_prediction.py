"""Causal local-motion evidence checks; no claim of calibrated angle bounds."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include "foot_local_prediction.h"
#include <cstdlib>
#include <iostream>
#include <cmath>
#define CHECK(test) do {if(!(test)){std::cerr<<"failed: "<<#test<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
using namespace foot_local_prediction;
int main(){
  Predictor p;CHECK(sizeof(Predictor)<128u);CHECK(sizeof(Projection)<128u);
  CHECK(p.observe(1,100,163,true,-.56573f,5.75716f)==Reason::Warmup);
  CHECK(p.observe(1,200,263,true,-.47464f,5.36924f)==Reason::Warmup);
  CHECK(p.observe(1,300,363,true,-.58475f,3.95754f)==Reason::Ready);
  const auto frozen=p.projection();const auto returning=frozen.predict(363);
  CHECK(returning.ready() && returning.method==Method::Local && returning.confidence==1.f);
  CHECK(returning.phase_left==Phase::Stationary && returning.phase_right==Phase::Returning);
  CHECK(returning.right<3.95754f && returning.right>3.8f && returning.right_rate<0.f);
  CHECK(returning.left_uncertainty<.75f && returning.right_uncertainty<.75f);
  CHECK(returning.age_ms==63u && returning.horizon_ms==63u);
  CHECK(frozen.predict(362).reason==Reason::Timestamp);
  CHECK(frozen.predict(461).reason==Reason::Stale);
  CHECK(frozen.predict(363,30).reason==Reason::Horizon);
  CHECK(frozen.predict(420).right_uncertainty>returning.right_uncertainty);
  p.observe(1,400,463,true,-.65778f,3.66896f);
  CHECK(frozen.predict(363).right==returning.right); // future insertion cannot alter a copy

  // A first negative interval after increasing motion is a turning zone,
  // not permission to extend that newest slope into an unseen current pose.
  p.reset();p.observe(1,100,163,true,0,3.03763f);
  p.observe(1,200,263,true,0,5.75716f);p.observe(1,300,363,true,0,5.36924f);
  const auto turning=p.predict(363);
  CHECK(turning.ready() && turning.phase_right==Phase::Turning);
  CHECK(turning.right==5.36924f && turning.right_uncertainty>=1.25f);

  // Increasing motion may already have reached an unobserved peak. Keep the
  // observed angle and expose a larger age-dependent margin instead of v*t.
  p.reset();p.observe(1,100,163,true,0,-.08771f);
  p.observe(1,200,263,true,0,3.03763f);p.observe(1,300,363,true,0,5.75716f);
  const auto forward=p.predict(363);
  CHECK(forward.ready() && forward.phase_right==Phase::Forward && forward.right_rate>0.f);
  CHECK(forward.right==5.75716f && forward.right_uncertainty>.75f);

  // Small jitter is stationary; larger inconsistent movement is uncertain.
  p.reset();p.observe(1,100,163,true,1,2);
  p.observe(1,200,263,true,1.1f,2.1f);p.observe(1,300,363,true,1.02f,1.98f);
  CHECK(p.predict(363).phase_left==Phase::Stationary);
  CHECK(p.predict(363).left_uncertainty<.75f);

  CHECK(p.observe(1,400,463,false,0,0)==Reason::Invalid);
  CHECK(p.observe(1,500,563,true,1,2)==Reason::Warmup);
  CHECK(p.observe(1,600,663,true,1,2)==Reason::Warmup);
  CHECK(p.observe(1,700,763,true,1,2)==Reason::Ready);
  CHECK(p.observe(1,700,763,true,1,2)==Reason::Timestamp);
  CHECK(!p.predict(800).ready());
  CHECK(p.observe(1,800,799,true,1,2)==Reason::Timestamp);
  CHECK(p.observe(1,900,1061,true,1,2)==Reason::Stale);
  CHECK(p.observe(2,100,163,true,1,2)==Reason::Warmup); // new run, new clock
  CHECK(p.observe(2,400,463,true,1,2)==Reason::Gap);
  CHECK(p.observe(2,500,563,true,1,2)==Reason::Warmup);
  CHECK(p.observe(2,600,663,true,1,2)==Reason::Ready);
  CHECK(p.observe(2,700,763,true,20,2)==Reason::Jump);
  CHECK(p.observe(2,800,863,true,NAN,2)==Reason::Invalid);
  CHECK(p.observe(0,900,963,true,1,2)==Reason::Invalid);
  std::cout<<"local delayed evidence, reversal restraint, frozen projection and invalidation PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix="foot_local_prediction_") as temporary:
    temporary = Path(temporary)
    cpp, exe = temporary / "test.cpp", temporary / "test.exe"
    cpp.write_text(SOURCE, encoding="utf8")
    subprocess.run([
        "g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I" + str(ROOT / "src"), str(cpp), "-o", str(exe)
    ], check=True)
    subprocess.run([str(exe)], check=True)
