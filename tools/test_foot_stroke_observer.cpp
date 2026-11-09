#include "foot_stroke_observer.h"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace foot_stroke_observer;
static bool near(float a,float b) { return std::fabs(a-b)<1e-5f; }

int main() {
  // Initial center -> forward (2), then back (-3) -> forward (2) = 5.
  // Reversal is confirmed on a later sample, never at the extremum itself.
  Observer a(+1,.5f,500);
  assert(a.observe(0,0,true).kind==Kind::None);
  assert(a.observe(10,1,true).kind==Kind::None);
  assert(a.observe(20,2,true).kind==Kind::None);
  auto half=a.observe(30,1.4f,true);
  assert(half.kind==Kind::InitialToForward && near(half.width,2));
  assert(half.from_ms==0 && half.to_ms==20 && half.confirmed_ms==30);
  a.observe(40,-3,true); a.observe(50,-2.4f,true); a.observe(60,2,true);
  auto full=a.observe(70,1.4f,true);
  assert(full.kind==Kind::BackToForward && near(full.width,5));
  assert(full.from_ms==40 && full.to_ms==60 && full.confirmed_ms==70);

  // Opposite camera direction, with an arbitrary offset, is equivalent.
  Observer b(-1,.5f,500);
  b.observe(0,100,true); b.observe(10,101,true); b.observe(20,103,true);
  b.observe(30,102.4f,true); b.observe(40,98,true);
  auto opposite=b.observe(50,98.6f,true);
  assert(opposite.kind==Kind::BackToForward && near(opposite.width,5));
  assert(opposite.from_ms==20 && opposite.to_ms==40);

  // Undefined sign produces directionless full spans, but NEVER calls the
  // first center-to-extremum movement a full span or a forward movement.
  Observer ul(0,.5f,500),ur(0,.5f,500);
  ul.observe(0,100,true);ur.observe(0,-10,true);
  ul.observe(10,102,true);ur.observe(10,-12,true);
  assert(ul.observe(20,101.4f,true).kind==Kind::None);
  assert(ur.observe(20,-11.4f,true).kind==Kind::None);
  ul.observe(30,98,true); ur.observe(30,-8,true);
  auto uleft=ul.observe(40,98.6f,true);
  auto uright=ur.observe(40,-8.6f,true);
  assert(uleft.kind==Kind::DirectionlessSpan && uright.kind==Kind::DirectionlessSpan);
  assert(near(uleft.width,4) && near(uright.width,4));
  assert(uleft.from_ms==10 && uleft.to_ms==30 && uleft.confirmed_ms==40);
  assert(uleft.direction==Direction::Decreasing && uright.direction==Direction::Increasing);
  Pairer unsigned_pair; unsigned_pair.left(uleft); unsigned_pair.right(uright);
  auto symmetric=unsigned_pair.take(40,100,20);
  assert(symmetric.valid && near(symmetric.difference,0));

  // Unequal full spans are compared in the common input coordinate unit.
  Observer unequal(0,.5f,500);
  unequal.observe(0,0,true); unequal.observe(10,3,true); unequal.observe(20,2.4f,true);
  unequal.observe(30,-3,true); auto six=unequal.observe(40,-2.4f,true);
  unsigned_pair.left(six); unsigned_pair.right(uright);
  auto different=unsigned_pair.take(40,100,20);
  assert(different.valid && near(different.difference,2));
  unsigned_pair.left(full); unsigned_pair.right(uright);
  assert(!unsigned_pair.take(70,100,100).valid); // mixed event semantics

  // A slowly drifting camera is intentionally NOT removed: successive raw
  // spans differ by direction. Calling them ground-relative strides would be
  // an unjustified interpretation.
  Observer drift(0,.5f,500);
  drift.observe(0,0,true);drift.observe(10,2.2f,true);drift.observe(20,1.6f,true);
  drift.observe(30,-1.6f,true); auto down=drift.observe(40,-1.f,true);
  drift.observe(50,2.6f,true); auto up=drift.observe(60,2.f,true);
  assert(down.kind==Kind::DirectionlessSpan && near(down.width,3.8f));
  assert(up.kind==Kind::DirectionlessSpan && near(up.width,4.2f));
  assert(down.direction==Direction::Decreasing && up.direction==Direction::Increasing);

  // Missing observations invalidate the old extremum. After re-acquisition
  // the first partial movement is discarded until two new extrema exist.
  Observer missing(0,.5f,100);
  missing.observe(0,0,true);missing.observe(10,2,true);missing.observe(20,1.4f,true);
  missing.observe(25,NAN,false);
  missing.observe(30,-2,true);missing.observe(40,2,true);
  assert(missing.observe(50,1.4f,true).kind==Kind::None);
  missing.observe(60,-2,true);
  assert(missing.observe(70,-1.4f,true).kind==Kind::DirectionlessSpan);

  // Out-of-domain sign values remain invalid rather than guessed.
  Observer unknown(2,.5f,500);
  assert(unknown.observe(0,0,true).kind==Kind::None);
  assert(unknown.status()==Status::InvalidConfiguration);
  Observer bad_threshold(1,0.f,500);
  assert(bad_threshold.observe(0,0,true).kind==Kind::None);
  assert(bad_threshold.status()==Status::InvalidConfiguration);

  // Invalid, reordered, or missing samples start a new tracking epoch and
  // cannot complete a stroke using an old foot extremum.
  Observer c(1,.5f,100);
  c.observe(0,0,true); c.observe(10,-3,true); c.observe(20,-2,true);
  c.observe(30,NAN,false); assert(c.status()==Status::InvalidSample);
  c.observe(40,2,true); assert(c.observe(50,1,true).kind==Kind::None);
  c.observe(60,0,true); c.observe(400,10,true); assert(c.status()==Status::GapReset);
  assert(c.observe(390,11,true).kind==Kind::None);
  assert(c.status()==Status::NonMonotonicTime);

  // Only new FULL forward strokes can be paired; the initial half and an
  // out-of-time pair cannot masquerade as equal, valid gait steps.
  Pairer pair;
  pair.left(half); pair.right(opposite);
  assert(!pair.take(70,100,100).valid);
  pair.left(full);
  auto cmp=pair.take(70,100,30);
  assert(cmp.valid && near(cmp.difference,0));
  assert(!pair.take(70,100,30).valid); // consumed once
  pair.left(full); pair.right(opposite);
  assert(!pair.take(70,100,5).valid); // peak skew 20ms
  pair.invalidateLeft();
  assert(!pair.take(70,100,100).valid);
  pair.left(full);
  assert(!pair.take(1000,100,100).valid); // stale
  auto late=opposite; late.to_ms=1; late.confirmed_ms=1000;
  auto current=full; current.to_ms=990; current.confirmed_ms=1000;
  pair.reset(); pair.left(current); pair.right(late);
  assert(!pair.take(1000,100,2000).valid); // fresh confirmation, old peak

  std::cout << "foot stroke observer tests passed\n";
}
