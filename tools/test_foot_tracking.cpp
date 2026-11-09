#include <cassert>
#include <cmath>
#include <vector>
#include <iostream>
#include <cstring>
#include "host_v46o/Arduino.h"
#include "../src/foot_zero.h"
#include "../src/foot_calibration_table.h"
#include "../src/white_marker_tracker.h"
#include "../src/foot_angle_estimator.h"
#include "../src/marker_identity_policy.h"
using Image = std::vector<uint8_t>;
static Image blank(uint8_t level=30){return Image(320*240,level);}
static void band(Image& image,int x,int y,int half_width=10,uint8_t level=230){
  for(int row=y-10;row<=y+10;++row)for(int col=x-half_width;col<=x+half_width;++col)
    if(row>=0&&row<240&&col>=0&&col<320)image[row*320+col]=level;
}
static WhiteMarkerObservation observe(WhiteMarker1DTracker& tracker,const Image& image){
  host_us+=100000;return tracker.process(image.data());
}
static void verticalRecovery(){
  for(int dy=-32;dy<=32;++dy)for(int x:{35,80,160,210,260}){
    WhiteMarker1DTracker a(0),b(1);
    auto image=blank();band(image,x,66+dy);band(image,320-x,160-dy);
    const auto ma=a.process(image.data()),mb=b.process(image.data());
    assert(ma.valid&&mb.valid);
    assert(std::abs(ma.center_x_px-x)<0.01&&std::abs(mb.center_x_px-(320-x))<0.01);
    assert(ma.candidate_count==1&&mb.candidate_count==1);
    assert(ma.templates_tested==17&&mb.templates_tested==17);
    assert(std::abs(ma.center_y_px-66)<=32&&std::abs(mb.center_y_px-160)<=32);
    auto upper=blank();band(upper,x,66+dy);WhiteMarker1DTracker isolatedB(1);
    assert(!isolatedB.process(upper.data()).valid);
    auto lower=blank();band(lower,x,160+dy);WhiteMarker1DTracker isolatedA(0);
    assert(!isolatedA.process(lower.data()).valid);
  }
  for(int dy:{-24,24}){
    auto image=blank();
    for(int row=66+dy-10;row<=66+dy+10;++row){
      const int x=160+(row-66-dy)/2;
      for(int col=x-10;col<=x+10;++col)image[row*320+col]=230;
    }
    WhiteMarker1DTracker a(0);const auto ma=a.process(image.data());
    assert(ma.valid&&std::abs(ma.center_x_px-160)<3);
  }
  for(int level:{0,30,230,255}){
    auto image=blank(level);WhiteMarker1DTracker a(0);const auto empty=a.process(image.data());
    assert(!empty.valid&&empty.reason==MarkerDetectionReason::LowContrast);
  }
  auto image=blank();band(image,160,90,10,50);WhiteMarker1DTracker dim(0);
  assert(!dim.process(image.data()).valid);
  image=blank();band(image,160,66,70);WhiteMarker1DTracker wide(0);
  assert(wide.process(image.data()).reason==MarkerDetectionReason::BadShape);
  image=blank();band(image,0,66,1);WhiteMarker1DTracker edge(0);
  assert(!edge.process(image.data()).valid);
}
static void wrongZeroReproduction(){
  WhiteMarker1DTracker a(0),b(1);FootZero zero;
  auto upright=blank();band(upright,172,66);band(upright,175,184);
  band(upright,102,160,10,140); // nominal-row reflection, not the left white marker
  for(uint32_t t=0;t<=2200;t+=100){
    host_us+=100000;const auto ma=a.process(upright.data()),mb=b.process(upright.data());
    assert(ma.valid&&mb.valid&&std::abs(mb.center_x_px-175)<.01);
    assert(mb.candidate_count==2&&std::abs(mb.alternate_x_px-102)<.01);
    zero.observe(t,1,true,ma.valid&&mb.valid,ma.center_x_px,mb.center_x_px);
  }
  assert(zero.ready&&std::abs(zero.a_zero-172)<.01&&std::abs(zero.b_zero-175)<.01);
  WhiteMarkerObservation ma,mb;
  for(int step=1;step<=20;++step){
    const float f=step/20.0f;auto image=blank();
    band(image,std::lround(172-137*f),std::lround(66+20*f));
    band(image,std::lround(175-140*f),std::lround(184-4*f));
    host_us+=100000;ma=a.process(image.data());mb=b.process(image.data());
    assert(ma.valid&&mb.valid);
    zero.observe(2300+step*100,2,false,true,ma.center_x_px,mb.center_x_px);
  }
  const auto right=estimateFootAngle(ma,zero.a_zero,true),left=estimateFootAngle(mb,zero.b_zero,true);
  assert(right.valid&&left.valid);
  // This synthetic image moves A by 137 px and B by 140 px. Verify each
  // independent displacement, not an artificial left/right angle constraint.
  assert(right.angle_deg>0&&left.angle_deg>0);
  std::cout<<"wrong-feature zero reproduction: right="<<right.angle_deg<<" left="<<left.angle_deg<<" deg\n";
}
static void identityAndAmbiguity(){
  auto image=blank();band(image,100,66);band(image,175,90);
  WhiteMarker1DTracker ambiguous(0);const auto amb=observe(ambiguous,image);
  assert(!amb.valid&&amb.reason==MarkerDetectionReason::Ambiguous&&amb.candidate_count==2);
  // Nearby separate peaks are never blended into a centroid in the dark gap.
  image=blank();band(image,145,66,4);band(image,172,66,4,140);
  WhiteMarker1DTracker islands(0);const auto island=observe(islands,image);
  assert(island.valid&&std::abs(island.center_x_px-145)<.01);
  // A stronger distant reflection cannot take over an established track.
  WhiteMarker1DTracker tracked(0);image=blank();band(image,60,66);
  assert(observe(tracked,image).valid);
  image=blank();band(image,65,70);band(image,230,66,10,255);
  const auto kept=observe(tracked,image);assert(kept.valid&&std::abs(kept.center_x_px-65)<.01);
  image=blank();band(image,230,66,10,255);const auto jump=observe(tracked,image);
  assert(!jump.valid&&jump.reason==MarkerDetectionReason::TrackJump);
  host_us+=600000;
  const auto first=observe(tracked,image),second=observe(tracked,image),third=observe(tracked,image);
  assert(!first.valid&&!second.valid&&third.valid);
  assert(first.reason==MarkerDetectionReason::Reacquiring);
  const auto missing=tracked.process(nullptr);
  assert(!missing.valid&&missing.reason==MarkerDetectionReason::NoFrame&&missing.center_x_px==0);
  // The two feet remain independent after zero; different actual angles must
  // not be forced to agree by a left/right constraint.
  WhiteMarkerObservation a,b;a.valid=b.valid=true;a.id=0;b.id=1;a.center_x_px=160;b.center_x_px=120;
  assert(estimateFootAngle(a,170,true).angle_deg>0);
  assert(estimateFootAngle(b,170,true).angle_deg>estimateFootAngle(a,170,true).angle_deg);
  assert(!estimateFootAngle(a,170,false).valid);
  a.center_x_px=10;const auto outside=estimateFootAngle(a,170,true);
  assert(outside.valid&&!outside.in_calibration_range);
}
static void zeroGates(){
  FootZero z;
  for(uint32_t t=0;t<1800;t+=100)z.observe(t,1,true,true,169,174);
  assert(!z.ready);
  z.observe(1800,2,true,true,171,176);assert(z.count==1);
  for(uint32_t t=1900;t<=3800;t+=100)z.observe(t,2,true,true,171,176);
  assert(z.ready&&z.a_zero==171&&z.b_zero==176);
  z.observe(5000,3,false,false,0,0);assert(z.ready&&z.a_zero==171);
  FootZero wrong;
  for(uint32_t t=0;t<10000;t+=100)wrong.observe(t,1,true,true,172,101.6366);
  assert(!wrong.ready&&wrong.reason==FootZeroReason::PositionMismatch);
  FootZero drift;
  for(uint32_t t=0;t<10000;t+=100)drift.observe(t,1,true,true,172,165+((t/100)%20));
  assert(!drift.ready); // stationary body with a moving foot is not a zero
  FootZero missed;
  for(uint32_t t=0;t<1500;t+=100)missed.observe(t,1,true,true,169,174);
  missed.observe(1900,1,true,true,169,174);assert(missed.count==1);
  missed.observe(2000,1,true,false,169,174);assert(missed.count==0);
  FootZero slow;for(uint32_t t=0;t<10000;t+=500)slow.observe(t,1,true,true,169,174);assert(!slow.ready);
}
static void weakDisplacedConfirmation(){
  // Replay the *reported candidate metrics*, not unavailable source pixels.
  // 0.47.15 run 55522052: selected Y 82 -> 50, contrast 215.2 -> 55.2,
  // weight 5633.06689 -> 341.79999. This warrants confirmation, not a claim
  // that the camera image proves which physical feature was seen.
  assert(marker_identity::weakDisplaced(169.29527f,82,215.2f,5633.06689f,204.94246f,50,55.2f,341.79999f));
  assert(!marker_identity::weakDisplaced(169,82,215.2f,5633.06689f,205,82,55.2f,341.79999f));
  assert(!marker_identity::weakDisplaced(169,82,215.2f,5633.06689f,169,50,55.2f,341.79999f));
  assert(!marker_identity::weakDisplaced(169,82,215.2f,5633.06689f,205,50,200,5000));
  WhiteMarker1DTracker tracker(0);
  auto strong=blank();band(strong,169,82,10,245);
  assert(observe(tracker,strong).valid);
  auto weak=blank();band(weak,205,50,4,90);
  const auto first=observe(tracker,weak);
  assert(!first.valid&&first.reason==MarkerDetectionReason::Reacquiring);
  // A one-frame weak island cannot move the accepted track/zero.
  auto returned=blank();band(returned,172,82,10,190);
  assert(observe(tracker,returned).valid);
  // A persistent dim displaced marker remains recoverable, with existing
  // three-frame confirmation; no permanent contrast or support exclusion.
  assert(!observe(tracker,weak).valid);
  assert(!observe(tracker,weak).valid);
  const auto confirmed=observe(tracker,weak);
  assert(confirmed.valid&&std::abs(confirmed.center_x_px-205)<.01);
  assert(estimateFootAngle(confirmed,171,true).valid);
  assert(!estimateFootAngle(confirmed,171,true).in_calibration_range);
  WhiteMarker1DTracker moving(0);assert(observe(moving,strong).valid);
  auto displaced=blank();band(displaced,205,50,10,245);
  assert(observe(moving,displaced).valid);
  WhiteMarker1DTracker dimming(0);assert(observe(dimming,strong).valid);
  auto same_location=blank();band(same_location,169,82,4,90);
  assert(observe(dimming,same_location).valid);
  std::cout<<"Weak displaced candidate confirmation, immediate true-track return, strong motion and dim tracking PASS\n";
}
static void fixedPoseCalibrationReplay(){
  // IMU-reference intermediate poses and averaged repeat endpoint.
  const float rows[][3]={{203.0819f,201.6569f,0},
    {182.4695f,173.8169f,3.688465f},{152.5749f,144.3562f,8.371281f},
    {111.0505f,102.703f,14.109958f},{67.5192f,63.6918f,20.626501f}};
  for(int side=0;side<2;++side){
    WhiteMarkerObservation marker;marker.valid=true;marker.id=side;
    const float zero=rows[0][side];
    for(const auto& r:rows){marker.center_x_px=r[side];const auto e=estimateFootAngle(marker,zero,true);
      assert(e.valid&&e.in_calibration_range&&std::abs(e.angle_deg-r[2])<.00002f);}
    for(int i=0;i<4;++i){marker.center_x_px=(rows[i][side]+rows[i+1][side])/2;
      assert(std::abs(estimateFootAngle(marker,zero,true).angle_deg-(rows[i][2]+rows[i+1][2])/2)<.00002f);}
    // No clamp or rejection just outside either endpoint. A moved per-boot zero
    // is evaluated through the same mapping and remains exactly zero there.
    marker.center_x_px=zero+5;auto e=estimateFootAngle(marker,zero,true);
    assert(e.valid&&!e.in_calibration_range&&e.angle_deg<0);
    marker.center_x_px=rows[4][side]-5;e=estimateFootAngle(marker,zero,true);
    assert(e.valid&&!e.in_calibration_range&&e.angle_deg>rows[4][2]);
    marker.center_x_px=zero+1;assert(std::abs(estimateFootAngle(marker,zero+1,true).angle_deg)<.00001f);
    assert(!estimateFootAngle(marker,zero,false).valid);
    marker.center_x_px=NAN;assert(!estimateFootAngle(marker,zero,true).valid);
    marker.center_x_px=zero;assert(!estimateFootAngle(marker,NAN,true).valid);
    marker.valid=false;assert(!estimateFootAngle(marker,zero,true).valid);
  }
  // The repeat pair is treated as the same fixed-leg camera-tilt protocol.
  // Measure change relative to its own upright capture, not another boot's zero.
  const float upright[]={203.0717f,201.2023f};
  const float tilted[]={67.2186f,64.1011f};
  float delta[2];
  for(int side=0;side<2;++side){
    const auto* points=side==0?foot_calibration::right:foot_calibration::left;
    delta[side]=foot_calibration::angleAt(points,tilted[side])-foot_calibration::angleAt(points,upright[side]);
    assert(std::abs(delta[side]-20.574717f)<.13f);
  }
  assert(std::abs(delta[0]-delta[1])<.20f);
  std::cout<<"Five-pose interpolation, endpoint extrapolation, zero and invalid observations PASS\n";
}
int main(){verticalRecovery();wrongZeroReproduction();identityAndAmbiguity();zeroGates();weakDisplacedConfirmation();fixedPoseCalibrationReplay();}
