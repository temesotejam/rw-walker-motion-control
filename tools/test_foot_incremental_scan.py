"""Compare incremental image preparation with the actual complete-frame tracker.

Uses sparse views that expose only admitted logical rows. Features, candidate
merge order and final-time temporal decisions must be bit-identical. This is a
host correctness test, not proof of sensor timing or a camera frame's identity.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

CPP = r'''
#include <Arduino.h>
#include <array>
#include <cassert>
#include <iostream>
#include <random>
#include "white_marker_tracker.h"

using Tracker=WhiteMarker1DTracker;
using Features=Tracker::PreparedFeatures;
using Scan=Tracker::PreparedScan;
using View=foot_image_rows::GrayRowView;
// Test-only explicit-instantiation access leaves production visibility and
// class definitions unchanged in both translation units.
template<class Tag,auto Member> struct Access {
 friend auto member(Tag){return Member;}
};
#define ACCESS(tag,type,field) \
 struct tag {friend auto member(tag);}; \
 template struct Access<tag,&type::field>
ACCESS(ViewIndices,View,indices_);
ACCESS(Candidates,Features,candidates_);
ACCESS(Observation,Features,observation_);
ACCESS(Lane,Features,marker_id_);
ACCESS(Count,Features,count_);
ACCESS(ImageValid,Features,image_valid_);
ACCESS(Overflow,Features,overflow_);
ACCESS(Admitted,Features,admitted_);
#undef ACCESS

unsigned feature_pairs=0,observations=0,append_calls=0,rejections=0;
uint32_t bits(float value){uint32_t out;memcpy(&out,&value,sizeof(out));return out;}
void equalObservation(const WhiteMarkerObservation& a,const WhiteMarkerObservation& b){
 ++observations;
 assert(a.valid==b.valid && a.reason==b.reason && a.id==b.id);
 assert(a.templates_tested==b.templates_tested && a.candidate_count==b.candidate_count);
 assert(bits(a.ambiguity_ratio)==bits(b.ambiguity_ratio));
 assert(bits(a.alternate_x_px)==bits(b.alternate_x_px) && bits(a.alternate_y_px)==bits(b.alternate_y_px));
 assert(bits(a.center_x_px)==bits(b.center_x_px) && bits(a.center_y_px)==bits(b.center_y_px));
 assert(bits(a.peak_contrast)==bits(b.peak_contrast) && bits(a.weight_sum)==bits(b.weight_sum));
 assert(a.peak_x_px==b.peak_x_px && a.bright_width_px==b.bright_width_px);
 assert(a.success_count==b.success_count && a.fail_count==b.fail_count);
 assert(a.processing_us==b.processing_us);
}
void equalFeatures(const Features& a,const Features& b){
 ++feature_pairs;
 assert(a.*member(Admitted{}) && b.*member(Admitted{}));
 assert(a.*member(Lane{})==b.*member(Lane{}));
 assert(a.*member(Count{})==b.*member(Count{}));
 assert(a.*member(ImageValid{})==b.*member(ImageValid{}));
 assert(a.*member(Overflow{})==b.*member(Overflow{}));
 assert(a.processing_us==b.processing_us);
 equalObservation(a.*member(Observation{}),b.*member(Observation{}));
 const auto& aa=a.*member(Candidates{});const auto& bb=b.*member(Candidates{});
 for(int i=0;i<appcfg::kWhiteMaxCandidates;++i){
  assert(bits(aa[i].x)==bits(bb[i].x) && bits(aa[i].y)==bits(bb[i].y));
  assert(bits(aa[i].contrast)==bits(bb[i].contrast) && bits(aa[i].weight)==bits(bb[i].weight));
  assert(aa[i].peak==bb[i].peak && aa[i].width==bb[i].width);
  assert(aa[i].min_y==bb[i].min_y && aa[i].max_y==bb[i].max_y);
 }
}
using Snapshot=std::array<unsigned char,sizeof(Tracker)>;
Snapshot snapshot(const Tracker& tracker){Snapshot bytes;memcpy(bytes.data(),&tracker,bytes.size());return bytes;}
void unchanged(const Tracker& tracker,const Snapshot& before){assert(snapshot(tracker)==before);}
void rejected(Tracker& tracker,const Features& features){
 const auto before=snapshot(tracker);const auto result=tracker.processPrepared(features);
 assert(!result.valid && result.reason==MarkerDetectionReason::NoFrame);
 unchanged(tracker,before);++rejections;
}
View prefix(const std::vector<uint8_t>& image,int maximum,std::array<uint8_t,240>& indices){
 for(int y=0;y<240;++y)indices[y]=y<=maximum?uint8_t(y):255;
 auto view=View::full(image.data());view.*member(ViewIndices{})=indices.data();return view;
}
void band(std::vector<uint8_t>& image,int cx,int cy,int half,int brightness=235){
 for(int y=cy-10;y<=cy+10;++y)for(int x=cx-half;x<=cx+half;++x)
  if(x>=0 && x<320 && y>=0 && y<240)image[y*320+x]=uint8_t(brightness);
}
std::vector<uint8_t> fixture(std::mt19937& rng,unsigned iteration){
 std::vector<uint8_t> image(320*240);
 for(auto& pixel:image)pixel=iteration%11==0?uint8_t(rng()):uint8_t(10+rng()%30);
 if(iteration%13)band(image,5+int(rng()%310),34+int(rng()%65),2+int(rng()%38));
 if(iteration%17)band(image,5+int(rng()%310),128+int(rng()%65),2+int(rng()%38));
 if(iteration%5==0){band(image,75,66,8);band(image,245,66,8);}
 if(iteration%7==0){band(image,100,160,4,115);band(image,250,160,20);}
 if(iteration%19==0){ // Many islands at separated template heights exercise merge/overflow order.
  std::fill(image.begin(),image.end(),uint8_t(10));
  for(int y:{34,62,90,128,156,184})for(int x=10;x<320;x+=30)band(image,x,y,3);
 }
 if(iteration%23==0)std::fill(image.begin(),image.end(),uint8_t(75));
 return image;
}
std::vector<int> schedule(unsigned kind,int lane,std::mt19937& rng){
 std::vector<int> values;
 if(kind==0)return {239};
 if(kind==1){for(int y=-1;y<240;++y)values.push_back(y);}
 if(kind==2){for(int y=(lane?190:74)-32;y<=(lane?190:74)+32;y+=4){values.push_back(y-1);values.push_back(y);}values.push_back(239);}
 if(kind==3){for(int y=-1;y<239;){values.push_back(y);y+=1+int(rng()%29);}values.push_back(239);}
 if(kind==4){for(int y=-1;y<239;y+=7){values.push_back(y);values.push_back(y);values.push_back(-1);}values.push_back(239);}
 if(kind==5){for(int y=5;y<239;y+=4)values.push_back(y);values.push_back(239);} // Raw 4-row chunks with logical origin +2.
 if(kind==6)return {-1,0,40,70,100,130,160,190,222,239};
 return values;
}
void equivalence(){
 std::mt19937 rng(260117);
 for(unsigned iteration=0;iteration<320;++iteration){
  auto image=fixture(rng,iteration);
  for(int lane:{0,1}){
   Tracker tracker(lane);const auto before=snapshot(tracker);
   const auto expected=tracker.prepare(View::full(image.data()));unchanged(tracker,before);
   for(unsigned kind=0;kind<7;++kind){
    auto scan=tracker.beginScan();assert(scan.valid() && !scan.complete() && !scan.templatesTested());
    std::array<uint8_t,240> indices;int highest=-1;
    for(int maximum:schedule(kind,lane,rng)){
     auto view=prefix(image,maximum,indices);
     foot_image_rows::readCounts()={};
     assert(tracker.appendAvailable(scan,view,maximum));++append_calls;
     highest=std::max(highest,maximum);
     const int first=(lane?190:74)-32;
     const int templates=highest<first?0:std::min(17,(highest-first)/4+1);
     assert(scan.templatesTested()==templates && scan.complete()==(templates==17));
     assert(foot_image_rows::readCounts().missing_rows==0);
     unchanged(tracker,before);
     if(!scan.complete())rejected(tracker,tracker.finishScan(scan));
    }
    assert(scan.complete() && scan.processingUs()==0);
    const auto prepared=tracker.finishScan(scan);equalFeatures(expected,prepared);
    Tracker direct(lane),incremental(lane);
    host_us+=12345;equalObservation(direct.process(image.data()),incremental.processPrepared(prepared));
   }
  }
 }
}
void errorsAndAbandonment(){
 std::vector<uint8_t> image(320*240,15);band(image,190,66,10);band(image,200,160,10);
 const auto view=View::full(image.data());
 for(int lane:{0,1}){
  Tracker tracker(lane);host_us+=100000;assert(tracker.process(view).valid);
  const auto before=snapshot(tracker);
  Scan default_scan;assert(!default_scan.valid());rejected(tracker,tracker.finishScan(default_scan));
  assert(!tracker.appendAvailable(default_scan,view,239));rejected(tracker,tracker.finishScan(default_scan));
  for(int invalid_max:{-2,240,1000}){
   auto scan=tracker.beginScan();assert(!tracker.appendAvailable(scan,view,invalid_max));
   assert(!scan.valid() && !scan.complete());rejected(tracker,tracker.finishScan(scan));
   assert(!tracker.appendAvailable(scan,view,239)); // Poison persists until explicit reset.
  }
  auto null_scan=tracker.beginScan();assert(!tracker.appendAvailable(null_scan,View{},239));
  rejected(tracker,tracker.finishScan(null_scan));
  Tracker other(1-lane);auto wrong=other.beginScan();
  assert(!tracker.appendAvailable(wrong,view,239));assert(!wrong.valid());
  rejected(tracker,tracker.finishScan(wrong));
  auto other_complete=other.beginScan();assert(other.appendAvailable(other_complete,view,239));
  assert(other_complete.complete());rejected(tracker,tracker.finishScan(other_complete));
  rejected(tracker,other.finishScan(other_complete)); // Completed other-lane features also reject.
  for(int y=0;y<240;++y){
   // Every row needed by this lane must be checked before evaluation.
   const int* markers=lane?appcfg::kWhiteMarkerBRows:appcfg::kWhiteMarkerARows;
   const int* references=lane?appcfg::kWhiteReferenceBRows:appcfg::kWhiteReferenceARows;
   const bool needed=foot_image_rows::requestedBy(y,markers,5)||foot_image_rows::requestedBy(y,references,3);
   if(!needed)continue;
   std::array<uint8_t,240> indices;auto missing=prefix(image,239,indices);indices[y]=255;
   auto scan=tracker.beginScan();assert(!tracker.appendAvailable(scan,missing,239));
   assert(!scan.valid() && !scan.complete());rejected(tracker,tracker.finishScan(scan));
  }
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
  { // A leaving row is no longer in the next template, but rolling contrast still reads it.
   std::array<uint8_t,240> indices;auto partial=prefix(image,239,indices);
   auto scan=tracker.beginScan();const int first=(lane?190:74)-32;
   assert(tracker.appendAvailable(scan,partial,first) && scan.templatesTested()==1);
   indices[(lane?152:58)-32]=255;
   assert(!tracker.appendAvailable(scan,partial,first+4));rejected(tracker,tracker.finishScan(scan));
  }
#endif
  unchanged(tracker,before);
  for(int maximum:{0,41,42,70,105,157,158,190,221}){
   auto scan=tracker.beginScan();assert(tracker.appendAvailable(scan,view,maximum));
   // Abandon this frame. No feature or temporal history may leak into its replacement.
   scan=tracker.beginScan();assert(scan.templatesTested()==0 && !scan.complete());
   auto replacement=image;band(replacement,40,lane?160:66,5);
   assert(tracker.appendAvailable(scan,View::full(replacement.data()),239));
   equalFeatures(tracker.prepare(View::full(replacement.data())),tracker.finishScan(scan));
  }
  unchanged(tracker,before);
  auto destroyed=image;
  auto scan=tracker.beginScan();assert(tracker.appendAvailable(scan,View::full(destroyed.data()),239));
  auto owned=tracker.finishScan(scan),expected=tracker.prepare(View::full(destroyed.data()));
  scan=tracker.beginScan();std::fill(destroyed.begin(),destroyed.end(),0);
  equalFeatures(expected,owned);
  // Existing prepare(null) must still count a failed image, unlike rejected scans.
  const auto null=tracker.processPrepared(tracker.prepare(View{}));
  assert(!null.valid && null.fail_count==1 && null.success_count==1);
 }
 Tracker invalid(-1);auto scan=invalid.beginScan();assert(!scan.valid());
 rejected(invalid,invalid.finishScan(scan));rejected(invalid,Features{});
 assert(!invalid.appendAvailable(scan,view,239));
}
void temporalFinalTime(){
 std::mt19937 rng(26981);
 for(int lane:{0,1}){
  Tracker direct(lane),incremental(lane);
  uint32_t time=0xffff0000u;
  for(unsigned iteration=0;iteration<450;++iteration){
   auto image=fixture(rng,iteration+1);const auto view=View::full(image.data());
   const auto before=snapshot(incremental);auto scan=incremental.beginScan();
   host_us=time;
   assert(incremental.appendAvailable(scan,view,(lane?190:74)-32+24));
   rejected(incremental,incremental.finishScan(scan));unchanged(incremental,before);
   time+=iteration%7==0?500001u:iteration%9==0?300001u:33333u;
   host_us=time;
   assert(incremental.appendAvailable(scan,view,239));
   const auto prepared=incremental.finishScan(scan);unchanged(incremental,before);
   // The temporal commit is deliberately later than prepare, across expiry and timer wrap.
   equalObservation(direct.process(view),incremental.processPrepared(prepared));
  }
  for(uint32_t delay:{499999u,500000u,500001u,800001u}){
   Tracker oracle(lane),staged(lane);std::vector<uint8_t> image(320*240,10);band(image,160,lane?160:66,10);
   host_us=0xffffff00u;assert(oracle.process(image.data()).valid);assert(staged.process(image.data()).valid);
   auto scan=staged.beginScan();assert(staged.appendAvailable(scan,View::full(image.data()),239));
   host_us+=delay;
   const auto expected=oracle.process(image.data());
   equalObservation(expected,staged.processPrepared(staged.finishScan(scan)));
   assert(expected.reason==(delay<=500000?MarkerDetectionReason::Detected:MarkerDetectionReason::Reacquiring));
  }
 }
}
int main(){
 static_assert(appcfg::kWhiteMaxTemplates==17 && appcfg::kWhiteMaxCandidates==24);
 equivalence();errorsAndAbandonment();temporalFinalTime();
 std::cout<<"incremental scan PASS: "<<feature_pairs<<" bit-exact feature pairs, "<<observations
          <<" observation pairs, "<<append_calls<<" staged appends, "<<rejections<<" rejected commits; "
          <<"PreparedScan="<<sizeof(Scan)<<" bytes, PreparedFeatures="<<sizeof(Features)<<" bytes\n";
}
'''

TIMING_CPP = r'''
#include <Arduino.h>
#include <cassert>
#include <iostream>
#include "white_marker_tracker.h"
int main(){
 std::vector<uint8_t> image(320*240,25);
 const auto view=foot_image_rows::GrayRowView::full(image.data());
 WhiteMarker1DTracker tracker(1);
 host_us=0xfffffffcu; // The first measured call crosses uint32_t wrap.
 auto scan=tracker.beginScan();
 assert(tracker.appendAvailable(scan,view,157));
 assert(scan.processingUs()==7 && scan.templatesTested()==0);
 host_us+=250000;
 assert(tracker.appendAvailable(scan,view,158));
 assert(scan.processingUs()==14 && scan.templatesTested()==1);
 host_us+=900000;
 assert(tracker.appendAvailable(scan,view,239));
 assert(scan.complete() && scan.processingUs()==21);
 const auto prepared=tracker.finishScan(scan);
 assert(prepared.processing_us==21); // Includes work calls; excludes waits between callbacks.
 assert(tracker.prepare(view).processing_us==7);
 const auto result=tracker.processPrepared(prepared);
 assert(result.processing_us==7 && result.fail_count==1);
 scan=tracker.beginScan();assert(scan.processingUs()==0);
 assert(!tracker.appendAvailable(scan,{},239));assert(scan.processingUs()==7);
 const auto rejected=tracker.processPrepared(tracker.finishScan(scan));
 assert(rejected.fail_count==1 && rejected.success_count==0);
 std::cout<<"incremental timing: accumulated work excludes callback gaps and survives clock wrap PASS\n";
}
'''


def main():
    compiler = shutil.which('g++')
    if not compiler:
        raise SystemExit('g++ is required for the native tracker test')
    with tempfile.TemporaryDirectory(prefix='foot_incremental_scan_') as temporary:
        path = Path(temporary)
        source = path/'test.cpp'
        source.write_text(CPP, encoding='utf8')
        for rolling in (0, 1):
            binary = path/('rolling_%d.exe' % rolling)
            subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                '-DFOOT_LOW_LATENCY_V20=1', '-DFOOT_SPEED_V22=%d' % rolling,
                '-DFOOT_CAPTURE_V23=1', '-DFOOT_ROW_OVERLAP_V24=1',
                '-DFOOT_NATIVE_SKIP_V25=1', '-DFOOT_NATIVE_STREAM_V26=1',
                '-DFOOT_IMAGE_ROWS_TEST_READS=1',
                '-I'+str(ROOT/'tools/host_v46o'), '-I'+str(ROOT/'src'), str(source),
                str(ROOT/'src/white_marker_tracker.cpp'), '-o', str(binary)], check=True)
            print('Rolling contrast:', rolling, flush=True)
            subprocess.run([str(binary)], check=True)
        # Use the same host shim with a deterministic advancing clock to verify
        # timing aggregation independently of the bit-exact pixel comparisons.
        shim = path/'shim'
        shim.mkdir()
        arduino = (ROOT/'tools/host_v46o/Arduino.h').read_text(encoding='utf8')
        old = 'inline uint32_t micros() { return host_us; }'
        assert old in arduino
        (shim/'Arduino.h').write_text(arduino.replace(old,
            'inline uint32_t micros() { uint32_t now=host_us; host_us+=7; return now; }'), encoding='utf8')
        source.write_text(TIMING_CPP, encoding='utf8')
        binary = path/'timing.exe'
        subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
            '-DFOOT_LOW_LATENCY_V20=1', '-DFOOT_SPEED_V22=1', '-DFOOT_ROW_OVERLAP_V24=1',
            '-DFOOT_NATIVE_STREAM_V26=1', '-I'+str(shim), '-I'+str(ROOT/'src'), str(source),
            str(ROOT/'src/white_marker_tracker.cpp'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        (shim/'Arduino.h').write_text('#pragma once\n#include <algorithm>\n#include <stdint.h>\n'
            'using std::min; using std::max; uint32_t micros();\n', encoding='utf8')
        subprocess.run([compiler, '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
            '-DFOOT_LOW_LATENCY_V20=1', '-DFOOT_SPEED_V22=1', '-DFOOT_ROW_OVERLAP_V24=1',
            '-DFOOT_NATIVE_STREAM_V26=1', '-I'+str(shim), '-I'+str(ROOT/'src'), '-c',
            str(ROOT/'src/white_marker_tracker.cpp'), '-o', str(path/'tracker_cxx11.o')], check=True)
        print('Incremental tracker C++11 compatibility PASS', flush=True)


if __name__ == '__main__':
    main()
