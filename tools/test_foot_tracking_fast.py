"""Compare v20 with the frozen pre-v20 tracker on identical pixels/timestamps.

Synthetic differential/state tests establish software equivalence only. Native
host timing is not an ESP32/PSRAM latency measurement or physical gait result.
"""
from pathlib import Path
import argparse
import json
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

# Frozen pre-v20 state/evaluation oracle; do not update from current source.
LEGACY_HEADER = r'''
class LegacyWhiteMarker1DTracker {
public:
    explicit LegacyWhiteMarker1DTracker(int marker_id);

    WhiteMarkerObservation process(const uint8_t* gray);

private:
    struct Candidate {
        float x = 0, y = 0, contrast = 0, weight = 0;
        int peak = 0, width = 0, min_y = 0, max_y = 0;
    };
    const int* markerRows() const;
    const int* referenceRows() const;
    int nominalCenterY() const;
    void evaluate(const uint8_t* gray, int y_offset, Candidate* candidates,
                  int& count, bool& overflow, WhiteMarkerObservation& weak) const;

    int _id;
    uint32_t _success_count = 0;
    uint32_t _fail_count = 0;
    bool _tracked = false;
    float _last_x = 0, _last_y = 0, _pending_x = 0, _pending_y = 0;
    float _last_contrast = 0, _last_weight = 0;
    uint32_t _last_us = 0, _pending_us = 0;
    uint8_t _pending_count = 0;
};
'''
LEGACY_SOURCE = r'''
#include "legacy_tracker.h"
#include <Arduino.h>
#include <math.h>
#include "foot_tracking_config.h"
#include "marker_identity_policy.h"

extern uint64_t legacy_pixel_reads;
static inline uint8_t legacyPixel(const uint8_t* gray,int index){
#if defined(FOOT_IMAGE_ROWS_TEST_READS) && FOOT_IMAGE_ROWS_TEST_READS
    ++legacy_pixel_reads;
#endif
    return gray[index];
}

namespace {
constexpr int kWidth = appcfg::kFrameWidth;
constexpr int kRadius = 2, kWindow = 5, kScale = 15;
static_assert(appcfg::kWhiteMarkerRowCount == 5 && appcfg::kWhiteReferenceRowCount == 3,
              "Integer contrast scale assumes five marker and three reference rows");
constexpr int kPeakMin = static_cast<int>(appcfg::kWhitePeakMinContrast * kScale);
constexpr int kBaseline = static_cast<int>(appcfg::kWhiteCentroidBaseline * kScale);
constexpr int kWeightMin = static_cast<int>(appcfg::kWhiteMinWeightSum * kScale);
}

const char* legacyMarkerDetectionReasonName(MarkerDetectionReason reason) {
    switch (reason) {
        case MarkerDetectionReason::Detected: return "detected";
        case MarkerDetectionReason::NoFrame: return "no_frame";
        case MarkerDetectionReason::LowContrast: return "low_contrast";
        case MarkerDetectionReason::LowWeight: return "low_weight";
        case MarkerDetectionReason::BadShape: return "bad_shape";
        case MarkerDetectionReason::Ambiguous: return "ambiguous";
        case MarkerDetectionReason::TrackJump: return "track_jump";
        case MarkerDetectionReason::Reacquiring: return "reacquiring";
    }
    return "unknown";
}
LegacyWhiteMarker1DTracker::LegacyWhiteMarker1DTracker(int marker_id) : _id(marker_id) {}
const int* LegacyWhiteMarker1DTracker::markerRows() const {
    return _id == appcfg::kMarkerAId ? appcfg::kWhiteMarkerARows : appcfg::kWhiteMarkerBRows;
}
const int* LegacyWhiteMarker1DTracker::referenceRows() const {
    return _id == appcfg::kMarkerAId ? appcfg::kWhiteReferenceARows : appcfg::kWhiteReferenceBRows;
}
int LegacyWhiteMarker1DTracker::nominalCenterY() const {
    return _id == appcfg::kMarkerAId ? appcfg::kWhiteMarkerACenterY : appcfg::kWhiteMarkerBCenterY;
}

WhiteMarkerObservation LegacyWhiteMarker1DTracker::process(const uint8_t* gray) {
    const uint32_t now = micros();
    WhiteMarkerObservation out;
    out.id = _id; out.center_y_px = nominalCenterY();
    if (gray) {
        Candidate candidates[appcfg::kWhiteMaxCandidates];
        int count = 0; bool overflow = false;
        out.reason = MarkerDetectionReason::LowContrast;
        // Always search all heights. A nominal-row distractor must not hide a
        // stronger displaced marker. Each distinct horizontal island is kept.
        for (int dy = -appcfg::kWhiteSearchRadiusYPx; dy <= appcfg::kWhiteSearchRadiusYPx;
             dy += appcfg::kWhiteSearchStepYPx) {
            evaluate(gray, dy, candidates, count, overflow, out);
            ++out.templates_tested;
        }
        out.candidate_count = count;
        if (count) {
            const bool recent = _tracked && static_cast<uint32_t>(now - _last_us) <= appcfg::kWhiteTrackMemoryUs;
            int best = -1, second = -1, global = 0;
            float best_score = -1, second_score = -1;
            for (int i = 0; i < count; ++i) {
                const auto& c = candidates[i];
                if (c.contrast > candidates[global].contrast ||
                    (c.contrast == candidates[global].contrast && c.weight > candidates[global].weight)) global = i;
                const float dx = fabsf(c.x - _last_x), dy = fabsf(c.y - _last_y);
                if (recent && (dx > appcfg::kWhiteMaxTrackStepXPx || dy > appcfg::kWhiteMaxTrackStepYPx)) continue;
                // Proximity is a modest preference; it cannot turn a weak
                // nearby reflection into a stronger current-image detection.
                const float distance = recent ? fmaxf(dx / appcfg::kWhiteMaxTrackStepXPx,
                                                       dy / appcfg::kWhiteMaxTrackStepYPx) : 0;
                const float score = c.contrast * (1.0f - 0.15f * distance);
                if (score > best_score) {
                    second = best; second_score = best_score; best = i; best_score = score;
                } else if (score > second_score) { second = i; second_score = score; }
            }
            const auto& chosen = candidates[best < 0 ? global : best];
            out.center_x_px = chosen.x; out.center_y_px = chosen.y;
            out.peak_contrast = chosen.contrast; out.weight_sum = chosen.weight;
            out.peak_x_px = chosen.peak; out.bright_width_px = chosen.width;
            if (second >= 0) {
                out.alternate_x_px = candidates[second].x; out.alternate_y_px = candidates[second].y;
                out.ambiguity_ratio = second_score / best_score;
            }
            out.reason = overflow ? MarkerDetectionReason::Ambiguous :
                best < 0 ? MarkerDetectionReason::TrackJump :
                out.ambiguity_ratio >= appcfg::kWhiteAmbiguousRatio ? MarkerDetectionReason::Ambiguous :
                MarkerDetectionReason::Detected;
            const bool weak_displaced = recent && marker_identity::weakDisplaced(
                _last_x, _last_y, _last_contrast, _last_weight,
                chosen.x, chosen.y, chosen.contrast, chosen.weight);
            if (out.reason == MarkerDetectionReason::Detected && _tracked && (!recent || weak_displaced)) {
                if (!_pending_count || static_cast<uint32_t>(now - _pending_us) > 300000 ||
                    fabsf(chosen.x - _pending_x) > 40 || fabsf(chosen.y - _pending_y) > 24) _pending_count = 0;
                _pending_x = chosen.x; _pending_y = chosen.y; _pending_us = now;
                if (++_pending_count < appcfg::kWhiteReacquireFrames) out.reason = MarkerDetectionReason::Reacquiring;
            } else { _pending_count = 0; }
            out.valid = out.reason == MarkerDetectionReason::Detected;
            if (out.valid) {
                _tracked = true; _last_x = chosen.x; _last_y = chosen.y; _last_us = now; _pending_count = 0;
                _last_contrast = chosen.contrast; _last_weight = chosen.weight;
            }
        } else { _pending_count = 0; }
    } else { _pending_count = 0; }
    if (out.valid) ++_success_count; else ++_fail_count;
    out.success_count = _success_count; out.fail_count = _fail_count;
    out.processing_us = micros() - now;
    return out;
}

void LegacyWhiteMarker1DTracker::evaluate(const uint8_t* gray, int dy, Candidate* candidates,
                                   int& count, bool& overflow, WhiteMarkerObservation& weak) const {
    int contrast[kWidth], smooth[kWidth] = {};
    const int* rows = markerRows(); const int* refs = referenceRows();
    for (int x = 0; x < kWidth; ++x) {
        int a = 0, b = 0;
        for (int i = 0; i < appcfg::kWhiteMarkerRowCount; ++i) a += legacyPixel(gray, (rows[i] + dy) * kWidth + x);
        for (int i = 0; i < appcfg::kWhiteReferenceRowCount; ++i) b += legacyPixel(gray, (refs[i] + dy) * kWidth + x);
        contrast[x] = 3 * a - 5 * b;
    }
    int rolling = 0;
    for (int x = 0; x < kWindow; ++x) rolling += contrast[x];
    for (int x = kRadius; x < kWidth - kRadius; ++x) {
        if (x > kRadius) rolling += contrast[x + kRadius] - contrast[x - kRadius - 1];
        smooth[x] = rolling / kWindow;
        if (smooth[x] > weak.peak_contrast * kScale) weak.peak_contrast = static_cast<float>(smooth[x]) / kScale;
    }
    for (int x = kRadius; x < kWidth - kRadius;) {
        if (smooth[x] <= kBaseline) { ++x; continue; }
        const int lo = x;
        int peak = x, peak_value = 0, weight = 0;
        int64_t weighted_x = 0;
        while (x < kWidth - kRadius && smooth[x] > kBaseline) {
            const int w = smooth[x] - kBaseline;
            weight += w; weighted_x += static_cast<int64_t>(x) * w;
            if (smooth[x] > peak_value) { peak_value = smooth[x]; peak = x; }
            ++x;
        }
        const int width = x - lo;
        if (peak_value < kPeakMin) continue;
        if (weight < kWeightMin) { weak.reason = MarkerDetectionReason::LowWeight; continue; }
        if (width < appcfg::kWhiteMinWidthPx || width > appcfg::kWhiteMaxWidthPx) {
            weak.reason = MarkerDetectionReason::BadShape; continue;
        }
        Candidate c;
        c.x = static_cast<float>(weighted_x) / weight; c.y = nominalCenterY() + dy;
        c.contrast = static_cast<float>(peak_value) / kScale; c.weight = static_cast<float>(weight) / kScale;
        c.peak = peak; c.width = width; c.min_y = c.max_y = static_cast<int>(c.y);
        // Adjacent height templates observing the same horizontal island are
        // one candidate, not competitors. Retain its strongest full-row view.
        int merge = -1;
        for (int i = 0; i < count; ++i) {
            if (fabsf(c.x - candidates[i].x) <= 8 && c.y >= candidates[i].min_y - 8 &&
                c.y <= candidates[i].max_y + 8) { merge = i; break; }
        }
        if (merge >= 0) {
            auto& old = candidates[merge];
            const int low_y = min(old.min_y, c.min_y), high_y = max(old.max_y, c.max_y);
            if (c.contrast > old.contrast || (c.contrast == old.contrast &&
                (c.weight > old.weight || (c.weight == old.weight &&
                 fabsf(c.y - nominalCenterY()) < fabsf(old.y - nominalCenterY()))))) old = c;
            old.min_y = low_y; old.max_y = high_y;
        } else if (count < appcfg::kWhiteMaxCandidates) { candidates[count++] = c; }
        else { overflow = true; }
    }
}
'''

CPP = r'''
#include "legacy_tracker.h"
#include "white_marker_tracker.h"
#include <Arduino.h>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <vector>
#undef assert
#define assert(test) do {if(!(test)){std::cerr<<"assertion failed: "<<#test<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
using Image=std::vector<uint8_t>;
namespace rows=foot_image_rows;
uint64_t legacy_pixel_reads=0;
static unsigned comparisons=0,reason_mask=0;
static uint32_t rng=0x51a8eb93u;
static uint32_t random32(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static Image blank(uint8_t value=30){return Image(rows::kFrameBytes,value);}
static void band(Image& image,int x,int y,int half_width=10,int level=230,int half_height=10){
 for(int yy=y-half_height;yy<=y+half_height;++yy)for(int xx=x-half_width;xx<=x+half_width;++xx)
  if(yy>=0 && yy<appcfg::kFrameHeight && xx>=0 && xx<appcfg::kFrameWidth)image[yy*appcfg::kFrameWidth+xx]=static_cast<uint8_t>(level);
}
static void equal(const WhiteMarkerObservation& old,const WhiteMarkerObservation& fast){
#define CMP(field) do {if(std::memcmp(&old.field,&fast.field,sizeof(old.field))!=0){std::cerr<<"field mismatch "<<#field<<" comparison "<<comparisons<<"\n";std::exit(2);}}while(false)
 CMP(valid);CMP(reason);CMP(templates_tested);CMP(candidate_count);CMP(ambiguity_ratio);
 CMP(alternate_x_px);CMP(alternate_y_px);CMP(id);CMP(center_x_px);CMP(center_y_px);
 CMP(peak_contrast);CMP(weight_sum);CMP(peak_x_px);CMP(bright_width_px);CMP(processing_us);
 CMP(success_count);CMP(fail_count);
#undef CMP
 ++comparisons;reason_mask|=1u<<static_cast<unsigned>(fast.reason);
}
struct Sequence {
 LegacyWhiteMarker1DTracker old_a{0},old_b{1};
 WhiteMarker1DTracker full_a{0},full_b{1},pointer_a{0},pointer_b{1},packed_a{0},packed_b{1};
 std::vector<uint8_t> storage=std::vector<uint8_t>(rows::kRequiredBytes);
 rows::RowCache cache{storage.data(),storage.size()};
 void step(const Image* image,uint32_t stamp){
  host_us=stamp;const auto a=old_a.process(image?image->data():nullptr),b=old_b.process(image?image->data():nullptr);
  host_us=stamp;equal(a,pointer_a.process(image?image->data():nullptr));equal(b,pointer_b.process(image?image->data():nullptr));
  const auto view=rows::GrayRowView::full(image?image->data():nullptr);
  host_us=stamp;equal(a,full_a.process(view));equal(b,full_b.process(view));
  assert(cache.capture(image?image->data():nullptr)==bool(image));
  host_us=stamp;equal(a,packed_a.process(cache.view()));equal(b,packed_b.process(cache.view()));
 }
};
static void cacheContract(){
 static_assert(rows::kRequiredRows==66 && rows::kRequiredBytes==21120,"Current complete sparse search");
 bool expected[appcfg::kFrameHeight]={};
 for(int dy=-appcfg::kWhiteSearchRadiusYPx;dy<=appcfg::kWhiteSearchRadiusYPx;dy+=appcfg::kWhiteSearchStepYPx){
  for(const auto& y:appcfg::kWhiteMarkerARows)expected[y+dy]=true;
  for(const auto& y:appcfg::kWhiteReferenceARows)expected[y+dy]=true;
  for(const auto& y:appcfg::kWhiteMarkerBRows)expected[y+dy]=true;
  for(const auto& y:appcfg::kWhiteReferenceBRows)expected[y+dy]=true;
 }
 auto image=blank();for(auto& p:image)p=static_cast<uint8_t>(random32());
 std::vector<uint8_t> protected_buffer(rows::kRequiredBytes+128,0xa5);
 rows::RowCache cache(protected_buffer.data()+64,rows::kRequiredBytes);
 assert(!cache.view().valid());assert(cache.capture(image.data(),image.size()));
 const auto packed=cache.view(),full=rows::GrayRowView::full(image.data(),image.size());
 unsigned count=0;
 for(int y=0;y<appcfg::kFrameHeight;++y){
  assert(rows::neededRow(y)==expected[y]);assert(full.row(y)==image.data()+y*appcfg::kFrameWidth);
  if(expected[y]){++count;assert(packed.row(y));assert(std::memcmp(packed.row(y),full.row(y),appcfg::kFrameWidth)==0);}
  else assert(!packed.row(y));
 }
 assert(count==66 && !packed.row(-1) && !packed.row(appcfg::kFrameHeight));
 for(unsigned i=0;i<64;++i)assert(protected_buffer[i]==0xa5 && protected_buffer[protected_buffer.size()-1-i]==0xa5);
 // The original frame may be returned/reused before recognition begins.
 const auto frozen=image;std::fill(image.begin(),image.end(),0);
 LegacyWhiteMarker1DTracker old(0);WhiteMarker1DTracker fast(0);
 equal(old.process(frozen.data()),fast.process(cache.view()));
 LegacyWhiteMarker1DTracker old_b(1);WhiteMarker1DTracker fast_b(1);
 equal(old_b.process(frozen.data()),fast_b.process(cache.view()));
 assert(!cache.capture(image.data(),rows::kFrameBytes-1));assert(!cache.view().valid());
 assert(fast.process(cache.view()).reason==MarkerDetectionReason::NoFrame);
 assert(cache.capture(image.data()));cache.invalidate();assert(!cache.view().valid());
 assert(!cache.capture(nullptr));assert(!cache.view().valid());
 rows::RowCache short_cache(protected_buffer.data()+64,rows::kRequiredBytes-1);
 assert(!short_cache.capture(image.data()) && !short_cache.view().valid());
 rows::RowCache missing_storage(nullptr,rows::kRequiredBytes);
 assert(!missing_storage.capture(image.data()) && !missing_storage.view().valid());
 assert(!rows::GrayRowView::full(nullptr).valid());assert(!rows::GrayRowView::full(image.data(),rows::kFrameBytes-1).valid());
}
static void deterministicCases(){
 for(int dy=-32;dy<=32;++dy)for(int x:{0,2,35,160,285,319}){
  auto image=blank();band(image,x,66+dy);band(image,319-x,160-dy);Sequence s;s.step(&image,1000000);
 }
 for(int value:{0,30,35,54,55,56,230,255}){auto image=blank(value);Sequence s;s.step(&image,1000);}
 for(int level:{54,55,56,90,100,255})for(int width:{0,1,2,3,10,35,36,70}){
  auto image=blank(0);band(image,150,66,width,level);band(image,170,160,width,level);Sequence s;s.step(&image,1000);
 }
 Sequence s;auto image=blank();s.step(nullptr,1000);s.step(&image,101000);
 band(image,60,66);band(image,180,160);s.step(&image,201000);
 image=blank();band(image,250,66);band(image,35,160);s.step(&image,301000); // TrackJump
 s.step(&image,901001);s.step(&image,1001001);s.step(&image,1101001); // Reacquire three frames
 image=blank();band(image,100,66);band(image,175,90);s.step(&image,1701002); // Ambiguous
 image=blank();band(image,100,66,70);s.step(&image,2301003); // BadShape
 image=blank();band(image,100,66,1,100);s.step(&image,2901004); // LowWeight
 s.step(nullptr,2901004);s.step(nullptr,0xfffffff0u);s.step(&image,0x20000u);
 Sequence weak;image=blank();band(image,169,82,10,245);weak.step(&image,1000000);
 image=blank();band(image,205,50,4,90);weak.step(&image,1100000);weak.step(&image,1200000);weak.step(&image,1300000);
}
static void fuzzCases(){
 Sequence series;uint32_t stamp=0xff000000u;
 for(unsigned i=0;i<1600;++i){
  auto image=blank(static_cast<uint8_t>(random32()));
  if(i%5==0)for(auto& p:image)p=static_cast<uint8_t>(random32());
  else {
   const unsigned islands=random32()%36;
   for(unsigned j=0;j<islands;++j)band(image,static_cast<int>(random32()%380)-30,static_cast<int>(random32()%280)-20,
       random32()%85,random32()%256,random32()%28);
   if(i%3==0){band(image,70+(i%150),66+static_cast<int>(i%65)-32);band(image,220-(i%140),160+static_cast<int>(i%65)-32);}
  }
  stamp+=i%37==0?600001u:i%17==0?0u:16667u;
  series.step(i%29==0?nullptr:&image,stamp);
  if(i%11==0){Sequence fresh;fresh.step(&image,stamp);}
 }
 assert(reason_mask==255u); // Every original detection result was exercised.
}
static void readCounts(){
#if defined(FOOT_IMAGE_ROWS_TEST_READS) && FOOT_IMAGE_ROWS_TEST_READS
 auto image=blank();band(image,160,66);band(image,170,160);
 LegacyWhiteMarker1DTracker old_a(0),old_b(1);legacy_pixel_reads=0;
 old_a.process(image.data());old_b.process(image.data());
 assert(legacy_pixel_reads==87040);
 WhiteMarker1DTracker a(0),b(1);rows::readCounts()=rows::ReadCounts{};
 a.process(image.data());b.process(image.data());
 assert(rows::readCounts().full_rows==272 && rows::readCounts().missing_rows==0);
 std::vector<uint8_t> storage(rows::kRequiredBytes);rows::RowCache cache(storage.data(),storage.size());
 rows::readCounts()=rows::ReadCounts{};assert(cache.capture(image.data()));
 a.process(cache.view());b.process(cache.view());
 assert(rows::readCounts().capture_rows==66 && rows::readCounts().packed_rows==272);
 assert(rows::readCounts().full_rows==0 && rows::readCounts().missing_rows==0);
 std::cout<<"READS {\"legacy_source_bytes\":"<<legacy_pixel_reads<<",\"cache_source_bytes\":"<<rows::readCounts().capture_rows*320
    <<",\"cache_internal_pixel_bytes\":"<<rows::readCounts().packed_rows*320<<",\"required_rows\":66}\n";
#endif
}
static volatile uint32_t bench_digest=0;
static double bench(int mode,const std::vector<Image>& images,unsigned frames){
 LegacyWhiteMarker1DTracker old_a(0),old_b(1);WhiteMarker1DTracker a(0),b(1);
 std::vector<uint8_t> storage(rows::kRequiredBytes);rows::RowCache cache(storage.data(),storage.size());
 const auto start=std::chrono::steady_clock::now();
 for(unsigned i=0;i<frames;++i){
  host_us=1000000+i*16667;const auto& image=images[i%images.size()];WhiteMarkerObservation left,right;
  if(mode==0){left=old_a.process(image.data());right=old_b.process(image.data());}
  else if(mode==1){left=a.process(image.data());right=b.process(image.data());}
  else{assert(cache.capture(image.data()));const auto view=cache.view();left=a.process(view);right=b.process(view);}
  bench_digest+=left.success_count+right.fail_count+left.peak_x_px+right.peak_x_px;
 }
 return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/frames;
}
static void benchmarks(unsigned frames){
 std::vector<Image> images;
 for(unsigned i=0;i<32;++i){auto image=blank(25);band(image,70+i*5,50+i%33,10,180+i%60);band(image,220-i*4,144+i%33,10,220);images.push_back(image);}
 double medians[3];for(int mode=0;mode<3;++mode){double times[3];for(auto& t:times)t=bench(mode,images,frames);std::sort(times,times+3);medians[mode]=times[1];}
 std::cout<<"BENCH {\"frames_per_pass\":"<<frames<<",\"legacy_ms_per_pair\":"<<medians[0]<<",\"fast_full_ms_per_pair\":"<<medians[1]
    <<",\"capture_and_fast_packed_ms_per_pair\":"<<medians[2]<<",\"host_only\":true}\n";
}
int main(int argc,char** argv){
 if(argc>1 && std::string(argv[1])=="bench"){benchmarks(argc>2?static_cast<unsigned>(std::strtoul(argv[2],nullptr,10)):500);return 0;}
 cacheContract();deterministicCases();fuzzCases();readCounts();
 std::cout<<"TRACKER bit-exact observations/state counters/cache lifetime/17 heights/bounds/missing/all8reasons PASS comparisons="<<comparisons<<"\n";
}
'''


def compile_probe(temp, counted, fast=True):
    source = temp / 'probe.cpp'
    source.write_text(CPP, encoding='utf8')
    (temp / 'legacy_tracker.h').write_text('#pragma once\n#include "white_marker_tracker.h"\n' + LEGACY_HEADER, encoding='utf8')
    legacy = temp / 'legacy.cpp'
    legacy.write_text(LEGACY_SOURCE, encoding='utf8')
    exe = temp / ('counted.exe' if counted and fast else 'fallback.exe' if counted else 'benchmark.exe')
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DFOOT_LOW_LATENCY_V20=' + str(int(fast)), '-DFOOT_IMAGE_ROWS_TEST_READS=' + str(int(counted)),
                    '-I' + str(ROOT / 'tools/host_v46o'), '-I' + str(ROOT / 'src'), '-I' + str(temp),
                    str(source), str(legacy), str(ROOT / 'src/white_marker_tracker.cpp'), '-o', str(exe)], check=True)
    return exe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bench-frames', type=int, default=500)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    assert args.bench_frames > 0
    report = {'semantics': 'native_synthetic_software_equivalence_and_host_timing_not_ESP_latency',
              'compiler': 'g++ -std=c++17 -O2 -Wall -Wextra -Werror'}
    with tempfile.TemporaryDirectory(prefix='foot_tracking_fast_') as folder:
        temp = Path(folder)
        counted = compile_probe(temp, True)
        output = subprocess.check_output([str(counted)], text=True)
        print(output.strip())
        report['reads'] = json.loads(next(line[6:] for line in output.splitlines() if line.startswith('READS ')))
        fallback = compile_probe(temp, True, fast=False)
        print('Flag-off pointer/view compatibility:', subprocess.check_output([str(fallback)], text=True).strip())
        ordinary = compile_probe(temp, False)
        output = subprocess.check_output([str(ordinary), 'bench', str(args.bench_frames)], text=True)
        print(output.strip())
        report['host_benchmark'] = json.loads(next(line[6:] for line in output.splitlines() if line.startswith('BENCH ')))
        compat = temp / 'compat.cpp'
        compat.write_text('#include "foot_image_rows.h"\nstatic_assert(foot_image_rows::kRequiredRows==66,"rows");\nint main(){return 0;}\n', encoding='utf8')
        subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-I' + str(ROOT / 'src'),
                        str(compat), '-o', str(temp / 'compat.exe')], check=True)
        print('Row-view/cache API C++11 compatibility PASS')
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')


if __name__ == '__main__':
    main()
