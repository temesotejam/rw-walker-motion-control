"""Verify progressive lower-row ownership using the production mailbox code.

Native and legacy cache geometry are exercised separately. State-transition
interleavings test coherent images and non-reentrant consumers, not device
timing, sensor image quality, or cross-core scheduling performance.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

CPP = r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <functional>
#include <iostream>
#include <vector>
#include "camera_row_overlap.h"
#include "foot_lower_rows.h"

using namespace foot_image_rows;
template<class Tag,typename Tag::Type Member> struct TestMemberAccess {
    friend typename Tag::Type member(Tag) {return Member;}
};
struct ViewIndices {
    using Type=const uint8_t* GrayRowView::*;
    friend Type member(ViewIndices);
};
template struct TestMemberAccess<ViewIndices,&GrayRowView::indices_>;
static uint64_t host_time=1000;
static std::function<void()> timer_hook;
extern "C" int64_t esp_timer_get_time() {
    if(timer_hook) {auto once=timer_hook;timer_hook={};once();}
    return ++host_time;
}
static uint8_t pixel(int y,int x,unsigned seed) {
    return static_cast<uint8_t>((13*y+7*x+seed)%251);
}
static std::vector<uint8_t> image(unsigned seed,bool logical=false) {
    const int height=logical?appcfg::kFrameHeight:kRawFrameHeight;
    std::vector<uint8_t> data(size_t(height)*appcfg::kFrameWidth);
    for(int y=0;y<height;++y)for(int x=0;x<appcfg::kFrameWidth;++x)
        data[size_t(y)*appcfg::kFrameWidth+x]=pixel(logical?y:y-kRawYOrigin,x,seed);
    return data;
}
static int lastRequired(int maximum) {
    for(int y=maximum;y>=0;--y)if(lowerNeededRow(y))return y;
    return -1;
}
static void checkLower(const GrayRowView& view,unsigned seed,int maximum) {
    assert(view.valid()==(lastRequired(maximum)>=0));
    assert(!view.row(-1) && !view.row(appcfg::kFrameHeight));
    unsigned count=0;
    for(int y=0;y<appcfg::kFrameHeight;++y) {
        const auto* row=view.row(y);
        assert(bool(row)==(lowerNeededRow(y) && y<=maximum));
        if(!row)continue;
        ++count;
        for(int x=0;x<appcfg::kFrameWidth;++x)assert(row[x]==pixel(y,x,seed));
    }
    if(maximum>=222)assert(count==40);
}
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
static void checkUpper(const GrayRowView& view,unsigned seed) {
    assert(view.valid());
    for(int y=0;y<appcfg::kFrameHeight;++y) {
        const auto* row=view.row(y);
        assert(bool(row)==upperNeededRow(y));
        if(row)for(int x=0;x<appcfg::kFrameWidth;++x)assert(row[x]==pixel(y,x,seed));
    }
}
#endif
static void cacheCases() {
    static_assert(kLowerRows==40 && kLowerBytes==12800,"fixed lower allocation");
    static_assert(lowerPlanValid(),"C++11 constant evaluation must build the row plan");
    unsigned plan_index=0;
    for(int y=0;y<appcfg::kFrameHeight;++y)if(lowerNeededRow(y)) {
        assert(plan_index<kLowerRows && kLowerLogicalPlan.logical[plan_index]==y);
        ++plan_index;
    }
    assert(plan_index==kLowerRows);
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    unsigned upper_index=0;
    for(int y=0;y<appcfg::kFrameHeight;++y) {
        const unsigned expected=upperNeededRow(y)?upper_index++:255;
        assert(kUpperIndexPlan.indices[y]==expected);
    }
    assert(upper_index==kUpperRows);
#endif
    std::vector<uint8_t> storage(kLowerBytes+2,0xa5);
    LowerRowCache cache(storage.data()+1,kLowerBytes);
    auto raw=image(11);
    assert(!cache.view().valid() && cache.maxLogicalY()==-1);
    // Every physical prefix boundary, including a partial last row. A partial
    // row must never become visible merely because some of its pixels arrived.
    for(size_t complete=0;complete<=size_t(kRawFrameHeight);++complete) {
        const size_t prefix=complete*kRawFrameWidth;
        for(size_t extra: {size_t(0),size_t(1),size_t(kRawFrameWidth-1)}) {
            if(prefix+extra>kRawFrameBytes)continue;
            const int maximum=static_cast<int>(complete)-1-kRawYOrigin;
            const int expected=lastRequired(std::min(maximum,appcfg::kFrameHeight-1));
            assert(cache.capture(raw.data(),prefix+extra)==(expected>=0));
            assert(cache.maxLogicalY()==expected);
            checkLower(cache.view(),11,expected);
            assert(storage.front()==0xa5 && storage.back()==0xa5);
        }
    }
    int previous=-1;
    for(int milestone:kLowerMilestones) {
        const size_t prefix=lowerPrefixThrough(milestone);
        assert(prefix==size_t(rawYForLogical(milestone)+1)*kRawFrameWidth);
        assert(lowerMilestoneForPrefix(prefix-1)==previous);
        assert(lowerMilestoneForPrefix(prefix)==milestone);
        assert(lowerMilestoneForPrefix(prefix+1)==milestone);
        previous=milestone;
    }
    assert(lowerMilestoneForPrefix(0)==-1);
    assert(lowerMilestoneForPrefix(kRawFrameBytes)==222);
    assert(lowerMilestoneForPrefix(kRawFrameBytes+1)==-1);
    assert(lowerPrefixThrough(-1)==0 && lowerPrefixThrough(240)==0);
    assert(cache.capture(raw.data(),raw.size()));
    std::fill(raw.begin(),raw.end(),0);
    checkLower(cache.view(),11,222); // No camera buffer alias survives.
    assert(!cache.capture(nullptr,kRawFrameBytes) && !cache.view().valid());
    assert(cache.maxLogicalY()==-1);
    assert(!cache.capture(raw.data(),kRawFrameBytes+1) && !cache.view().valid());
    LowerRowCache small(storage.data()+1,kLowerBytes-1),missing;
    assert(!small.capture(raw.data(),raw.size()) && !small.view().valid());
    assert(!missing.capture(raw.data(),raw.size()) && !missing.view().valid());
}
static void viewCases() {
    // The consumer reuses a larger scratch allocation. The packed copy must
    // retain its own indices after the mailbox cache object ceases to exist.
    std::vector<uint8_t> storage(kRequiredBytes+2,0xa5);
    LowerRowCache scratch(storage.data()+1,kRequiredBytes);
    {
        auto raw=image(31);
        std::vector<uint8_t> source(kLowerBytes);
        LowerRowCache mailbox(source.data(),source.size());
        assert(mailbox.capture(raw.data(),raw.size()));
        assert(scratch.capture(mailbox.view(),190));
        std::fill(source.begin(),source.end(),0);
        std::fill(raw.begin(),raw.end(),0);
    }
    checkLower(scratch.view(),31,190);
    assert(scratch.maxLogicalY()==190);
    assert(storage.front()==0xa5);
    assert(std::all_of(storage.begin()+1+kLowerBytes,storage.end(),
                       [](uint8_t value){return value==0xa5;}));
    auto full=image(43,true);
    const auto view=GrayRowView::full(full.data());
    for(int maximum=0;maximum<appcfg::kFrameHeight;++maximum) {
        const int expected=lastRequired(maximum);
        assert(scratch.capture(view,maximum)==(expected>=0));
        assert(scratch.maxLogicalY()==expected);
        checkLower(scratch.view(),43,expected);
    }
    assert(!scratch.capture(view,-1) && !scratch.view().valid());
    assert(!scratch.capture(view,240) && !scratch.view().valid());
    assert(!scratch.capture(GrayRowView{},222) && !scratch.view().valid());
    LowerRowCache small(storage.data()+1,kLowerBytes-1),missing;
    assert(!small.capture(view,222) && !small.view().valid());
    assert(!missing.capture(view,222) && !missing.view().valid());
    std::array<uint8_t,appcfg::kFrameHeight> indices;
    for(int y=0;y<appcfg::kFrameHeight;++y)indices[y]=static_cast<uint8_t>(y);
    auto incomplete=view;
    incomplete.*member(ViewIndices{})=indices.data();
    for(int absent=0;absent<appcfg::kFrameHeight;++absent)if(lowerNeededRow(absent)) {
        assert(scratch.capture(view,222));
        std::fill(storage.begin(),storage.end(),0xa5);
        indices[absent]=255;
        assert(!scratch.capture(incomplete,222) && !scratch.view().valid());
        assert(scratch.maxLogicalY()==-1);
        assert(std::all_of(storage.begin(),storage.end(),[](uint8_t v){return v==0xa5;}));
        // A missing future row does not invalidate an earlier complete stage.
        if(absent>174)assert(scratch.capture(incomplete,174));
        indices[absent]=static_cast<uint8_t>(absent);
    }
}
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
struct Context {
    uint64_t expected=0;
    unsigned seed=0,lower_calls=0,upper_calls=0;
    int maximum=-1;
    std::function<void()> reading,upper_reading;
    std::vector<char> order;
};
static void lowerConsumer(const GrayRowView& rows,uint64_t stamp,uint32_t copy_us,int maximum,void* ptr) {
    auto& c=*static_cast<Context*>(ptr);
    assert(stamp==c.expected && copy_us>0 && maximum==c.maximum);
    ++c.lower_calls;c.order.push_back('L');
    checkLower(rows,c.seed,maximum);
    if(c.reading) {auto once=c.reading;c.reading={};once();}
    checkLower(rows,c.seed,maximum);
}
static void upperConsumer(const GrayRowView& rows,uint64_t stamp,uint32_t copy_us,void* ptr) {
    auto& c=*static_cast<Context*>(ptr);
    assert(stamp==c.expected && copy_us>0);
    ++c.upper_calls;c.order.push_back('U');
    checkUpper(rows,c.seed);
    if(c.upper_reading) {auto once=c.upper_reading;c.upper_reading={};once();}
    const unsigned before=c.lower_calls;
    rw_camera_overlap_poll(); // Cannot enter the lower callback inside upper.
    assert(c.lower_calls==before);
    checkUpper(rows,c.seed);
}
static void mailboxCases() {
    const uint64_t base=(uint64_t(1)<<40)+98765;
    Context c;
    std::vector<uint8_t> storage(kLowerBytes+2,0xa5),other(kLowerBytes);
    auto raw=image(51);
    rw_camera_overlap_offer(raw.data(),raw.size(),base);rw_camera_overlap_poll();
    assert(rw_camera_lower_overlap_stats().offered==0 && c.lower_calls==0);
    assert(!camera_row_overlap::initializeLower(nullptr,kLowerBytes,lowerConsumer,&c));
    assert(!camera_row_overlap::initializeLower(storage.data()+1,kLowerBytes-1,lowerConsumer,&c));
    assert(!camera_row_overlap::initializeLower(storage.data()+1,kLowerBytes,nullptr,&c));
    assert(camera_row_overlap::initializeLower(storage.data()+1,kLowerBytes,lowerConsumer,&c));
    assert(!camera_row_overlap::initializeLower(other.data(),other.size(),lowerConsumer,&c));
    rw_camera_overlap_offer(nullptr,raw.size(),base);
    rw_camera_overlap_offer(raw.data(),raw.size(),0);
    rw_camera_overlap_offer(raw.data(),kRawFrameBytes+1,base);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(174)-1,base);
    assert(rw_camera_lower_overlap_stats().offered==0);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(174),base);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(174),base);
    assert(rw_camera_lower_overlap_stats().offered==1);
    raw=image(53);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(190),base);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(174),base);
    std::fill(raw.begin(),raw.end(),0);
    c.expected=base;c.seed=53;c.maximum=190;rw_camera_overlap_poll();
    auto stats=rw_camera_lower_overlap_stats();
    assert(c.lower_calls==1 && stats.offered==2 && stats.replaced==1 && stats.consumed==1);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(190),base);rw_camera_overlap_poll();
    assert(c.lower_calls==1); // Successful stages cannot be published twice.
    raw=image(55);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(206),base);
    c.seed=55;c.maximum=206;rw_camera_overlap_poll();
    raw=image(57);
    rw_camera_overlap_offer(raw.data(),raw.size(),base);
    c.seed=57;c.maximum=222;rw_camera_overlap_poll();
    assert(c.lower_calls==3);

    // Seeing the start of a new frame excludes a late old frame, even if its
    // prefix is longer. No pixels may be relabeled with the newer timestamp.
    raw=image(59);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(174)-1,base+1);
    const auto old_offered=rw_camera_lower_overlap_stats().offered;
    rw_camera_overlap_offer(raw.data(),raw.size(),base);
    assert(rw_camera_lower_overlap_stats().offered==old_offered);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(174),base+1);
    c.expected=base+1;c.seed=59;c.maximum=174;
    auto competing=image(61);
    const auto busy_before=rw_camera_lower_overlap_stats().busy;
    c.reading=[&] {
        rw_camera_overlap_offer(competing.data(),lowerPrefixThrough(190),base+2);
        rw_camera_overlap_poll();
        assert(c.lower_calls==4);
        assert(rw_camera_lower_overlap_stats().busy==busy_before+1);
    };
    rw_camera_overlap_poll();assert(c.lower_calls==4);
    rw_camera_overlap_offer(raw.data(),raw.size(),base+1);
    rw_camera_overlap_poll();assert(c.lower_calls==4);
    // READING refusal did not mark a stage consumed: a later DMA chunk retries.
    rw_camera_overlap_offer(competing.data(),lowerPrefixThrough(190)+17,base+2);
    c.expected=base+2;c.seed=61;c.maximum=190;rw_camera_overlap_poll();
    assert(c.lower_calls==5);
    raw=image(63);
    timer_hook=[&] {rw_camera_overlap_poll();assert(c.lower_calls==5);};
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(206),base+3);
    assert(!timer_hook && c.lower_calls==5);
    c.expected=base+3;c.seed=63;c.maximum=206;rw_camera_overlap_poll();
    assert(c.lower_calls==6); // WRITING never published partially copied data.
    raw=image(65);
    rw_camera_overlap_offer(raw.data(),raw.size(),base+4);
    raw=image(67);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(174),base+5);
    c.expected=base+5;c.seed=67;c.maximum=174;rw_camera_overlap_poll();
    assert(c.lower_calls==7); // New frame replaced READY and hid old future rows.

    // Enable upper after isolated lower tests. The upper duplicate guard must
    // not block later lower milestones from the same acquisition.
    std::vector<uint8_t> upper_storage(kUpperBytes+2,0x5a);
    assert(camera_row_overlap::initialize(upper_storage.data()+1,kUpperBytes,upperConsumer,&c));
    raw=image(71);
    rw_camera_overlap_offer(raw.data(),kUpperPrefixBytes,base+6);
    c.expected=base+6;c.seed=71;c.order.clear();rw_camera_overlap_poll();
    assert(c.upper_calls==1 && c.lower_calls==7 && c.order==std::vector<char>{'U'});
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(174),base+6);
    c.maximum=174;c.order.clear();rw_camera_overlap_poll();
    assert(c.upper_calls==1 && c.lower_calls==8 && c.order==std::vector<char>{'L'});
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(190),base+6);
    c.maximum=190;rw_camera_overlap_poll();assert(c.lower_calls==9);
    raw=image(73);
    rw_camera_overlap_offer(raw.data(),lowerPrefixThrough(206)+31,base+7);
    c.expected=base+7;c.seed=73;c.maximum=206;c.order.clear();rw_camera_overlap_poll();
    const std::vector<char> expected_order={'U','L'};
    assert(c.order==expected_order && c.upper_calls==2 && c.lower_calls==10);
    rw_camera_overlap_poll();assert(c.lower_calls==10 && c.upper_calls==2);
    assert(storage.front()==0xa5 && storage.back()==0xa5);
    assert(upper_storage.front()==0x5a && upper_storage.back()==0x5a);
    stats=rw_camera_lower_overlap_stats();
    assert(stats.consumed==c.lower_calls && stats.busy==1 && stats.replaced==2);
    assert(stats.offered==stats.consumed+stats.replaced);

    // V26 upper must retry a frame skipped while an earlier callback owned
    // the slot. Previously the busy encounter consumed the frame's only offer.
    // The legacy regression separately preserves that old profile behavior.
    raw=image(79);
    rw_camera_overlap_offer(raw.data(),kUpperPrefixBytes,base+8);
    c.expected=base+8;c.seed=79;
    auto next_upper=image(83);
    const auto upper_busy_before=rw_camera_overlap_stats().busy;
    c.upper_reading=[&] {
        rw_camera_overlap_offer(next_upper.data(),kUpperPrefixBytes,base+9);
        rw_camera_overlap_poll();
        assert(c.upper_calls==3 && c.lower_calls==10);
        assert(rw_camera_overlap_stats().busy==upper_busy_before+1);
    };
    rw_camera_overlap_poll();assert(c.upper_calls==3);
    const auto upper_offers_before=rw_camera_overlap_stats().offered;
    rw_camera_overlap_offer(next_upper.data(),kUpperPrefixBytes+1280,base+9);
    assert(rw_camera_overlap_stats().offered==upper_offers_before+1);
    c.expected=base+9;c.seed=83;rw_camera_overlap_poll();
    assert(c.upper_calls==4 && c.lower_calls==10);
    // A successful retry still deduplicates later chunks of the same frame.
    rw_camera_overlap_offer(next_upper.data(),kUpperPrefixBytes+2560,base+9);
    rw_camera_overlap_poll();assert(c.upper_calls==4);
    assert(upper_storage.front()==0x5a && upper_storage.back()==0x5a);
}
#endif
int main() {
    cacheCases();viewCases();
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    mailboxCases();
    std::cout<<"PASS V26 lower mailbox: cumulative stages, complete-row bounds, 64-bit stamps, new-frame replacement, reading/writing exclusion, busy retry, non-reentrant upper/lower order\n";
#else
    std::cout<<"PASS lower cache geometry raw="<<kRawFrameHeight<<": every prefix boundary, copied-view ownership, exact future-row hiding, incomplete-view rejection\n";
#endif
}
'''


def main():
    compiler = shutil.which('g++')
    if not compiler:
        raise SystemExit('g++ is required for the production mailbox test')
    variants = [
        ('legacy', 'c++17', []),
        ('native', 'c++17', ['FOOT_NATIVE_SKIP_V25=1']),
        ('stream', 'c++17', ['FOOT_NATIVE_SKIP_V25=1', 'FOOT_NATIVE_STREAM_V26=1']),
        ('stream_cpp11', 'c++11', ['FOOT_NATIVE_SKIP_V25=1', 'FOOT_NATIVE_STREAM_V26=1']),
    ]
    with tempfile.TemporaryDirectory(prefix='camera_lower_overlap_') as folder:
        path = Path(folder)
        (path/'esp_timer.h').write_text(
            '#pragma once\n#include <stdint.h>\nextern "C" int64_t esp_timer_get_time();\n',
            encoding='utf8')
        (path/'test.cpp').write_text(CPP, encoding='utf8')
        for name, standard, flags in variants:
            binary = path/(name+'.exe')
            subprocess.run([compiler, '-std='+standard, '-O2', '-Wall', '-Wextra', '-Werror',
                            '-DFOOT_ROW_OVERLAP_V24=1', *['-D'+flag for flag in flags],
                            '-I'+str(path), '-I'+str(ROOT/'src'), str(path/'test.cpp'),
                            str(ROOT/'src/camera_row_overlap.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
