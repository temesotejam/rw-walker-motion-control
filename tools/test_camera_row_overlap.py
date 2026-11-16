"""Exercise the production upper-row cache and mailbox ownership protocol.

Producer/consumer interleavings are injected at state transitions. This checks
pixel ownership and admission, not sensor timing or target CPU scheduling.
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

using namespace foot_image_rows;
// Test-only friend injection constructs an incomplete view without changing
// production visibility or the class definition in either translation unit.
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
static uint8_t pixel(unsigned y,unsigned x,unsigned seed) {
    return static_cast<uint8_t>((13*y+7*x+seed)%251);
}
static std::vector<uint8_t> image(unsigned seed,size_t size=kFrameBytes) {
    std::vector<uint8_t> data(size);
    for(size_t i=0;i<size;++i)data[i]=pixel(i/appcfg::kFrameWidth,i%appcfg::kFrameWidth,seed);
    return data;
}
static void checkView(const GrayRowView& view,unsigned seed) {
    assert(view.valid());
    assert(!view.row(-1) && !view.row(appcfg::kFrameHeight));
    unsigned count=0;
    for(int y=0;y<appcfg::kFrameHeight;++y) {
        const auto* row=view.row(y);
        assert(bool(row)==upperNeededRow(y));
        if(!row)continue;
        ++count;
        for(int x=0;x<appcfg::kFrameWidth;++x)assert(row[x]==pixel(y,x,seed));
    }
    assert(count==26 && count==kUpperRows);
    // Every pixel required by all 17 upper-lane templates must be available.
    for(int dy=-appcfg::kWhiteSearchRadiusYPx;dy<=appcfg::kWhiteSearchRadiusYPx;
        dy+=appcfg::kWhiteSearchStepYPx) {
        for(int y:appcfg::kWhiteMarkerARows)assert(view.row(y+dy));
        for(int y:appcfg::kWhiteReferenceARows)assert(view.row(y+dy));
    }
}
struct Context {
    uint64_t expected=0;
    unsigned seed=0,calls=0;
    bool reentrant=false;
};
static void consumer(const GrayRowView& view,uint64_t stamp,uint32_t copy_us,void* ptr) {
    auto& c=*static_cast<Context*>(ptr);
    assert(stamp==c.expected && copy_us>0);
    ++c.calls;
    checkView(view,c.seed);
    if(c.reentrant) {
        auto competing=image(77);
        rw_camera_overlap_offer(competing.data(),competing.size(),stamp+1);
        rw_camera_overlap_poll(); // READING cannot be consumed twice.
        checkView(view,c.seed); // Producer must not overwrite owned pixels.
        const auto s=rw_camera_overlap_stats();assert(s.busy==1);
    }
}
static void cacheCases() {
    static_assert(kUpperRows==26 && kUpperBytes==8320 && kUpperPrefixBytes==34240);
    std::vector<uint8_t> storage(kUpperBytes+2,0xa5);
    auto prefix=image(11,kUpperPrefixBytes);
    UpperRowCache rows(storage.data()+1,kUpperBytes);
    assert(!rows.view().valid());
    assert(!rows.capture(prefix.data(),kUpperPrefixBytes-1));
    assert(!rows.view().valid());
    assert(rows.capture(prefix.data(),prefix.size()));
    std::fill(prefix.begin(),prefix.end(),0); // Only independent pixels survive.
    checkView(rows.view(),11);
    assert(storage.front()==0xa5 && storage.back()==0xa5);
    assert(!rows.capture(nullptr,kUpperPrefixBytes) && !rows.view().valid());
    assert(!rows.capture(prefix.data(),kFrameBytes+1) && !rows.view().valid());
    UpperRowCache too_small(storage.data()+1,kUpperBytes-1),missing;
    assert(!too_small.capture(prefix.data(),prefix.size()));
    assert(!missing.capture(prefix.data(),prefix.size()));
}
static void viewCaptureCases() {
    // Match the consumer's larger, reused internal allocation. The compact
    // copy must touch only its own 8320 bytes and own both pixels and indices.
    std::vector<uint8_t> storage(kRequiredBytes+2,0xa5);
    UpperRowCache scratch(storage.data()+1,kRequiredBytes);
    {
        auto full=image(43);
        std::vector<uint8_t> mailbox_storage(kUpperBytes);
        UpperRowCache mailbox(mailbox_storage.data(),mailbox_storage.size());
        assert(mailbox.capture(full.data(),full.size()));
        assert(scratch.capture(mailbox.view()));
        std::fill(mailbox_storage.begin(),mailbox_storage.end(),0);
        std::fill(full.begin(),full.end(),0);
        checkView(scratch.view(),43);
    } // Source pixels and source row-index table have both expired.
    checkView(scratch.view(),43);
    assert(storage.front()==0xa5);
    assert(std::all_of(storage.begin()+1+kUpperBytes,storage.end(),
                       [](uint8_t value){return value==0xa5;}));

    auto full=image(47);
    const auto full_view=GrayRowView::full(full.data());
    assert(scratch.capture(full_view));checkView(scratch.view(),47);
    assert(!scratch.capture(GrayRowView{}) && !scratch.view().valid());
    UpperRowCache too_small(storage.data()+1,kUpperBytes-1),missing;
    assert(!too_small.capture(full_view) && !too_small.view().valid());
    assert(!missing.capture(full_view) && !missing.view().valid());

    // A nominally valid view may lack any required row. Validate the entire
    // row set before copying: even a missing final row must leave bytes intact.
    std::array<uint8_t,appcfg::kFrameHeight> indices;
    for(int y=0;y<appcfg::kFrameHeight;++y)indices[y]=static_cast<uint8_t>(y);
    auto partial=full_view;
    partial.*member(ViewIndices{})=indices.data();
    for(int absent=0;absent<appcfg::kFrameHeight;++absent)if(upperNeededRow(absent)) {
        assert(scratch.capture(full_view)); // Failure also clears a prior success.
        std::fill(storage.begin(),storage.end(),0xa5);
        indices[absent]=255;
        assert(partial.valid() && !partial.row(absent));
        assert(!scratch.capture(partial) && !scratch.view().valid());
        assert(std::all_of(storage.begin(),storage.end(),
                           [](uint8_t value){return value==0xa5;}));
        indices[absent]=static_cast<uint8_t>(absent);
    }
    assert(scratch.capture(partial));checkView(scratch.view(),47);
}
int main() {
    cacheCases();
    viewCaptureCases();
    Context c;
    std::vector<uint8_t> storage(kUpperBytes+2,0xa5),other(kUpperBytes);
    auto first=image(19,kUpperPrefixBytes);
    rw_camera_overlap_offer(first.data(),first.size(),100);rw_camera_overlap_poll();
    assert(!rw_camera_overlap_stats().offered && !c.calls);
    assert(!camera_row_overlap::initialize(nullptr,kUpperBytes,consumer,&c));
    assert(!camera_row_overlap::initialize(storage.data()+1,kUpperBytes-1,consumer,&c));
    assert(!camera_row_overlap::initialize(storage.data()+1,kUpperBytes,nullptr,&c));
    assert(camera_row_overlap::initialize(storage.data()+1,kUpperBytes,consumer,&c));
    assert(!camera_row_overlap::initialize(other.data(),other.size(),consumer,&c));

    rw_camera_overlap_offer(nullptr,first.size(),100);
    rw_camera_overlap_offer(first.data(),first.size(),0);
    rw_camera_overlap_offer(first.data(),kFrameBytes+1,100);
    rw_camera_overlap_offer(first.data(),first.size()-1,100);
    assert(!rw_camera_overlap_stats().offered);
    rw_camera_overlap_offer(first.data(),first.size(),100);
    rw_camera_overlap_offer(first.data(),first.size(),100);
    assert(rw_camera_overlap_stats().offered==1);
    std::fill(first.begin(),first.end(),0);
    c.expected=100;c.seed=19;rw_camera_overlap_poll();
    assert(c.calls==1 && rw_camera_overlap_stats().consumed==1);
    rw_camera_overlap_offer(first.data(),first.size(),100);rw_camera_overlap_poll();
    assert(c.calls==1); // Same frame is never offered twice, even after release.

    auto a=image(21),b=image(23);
    rw_camera_overlap_offer(a.data(),a.size(),101);
    rw_camera_overlap_offer(b.data(),b.size(),102); // Latest READY replaces old READY.
    std::fill(b.begin(),b.end(),0);
    c.expected=102;c.seed=23;rw_camera_overlap_poll();
    auto stats=rw_camera_overlap_stats();
    assert(c.calls==2 && stats.offered==3 && stats.replaced==1 && stats.consumed==2);

    rw_camera_overlap_offer(a.data(),a.size(),103);
    c.expected=103;c.seed=21;c.reentrant=true;rw_camera_overlap_poll();c.reentrant=false;
    assert(c.calls==3 && rw_camera_overlap_stats().busy==1);
    rw_camera_overlap_offer(a.data(),a.size(),104);rw_camera_overlap_poll();
    assert(c.calls==3); // The frame skipped while READING is not retried later.
    rw_camera_overlap_offer(a.data(),a.size(),105);
    c.expected=105;rw_camera_overlap_poll();assert(c.calls==4);

    // Polls during WRITING cannot observe a stamp with partially copied rows.
    timer_hook=[&]{rw_camera_overlap_poll();assert(c.calls==4);};
    rw_camera_overlap_offer(a.data(),a.size(),106);
    assert(!timer_hook && c.calls==4);
    c.expected=106;rw_camera_overlap_poll();assert(c.calls==5);
    rw_camera_overlap_poll();assert(c.calls==5);
    assert(storage.front()==0xa5 && storage.back()==0xa5);
    stats=rw_camera_overlap_stats();
    assert(stats.offered==6 && stats.busy==1 && stats.replaced==1 && stats.consumed==5);
    std::cout<<"PASS upper cache and actual mailbox: all rows/prefix boundary, independent view copy, null/small/missing-row rejection, owned pixels, init refusal, latest replacement, WRITING/READING exclusion, reentrant producer, duplicate suppression\n";
}
'''


def main():
    compiler = shutil.which('g++')
    if not compiler:
        raise SystemExit('g++ is required for the production mailbox test')
    with tempfile.TemporaryDirectory(prefix='camera_row_overlap_') as folder:
        path = Path(folder)
        (path/'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nextern "C" int64_t esp_timer_get_time();\n', encoding='utf8')
        (path/'test.cpp').write_text(CPP, encoding='utf8')
        binary = path/'test.exe'
        subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-DFOOT_ROW_OVERLAP_V24=1', '-I'+str(path), '-I'+str(ROOT/'src'),
                        str(path/'test.cpp'), str(ROOT/'src/camera_row_overlap.cpp'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
