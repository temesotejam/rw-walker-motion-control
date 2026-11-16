"""Verify speculative image preparation without committing temporal tracker state.

Reuses the frozen detector oracle and broad image/time corpus. All clocks are
synthetic; this checks equivalence and lifetime, not ESP32 latency.
"""
from pathlib import Path
import subprocess
import tempfile
import test_foot_tracking_fast as oracle

ROOT = Path(__file__).resolve().parents[1]


def fixture():
    cpp = oracle.CPP.replace(
        'WhiteMarker1DTracker full_a{0},full_b{1},pointer_a{0},pointer_b{1},packed_a{0},packed_b{1};',
        'WhiteMarker1DTracker full_a{0},full_b{1},pointer_a{0},pointer_b{1},packed_a{0},packed_b{1},early_a{0},early_b{1};')
    needle = '  host_us=stamp;equal(a,packed_a.process(cache.view()));equal(b,packed_b.process(cache.view()));'
    extra = r'''
  unsigned char before_a[sizeof(early_a)],before_b[sizeof(early_b)];
  std::memcpy(before_a,&early_a,sizeof(early_a));std::memcpy(before_b,&early_b,sizeof(early_b));
  host_us=stamp-25000u;
  // Every frame has an extra abandoned speculation: it must not count as a
  // failed measurement, increment reacquisition, or change track memory.
  auto discarded=blank();band(discarded,10,66);band(discarded,310,160);
  (void)early_a.prepare(rows::GrayRowView::full(discarded.data()));
  (void)early_b.prepare(rows::GrayRowView::full(discarded.data()));
  const auto prepared_a=early_a.prepare(cache.view()),prepared_b=early_b.prepare(cache.view());
  assert(std::memcmp(before_a,&early_a,sizeof(early_a))==0);
  assert(std::memcmp(before_b,&early_b,sizeof(early_b))==0);
  // Feature values own all pixels they need after prepare returns.
  std::fill(storage.begin(),storage.end(),0);
  host_us=stamp;equal(a,early_a.processPrepared(prepared_a));equal(b,early_b.processPrepared(prepared_b));
'''
    assert needle in cpp
    cpp = cpp.replace(needle, needle + extra)
    cases = r'''
static void preparedAdmissionCases(){
 WhiteMarker1DTracker ordinary(0),early(0),other(1);
 auto image=blank();band(image,80,66);band(image,170,160);
 auto run=[&](uint32_t preparation,uint32_t commit){
  host_us=preparation;auto features=early.prepare(rows::GrayRowView::full(image.data()));
  host_us=commit;auto a=ordinary.process(image.data());host_us=commit;equal(a,early.processPrepared(features));
 };
 run(975000,1000000);
 image=blank();band(image,250,66);
 run(1475001,1500001); // Commit crosses the 500ms track-memory boundary.
 run(1775001,1800001); // At the pending 300ms boundary.
 run(2075002,2100002); // Beyond pending validity, despite early preparation.
 run(0xffff7ff0u,0xfffffff0u);run(0x100u,0x20000u);
 // Wrong lane/default prepared value is rejected without changing any state.
 unsigned char before[sizeof(early)];std::memcpy(before,&early,sizeof(early));
 host_us=0x30000u;const auto wrong=other.prepare(rows::GrayRowView::full(image.data()));
 auto bad=early.processPrepared(wrong);
 assert(!bad.valid && bad.reason==MarkerDetectionReason::NoFrame && bad.id==0);
 assert(std::memcmp(before,&early,sizeof(early))==0);
 bad=early.processPrepared(WhiteMarker1DTracker::PreparedFeatures{});
 assert(!bad.valid && std::memcmp(before,&early,sizeof(early))==0);
 run(0x32000u,0x40000u);
 // Invalid image preparation retains the regular no-frame transition semantics.
 host_us=0x50000u;const auto missing=early.prepare(rows::GrayRowView{});
 host_us=0x60000u;auto a=ordinary.process(nullptr);host_us=0x60000u;equal(a,early.processPrepared(missing));
 std::cout<<"prepared marker identity, abandoned speculation, copied features, commit-time thresholds/wrap PASS\n";
}
'''
    needle = 'int main(int argc,char** argv){'
    assert needle in cpp
    cpp = cpp.replace(needle, cases + '\n' + needle)
    cpp = cpp.replace(' cacheContract();deterministicCases();fuzzCases();readCounts();',
                      ' cacheContract();deterministicCases();fuzzCases();readCounts();preparedAdmissionCases();')
    return cpp


def main():
    with tempfile.TemporaryDirectory(prefix='foot_prepared_') as folder:
        path = Path(folder)
        (path / 'test.cpp').write_text(fixture(), encoding='utf8')
        (path / 'legacy_tracker.h').write_text(
            '#pragma once\n#include "white_marker_tracker.h"\n' + oracle.LEGACY_HEADER, encoding='utf8')
        (path / 'legacy.cpp').write_text(oracle.LEGACY_SOURCE, encoding='utf8')
        exe = path / 'test.exe'
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-DFOOT_LOW_LATENCY_V20=1', '-DFOOT_SPEED_V22=1', '-DFOOT_CAPTURE_V23=1',
                        '-DFOOT_ROW_OVERLAP_V24=1', '-DFOOT_IMAGE_ROWS_TEST_READS=1',
                        '-I' + str(ROOT / 'tools/host_v46o'), '-I' + str(ROOT / 'src'), '-I' + str(path),
                        str(path / 'test.cpp'), str(path / 'legacy.cpp'),
                        str(ROOT / 'src/white_marker_tracker.cpp'), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
