#pragma once
#include <atomic>
#include <cstdint>
// Control task writes. Export obtains an atomic, bounded snapshot.
namespace early_diag {
constexpr unsigned kReasonCount=16,kAsyncReasonCount=8;
struct Snapshot {
 uint32_t run=0,frames=0,invalid_frames=0,submitted=0,submit_failed=0,pose_dropped=0;
 uint32_t replies=0,valid=0,armed=0,stale_replies=0;
 uint32_t max_compute_us=0,max_step_us=0,min_stack_raw=0;bool worker_ready=false;
 uint32_t reason[kReasonCount]={},async_reason[kAsyncReasonCount]={};
};
struct Counters {
 std::atomic<uint32_t> run{0},frames{0},invalid_frames{0},submitted{0},submit_failed{0},pose_dropped{0};
 std::atomic<uint32_t> replies{0},valid{0},armed{0},stale_replies{0};
 std::atomic<uint32_t> max_compute_us{0},max_step_us{0},min_stack_raw{0};
 std::atomic<bool> worker_ready{false};
 std::atomic<uint32_t> reason[kReasonCount]{},async_reason[kAsyncReasonCount]{};
 void reset(uint32_t r){
   run.store(0);frames.store(0);invalid_frames.store(0);submitted.store(0);submit_failed.store(0);pose_dropped.store(0);
   replies.store(0);valid.store(0);armed.store(0);stale_replies.store(0);
   max_compute_us.store(0);max_step_us.store(0);min_stack_raw.store(0);
   for(auto& x:reason)x.store(0);
   for(auto& x:async_reason)x.store(0);
   run.store(r);
 }
 Snapshot snapshot()const{
   Snapshot s;s.run=run.load();s.frames=frames.load();s.invalid_frames=invalid_frames.load();
   s.submitted=submitted.load();s.submit_failed=submit_failed.load();s.pose_dropped=pose_dropped.load();
   s.replies=replies.load();s.valid=valid.load();s.armed=armed.load();s.stale_replies=stale_replies.load();
   s.max_compute_us=max_compute_us.load();s.max_step_us=max_step_us.load();
   s.min_stack_raw=min_stack_raw.load();s.worker_ready=worker_ready.load();
   for(unsigned i=0;i<kReasonCount;++i)s.reason[i]=reason[i].load();
   for(unsigned i=0;i<kAsyncReasonCount;++i)s.async_reason[i]=async_reason[i].load();
   return s;
 }
};
}
extern early_diag::Counters g_early_diag;
