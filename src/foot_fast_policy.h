#pragma once
#include <cstddef>
#include <cstdint>
#include <stdint.h>
#include <stdio.h>

// v20 bounds capture/recognition load. Actual sensor FPS and end-to-end age
// must be measured on the board; these settings are not performance claims.
namespace foot_fast_policy {
#if defined(FOOT_CADENCE20_V37) && FOOT_CADENCE20_V37
constexpr uint32_t kWorkerPeriodUs=20000;
#elif defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
constexpr uint32_t kWorkerPeriodUs=23000;
#elif defined(FOOT_WORKER26_V34) && FOOT_WORKER26_V34
constexpr uint32_t kWorkerPeriodUs=26000;
#elif defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
constexpr uint32_t kWorkerPeriodUs=33333;
#else
constexpr uint32_t kWorkerPeriodUs=40000;
#endif
constexpr uint32_t kControlPollMs=2;
constexpr unsigned kCameraFrameBuffers=2;
#if defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
#if defined(FOOT_CADENCE20_V37) && FOOT_CADENCE20_V37
constexpr uint32_t kArchiveCapacity=2048;
#else
constexpr uint32_t kArchiveCapacity=1760;
#endif
constexpr size_t kMetadataHardBudgetBytes=1408U*1024U;
#if defined(SUPPORT_TRANSFER_V39) && SUPPORT_TRANSFER_V39
// All published metadata, including its footer, fits this limit. reserve() adds the NUL byte.
constexpr size_t kMetadataReserveBytes=kMetadataHardBudgetBytes;
#else
constexpr size_t kMetadataReserveBytes=1536U*1024U;
#endif
static_assert((kArchiveCapacity-1ULL)*kWorkerPeriodUs>=40000000ULL,"Retain the full 40-second capture");
#elif defined(FOOT_WORKER26_V34) && FOOT_WORKER26_V34
constexpr uint32_t kArchiveCapacity=1600; // 5s start +30s trial +5s end, plus margin.
constexpr size_t kMetadataHardBudgetBytes=1280U*1024U;
constexpr size_t kMetadataReserveBytes=1408U*1024U;
static_assert((kArchiveCapacity-1ULL)*kWorkerPeriodUs>=40000000ULL,"Retain the full 40-second capture");
#else
constexpr uint32_t kArchiveCapacity=1280; // 40 seconds *25Hz plus margin.
constexpr size_t kMetadataHardBudgetBytes=1024U*1024U;
constexpr size_t kMetadataReserveBytes=1152U*1024U;
#endif
// Raw foot frames and controller audits take precedence over forecast copies.
constexpr size_t kOptionalPredictionEndBytes=kMetadataHardBudgetBytes*65U/100U;
constexpr size_t kEnergyAuditEndBytes=kMetadataHardBudgetBytes*96U/100U;
constexpr size_t kSupportAuditEndBytes=kMetadataHardBudgetBytes*99U/100U;
}

// Owner must hold the observer mutex. One update per first control receipt.
// Bins [i*250,(i+1)*250) us; last bin includes everything >=63750 us.
// Boot-lifetime aggregates, not an archive or an exposure timestamp correction.
struct FootLatencyHistogram {
  static constexpr unsigned kBins=256, kBinUs=250;
  uint32_t bins[kBins]={};
  uint32_t count=0,both_detected=0,invalid_time=0,max_us=0,min_us=UINT32_MAX;
  uint32_t max_gap_us=0;
  uint64_t sum_us=0,last_receipt_us=0;
#if defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
  struct Window {uint32_t epoch=0,receipts=0,detected=0;};
  Window windows[16]; // Last 80 seconds, five seconds per bucket.
  uint32_t valid_gap_bins[128]={},valid_gaps=0,valid_gaps_over_50ms=0;
  uint64_t last_valid_receipt_us=0;
#endif
  void record(uint64_t frame_us,uint64_t receipt_us,bool detected) {
    if(!frame_us || receipt_us<frame_us || receipt_us-frame_us>UINT32_MAX) {
      ++invalid_time; return;
    }
    const uint32_t age=static_cast<uint32_t>(receipt_us-frame_us);
    const unsigned bin=age/kBinUs;
    ++bins[bin<kBins?bin:kBins-1];++count;if(detected)++both_detected;
    sum_us+=age;if(age>max_us)max_us=age;if(age<min_us)min_us=age;
    if(last_receipt_us && receipt_us>=last_receipt_us && receipt_us-last_receipt_us<=UINT32_MAX) {
      const auto gap=static_cast<uint32_t>(receipt_us-last_receipt_us);
      if(gap>max_gap_us)max_gap_us=gap;
    }
    last_receipt_us=receipt_us;
#if defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
    const uint32_t epoch=static_cast<uint32_t>(receipt_us/5000000ULL);
    auto& window=windows[epoch%16];
    if(window.epoch!=epoch){window=Window{};window.epoch=epoch;}
    ++window.receipts;
    if(detected) {
      ++window.detected;
      if(last_valid_receipt_us && receipt_us>=last_valid_receipt_us) {
        const uint64_t gap=receipt_us-last_valid_receipt_us;
        const uint64_t index=gap/500;
        ++valid_gap_bins[index<128?static_cast<unsigned>(index):127];
        ++valid_gaps;if(gap>50000)++valid_gaps_over_50ms;
      }
      last_valid_receipt_us=receipt_us;
    }
#endif
  }
  template<class Text> void appendJson(Text& out) const {
    char text[384];
    snprintf(text,sizeof(text),"{\"count\":%lu,\"both_detected\":%lu,\"invalid_time\":%lu,\"sum_us\":%llu,\"min_us\":%lu,\"max_us\":%lu,\"max_gap_us\":%lu,\"bin_us\":250,\"overflow_from_us\":63750,\"bins\":[",
      static_cast<unsigned long>(count),static_cast<unsigned long>(both_detected),static_cast<unsigned long>(invalid_time),
      static_cast<unsigned long long>(sum_us),static_cast<unsigned long>(count?min_us:0),
      static_cast<unsigned long>(max_us),static_cast<unsigned long>(max_gap_us));
    out+=text;
    for(unsigned i=0;i<kBins;++i){snprintf(text,sizeof(text),"%s%lu",i?",":"",static_cast<unsigned long>(bins[i]));out+=text;}
    out+="]";
#if defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
    snprintf(text,sizeof(text),",\"last_receipt_us\":%llu,\"last_valid_receipt_us\":%llu,\"valid_gaps\":%lu,\"valid_gaps_over_50ms\":%lu,\"valid_gap_bin_us\":500,\"valid_gap_overflow_from_us\":63500,\"valid_gap_bins\":[",
      static_cast<unsigned long long>(last_receipt_us),static_cast<unsigned long long>(last_valid_receipt_us),static_cast<unsigned long>(valid_gaps),static_cast<unsigned long>(valid_gaps_over_50ms));
    out+=text;
    for(unsigned i=0;i<128;++i){snprintf(text,sizeof(text),"%s%lu",i?",":"",static_cast<unsigned long>(valid_gap_bins[i]));out+=text;}
    out+="],\"window_us\":5000000,\"windows\":[";
    for(unsigned i=0;i<16;++i){const auto& w=windows[i];snprintf(text,sizeof(text),"%s[%lu,%lu,%lu]",i?",":"",static_cast<unsigned long>(w.epoch),static_cast<unsigned long>(w.receipts),static_cast<unsigned long>(w.detected));out+=text;}
    out+="]";
#endif
    out+="}";
  }
};
