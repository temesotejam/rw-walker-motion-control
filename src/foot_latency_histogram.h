#pragma once
#include <stdint.h>
#include <stdio.h>

// Owner must hold the observer mutex. One update per first control receipt.
// Bins [i*250,(i+1)*250) us; last bin includes everything >=63750 us.
// Boot-lifetime aggregates, not an archive or an exposure timestamp correction.
struct FootLatencyHistogram {
  static constexpr unsigned kBins=256, kBinUs=250;
  uint32_t bins[kBins]={};
  uint32_t count=0,both_detected=0,invalid_time=0,max_us=0,min_us=UINT32_MAX;
  uint32_t max_gap_us=0;
  uint64_t sum_us=0,last_receipt_us=0;
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
  }
  template<class Text> void appendJson(Text& out) const {
    char text[384];
    snprintf(text,sizeof(text),"{\"count\":%lu,\"both_detected\":%lu,\"invalid_time\":%lu,\"sum_us\":%llu,\"min_us\":%lu,\"max_us\":%lu,\"max_gap_us\":%lu,\"bin_us\":250,\"overflow_from_us\":63750,\"bins\":[",
      static_cast<unsigned long>(count),static_cast<unsigned long>(both_detected),static_cast<unsigned long>(invalid_time),
      static_cast<unsigned long long>(sum_us),static_cast<unsigned long>(count?min_us:0),
      static_cast<unsigned long>(max_us),static_cast<unsigned long>(max_gap_us));
    out+=text;
    for(unsigned i=0;i<kBins;++i){snprintf(text,sizeof(text),"%s%lu",i?",":"",static_cast<unsigned long>(bins[i]));out+=text;}
    out+="]}";
  }
};
