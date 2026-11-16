#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "foot_tracking_config.h"

// A view has the original 320-column/Y coordinates even when only requested
// rows are stored. The caller owns the image buffer and the cache lifetime.
// No allocation or camera lifetime is hidden in either type.
namespace foot_image_rows {
constexpr size_t kFrameBytes=static_cast<size_t>(appcfg::kFrameWidth)*appcfg::kFrameHeight;
constexpr bool reachesRow(int y,int base) {
  return y-base>=-appcfg::kWhiteSearchRadiusYPx && y-base<=appcfg::kWhiteSearchRadiusYPx &&
      (y-base+appcfg::kWhiteSearchRadiusYPx)%appcfg::kWhiteSearchStepYPx==0;
}
constexpr bool requestedBy(int y,const int* rows,int count) {
  return count>0 && (reachesRow(y,rows[count-1]) || requestedBy(y,rows,count-1));
}
constexpr bool neededRow(int y) {
  return y>=0 && y<appcfg::kFrameHeight &&
      (requestedBy(y,appcfg::kWhiteMarkerARows,appcfg::kWhiteMarkerRowCount) ||
       requestedBy(y,appcfg::kWhiteReferenceARows,appcfg::kWhiteReferenceRowCount) ||
       requestedBy(y,appcfg::kWhiteMarkerBRows,appcfg::kWhiteMarkerRowCount) ||
       requestedBy(y,appcfg::kWhiteReferenceBRows,appcfg::kWhiteReferenceRowCount));
}
constexpr size_t countRows(int y=0) {
  return y>=appcfg::kFrameHeight?0:(neededRow(y)?1:0)+countRows(y+1);
}
constexpr size_t kRequiredRows=countRows();
constexpr size_t kRequiredBytes=kRequiredRows*appcfg::kFrameWidth;
static_assert(kRequiredRows>0 && kRequiredRows<255,"Packed row index fits one byte");
static_assert(appcfg::kWhiteSearchStepYPx>0,"Finite complete height search");

#if defined(FOOT_IMAGE_ROWS_TEST_READS) && FOOT_IMAGE_ROWS_TEST_READS
// Native verification only. Row counts become byte reads by multiplying by
// width in the row-major evaluator, which scans every returned row once.
struct ReadCounts {uint64_t full_rows=0,packed_rows=0,capture_rows=0,missing_rows=0;};
inline ReadCounts& readCounts(){static ReadCounts counts;return counts;}
#endif

class RowCache;
class UpperRowCache;
class GrayRowView {
 public:
  GrayRowView()=default;
  static GrayRowView full(const uint8_t* gray,size_t bytes=kFrameBytes) {
    return gray && bytes>=kFrameBytes?GrayRowView(gray,nullptr):GrayRowView{};
  }
  bool valid()const{return data_!=nullptr;}
  const uint8_t* row(int y)const {
    if(!data_ || y<0 || y>=appcfg::kFrameHeight || (indices_ && indices_[y]==255)) {
#if defined(FOOT_IMAGE_ROWS_TEST_READS) && FOOT_IMAGE_ROWS_TEST_READS
      ++readCounts().missing_rows;
#endif
      return nullptr;
    }
#if defined(FOOT_IMAGE_ROWS_TEST_READS) && FOOT_IMAGE_ROWS_TEST_READS
    if(indices_)++readCounts().packed_rows;else ++readCounts().full_rows;
#endif
    return data_+static_cast<size_t>(indices_?indices_[y]:y)*appcfg::kFrameWidth;
  }
 private:
  friend class RowCache;
  friend class UpperRowCache;
  GrayRowView(const uint8_t* data,const uint8_t* indices):data_(data),indices_(indices){}
  const uint8_t* data_=nullptr;
  const uint8_t* indices_=nullptr;
};

class RowCache {
 public:
  RowCache(uint8_t* storage,size_t capacity):storage_(storage),capacity_(capacity) {
    uint8_t index=0;
    for(int y=0;y<appcfg::kFrameHeight;++y)indices_[y]=neededRow(y)?index++:255;
  }
  // Source and caller storage must not overlap. A failed capture invalidates
  // the previous cache instead of silently reusing older pixels.
  bool capture(const uint8_t* gray,size_t bytes=kFrameBytes) {
    invalidate();
    if(!gray || bytes<kFrameBytes || !storage_ || capacity_<kRequiredBytes)return false;
    for(int y=0;y<appcfg::kFrameHeight;++y)if(indices_[y]!=255) {
      memcpy(storage_+static_cast<size_t>(indices_[y])*appcfg::kFrameWidth,
             gray+static_cast<size_t>(y)*appcfg::kFrameWidth,appcfg::kFrameWidth);
#if defined(FOOT_IMAGE_ROWS_TEST_READS) && FOOT_IMAGE_ROWS_TEST_READS
      ++readCounts().capture_rows;
#endif
    }
    ready_=true;return true;
  }
  // A view aliases caller storage and this index table. It remains usable
  // only until this cache is recaptured/invalidated or either owner expires.
  GrayRowView view()const{return ready_?GrayRowView(storage_,indices_):GrayRowView{};}
  void invalidate(){ready_=false;}
 private:
  uint8_t* storage_=nullptr;
  size_t capacity_=0;
  uint8_t indices_[appcfg::kFrameHeight];
  bool ready_=false;
};
}
