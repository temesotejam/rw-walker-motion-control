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
// Logical tracker/calibration coordinates remain 320x240. Native sensor bytes
// have a distinct shape and must enter through an explicitly mapping cache.
constexpr int kRawFrameWidth=appcfg::kFrameWidth;
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
constexpr int kRawFrameHeight=236,kRawYOrigin=-2;
#else
constexpr int kRawFrameHeight=appcfg::kFrameHeight,kRawYOrigin=0;
#endif
constexpr size_t kRawFrameBytes=static_cast<size_t>(kRawFrameWidth)*kRawFrameHeight;
constexpr int rawYForLogical(int logical_y) {return logical_y+kRawYOrigin;}
constexpr bool rawFrameBytesValid(size_t bytes) {
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  return bytes==kRawFrameBytes;
#else
  return bytes>=kRawFrameBytes;
#endif
}
// Display-only padding. Missing sensor rows never become tracker evidence.
inline uint8_t rawPixelOrZero(const uint8_t* raw,size_t bytes,int logical_x,int logical_y) {
  const int raw_y=rawYForLogical(logical_y);
  return raw && rawFrameBytesValid(bytes) && logical_x>=0 && logical_x<kRawFrameWidth &&
      logical_y>=0 && logical_y<appcfg::kFrameHeight && raw_y>=0 && raw_y<kRawFrameHeight
      ? raw[static_cast<size_t>(raw_y)*kRawFrameWidth+logical_x] : 0;
}
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
// Display-only 2:1 sampling in logical coordinates. Validate the two buffers
// once, then map each row once instead of checking all 19,200 output pixels.
// Source and destination must not overlap; invalid inputs leave output intact.
inline bool copyNativeThumbnail(const uint8_t* raw,size_t bytes,uint8_t* output,size_t capacity) {
  constexpr size_t width=appcfg::kFrameWidth/2,height=appcfg::kFrameHeight/2;
  if(!raw || !rawFrameBytesValid(bytes) || !output || capacity<width*height)return false;
  for(size_t y=0;y<height;++y) {
    uint8_t* destination=output+y*width;
    const int raw_y=rawYForLogical(static_cast<int>(y)*2);
    if(raw_y<0 || raw_y>=kRawFrameHeight)memset(destination,0,width);
    else {
      const uint8_t* source=raw+static_cast<size_t>(raw_y)*kRawFrameWidth;
      for(size_t x=0;x<width;++x)destination[x]=source[x*2];
    }
  }
  return true;
}
#endif
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
constexpr bool requiredRawRowsValid(int y=0) {
  return y>=appcfg::kFrameHeight || ((!neededRow(y) ||
      (rawYForLogical(y)>=0 && rawYForLogical(y)<kRawFrameHeight)) && requiredRawRowsValid(y+1));
}
static_assert(requiredRawRowsValid(),"Every requested logical row exists in raw sensor geometry");
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
class LowerRowCache;
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
  friend class LowerRowCache;
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
  bool capture(const uint8_t* gray,size_t bytes=kRawFrameBytes) {
    invalidate();
    if(!gray || !rawFrameBytesValid(bytes) || !storage_ || capacity_<kRequiredBytes)return false;
    for(int y=0;y<appcfg::kFrameHeight;++y)if(indices_[y]!=255) {
      memcpy(storage_+static_cast<size_t>(indices_[y])*appcfg::kFrameWidth,
             gray+static_cast<size_t>(rawYForLogical(y))*kRawFrameWidth,appcfg::kFrameWidth);
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

namespace foot_image_rows {
constexpr bool upperNeededRow(int y) {
  return y>=0 && y<appcfg::kFrameHeight &&
      (requestedBy(y,appcfg::kWhiteMarkerARows,appcfg::kWhiteMarkerRowCount) ||
       requestedBy(y,appcfg::kWhiteReferenceARows,appcfg::kWhiteReferenceRowCount));
}
constexpr unsigned upperCount(int y=0) {
  return y>=appcfg::kFrameHeight?0:(upperNeededRow(y)?1:0)+upperCount(y+1);
}
constexpr unsigned upperLast(int y=appcfg::kFrameHeight-1) {
  return y<0?0:(upperNeededRow(y)?y:upperLast(y-1));
}
constexpr unsigned kUpperRows=upperCount();
constexpr unsigned kUpperBytes=kUpperRows*appcfg::kFrameWidth;
constexpr unsigned kUpperPrefixBytes=(rawYForLogical(upperLast())+1)*kRawFrameWidth;
static_assert(kUpperRows>0 && kUpperRows<255,"Upper row indices fit");
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
// Build the sparse index table at compile time. Acquisition callbacks create
// scratch caches repeatedly; they must not rediscover template membership.
constexpr unsigned upperCountBefore(int y) {
  return y<=0?0:upperCountBefore(y-1)+(upperNeededRow(y-1)?1:0);
}
template<unsigned... Y> struct UpperRowSequence {};
template<unsigned N,unsigned... Y> struct MakeUpperRowSequence:MakeUpperRowSequence<N-1,N-1,Y...> {};
template<unsigned... Y> struct MakeUpperRowSequence<0,Y...> {using Type=UpperRowSequence<Y...>;};
struct UpperIndexPlan {uint8_t indices[appcfg::kFrameHeight];};
template<unsigned... Y> constexpr UpperIndexPlan makeUpperIndexPlan(UpperRowSequence<Y...>) {
  return {{static_cast<uint8_t>(upperNeededRow(Y)?upperCountBefore(Y):255)...}};
}
constexpr UpperIndexPlan kUpperIndexPlan=makeUpperIndexPlan(
    MakeUpperRowSequence<appcfg::kFrameHeight>::Type{});
#endif
class UpperRowCache {
 public:
  UpperRowCache(uint8_t* storage=nullptr,size_t capacity=0):storage_(storage),capacity_(capacity) {
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    memcpy(indices_,kUpperIndexPlan.indices,sizeof(indices_));
#else
    uint8_t index=0;
    for(int y=0;y<appcfg::kFrameHeight;++y)indices_[y]=upperNeededRow(y)?index++:255;
#endif
  }
  bool capture(const uint8_t* gray,size_t prefix_bytes) {
    ready_=false;
    if(!gray || !storage_ || capacity_<kUpperBytes || prefix_bytes<kUpperPrefixBytes ||
       prefix_bytes>kRawFrameBytes)return false;
    for(int y=0;y<appcfg::kFrameHeight;++y)if(indices_[y]!=255)
      memcpy(storage_+size_t(indices_[y])*appcfg::kFrameWidth,
             gray+size_t(rawYForLogical(y))*kRawFrameWidth,appcfg::kFrameWidth);
    ready_=true;return true;
  }
  // Copy a packed mailbox to consumer scratch. Source and storage must not alias.
  bool capture(const GrayRowView& gray) {
    ready_=false;
    if(!gray.valid() || !storage_ || capacity_<kUpperBytes)return false;
    for(int y=0;y<appcfg::kFrameHeight;++y)if(indices_[y]!=255 && !gray.row(y))return false;
    for(int y=0;y<appcfg::kFrameHeight;++y)if(indices_[y]!=255)
      memcpy(storage_+size_t(indices_[y])*appcfg::kFrameWidth,gray.row(y),appcfg::kFrameWidth);
    ready_=true;return true;
  }
  GrayRowView view()const{return ready_?GrayRowView(storage_,indices_):GrayRowView{};}
 private:
  uint8_t* storage_=nullptr;
  size_t capacity_=0;
  uint8_t indices_[appcfg::kFrameHeight];
  bool ready_=false;
};
}

namespace foot_image_rows {
constexpr bool lowerNeededRow(int y) {
  return y>=0 && y<appcfg::kFrameHeight &&
      (requestedBy(y,appcfg::kWhiteMarkerBRows,appcfg::kWhiteMarkerRowCount) ||
       requestedBy(y,appcfg::kWhiteReferenceBRows,appcfg::kWhiteReferenceRowCount));
}
constexpr unsigned lowerCount(int y=0) {
  return y>=appcfg::kFrameHeight?0:(lowerNeededRow(y)?1:0)+lowerCount(y+1);
}
constexpr unsigned kLowerRows=lowerCount();
constexpr unsigned kLowerBytes=kLowerRows*appcfg::kFrameWidth;
// C++11 constexpr generation: no first-use initialization, heap allocation, or
// runtime membership search in either the HAL producer or consumer callback.
constexpr int lowerLogicalRow(unsigned index,int y=0) {
  return y>=appcfg::kFrameHeight?appcfg::kFrameHeight:
      lowerNeededRow(y)?(index?lowerLogicalRow(index-1,y+1):y):lowerLogicalRow(index,y+1);
}
template<unsigned... I> struct LowerRowSequence {};
template<unsigned N,unsigned... I> struct MakeLowerRowSequence:MakeLowerRowSequence<N-1,N-1,I...> {};
template<unsigned... I> struct MakeLowerRowSequence<0,I...> {using Type=LowerRowSequence<I...>;};
struct LowerLogicalPlan {uint8_t logical[kLowerRows];};
template<unsigned... I> constexpr LowerLogicalPlan makeLowerLogicalPlan(LowerRowSequence<I...>) {
  return {{static_cast<uint8_t>(lowerLogicalRow(I))...}};
}
constexpr LowerLogicalPlan kLowerLogicalPlan=makeLowerLogicalPlan(MakeLowerRowSequence<kLowerRows>::Type{});
constexpr bool lowerPlanValid(unsigned index=0) {
  return index>=kLowerRows || (lowerNeededRow(kLowerLogicalPlan.logical[index]) &&
      (!index || kLowerLogicalPlan.logical[index]>kLowerLogicalPlan.logical[index-1]) &&
      lowerPlanValid(index+1));
}
static_assert(lowerPlanValid(),"Complete ordered lower row plan comes from tracker configuration");
constexpr int kLowerMilestones[]={174,190,206,222};
constexpr size_t lowerPrefixThrough(int logical_y) {
  return logical_y>=0 && logical_y<appcfg::kFrameHeight &&
      rawYForLogical(logical_y)>=0 && rawYForLogical(logical_y)<kRawFrameHeight
      ?size_t(rawYForLogical(logical_y)+1)*kRawFrameWidth:0;
}
constexpr int lowerMilestoneForPrefix(size_t bytes) {
  return bytes>kRawFrameBytes?-1:
      bytes>=lowerPrefixThrough(222)?222:
      bytes>=lowerPrefixThrough(206)?206:
      bytes>=lowerPrefixThrough(190)?190:
      bytes>=lowerPrefixThrough(174)?174:-1;
}
static_assert(kLowerRows==40 && kLowerBytes==12800,"Complete lower search row storage");
static_assert(lowerNeededRow(174) && lowerNeededRow(190) &&
              lowerNeededRow(206) && lowerNeededRow(222),"Stages end on required rows");

// The caller owns storage. Copies are cumulative, never a mixture of cached
// earlier pixels and later arrivals. Source and destination must not overlap.
// A view aliases this object's index table and storage until the next capture.
class LowerRowCache {
 public:
  LowerRowCache(uint8_t* storage=nullptr,size_t capacity=0):storage_(storage),capacity_(capacity) {
    invalidate();
  }
  bool capture(const uint8_t* raw,size_t prefix_bytes) {
    invalidate();
    if(!raw || !storage_ || capacity_<kLowerBytes || prefix_bytes>kRawFrameBytes)return false;
    const size_t complete_rows=prefix_bytes/kRawFrameWidth;
    uint8_t index=0;
    for(unsigned i=0;i<kLowerRows;++i) {
      const int y=kLowerLogicalPlan.logical[i],raw_y=rawYForLogical(y);
      if(raw_y<0)continue;
      if(size_t(raw_y)>=complete_rows)break;
      memcpy(storage_+size_t(index)*appcfg::kFrameWidth,
             raw+size_t(raw_y)*kRawFrameWidth,appcfg::kFrameWidth);
      indices_[y]=index++;max_y_=y;
    }
    ready_=index!=0;
    return ready_;
  }
  // Missing required rows fail before any pixels are copied. Future rows are
  // hidden even when the source view contains them, so staged work cannot see
  // pixels outside its declared complete prefix.
  bool capture(const GrayRowView& rows,int max_logical_y) {
    invalidate();
    if(!rows.valid() || !storage_ || capacity_<kLowerBytes ||
        max_logical_y<0 || max_logical_y>=appcfg::kFrameHeight)return false;
    unsigned count=0;
    for(unsigned i=0;i<kLowerRows && kLowerLogicalPlan.logical[i]<=max_logical_y;++i) {
      const int y=kLowerLogicalPlan.logical[i];
      if(!rows.row(y))return false;
      ++count;
    }
    if(!count)return false;
    for(unsigned i=0;i<count;++i) {
      const int y=kLowerLogicalPlan.logical[i];
      memcpy(storage_+size_t(i)*appcfg::kFrameWidth,rows.row(y),appcfg::kFrameWidth);
      indices_[y]=static_cast<uint8_t>(i);max_y_=y;
    }
    ready_=true;return true;
  }
  GrayRowView view()const{return ready_?GrayRowView(storage_,indices_):GrayRowView{};}
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
  // Only a stable, immutable prefix of the SAME stamped frame may be extended.
  // New stamps or scratch invalidation force every required row to be copied.
  bool extend(const uint8_t* raw,size_t prefix_bytes,uint64_t stamp) {
    if(!raw || !storage_ || capacity_<kLowerBytes || !stamp || prefix_bytes>kRawFrameBytes) {
      invalidate();return false;
    }
    const size_t complete=prefix_bytes/kRawFrameWidth;
    unsigned count=0;
    while(count<kLowerRows && size_t(rawYForLogical(kLowerLogicalPlan.logical[count]))<complete)++count;
    if(!admitExtension(stamp,count))return false;
    for(unsigned i=copied_rows_;i<count;++i) {
      const int y=kLowerLogicalPlan.logical[i];
      memcpy(storage_+size_t(i)*appcfg::kFrameWidth,
             raw+size_t(rawYForLogical(y))*kRawFrameWidth,appcfg::kFrameWidth);
      indices_[y]=static_cast<uint8_t>(i);
    }
    finishExtension(count);return true;
  }
  bool extend(const GrayRowView& rows,int maximum,uint64_t stamp) {
    if(!rows.valid() || !storage_ || capacity_<kLowerBytes || !stamp ||
       maximum<0 || maximum>=appcfg::kFrameHeight) {invalidate();return false;}
    unsigned count=0;
    while(count<kLowerRows && kLowerLogicalPlan.logical[count]<=maximum) {
      // Validate the entire advertised prefix before retaining prior pixels.
      if(!rows.row(kLowerLogicalPlan.logical[count])){invalidate();return false;}
      ++count;
    }
    if(!admitExtension(stamp,count))return false;
    for(unsigned i=copied_rows_;i<count;++i) {
      const int y=kLowerLogicalPlan.logical[i];
      memcpy(storage_+size_t(i)*appcfg::kFrameWidth,rows.row(y),appcfg::kFrameWidth);
      indices_[y]=static_cast<uint8_t>(i);
    }
    finishExtension(count);return true;
  }
  unsigned lastCopiedBytes()const{return copied_bytes_;}
#endif
  int maxLogicalY()const{return ready_?max_y_:-1;}
  void invalidate() {
    ready_=false;max_y_=-1;
    memset(indices_,255,sizeof(indices_));
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
    frame_stamp_=0;copied_rows_=0;copied_bytes_=0;
#endif
  }
 private:
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
  bool admitExtension(uint64_t stamp,unsigned count) {
    if(!count || (frame_stamp_ && stamp<frame_stamp_)) {invalidate();return false;}
    if(stamp!=frame_stamp_){invalidate();frame_stamp_=stamp;}
    if(count<copied_rows_){invalidate();return false;}
    copied_bytes_=(count-copied_rows_)*appcfg::kFrameWidth;
    return true;
  }
  void finishExtension(unsigned count) {
    copied_rows_=count;max_y_=kLowerLogicalPlan.logical[count-1];ready_=true;
  }
  uint64_t frame_stamp_=0;
  unsigned copied_rows_=0,copied_bytes_=0;
#endif
  uint8_t* storage_=nullptr;
  size_t capacity_=0;
  uint8_t indices_[appcfg::kFrameHeight];
  int max_y_=-1;
  bool ready_=false;
};
}
