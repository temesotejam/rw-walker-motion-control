#pragma once
#include "foot_image_rows.h"

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
  int maxLogicalY()const{return ready_?max_y_:-1;}
  void invalidate() {
    ready_=false;max_y_=-1;
    memset(indices_,255,sizeof(indices_));
  }
 private:
  uint8_t* storage_=nullptr;
  size_t capacity_=0;
  uint8_t indices_[appcfg::kFrameHeight];
  int max_y_=-1;
  bool ready_=false;
};
}
