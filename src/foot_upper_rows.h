#pragma once
#include "foot_image_rows.h"

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
constexpr unsigned kUpperPrefixBytes=(upperLast()+1)*appcfg::kFrameWidth;
static_assert(kUpperRows>0 && kUpperRows<255,"Upper row indices fit");
class UpperRowCache {
 public:
  UpperRowCache(uint8_t* storage=nullptr,size_t capacity=0):storage_(storage),capacity_(capacity) {
    uint8_t index=0;
    for(int y=0;y<appcfg::kFrameHeight;++y)indices_[y]=upperNeededRow(y)?index++:255;
  }
  bool capture(const uint8_t* gray,size_t prefix_bytes) {
    ready_=false;
    if(!gray || !storage_ || capacity_<kUpperBytes || prefix_bytes<kUpperPrefixBytes ||
       prefix_bytes>kFrameBytes)return false;
    for(int y=0;y<appcfg::kFrameHeight;++y)if(indices_[y]!=255)
      memcpy(storage_+size_t(indices_[y])*appcfg::kFrameWidth,
             gray+size_t(y)*appcfg::kFrameWidth,appcfg::kFrameWidth);
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
