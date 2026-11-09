#pragma once
#include <cmath>
#include <cstdint>

// One bounded identification probe, not an identified motion model.
// Spans are camera relative, not ground stride; no outcome is persisted.
namespace positive_sway_probe {
class Controller {
 public:
  void reset(){*this=Controller{};}
  bool active()const{return active_;}
  bool attempted()const{return attempted_;}
  uint32_t lastApplied()const{return last_applied_;}
  bool effectiveCommand(uint32_t ms)const {for(unsigned i=0;i<8;++i)if(ms && commands_[i]==ms)return true;return false;}
  void interrupt(){if(active_)active_=false;count_=0;next_=0;}
  void cycle(uint32_t start,uint32_t end,float left,float right,float plus,float minus,
             bool valid,bool healthy,bool neutral,bool effective) {
    if(!healthy){interrupt();return;}
    if(!start || end<=start || !valid || !std::isfinite(left) || !std::isfinite(right) ||
       !std::isfinite(plus) || !std::isfinite(minus) || left<0 || right<0 || plus<=0 || minus<=0) {
      interrupt();return;
    }
    if(active_) {
      // Overlapping samples cannot measure response. Invalid/ineffective
      // post-change samples interrupt, never become missing zeroes.
      if(start<=started_)return;
      if(!effective){interrupt();return;}
      if(response_end_ && start!=response_end_){interrupt();return;}
      if(plus>9.5f || minus>9.5f || plus<ref_plus_*.65f || minus<ref_minus_*.65f ||
         left<ref_left_*.4f || right<ref_right_*.4f) {interrupt();return;}
      response_end_=end;
      if(++responses_>=2)interrupt(); // always restore; observational trial only
      return;
    }
    if(attempted_ || !neutral || (last_end_ && start!=last_end_)){count_=0;next_=0;}
    if(attempted_ || !neutral){last_end_=end;return;}
    if(plus>=6.f || minus<4.5f || minus>8.5f || minus-plus<1.f || left<.2f || right<.2f) {
      count_=0;next_=0;last_end_=end;return;
    }
    plus_[next_]=plus;minus_[next_]=minus;left_[next_]=left;right_[next_]=right;next_=(next_+1)%4;
    if(count_<4)++count_;
    ref_plus_=ref_minus_=ref_left_=ref_right_=0;
    for(unsigned i=0;i<count_;++i){ref_plus_+=plus_[i];ref_minus_+=minus_[i];ref_left_+=left_[i];ref_right_+=right_[i];}
    ref_plus_/=count_;ref_minus_/=count_;ref_left_/=count_;ref_right_/=count_;last_end_=end;
  }
  bool candidate(uint32_t now,bool healthy,bool foot_idle)const {
    return healthy && foot_idle && !active_ && !attempted_ && count_>=4 &&
      now>=last_end_ && now-last_end_<=1500;
  }
  bool request(uint32_t now,bool healthy,bool foot_idle) {
    if(active_ && (!healthy || !foot_idle || !now || now<started_ || now-started_>3000))interrupt();
    return active_ || candidate(now,healthy,foot_idle);
  }
  void confirm(uint32_t now,uint16_t baseline,uint16_t actual,bool accepted,bool start) {
    if(!accepted || !now || actual<=baseline || actual-baseline>5){if(active_)interrupt();return;}
    if(!active_) {if(!start || attempted_)return;attempted_=true;active_=true;started_=now;responses_=0;response_end_=0;}
    last_applied_=now;commands_[command_next_]=now;command_next_=(command_next_+1)%8;
  }
 private:
  uint32_t last_end_=0,started_=0,last_applied_=0,response_end_=0;
  unsigned count_=0,responses_=0,next_=0,command_next_=0;
  uint32_t commands_[8]={};float plus_[4]={},minus_[4]={},left_[4]={},right_[4]={};
  float ref_plus_=0,ref_minus_=0,ref_left_=0,ref_right_=0;
  bool active_=false,attempted_=false;
};
}
