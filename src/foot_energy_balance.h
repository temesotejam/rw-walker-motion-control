#pragma once
#include <cmath>
#include <cstdint>

// Experimental identification within normal 8-degree walking. The quantity is
// camera-frame peak-to-peak motion, not ground step length or contact.
namespace foot_energy_balance {
class Controller {
 public:
  void reset(uint16_t id) {*this=Controller{};run_id_=id;}
  __attribute__((noinline)) void frame(uint16_t id,uint32_t ms,bool lv,bool rv,float l,float r) {
    if(id!=run_id_)return;
    Frame& f=frames_[next_];f.ms=ms;f.l=l;f.r=r;f.lv=lv&&std::isfinite(l)&&std::fabs(l)<=80.f;f.rv=rv&&std::isfinite(r)&&std::fabs(r)<=80.f;
    next_=(next_+1)%32;if(count_<32)++count_;
    if(pending_ && ms>=end_) {finishCycle();pending_=false;}
  }
  __attribute__((noinline)) void peak(int side,uint32_t ms) {
    if(side!=-1)return;
    if(have_boundary_ && ms>boundary_ && ms-boundary_>=250 && ms-boundary_<=2000) {
      start_=boundary_;end_=ms;pending_=true;
      if(count_ && frames_[(next_+31)%32].ms>=end_) {finishCycle();pending_=false;}
    }
    boundary_=ms;have_boundary_=true;
  }
  // Called once per zero crossing, including rejected/no-output decisions.
  __attribute__((noinline)) void decision(uint32_t ms,int side,bool executed,float charge,float rpm,float vbat) {
    if(side!=1 && side!=-1)return;
    executed=executed && std::isfinite(rpm) && std::isfinite(vbat) && vbat>0;
    sides_[support_next_]=side;supports_[support_next_]=executed;
    support_next_=(support_next_+1)%8;if(support_count_<8)++support_count_;
    if(phase_>=1 && phase_<=4 && !executed) {abort(3);return;}
    if(phase_<1 || phase_>3 || !settled_ || ms<segment_ms_+1000)return;
    const unsigned i=side>0?0u:1u;
    ++attempts_[i];
    if(executed && std::isfinite(charge)) {++outputs_[i];charge_[i]+=std::fabs(charge);rpm_sum_[i]+=rpm;vbat_sum_+=vbat;}
  }
  __attribute__((noinline)) void update(uint32_t ms,float stage) {
    if(!std::isfinite(stage))return;
    if(!have_stage_ || std::fabs(stage-stage_)>.01f) {
      if(have_stage_ && phase_>0) abort(2); // cancel when the baseline changes
      stage_=stage;stage_ms_=ms;have_stage_=true;
    }
    const bool fresh=cycle_time_ && ms>=cycle_time_ && ms-cycle_time_<=2000;
    valid_=fresh && cycle_count_>=2;
    if(phase_>0 && phase_<5 && !valid_) abort(1);
    if(phase_==0 && ms>=7000 && ms>=stage_ms_+3000 && valid_ && left_mean_+right_mean_>=3.f && supportReady()) {phase_=1;settled_=false;}
    float desired=0;
    if(phase_>=1 && phase_<=3) desired=phase_==2?.02f:-.02f;
    else if(phase_==4) desired=chosen_;
    if(!have_update_ || (ms>update_ms_ && ms-update_ms_>=1000)) {
      bias_+=std::fmax(-.01f,std::fmin(.01f,desired-bias_));
      update_ms_=ms;have_update_=true;
    }
    if(phase_>=1 && phase_<=3) {
      if(std::fabs(bias_-desired)<.0001f && !settled_) {
        settled_=true;segment_ms_=ms;resetSegment();seen_cycle_=cycle_seq_;
      }
      if(settled_ && cycle_seq_!=seen_cycle_) {
        seen_cycle_=cycle_seq_;
        if(cycle_start_>=segment_ms_+1000 && valid_) {err_sum_+=error_;++obs_;}
      }
      if(settled_ && obs_>=3 && ms>=segment_ms_+3000) {
        if(outputs_[0]<2 || outputs_[1]<2 || outputs_[0]!=attempts_[0] || outputs_[1]!=attempts_[1]) {abort(3);return;}
        const unsigned k=phase_-1;
        errs_[k]=err_sum_/obs_;qs_[k]=charge_[1]/outputs_[1]-charge_[0]/outputs_[0];
        rpms_[k][0]=rpm_sum_[0]/outputs_[0];rpms_[k][1]=rpm_sum_[1]/outputs_[1];
        vbats_[k]=vbat_sum_/(outputs_[0]+outputs_[1]);
        if(phase_<3) {++phase_;settled_=false;}
        else {
          const float d1=errs_[1]-errs_[0],d2=errs_[1]-errs_[2];
          const float q1=qs_[1]-qs_[0],q2=qs_[1]-qs_[2];
          // Both reversals must agree and cause an observable commanded charge change.
          if(std::fabs(rpms_[1][0]-rpms_[0][0])<=100 && std::fabs(rpms_[1][0]-rpms_[2][0])<=100 &&
             std::fabs(rpms_[1][1]-rpms_[0][1])<=100 && std::fabs(rpms_[1][1]-rpms_[2][1])<=100 &&
             std::fabs(vbats_[1]-vbats_[0])<=200 && std::fabs(vbats_[1]-vbats_[2])<=200 &&
             d1*d2>0 && std::fabs(d1)>=.04f && std::fabs(d2)>=.04f &&
             q1>=.3f && q2>=.3f && std::fabs(errs_[0]-errs_[2])<=.04f) {
            const float a=(errs_[0]+errs_[2])*.5f,b=errs_[1],center=(a+b)*.5f;
            const float ea=std::fabs(a),eb=std::fabs(b),ec=std::fabs(center);
            baseline_error_=ec;
            // The inferred neutral point must be worse by a meaningful margin;
            // never infer beyond the measured pair or force a choice near zero.
            if(ea+.04f<ec && ea<eb)chosen_=-.02f;
            else if(eb+.04f<ec && eb<ea)chosen_=.02f;
            else chosen_=0.f;
            if(chosen_!=0.f) {phase_=4;seen_cycle_=cycle_seq_;bad_=0;}
            else abort(4);
          } else abort(4);
          settled_=false;
        }
      }
    } else if(phase_==4) {
      // A selected candidate can be opposite the final A probe. Wait for the
      // slew and a complete settled cycle before judging that candidate.
      if(!settled_ && std::fabs(bias_-chosen_)<.0001f) {
        settled_=true;segment_ms_=ms;seen_cycle_=cycle_seq_;bad_=0;
      }
      if(settled_ && cycle_seq_!=seen_cycle_) {
        seen_cycle_=cycle_seq_;
        if(cycle_start_>=segment_ms_+1000) {
          if(std::fabs(error_)>baseline_error_+.08f) ++bad_; else bad_=0;
          if(bad_>=3) abort(5);
        }
      }
    }
  }
  float factor(int side)const{return side>0?1.f-bias_:1.f+bias_;}
  float bias()const{return bias_;}float leftMean()const{return cycle_count_?left_mean_:NAN;}
  float rightMean()const{return cycle_count_?right_mean_:NAN;}bool valid()const{return valid_;}
  uint8_t phase()const{return phase_;}uint8_t reason()const{return reason_;} // 0 waiting, 1..3 probes, 4 identified, 5 neutral fallback
 private:
  struct Frame {uint32_t ms=0;float l=NAN,r=NAN;bool lv=false,rv=false;};
  __attribute__((noinline)) void finishCycle() {
    float lo[2]={1e9f,1e9f},hi[2]={-1e9f,-1e9f};unsigned n[2]={};
    uint32_t first[2]={},last[2]={};float prior[2]={};bool gap[2]={};
    for(unsigned j=0;j<count_;++j) {
      const Frame& f=frames_[(next_+32-count_+j)%32];if(f.ms<start_||f.ms>end_)continue;
      for(unsigned s=0;s<2;++s) {
        const bool ok=s==0?f.lv:f.rv;const float x=s==0?f.l:f.r;if(!ok)continue;
        if(n[s] && (f.ms-last[s]>250 || f.ms<last[s] || std::fabs(x-prior[s])>30.f))gap[s]=true;
        prior[s]=x;
        if(!n[s])first[s]=f.ms;
        last[s]=f.ms;++n[s];
        lo[s]=std::fmin(lo[s],x);hi[s]=std::fmax(hi[s],x);
      }
    }
    const uint32_t edge=(end_-start_)/5<150?(end_-start_)/5:150;
    if(n[0]<5 || n[1]<5 || gap[0] || gap[1] || first[0]-start_>edge || first[1]-start_>edge || end_-last[0]>edge || end_-last[1]>edge) {
      cycle_count_=0;cycle_time_=0;return;
    }
    left_mean_=hi[0]-lo[0];right_mean_=hi[1]-lo[1];
    error_=(left_mean_-right_mean_)/(left_mean_+right_mean_+2.f);
    cycle_start_=start_;cycle_time_=end_;++cycle_seq_;++cycle_count_;
  }
  void abort(uint8_t reason){phase_=5;reason_=reason;settled_=false;}
  bool supportReady()const {
    if(support_count_<8)return false;
    unsigned positive=0,negative=0,good=0;
    for(unsigned i=0;i<8;++i)if(supports_[i]) {++good;if(sides_[i]>0)++positive;else ++negative;}
    return good==8 && positive>=3 && negative>=3;
  }
  void resetSegment(){obs_=0;err_sum_=0;vbat_sum_=0;for(unsigned i=0;i<2;++i){attempts_[i]=outputs_[i]=0;charge_[i]=rpm_sum_[i]=0;}}
  uint16_t run_id_=0;Frame frames_[32];unsigned next_=0,count_=0;
  uint32_t boundary_=0,start_=0,end_=0,cycle_time_=0,cycle_start_=0,cycle_seq_=0,seen_cycle_=0,cycle_count_=0;
  bool have_boundary_=false,pending_=false,valid_=false,have_stage_=false,have_update_=false,settled_=false;
  uint32_t stage_ms_=0,update_ms_=0,segment_ms_=0;float stage_=0,bias_=0,chosen_=0,baseline_error_=0;
  float left_mean_=NAN,right_mean_=NAN,error_=0,err_sum_=0,errs_[3]={},qs_[3]={},charge_[2]={};
  float rpm_sum_[2]={},rpms_[3][2]={{0,0},{0,0},{0,0}},vbats_[3]={},vbat_sum_=0;
  unsigned obs_=0,attempts_[2]={},outputs_[2]={},bad_=0;
  int sides_[8]={};bool supports_[8]={};unsigned support_next_=0,support_count_=0;uint8_t phase_=0,reason_=0;
};
}
