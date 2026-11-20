// Compile against either the reference or current support_transfer_plan.h.
#include "support_transfer_plan.h"
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace st = support_transfer_plan;
struct Frame { uint32_t peak, packet, gate; st::Input in; };
struct Receiving { size_t index; st::Progress progress; };

static float number() {
  std::string field;
  std::cin >> field;
  return std::stof(field);
}

template<typename F> double elapsedUs(F solve) {
  using Clock = std::chrono::steady_clock;
  constexpr int repeats=32;
  const auto start=Clock::now();
  for (int j=0;j<repeats;++j) {
    solve();
    std::atomic_signal_fence(std::memory_order_seq_cst);
  }
  return std::chrono::duration<double,std::micro>(Clock::now()-start).count()/repeats;
}

int main() {
  lateral_energy::CurrentTable current;
  if (!current.prepare(number())) return 1;
  std::vector<Frame> frames;
  Frame f{};
  while (std::cin>>f.peak>>f.packet>>f.gate>>f.in.side) {
    auto& in=f.in;
    in.angle_deg=number(); in.rate_dps=number(); in.accel_dps2=number(); in.fore_aft_deg=number();
    in.left_deg=number(); in.right_deg=number(); in.left_rate_dps=number(); in.right_rate_dps=number();
    in.initial_current_ma=number(); in.goal_current_ma=number();
    if (!(std::cin>>in.cap_ms)) return 1;
    frames.push_back(f);
  }
  if (frames.empty()) return 1;
  std::vector<st::Decision> result(frames.size());
  std::vector<Receiving> receiving;
  auto trace=[&](bool save) {
    st::Progress progress{};
    for (size_t k=0;k<frames.size();++k) {
      const auto& row=frames[k];
      progress.observe(row.peak,row.packet,row.in,(row.gate&7u)==0);
      auto& d=result[k];
      d=(row.gate&~256u) ? st::Decision{} : st::select(current,row.in,progress);
      if (save && d.phase==st::Phase::Receiving) receiving.push_back({k,progress});
    }
  };
  trace(true);
  std::vector<st::Decision> cached(receiving.size());
  auto selectReceiving=[&] {
    for (size_t j=0;j<receiving.size();++j) {
      const auto& entry=receiving[j];
      cached[j]=st::select(current,frames[entry.index].in,entry.progress);
    }
  };
  selectReceiving();
  const auto all_us=elapsedUs([&]{trace(false);});
  const auto receiving_us=elapsedUs(selectReceiving);
  std::cout<<std::setprecision(9)<<all_us<<' '<<receiving_us<<' '<<receiving.size()<<'\n';
  for (const auto& d:result)
    std::cout<<int(d.reason)<<' '<<int(d.phase)<<' '<<int(d.anchor)<<' '<<d.side<<' '
             <<d.delay_ms<<' '<<d.width_ms<<' '<<d.work_supported<<' '
             <<d.transfer_ms<<' '<<d.transfer_deg<<' '<<d.terminal_deg<<' '<<d.terminal_ms<<' '
             <<d.coast_margin_j<<' '<<d.margin_j<<' '<<d.work_j<<'\n';
  for (size_t j=0;j<cached.size();++j)
    if (cached[j].reason!=result[receiving[j].index].reason) return 1;
}
