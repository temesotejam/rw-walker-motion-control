#include "gait_turn_run.h"
#include "foot_fast_policy.h"
#include "psram_string.h"
#include <cassert>
#include <iostream>

void exportRun(const GaitTurnRun& run, size_t prefix = 0) {
  PsramString json;
  assert(json.reserve(foot_fast_policy::kMetadataHardBudgetBytes));
  json += "{\"start\":0";
  if (prefix) { const std::string spaces(prefix, ' '); json.append(spaces.data(), spaces.size()); }
  run.append(json);
  json += "}";
  assert(json.ok());
  // Omit padding from the serialized test output.
  for (const char* c = json.c_str(); *c; ++c) if (*c != ' ') std::cout << *c;
  std::cout << '\n';
}

void cycles(GaitTurnRun& run, unsigned count) {
  constexpr uint64_t epoch = 2300000;
  run.peak(1, 100, 0);
  for (unsigned i = 0; i < count; ++i) {
    for (unsigned j = 1; j <= 24; ++j) {
      const uint32_t ms = 100 + i * 600 + j * 25;
      periodic_foot::Packet packet;
      packet.count = packet.left.sequence = packet.right.sequence = i * 24 + j;
      packet.epoch_us = epoch;
      packet.published_us = epoch + ms * 1000;
      packet.left.image_us = packet.right.image_us = packet.published_us - 10000;
      const float reach = i == 2 ? 2.5f : 8.f;
      packet.left.measured = j > 12 ? reach : 0.f;
      packet.right.measured = j <= 12 ? 1 : 0;
      run.observe(packet);
      if (j == 12 || j == 24) run.peak(j == 12 ? -1 : 1, ms, -.01f * (ms - 100));
    }
  }
}

int main() {
  GaitTurnRun run;
  run.begin(ps_malloc);
  run.start(false);
  cycles(run, 2);
  assert(!run.status().enabled && !run.status().confirmed_ms);
  exportRun(run);
  run.start(true);
  cycles(run, 3);
  assert(run.status().confirmed_ms == 1900 && run.status().confirmed_side == 1);
  run.peak(-1, 2200, -21); // Next cycle has no camera observations.
  run.peak(1, 2500, -24);
  assert(run.status().latest.reason == gait_turn::Reason::Coverage && run.status().confirmed_ms == 1900);
  exportRun(run);
  exportRun(run, foot_fast_policy::kSupportAuditEndBytes - 4096);
  run.start(false);
  assert(!run.status().confirmed_ms && !run.status().latest.end_ms);
  exportRun(run);
  GaitTurnRun unavailable;
  unavailable.begin([](size_t) -> void* { return nullptr; });
  unavailable.start(true);
  cycles(unavailable, 2);
  assert(unavailable.status().confirmed_side == 1);
  exportRun(unavailable);
}
