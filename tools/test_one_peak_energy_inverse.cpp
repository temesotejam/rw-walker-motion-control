#include <cassert>
#include <cmath>
#include <cstdio>
#include "../src/one_peak_energy_inverse.h"

int main() {
  using namespace one_peak_energy_inverse;
  const State state{69.2992f, 234.77f, 7.58116f, 7250.f, 1.932539f};
  const auto predicted = predict(state, 25);
  assert(predicted.valid);
  assert(std::fabs(predicted.energy_mj - 1.1042532603f) < 0.000002f);
  assert(std::fabs(predicted.work_change_mj -
      (predicted.on_mj + predicted.after_mj - predict(state, 0).after_mj)) < 0.000002f);
  const auto target8 = choose(state, 0.7334225991f, 0.f);
  assert(target8.valid && target8.width_ms == 25 && target8.below_reachable);
  const auto target105 = choose(state, 1.1443124619f, 0.f);
  assert(target105.valid && target105.width_ms == 20);
  const auto extended = choose(state, 0.7334225991f, 0.f, 1);
  assert(extended.valid && extended.width_ms == 26 && extended.extrapolated);
  const auto protected_choice = choose(state, 0.7334225991f, 0.f, 5, true);
  assert(protected_choice.width_ms == 0);
  // target - measured is negative when the observed swing exceeds target.
  assert(nextAdaptiveExtraMs(0, true, 10.4f, -2.4f, 25) == 1);
  assert(nextAdaptiveExtraMs(1, true, 10.4f, -2.4f, 26) == 2);
  assert(nextAdaptiveExtraMs(5, true, 10.4f, -2.4f, 30) == 5);
  assert(nextAdaptiveExtraMs(2, true, 7.0f, 1.0f, 27) == 1);
  assert(nextAdaptiveExtraMs(2, false, 10.4f, -2.4f, 27) == 0);
  assert(nextAdaptiveExtraMs(2, true, 4.9f, -2.4f, 27) == 0);
  State recaptured_reference = state;
  recaptured_reference.reference_abs_pitch_deg = 1.55f;
  assert(choose(recaptured_reference, 0.7334225991f, 0.f).valid);
  recaptured_reference.reference_abs_pitch_deg = NAN;
  assert(!choose(recaptured_reference, 0.7334225991f, 0.f).valid);
  State low_voltage = state;
  low_voltage.battery_mv = 6100.f;
  assert(!choose(low_voltage, 0.7334225991f, 0.f).valid);
  std::puts("One-peak firmware inverse and support guards PASS");
}
