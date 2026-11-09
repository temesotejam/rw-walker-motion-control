#include <algorithm>
#include <array>
#include <cassert>
#include "positive_current_batch.h"

int main() {
  static_assert(positive_current_batch::kNegativeWidthMs == 40);
  static_assert(positive_current_batch::kDurationMs == 30000);
  bool seed_changes_order = false;
  for (uint32_t seed : {1U, 42U, 12345U, 0xffffffffU}) {
    for (uint16_t block = 0; block < 100; ++block) {
      std::array<int16_t, 3> values{};
      for (uint16_t i = 0; i < 3; ++i) {
        values[i] = positive_current_batch::current(seed, block * 3 + i);
        assert(values[i] == positive_current_batch::current(seed, block * 3 + i));
        if (values[i] != positive_current_batch::current(1, block * 3 + i)) seed_changes_order = true;
      }
      std::sort(values.begin(), values.end());
      assert((values == std::array<int16_t, 3>{300, 350, 400}));
    }
  }
  assert(seed_changes_order);
}
