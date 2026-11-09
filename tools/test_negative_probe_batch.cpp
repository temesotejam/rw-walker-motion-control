#include <cassert>
#include <cstdint>
#include <initializer_list>
#include "negative_probe_batch.h"
#include "negative_probe_extended_batch.h"

int main() {
  static_assert(negative_probe_batch::kSelectorMs == 25, "batch selector changed");
  static_assert(negative_probe_batch::kDurationMs == 30000, "batch duration changed");
  const uint16_t expected[5] = {0, 10, 15, 20, 25};
  bool different_order_found = false;
  for (const uint32_t seed : {0U, 1U, 12345U, 0xffffffffU}) {
    for (uint16_t block = 0; block < 20; ++block) {
      uint8_t counts[5] = {};
      for (uint8_t pos = 0; pos < 5; ++pos) {
        const uint16_t index = block * 5 + pos;
        const uint16_t chosen = negative_probe_batch::width(seed, index);
        assert(chosen == negative_probe_batch::width(seed, index));
        for (uint8_t j = 0; j < 5; ++j)
          if (chosen == expected[j]) ++counts[j];
      }
      for (const auto count : counts) assert(count == 1);
    }
  }
  for (uint8_t i = 0; i < 5; ++i)
    if (negative_probe_batch::width(1, i) != negative_probe_batch::width(12345, i))
      different_order_found = true;
  assert(different_order_found);
  static_assert(negative_probe_extended_batch::kSelectorMs == 40, "extended selector changed");
  static_assert(negative_probe_extended_batch::kDurationMs == 30000, "extended duration changed");
  for (const uint32_t seed : {0U, 1U, 12345U, 0xffffffffU}) {
    for (uint16_t block = 0; block < 20; ++block) {
      uint8_t counts[3] = {};
      for (uint8_t pos = 0; pos < 3; ++pos) {
        const uint16_t chosen = negative_probe_extended_batch::width(seed, block * 3 + pos);
        assert(chosen == negative_probe_extended_batch::width(seed, block * 3 + pos));
        if (chosen == 30) ++counts[0];
        else if (chosen == 35) ++counts[1];
        else if (chosen == 40) ++counts[2];
        else assert(false);
      }
      for (const auto count : counts) assert(count == 1);
    }
  }
}
