#pragma once
#include <cstdint>

// Both factors are ±1, so this factor converts logical excitation to the
// Roller register and converts its signed current reading back to excitation.
constexpr int walkingCurrentPolarity(int normal_output_sign, int driver_polarity) {
  return normal_output_sign * driver_polarity;
}
