#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Two slots in RTC RAM, not flash. Each journal has exactly one writer. Only the
// next boot reads it; USB prints a DRAM copy captured before either journal is
// changed. A reset during a write must leave the preceding sample readable.
namespace diagnostic_journal {
inline uint32_t crc32(const void* data, size_t size, uint32_t crc = 0) {
  crc = ~crc;
  const auto* bytes = static_cast<const uint8_t*>(data);
  while (size--) {
    crc ^= *bytes++;
    for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}
template<class T, uint32_t Magic> struct Journal {
  struct Slot { uint32_t magic, sequence; T value; uint32_t crc; } slots[2];
  static uint32_t checksum(const Slot& s) {
    return crc32(&s.value, sizeof(T), crc32(&s.sequence, sizeof(s.sequence)));
  }
  static Slot copy(const volatile Slot& from) {
    Slot to;
    auto* out = reinterpret_cast<uint8_t*>(&to);
    const auto* in = reinterpret_cast<const volatile uint8_t*>(&from);
    for (size_t i = 0; i < sizeof(Slot); ++i) out[i] = in[i];
    return to;
  }
  int latest(T& out, uint32_t* sequence = nullptr) const volatile {
    const Slot a = copy(slots[0]), b = copy(slots[1]);
    const bool av = a.magic == Magic && a.crc == checksum(a);
    const bool bv = b.magic == Magic && b.crc == checksum(b);
    if (!av && !bv) return -1;
    const int index = bv && (!av || static_cast<int32_t>(b.sequence - a.sequence) > 0) ? 1 : 0;
    const Slot& s = index ? b : a;
    out = s.value;
    if (sequence) *sequence = s.sequence;
    return index;
  }
  void clear() volatile {
    slots[0].magic = 0; slots[1].magic = 0;
    __sync_synchronize();
  }
  void save(const T& value) volatile {
    T previous;
    uint32_t sequence = 0;
    const int old = latest(previous, &sequence);
    volatile Slot& target = slots[old == 0 ? 1 : 0];
    Slot next{};
    next.sequence = sequence + 1; next.value = value; next.crc = checksum(next);
    target.magic = 0;
    __sync_synchronize();
    target.sequence = next.sequence;
    auto* out = reinterpret_cast<volatile uint8_t*>(&target.value);
    const auto* in = reinterpret_cast<const uint8_t*>(&next.value);
    for (size_t i = 0; i < sizeof(T); ++i) out[i] = in[i];
    target.crc = next.crc;
    __sync_synchronize();
    target.magic = Magic;
    __sync_synchronize();
  }
};
}
