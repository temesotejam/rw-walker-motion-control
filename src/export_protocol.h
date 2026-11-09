#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace export_protocol {
constexpr uint32_t kChunkBytes = 4096;
constexpr uint32_t kChunkMagic = 0x31484346; // FCH1, little endian
inline uint32_t crc32(uint32_t crc, const uint8_t* data, size_t length) {
  crc = ~crc;
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}
struct Span { const uint8_t* data; size_t length; };
inline bool validRange(size_t total, size_t offset, size_t length, size_t limit) {
  return length && length <= limit && offset <= total && length <= total - offset;
}
inline bool copy(const Span* spans, size_t count, size_t total, size_t offset,
                 uint8_t* out, size_t length) {
  if (!out || !validRange(total, offset, length, kChunkBytes)) return false;
  for (size_t i = 0; i < count && length; ++i) {
    if (offset >= spans[i].length) { offset -= spans[i].length; continue; }
    const size_t n = length < spans[i].length - offset ? length : spans[i].length - offset;
    memcpy(out, spans[i].data + offset, n); out += n; length -= n; offset = 0;
  }
  return length == 0;
}
} // namespace export_protocol
