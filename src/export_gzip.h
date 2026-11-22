#pragma once
#include <algorithm>
#include <esp32s3/rom/miniz.h>
#include "export_protocol.h"

namespace export_gzip {
// One gzip member over ordinary FCH1 packets; zero requests an identity response.
// The caller owns all workspace. The compressed result must reduce wire bytes.
template <class Fill>
size_t encode(tdefl_compressor& codec, uint8_t* encoded, size_t capacity, uint8_t* packet,
              uint32_t offset, uint32_t length, Fill fill) {
  if (!length || length > export_protocol::kStreamChunks * export_protocol::kChunkBytes ||
      offset > UINT32_MAX - length || !encoded || !packet || capacity <= 18) return 0;
  const uint32_t plain = export_protocol::packetBytes(length);
  const uint8_t gzip[] = {0x1f, 0x8b, 8, 0, 0, 0, 0, 0, 4, 255};
  memcpy(encoded, gzip, sizeof(gzip));
  size_t used = sizeof(gzip);
  uint32_t checksum = 0;
  if (tdefl_init(&codec, nullptr, nullptr, 1) != TDEFL_STATUS_OKAY) return 0;
  for (uint32_t done = 0; done < length;) {
    const uint32_t n = std::min(export_protocol::kChunkBytes, length - done);
    if (!fill(offset + done, n)) return 0;
    size_t input = n + 16, output = capacity - 8 - used;
    if (!output) return 0;
    checksum = export_protocol::crc32(checksum, packet, input);
    done += n;
    // Flush packet boundaries so the browser can persist full packets from a truncated response.
    const bool final = done == length;
    const auto status = tdefl_compress(&codec, packet, &input, encoded + used, &output,
                                       final ? TDEFL_FINISH : TDEFL_SYNC_FLUSH);
    if (input != n + 16 || status != (final ? TDEFL_STATUS_DONE : TDEFL_STATUS_OKAY)) return 0;
    used += output;
  }
  if (used + 8 >= plain) return 0;
  const uint32_t trailer[] = {checksum, plain};  // gzip trailer is little endian, as is ESP32-S3
  memcpy(encoded + used, trailer, sizeof(trailer));
  return used + sizeof(trailer);
}
} // namespace export_gzip
