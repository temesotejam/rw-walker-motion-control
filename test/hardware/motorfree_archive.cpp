// Included only in the isolated, motor-free fixture by prepare_motorfree.py.
#include <Arduino.h>
#include <esp32s3/rom/miniz.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <memory>
#include "archive_layout.h"
#include "export_protocol.h"
#include "psram_logger.h"

extern "C" const uint8_t replay_blob_start[], replay_blob_end[];

template<class Consume>
static bool unpack(uint32_t start, uint32_t end, size_t expected, uint32_t crc, Consume consume) {
  using Allocation = std::unique_ptr<void, decltype(&free)>;
  Allocation state(ps_malloc(sizeof(tinfl_decompressor)), free);
  Allocation block(ps_malloc(replay_layout::kBlockBytes + 1), free);
  if (!state || !block || end > static_cast<size_t>(replay_blob_end - replay_blob_start)) return false;
  auto* codec = static_cast<tinfl_decompressor*>(state.get());
  auto* out = static_cast<uint8_t*>(block.get());
  uint32_t checked = 0;
  size_t total = 0;
  for (uint32_t at = start; at < end;) {
    if (end - at < 4) return false;
    const auto* p = replay_blob_start + at;
    const unsigned stored = p[0] | (p[1] << 8), decoded = p[2] | (p[3] << 8);
    if (!stored || !decoded || decoded > replay_layout::kBlockBytes || stored > end - at - 4)
      return false;
    size_t input = stored, output = decoded + 1;
    tinfl_init(codec);
    const auto result = tinfl_decompress(codec, p + 4, &input, out, out, &output,
                                         TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    if (result != TINFL_STATUS_DONE || output != decoded || input != stored ||
        !consume(out, decoded)) return false;
    checked = export_protocol::crc32(checked, out, decoded);
    total += decoded;
    at += 4 + stored;
    delay(1);
  }
  return total == expected && checked == crc;
}

bool replayArchiveMetadata(PsramString& json) {
  json.clear();
  return unpack(0, replay_layout::kSampleBlockOffset, replay_layout::kMetadataSize,
                replay_layout::kMetadataCrc, [&](const uint8_t* p, size_t n) {
    json.append(reinterpret_cast<const char*>(p), n);
    return json.ok();
  });
}

bool replayArchiveSamples(PsramLogger& logger) {
  RwLogFileHeader original;
  static_assert(sizeof(original) == sizeof(replay_layout::kHeader), "Archived header type changed");
  memcpy(&original, replay_layout::kHeader, sizeof(original));
  if (original.log_sample_size != sizeof(LogSample) || original.sample_count > logger.sampleCapacity())
    return false;
  logger.startRun(original.run_id, original.run_start_us, 0, 0, 0, false, 0, 0, 0, false,
                  NAN, false, 0, false, true);
  if (!unpack(replay_layout::kSampleBlockOffset, replay_blob_end - replay_blob_start,
              original.sample_count * sizeof(LogSample), replay_layout::kSamplesCrc,
              [&](const uint8_t* p, size_t n) {
    if (n % sizeof(LogSample)) return false;
    for (size_t at = 0; at < n; at += sizeof(LogSample)) {
      LogSample row;
      memcpy(&row, p + at, sizeof(row));
      if (!logger.addSample(row)) return false;
    }
    return true;
  })) return false;
  logger.seal();
  const auto generated = logger.buildHeader(original.metadata_json_size);
  return logger.rwlogDownloadable() && !memcmp(&original, &generated, sizeof(original));
}

// Retain heap only after network/camera/worker startup. Match both measured
// headroom and contiguous capacity; never reduce either past these floors.
void replayMatchInternalHeap() {
  constexpr uint32_t cap = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  static void* held[16]{};
  for (auto& slot : held) {
    const size_t free_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t largest = heap_caps_get_largest_free_block(cap);
    if (free_now <= replay_layout::kTargetFree + 256 || largest <= replay_layout::kTargetLargest + 256)
      break;
    size_t n = std::min(free_now - replay_layout::kTargetFree, largest - replay_layout::kTargetLargest);
    n = (n - 128) & ~size_t(15);
    slot = heap_caps_malloc(n, cap);
    if (!slot) break;
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < replay_layout::kTargetFree ||
        heap_caps_get_largest_free_block(cap) < replay_layout::kTargetLargest) {
      heap_caps_free(slot);
      slot = nullptr;
      break;
    }
  }
}
