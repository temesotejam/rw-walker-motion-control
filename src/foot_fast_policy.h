#pragma once
#include <cstddef>
#include <cstdint>

// v20 bounds capture/recognition load. Actual sensor FPS and end-to-end age
// must be measured on the board; these settings are not performance claims.
namespace foot_fast_policy {
constexpr uint32_t kWorkerPeriodUs=40000;
constexpr uint32_t kControlPollMs=2;
constexpr unsigned kCameraFrameBuffers=2;
constexpr uint32_t kArchiveCapacity=1280; // 40 seconds *25Hz plus margin.
constexpr size_t kMetadataHardBudgetBytes=1024U*1024U;
constexpr size_t kMetadataReserveBytes=1152U*1024U;
// Raw foot frames and controller audits take precedence over forecast copies.
constexpr size_t kOptionalPredictionEndBytes=kMetadataHardBudgetBytes*65U/100U;
constexpr size_t kEnergyAuditEndBytes=kMetadataHardBudgetBytes*96U/100U;
constexpr size_t kSupportAuditEndBytes=kMetadataHardBudgetBytes*99U/100U;
}
