#pragma once
#include <cstddef>
#include <cstdint>

// v20 bounds capture/recognition load. Actual sensor FPS and end-to-end age
// must be measured on the board; these settings are not performance claims.
namespace foot_fast_policy {
#if defined(FOOT_CADENCE20_V37) && FOOT_CADENCE20_V37
constexpr uint32_t kWorkerPeriodUs=20000;
#elif defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
constexpr uint32_t kWorkerPeriodUs=23000;
#elif defined(FOOT_WORKER26_V34) && FOOT_WORKER26_V34
constexpr uint32_t kWorkerPeriodUs=26000;
#elif defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
constexpr uint32_t kWorkerPeriodUs=33333;
#else
constexpr uint32_t kWorkerPeriodUs=40000;
#endif
constexpr uint32_t kControlPollMs=2;
constexpr unsigned kCameraFrameBuffers=2;
#if defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
#if defined(FOOT_CADENCE20_V37) && FOOT_CADENCE20_V37
constexpr uint32_t kArchiveCapacity=2048;
#else
constexpr uint32_t kArchiveCapacity=1760;
#endif
constexpr size_t kMetadataHardBudgetBytes=1408U*1024U;
constexpr size_t kMetadataReserveBytes=1536U*1024U;
static_assert((kArchiveCapacity-1ULL)*kWorkerPeriodUs>=40000000ULL,"Retain the full 40-second capture");
#elif defined(FOOT_WORKER26_V34) && FOOT_WORKER26_V34
constexpr uint32_t kArchiveCapacity=1600; // 5s start +30s trial +5s end, plus margin.
constexpr size_t kMetadataHardBudgetBytes=1280U*1024U;
constexpr size_t kMetadataReserveBytes=1408U*1024U;
static_assert((kArchiveCapacity-1ULL)*kWorkerPeriodUs>=40000000ULL,"Retain the full 40-second capture");
#else
constexpr uint32_t kArchiveCapacity=1280; // 40 seconds *25Hz plus margin.
constexpr size_t kMetadataHardBudgetBytes=1024U*1024U;
constexpr size_t kMetadataReserveBytes=1152U*1024U;
#endif
// Raw foot frames and controller audits take precedence over forecast copies.
constexpr size_t kOptionalPredictionEndBytes=kMetadataHardBudgetBytes*65U/100U;
constexpr size_t kEnergyAuditEndBytes=kMetadataHardBudgetBytes*96U/100U;
constexpr size_t kSupportAuditEndBytes=kMetadataHardBudgetBytes*99U/100U;
}
