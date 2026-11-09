#pragma once

#include <stdint.h>

namespace appcfg {

static constexpr int kFrameWidth = 320;
static constexpr int kFrameHeight = 240;
static constexpr int kCameraFpsTarget = 15;

static constexpr int kMarkerAId = 0;
static constexpr int kMarkerBId = 1;

// Sparse-line candidates across the complete bounded vertical search.
// Upper lane = A, lower lane = B.
static constexpr int kWhiteMarkerRowCount = 5;
static constexpr int kWhiteReferenceRowCount = 3;

static constexpr int kWhiteMarkerARows[kWhiteMarkerRowCount] =
    {58, 62, 66, 70, 74};
static constexpr int kWhiteReferenceARows[kWhiteReferenceRowCount] =
    {38, 42, 46};
static constexpr int kWhiteMarkerACenterY = 66;

static constexpr int kWhiteMarkerBRows[kWhiteMarkerRowCount] =
    {152, 156, 160, 164, 168};
static constexpr int kWhiteReferenceBRows[kWhiteReferenceRowCount] =
    {182, 186, 190};
static constexpr int kWhiteMarkerBCenterY = 160;

static constexpr char kWhiteDetectorRevision[] = "sparse_rows_identity_v3";
static constexpr int kWhiteSearchRadiusYPx = 32;
static constexpr int kWhiteSearchStepYPx = 4;
static constexpr int kWhiteMaxTemplates =
    1 + 2 * kWhiteSearchRadiusYPx / kWhiteSearchStepYPx;
static_assert(kWhiteSearchRadiusYPx % kWhiteSearchStepYPx == 0, "Complete vertical search");
static_assert(kWhiteReferenceARows[0] - kWhiteSearchRadiusYPx >= 0, "Upper reference in frame");
static_assert(kWhiteReferenceBRows[kWhiteReferenceRowCount - 1] + kWhiteSearchRadiusYPx < kFrameHeight,
              "Lower reference in frame");
static_assert(kWhiteMarkerARows[kWhiteMarkerRowCount - 1] + kWhiteSearchRadiusYPx <
                  kWhiteMarkerBRows[0] - kWhiteSearchRadiusYPx,
              "Right and left marker search rows must not overlap");

static constexpr float kWhitePeakMinContrast = 55.0f;
static constexpr float kWhiteCentroidBaseline = 35.0f;
static constexpr float kWhiteMinWeightSum = 150.0f;
static constexpr int kWhiteCentroidHalfWindowPx = 35;
static constexpr int kWhiteMinWidthPx = 4;
static constexpr int kWhiteMaxWidthPx = 72;
static constexpr int kWhiteMaxCandidates = 24;
static constexpr float kWhiteAmbiguousRatio = 0.82f;
static constexpr float kWhiteMaxTrackStepXPx = 80.0f;
static constexpr float kWhiteMaxTrackStepYPx = 32.0f;
static constexpr uint32_t kWhiteTrackMemoryUs = 500000;
static constexpr uint8_t kWhiteReacquireFrames = 3;
// Confirm a displaced candidate if both independent quality measures collapse.
// A brightness change at the same location, or a strong displaced marker, is
// still accepted immediately. This does not impose an angle/support boundary.
static constexpr float kWhiteWeakContrastRatio = 0.5f;
static constexpr float kWhiteWeakWeightRatio = 0.1f;
static constexpr float kWhiteWeakXStepPx = kWhiteMaxTrackStepXPx * 0.25f;
static constexpr float kWhiteWeakYStepPx = kWhiteMaxTrackStepYPx * 0.5f;

// Current calibration: five fixed-leg poses with repeat endpoint, 2026-11-01.
// See foot_calibration_table.h. The scalar gains below are endpoint averages
// for metadata compatibility only; the estimator uses the nonlinear table.
// Historical fixtures preserve the old mapping for old logs.
// Definition:
//   foot_angle_deg = angle of the rigid foot/leg link relative to the body.
//   Upright initial posture = 0 deg.
//   Positive direction = the direction observed when marker X moves left.
//
// Seven hand-held snapshots are provisional MEKF references, with no
// independent held-out angular validation. Actual zero is still measured
// independently for each foot on every boot; nominal zeros center that gate.
static constexpr float kFootAngleAZeroXPx = 203.081900000f;
static constexpr float kFootAngleBZeroXPx = 201.656900000f;
static constexpr float kFootAngleADegPerPx = 0.152154693f;
static constexpr float kFootAngleBDegPerPx = 0.149505208f;
static constexpr float kFootCalibrationBodySpanDeg = 20.626501000f;
static constexpr float kFootCalibrationAUprightX = 203.081900000f;
static constexpr float kFootCalibrationATiltedX = 67.519200000f;
static constexpr float kFootCalibrationBUprightX = 201.656900000f;
static constexpr float kFootCalibrationBTiltedX = 63.691800000f;

// Current measured envelope is diagnostic only. Keep original historical bounds
// separately for provenance; extrapolated estimates are never clipped/rejected.
static constexpr float kFootAngleAOriginalMinCalXPx = 42.0f;
static constexpr float kFootAngleAOriginalMaxCalXPx = 173.0f;
static constexpr float kFootAngleBOriginalMinCalXPx = 43.5f;
static constexpr float kFootAngleBOriginalMaxCalXPx = 177.5f;
static constexpr float kFootObservedAMinXPx = 67.5192f;
static constexpr float kFootObservedAMaxXPx = 203.0819f;
static constexpr float kFootObservedBMinXPx = 63.6918f;
static constexpr float kFootObservedBMaxXPx = 201.6569f;
static constexpr float kFootObservedMarginPx = 0.0f;
static constexpr float kFootAngleAMinCalXPx = 67.519200000f;
static constexpr float kFootAngleAMaxCalXPx = 203.081900000f;
static constexpr float kFootAngleBMinCalXPx = 63.691800000f;
static constexpr float kFootAngleBMaxCalXPx = 201.656900000f;

// IMU diagnostics retained to validate body motion independently.
static constexpr float kBodyTiltComplementaryTauS = 0.50f;
static constexpr float kTiltStaticMaxGyroDps = 3.0f;
static constexpr float kTiltStaticAccelNormToleranceG = 0.05f;

// Automatic upright zeroing.
//
// On the current AtomS3R-CAM mount, gravity points approximately along IMU -Z
// when the body is upright. Upright recognition is intentionally based only
// on IMU gravity direction + stability; marker X is never used to decide
// whether the body is upright.
//
// IMU upright does not establish foot neutrality or marker identity. The
// measured neutral-image neighborhood is a plausibility gate, not the zero
// value: accepted positions are still averaged independently on every boot.
static constexpr float kAutoZeroMaxNominalOffsetXPx = 35.0f;
static constexpr float kAutoZeroMaxSpreadXPx = 4.0f;
// All conditions must remain continuously true before the reference locks.
// MEKF pitch-axis gravity tilt relative to the measured upright reference.
// A stable fore/aft roll offset is permitted during foot zeroing.
static constexpr float kAutoZeroMaxPitchErrorDeg = 5.0f;
static constexpr float kAutoZeroMaxGyroDps = 1.5f;
static constexpr float kAutoZeroAccelNormToleranceG = 0.03f;
static constexpr uint32_t kAutoZeroStableMs = 2000;
static constexpr uint32_t kAutoZeroMinVisionSamples = 15;

static constexpr uint32_t kImuControlPeriodUs = 5000; // 200 Hz
static constexpr uint32_t kTelemetryPeriodMs = 100;   // 10 Hz JSON

} // namespace appcfg
