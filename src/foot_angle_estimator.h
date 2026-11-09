#pragma once

#include "white_marker_tracker.h"

struct FootAngleEstimate {
    bool valid = false;
    // Diagnostic only: a finite extrapolated angle can still be valid/saved.
    bool in_calibration_range = false;
    float angle_deg = 0.0f;
    float zero_x_px = 0.0f;
    float deg_per_px = 0.0f;
};

FootAngleEstimate estimateFootAngle(
    const WhiteMarkerObservation& marker,
    float zero_x_px,
    bool zero_ready);
