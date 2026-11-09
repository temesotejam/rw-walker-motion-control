#pragma once

#include <stdint.h>

enum class MarkerDetectionReason : uint8_t {
    Detected, NoFrame, LowContrast, LowWeight, BadShape, Ambiguous, TrackJump, Reacquiring
};
const char* markerDetectionReasonName(MarkerDetectionReason reason);

struct WhiteMarkerObservation {
    bool valid = false;
    MarkerDetectionReason reason = MarkerDetectionReason::NoFrame;
    uint8_t templates_tested = 0;
    uint8_t candidate_count = 0;
    float ambiguity_ratio = 0.0f;
    float alternate_x_px = -1.0f, alternate_y_px = -1.0f;
    int id = -1;
    float center_x_px = 0.0f;
    // Center of the selected sparse-row template, not a measured Y centroid.
    float center_y_px = 0.0f;
    float peak_contrast = 0.0f;
    float weight_sum = 0.0f;
    int peak_x_px = 0;
    int bright_width_px = 0;
    uint32_t processing_us = 0;
    uint32_t success_count = 0;
    uint32_t fail_count = 0;
};

class WhiteMarker1DTracker {
public:
    explicit WhiteMarker1DTracker(int marker_id);

    WhiteMarkerObservation process(const uint8_t* gray);

private:
    struct Candidate {
        float x = 0, y = 0, contrast = 0, weight = 0;
        int peak = 0, width = 0, min_y = 0, max_y = 0;
    };
    const int* markerRows() const;
    const int* referenceRows() const;
    int nominalCenterY() const;
    void evaluate(const uint8_t* gray, int y_offset, Candidate* candidates,
                  int& count, bool& overflow, WhiteMarkerObservation& weak) const;

    int _id;
    uint32_t _success_count = 0;
    uint32_t _fail_count = 0;
    bool _tracked = false;
    float _last_x = 0, _last_y = 0, _pending_x = 0, _pending_y = 0;
    float _last_contrast = 0, _last_weight = 0;
    uint32_t _last_us = 0, _pending_us = 0;
    uint8_t _pending_count = 0;
};
