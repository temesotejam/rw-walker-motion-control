#pragma once

#include <stdint.h>
#include "foot_image_rows.h"
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26 && (!defined(FOOT_ROW_OVERLAP_V24) || !FOOT_ROW_OVERLAP_V24)
#error "Incremental preparation requires the existing prepared-feature interface"
#endif

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
    struct Candidate {
        float x = 0, y = 0, contrast = 0, weight = 0;
        int peak = 0, width = 0, min_y = 0, max_y = 0;
    };
public:
    explicit WhiteMarker1DTracker(int marker_id);

    WhiteMarkerObservation process(const uint8_t* gray);
    WhiteMarkerObservation process(const foot_image_rows::GrayRowView& gray);
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    class PreparedScan;
#endif
    // Image-only work may run before the complete frame is admitted. This
    // value owns its candidate features and never aliases camera pixels.
    class PreparedFeatures {
    public:
        uint32_t processing_us = 0;
    private:
        friend class WhiteMarker1DTracker;
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
        friend class PreparedScan;
        // An incomplete incremental scan is an admission failure, not NoFrame.
        bool admitted_ = false;
#endif
        Candidate candidates_[appcfg::kWhiteMaxCandidates];
        WhiteMarkerObservation observation_;
        int marker_id_ = -1, count_ = 0;
        bool image_valid_ = false, overflow_ = false;
    };
    PreparedFeatures prepare(const foot_image_rows::GrayRowView& gray) const;
    WhiteMarkerObservation processPrepared(const PreparedFeatures& prepared);
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    class PreparedScan {
    public:
        bool valid() const { return !failed_ && (features_.marker_id_ == appcfg::kMarkerAId ||
                                               features_.marker_id_ == appcfg::kMarkerBId); }
        bool complete() const { return valid() && next_dy_ > appcfg::kWhiteSearchRadiusYPx; }
        uint8_t templatesTested() const { return features_.observation_.templates_tested; }
        uint32_t processingUs() const { return features_.processing_us; }
    private:
        friend class WhiteMarker1DTracker;
        PreparedFeatures features_;
        int contrast_[appcfg::kFrameWidth] = {};
        int next_dy_ = -appcfg::kWhiteSearchRadiusYPx;
        bool failed_ = true;
    };
    PreparedScan beginScan() const;
    // maxLogicalRow is inclusive, in the original logical 320 x 240 coordinates.
    // Rows belong to one immutable frame and must remain available, including
    // the leaving rows used by rolling contrast. No view/pixel pointer is kept.
    // True means a valid scan (possibly waiting for more rows); use complete().
    // False poisons this scan; restart it with beginScan() for another frame.
    bool appendAvailable(PreparedScan& scan, const foot_image_rows::GrayRowView& gray,
                         int maxLogicalRow) const;
    // Incomplete, failed or wrong-lane scans return rejected features. Only the
    // caller can admit the full frame identity before processPrepared commits.
    PreparedFeatures finishScan(const PreparedScan& scan) const;
#endif
#endif

private:
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
    WhiteMarkerObservation processPreparedAt(const PreparedFeatures& prepared, uint32_t now);
#endif
    const int* markerRows() const;
    const int* referenceRows() const;
    int nominalCenterY() const;
    void evaluate(const foot_image_rows::GrayRowView& gray, int y_offset, Candidate* candidates,
                  int& count, bool& overflow, WhiteMarkerObservation& weak,
                  int* shared_contrast=nullptr, bool slide=false) const;

    int _id;
    uint32_t _success_count = 0;
    uint32_t _fail_count = 0;
    bool _tracked = false;
    float _last_x = 0, _last_y = 0, _pending_x = 0, _pending_y = 0;
    float _last_contrast = 0, _last_weight = 0;
    uint32_t _last_us = 0, _pending_us = 0;
    uint8_t _pending_count = 0;
};
