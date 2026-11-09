#include "white_marker_tracker.h"
#include <Arduino.h>
#include <math.h>
#include "foot_tracking_config.h"
#include "marker_identity_policy.h"

namespace {
constexpr int kWidth = appcfg::kFrameWidth;
constexpr int kRadius = 2, kWindow = 5, kScale = 15;
static_assert(appcfg::kWhiteMarkerRowCount == 5 && appcfg::kWhiteReferenceRowCount == 3,
              "Integer contrast scale assumes five marker and three reference rows");
constexpr int kPeakMin = static_cast<int>(appcfg::kWhitePeakMinContrast * kScale);
constexpr int kBaseline = static_cast<int>(appcfg::kWhiteCentroidBaseline * kScale);
constexpr int kWeightMin = static_cast<int>(appcfg::kWhiteMinWeightSum * kScale);
}

const char* markerDetectionReasonName(MarkerDetectionReason reason) {
    switch (reason) {
        case MarkerDetectionReason::Detected: return "detected";
        case MarkerDetectionReason::NoFrame: return "no_frame";
        case MarkerDetectionReason::LowContrast: return "low_contrast";
        case MarkerDetectionReason::LowWeight: return "low_weight";
        case MarkerDetectionReason::BadShape: return "bad_shape";
        case MarkerDetectionReason::Ambiguous: return "ambiguous";
        case MarkerDetectionReason::TrackJump: return "track_jump";
        case MarkerDetectionReason::Reacquiring: return "reacquiring";
    }
    return "unknown";
}
WhiteMarker1DTracker::WhiteMarker1DTracker(int marker_id) : _id(marker_id) {}
const int* WhiteMarker1DTracker::markerRows() const {
    return _id == appcfg::kMarkerAId ? appcfg::kWhiteMarkerARows : appcfg::kWhiteMarkerBRows;
}
const int* WhiteMarker1DTracker::referenceRows() const {
    return _id == appcfg::kMarkerAId ? appcfg::kWhiteReferenceARows : appcfg::kWhiteReferenceBRows;
}
int WhiteMarker1DTracker::nominalCenterY() const {
    return _id == appcfg::kMarkerAId ? appcfg::kWhiteMarkerACenterY : appcfg::kWhiteMarkerBCenterY;
}

WhiteMarkerObservation WhiteMarker1DTracker::process(const uint8_t* gray) {
    const uint32_t now = micros();
    WhiteMarkerObservation out;
    out.id = _id; out.center_y_px = nominalCenterY();
    if (gray) {
        Candidate candidates[appcfg::kWhiteMaxCandidates];
        int count = 0; bool overflow = false;
        out.reason = MarkerDetectionReason::LowContrast;
        // Always search all heights. A nominal-row distractor must not hide a
        // stronger displaced marker. Each distinct horizontal island is kept.
        for (int dy = -appcfg::kWhiteSearchRadiusYPx; dy <= appcfg::kWhiteSearchRadiusYPx;
             dy += appcfg::kWhiteSearchStepYPx) {
            evaluate(gray, dy, candidates, count, overflow, out);
            ++out.templates_tested;
        }
        out.candidate_count = count;
        if (count) {
            const bool recent = _tracked && static_cast<uint32_t>(now - _last_us) <= appcfg::kWhiteTrackMemoryUs;
            int best = -1, second = -1, global = 0;
            float best_score = -1, second_score = -1;
            for (int i = 0; i < count; ++i) {
                const auto& c = candidates[i];
                if (c.contrast > candidates[global].contrast ||
                    (c.contrast == candidates[global].contrast && c.weight > candidates[global].weight)) global = i;
                const float dx = fabsf(c.x - _last_x), dy = fabsf(c.y - _last_y);
                if (recent && (dx > appcfg::kWhiteMaxTrackStepXPx || dy > appcfg::kWhiteMaxTrackStepYPx)) continue;
                // Proximity is a modest preference; it cannot turn a weak
                // nearby reflection into a stronger current-image detection.
                const float distance = recent ? fmaxf(dx / appcfg::kWhiteMaxTrackStepXPx,
                                                       dy / appcfg::kWhiteMaxTrackStepYPx) : 0;
                const float score = c.contrast * (1.0f - 0.15f * distance);
                if (score > best_score) {
                    second = best; second_score = best_score; best = i; best_score = score;
                } else if (score > second_score) { second = i; second_score = score; }
            }
            const auto& chosen = candidates[best < 0 ? global : best];
            out.center_x_px = chosen.x; out.center_y_px = chosen.y;
            out.peak_contrast = chosen.contrast; out.weight_sum = chosen.weight;
            out.peak_x_px = chosen.peak; out.bright_width_px = chosen.width;
            if (second >= 0) {
                out.alternate_x_px = candidates[second].x; out.alternate_y_px = candidates[second].y;
                out.ambiguity_ratio = second_score / best_score;
            }
            out.reason = overflow ? MarkerDetectionReason::Ambiguous :
                best < 0 ? MarkerDetectionReason::TrackJump :
                out.ambiguity_ratio >= appcfg::kWhiteAmbiguousRatio ? MarkerDetectionReason::Ambiguous :
                MarkerDetectionReason::Detected;
            const bool weak_displaced = recent && marker_identity::weakDisplaced(
                _last_x, _last_y, _last_contrast, _last_weight,
                chosen.x, chosen.y, chosen.contrast, chosen.weight);
            if (out.reason == MarkerDetectionReason::Detected && _tracked && (!recent || weak_displaced)) {
                if (!_pending_count || static_cast<uint32_t>(now - _pending_us) > 300000 ||
                    fabsf(chosen.x - _pending_x) > 40 || fabsf(chosen.y - _pending_y) > 24) _pending_count = 0;
                _pending_x = chosen.x; _pending_y = chosen.y; _pending_us = now;
                if (++_pending_count < appcfg::kWhiteReacquireFrames) out.reason = MarkerDetectionReason::Reacquiring;
            } else { _pending_count = 0; }
            out.valid = out.reason == MarkerDetectionReason::Detected;
            if (out.valid) {
                _tracked = true; _last_x = chosen.x; _last_y = chosen.y; _last_us = now; _pending_count = 0;
                _last_contrast = chosen.contrast; _last_weight = chosen.weight;
            }
        } else { _pending_count = 0; }
    } else { _pending_count = 0; }
    if (out.valid) ++_success_count; else ++_fail_count;
    out.success_count = _success_count; out.fail_count = _fail_count;
    out.processing_us = micros() - now;
    return out;
}

void WhiteMarker1DTracker::evaluate(const uint8_t* gray, int dy, Candidate* candidates,
                                   int& count, bool& overflow, WhiteMarkerObservation& weak) const {
    int contrast[kWidth], smooth[kWidth] = {};
    const int* rows = markerRows(); const int* refs = referenceRows();
    for (int x = 0; x < kWidth; ++x) {
        int a = 0, b = 0;
        for (int i = 0; i < appcfg::kWhiteMarkerRowCount; ++i) a += gray[(rows[i] + dy) * kWidth + x];
        for (int i = 0; i < appcfg::kWhiteReferenceRowCount; ++i) b += gray[(refs[i] + dy) * kWidth + x];
        contrast[x] = 3 * a - 5 * b;
    }
    int rolling = 0;
    for (int x = 0; x < kWindow; ++x) rolling += contrast[x];
    for (int x = kRadius; x < kWidth - kRadius; ++x) {
        if (x > kRadius) rolling += contrast[x + kRadius] - contrast[x - kRadius - 1];
        smooth[x] = rolling / kWindow;
        if (smooth[x] > weak.peak_contrast * kScale) weak.peak_contrast = static_cast<float>(smooth[x]) / kScale;
    }
    for (int x = kRadius; x < kWidth - kRadius;) {
        if (smooth[x] <= kBaseline) { ++x; continue; }
        const int lo = x;
        int peak = x, peak_value = 0, weight = 0;
        int64_t weighted_x = 0;
        while (x < kWidth - kRadius && smooth[x] > kBaseline) {
            const int w = smooth[x] - kBaseline;
            weight += w; weighted_x += static_cast<int64_t>(x) * w;
            if (smooth[x] > peak_value) { peak_value = smooth[x]; peak = x; }
            ++x;
        }
        const int width = x - lo;
        if (peak_value < kPeakMin) continue;
        if (weight < kWeightMin) { weak.reason = MarkerDetectionReason::LowWeight; continue; }
        if (width < appcfg::kWhiteMinWidthPx || width > appcfg::kWhiteMaxWidthPx) {
            weak.reason = MarkerDetectionReason::BadShape; continue;
        }
        Candidate c;
        c.x = static_cast<float>(weighted_x) / weight; c.y = nominalCenterY() + dy;
        c.contrast = static_cast<float>(peak_value) / kScale; c.weight = static_cast<float>(weight) / kScale;
        c.peak = peak; c.width = width; c.min_y = c.max_y = static_cast<int>(c.y);
        // Adjacent height templates observing the same horizontal island are
        // one candidate, not competitors. Retain its strongest full-row view.
        int merge = -1;
        for (int i = 0; i < count; ++i) {
            if (fabsf(c.x - candidates[i].x) <= 8 && c.y >= candidates[i].min_y - 8 &&
                c.y <= candidates[i].max_y + 8) { merge = i; break; }
        }
        if (merge >= 0) {
            auto& old = candidates[merge];
            const int low_y = min(old.min_y, c.min_y), high_y = max(old.max_y, c.max_y);
            if (c.contrast > old.contrast || (c.contrast == old.contrast &&
                (c.weight > old.weight || (c.weight == old.weight &&
                 fabsf(c.y - nominalCenterY()) < fabsf(old.y - nominalCenterY()))))) old = c;
            old.min_y = low_y; old.max_y = high_y;
        } else if (count < appcfg::kWhiteMaxCandidates) { candidates[count++] = c; }
        else { overflow = true; }
    }
}
