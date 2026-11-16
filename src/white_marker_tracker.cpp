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
    return process(foot_image_rows::GrayRowView::full(gray));
}
WhiteMarkerObservation WhiteMarker1DTracker::process(const foot_image_rows::GrayRowView& gray) {
    const uint32_t now = micros();
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
    // Preserve the ordinary path's temporal decision clock from before scan.
    return processPreparedAt(prepare(gray),now);
#else
    WhiteMarkerObservation out;
    out.id = _id; out.center_y_px = nominalCenterY();
    if (gray.valid()) {
        Candidate candidates[appcfg::kWhiteMaxCandidates];
        int count = 0; bool overflow = false;
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
        int contrast_state[kWidth];
        bool slide=false;
#endif
        out.reason = MarkerDetectionReason::LowContrast;
        // Always search all heights. A nominal-row distractor must not hide a
        // stronger displaced marker. Each distinct horizontal island is kept.
        for (int dy = -appcfg::kWhiteSearchRadiusYPx; dy <= appcfg::kWhiteSearchRadiusYPx;
             dy += appcfg::kWhiteSearchStepYPx) {
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
            evaluate(gray, dy, candidates, count, overflow, out, contrast_state, slide);
            slide=true;
#else
            evaluate(gray, dy, candidates, count, overflow, out);
#endif
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
#endif
}

#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
WhiteMarker1DTracker::PreparedFeatures WhiteMarker1DTracker::prepare(const foot_image_rows::GrayRowView& gray) const {
    const uint32_t start=micros();
    PreparedFeatures prepared;
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    prepared.admitted_=true; // Preserve ordinary NoFrame handling as well.
#endif
    prepared.marker_id_=_id; prepared.image_valid_=gray.valid();
    auto& out=prepared.observation_;
    out.id=_id; out.center_y_px=nominalCenterY();
    if(prepared.image_valid_) {
        auto* candidates=prepared.candidates_;
        int& count=prepared.count_; bool& overflow=prepared.overflow_;
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
        int contrast_state[kWidth];
        bool slide=false;
#endif
        out.reason = MarkerDetectionReason::LowContrast;
        // Always search all heights. A nominal-row distractor must not hide a
        // stronger displaced marker. Each distinct horizontal island is kept.
        for (int dy = -appcfg::kWhiteSearchRadiusYPx; dy <= appcfg::kWhiteSearchRadiusYPx;
             dy += appcfg::kWhiteSearchStepYPx) {
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
            evaluate(gray, dy, candidates, count, overflow, out, contrast_state, slide);
            slide=true;
#else
            evaluate(gray, dy, candidates, count, overflow, out);
#endif
            ++out.templates_tested;
        }
        out.candidate_count = count;
    }
    prepared.processing_us=micros()-start;
    return prepared;
}
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
WhiteMarker1DTracker::PreparedScan WhiteMarker1DTracker::beginScan() const {
    PreparedScan scan;
    if(_id!=appcfg::kMarkerAId && _id!=appcfg::kMarkerBId)return scan;
    scan.failed_=false;
    auto& prepared=scan.features_;
    prepared.marker_id_=_id; prepared.image_valid_=true;
    prepared.observation_.id=_id;
    prepared.observation_.center_y_px=nominalCenterY();
    prepared.observation_.reason=MarkerDetectionReason::LowContrast;
    return scan;
}
bool WhiteMarker1DTracker::appendAvailable(PreparedScan& scan,
        const foot_image_rows::GrayRowView& gray,int maxLogicalRow) const {
    const uint32_t start=micros();
    auto& prepared=scan.features_;
    if(!scan.valid() || prepared.marker_id_!=_id || !gray.valid() ||
       maxLogicalRow < -1 || maxLogicalRow >= appcfg::kFrameHeight) {
        scan.failed_=true;
        prepared.processing_us+=micros()-start;
        return false;
    }
    const int* marker=markerRows(); const int* reference=referenceRows();
    while(scan.next_dy_<=appcfg::kWhiteSearchRadiusYPx) {
        const int dy=scan.next_dy_;
        bool available=true;
        // Check all current-template rows before asking the view for any pixel.
        for(int i=0;i<appcfg::kWhiteMarkerRowCount;++i)
            if(marker[i]+dy>maxLogicalRow)available=false;
        for(int i=0;i<appcfg::kWhiteReferenceRowCount;++i)
            if(reference[i]+dy>maxLogicalRow)available=false;
        if(!available)break;
        bool rows_present=true;
        for(int i=0;i<appcfg::kWhiteMarkerRowCount;++i)
            if(!gray.row(marker[i]+dy))rows_present=false;
        for(int i=0;i<appcfg::kWhiteReferenceRowCount;++i)
            if(!gray.row(reference[i]+dy))rows_present=false;
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
        const bool slide=dy!=-appcfg::kWhiteSearchRadiusYPx;
        if(slide && (!gray.row(marker[0]+dy-appcfg::kWhiteSearchStepYPx) ||
                     !gray.row(reference[0]+dy-appcfg::kWhiteSearchStepYPx)))rows_present=false;
#endif
        if(!rows_present){scan.failed_=true;break;}
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
        evaluate(gray,dy,prepared.candidates_,prepared.count_,prepared.overflow_,
                 prepared.observation_,scan.contrast_,slide);
#else
        evaluate(gray,dy,prepared.candidates_,prepared.count_,prepared.overflow_,prepared.observation_);
#endif
        ++prepared.observation_.templates_tested;
        prepared.observation_.candidate_count=prepared.count_;
        scan.next_dy_+=appcfg::kWhiteSearchStepYPx;
    }
    prepared.processing_us+=micros()-start;
    return !scan.failed_;
}
WhiteMarker1DTracker::PreparedFeatures WhiteMarker1DTracker::finishScan(const PreparedScan& scan) const {
    if(!scan.complete() || scan.features_.marker_id_!=_id)return PreparedFeatures{};
    PreparedFeatures prepared=scan.features_;
    prepared.admitted_=true;
    return prepared;
}
#endif
WhiteMarkerObservation WhiteMarker1DTracker::processPrepared(const PreparedFeatures& prepared) {
    return processPreparedAt(prepared,micros());
}
WhiteMarkerObservation WhiteMarker1DTracker::processPreparedAt(const PreparedFeatures& prepared,uint32_t now) {
    // A different lane/default value is a programming/admission error, not a
    // new failed image. Refuse it without changing temporal state or counters.
    if(prepared.marker_id_!=_id
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
       || !prepared.admitted_
#endif
    ) {
        WhiteMarkerObservation out;
        out.id=_id; out.center_y_px=nominalCenterY();
        out.success_count=_success_count; out.fail_count=_fail_count;
        out.processing_us=micros()-now;
        return out;
    }
    auto out=prepared.observation_;
    if(prepared.image_valid_) {
        const auto* candidates=prepared.candidates_;
        const int count=prepared.count_;const bool overflow=prepared.overflow_;
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
#endif

void WhiteMarker1DTracker::evaluate(const foot_image_rows::GrayRowView& gray, int dy, Candidate* candidates,
                                   int& count, bool& overflow, WhiteMarkerObservation& weak,
                                   int* shared_contrast, bool slide) const {
    int local_contrast[kWidth], smooth[kWidth] = {};
    int* contrast=shared_contrast ? shared_contrast : local_contrast;
    const int* rows = markerRows(); const int* refs = referenceRows();
    if (slide) {
        // Each template shifts by one sampled row. Keep exactly the same
        // integer contrast, replacing only rows entering/leaving each band.
        const int step=appcfg::kWhiteSearchStepYPx;
        const uint8_t* old_m=gray.row(rows[0]+dy-step);
        const uint8_t* new_m=gray.row(rows[appcfg::kWhiteMarkerRowCount-1]+dy);
        const uint8_t* old_r=gray.row(refs[0]+dy-step);
        const uint8_t* new_r=gray.row(refs[appcfg::kWhiteReferenceRowCount-1]+dy);
        for(int x=0;x<kWidth;++x) contrast[x]+=3*(int(new_m[x])-old_m[x])-5*(int(new_r[x])-old_r[x]);
    } else {
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
    // Stream complete rows instead of alternating eight distant image rows
    // at every column. All sums are integer and keep the original scale.
    const uint8_t* first = gray.row(rows[0] + dy);
    for (int x = 0; x < kWidth; ++x) contrast[x] = first[x];
    for (int i = 1; i < appcfg::kWhiteMarkerRowCount; ++i) {
        const uint8_t* row = gray.row(rows[i] + dy);
        for (int x = 0; x < kWidth; ++x) contrast[x] += row[x];
    }
    for (int x = 0; x < kWidth; ++x) contrast[x] *= 3;
    for (int i = 0; i < appcfg::kWhiteReferenceRowCount; ++i) {
        const uint8_t* row = gray.row(refs[i] + dy);
        for (int x = 0; x < kWidth; ++x) contrast[x] -= 5 * row[x];
    }
#else
    const uint8_t* marker_rows[appcfg::kWhiteMarkerRowCount];
    const uint8_t* reference_rows[appcfg::kWhiteReferenceRowCount];
    for (int i = 0; i < appcfg::kWhiteMarkerRowCount; ++i) marker_rows[i] = gray.row(rows[i] + dy);
    for (int i = 0; i < appcfg::kWhiteReferenceRowCount; ++i) reference_rows[i] = gray.row(refs[i] + dy);
    for (int x = 0; x < kWidth; ++x) {
        int a = 0, b = 0;
        for (int i = 0; i < appcfg::kWhiteMarkerRowCount; ++i) a += marker_rows[i][x];
        for (int i = 0; i < appcfg::kWhiteReferenceRowCount; ++i) b += reference_rows[i][x];
        contrast[x] = 3 * a - 5 * b;
    }
#endif
    }
    int rolling = 0;
#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
    int strongest = 0;
#endif
    for (int x = 0; x < kWindow; ++x) rolling += contrast[x];
    for (int x = kRadius; x < kWidth - kRadius; ++x) {
        if (x > kRadius) rolling += contrast[x + kRadius] - contrast[x - kRadius - 1];
        smooth[x] = rolling / kWindow;
#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
        if(smooth[x]>strongest)strongest=smooth[x];
#else
        if (smooth[x] > weak.peak_contrast * kScale) weak.peak_contrast = static_cast<float>(smooth[x]) / kScale;
#endif
    }
#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
    // One conversion per height rather than per pixel; the same integer maximum.
    if(strongest>weak.peak_contrast*kScale)weak.peak_contrast=static_cast<float>(strongest)/kScale;
#endif
    for (int x = kRadius; x < kWidth - kRadius;) {
        if (smooth[x] <= kBaseline) { ++x; continue; }
        const int lo = x;
        int peak = x, peak_value = 0, weight = 0;
#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
        // Worst case: every column at maximum positive 15*255 contrast.
        // The exact centroid sum fits int32; avoid software 64-bit multiplies.
        static_assert(int64_t(kWidth)*(kWidth-1)/2*15*255<INT32_MAX,"Centroid accumulator fits");
        int32_t weighted_x = 0;
#else
        int64_t weighted_x = 0;
#endif
        while (x < kWidth - kRadius && smooth[x] > kBaseline) {
            const int w = smooth[x] - kBaseline;
            weight += w;
#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
            weighted_x += x*w;
#else
            weighted_x += static_cast<int64_t>(x) * w;
#endif
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
