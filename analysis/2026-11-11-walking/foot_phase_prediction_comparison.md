# Causal foot prediction comparison

The candidate was evaluated by compiling the actual C++ header. Observations enter only at recorded image-delivery + recognition-processing + saved CAD-compute completion. Future camera data is used only by the scorer.

Primary score: prediction at the timestamp of a not-yet-arrived camera frame, compared directly to that frame. This avoids linear interpolation of the target and still uses the same camera recognition algorithm, not independent angle truth.

| Log | Common angle values | Candidate MAE | Hold MAE | Old linear MAE | Candidate p95 | Candidate max | Candidate coverage |
|---|---:|---:|---:|---:|---:|---:|---:|
| 137267573 | 368 | 0.2729 | 0.3035 | 0.4348 | 1.0873 | 2.6009 | 75.4% |
| 267557373 | 386 | 0.2831 | 0.3297 | 0.4619 | 1.0834 | 3.5070 | 77.8% |
| 27908524 | 428 | 0.3194 | 0.3467 | 0.5150 | 1.5486 | 3.8710 | 82.9% |
| 64040782 | 492 | 0.4442 | 0.5128 | 0.6910 | 1.8268 | 8.2464 | 88.2% |
| 84815132 | 466 | 0.2572 | 0.2645 | 0.3681 | 1.3478 | 4.9452 | 87.9% |
| 57814129 | 138 | 0.4160 | 0.4127 | 0.6455 | 1.8436 | 2.4885 | 47.9% |
| 199015154 | 568 | 0.3997 | 0.5666 | 0.7433 | 1.6714 | 3.6238 | 96.9% |

All angles are degrees. Common angle values include both feet and correlated targets from the same trajectory; they are not independent experiments.

The candidate has lower primary MAE than holding on 6 of these 7 recordings. Non-improved recordings: 57814129. This is an internal retrospective diagnostic, not a prospective held-out trial, and does not establish walking improvement.

Large errors remain. Confidence is an uncalibrated history-match score, not a bound on angle error or a contact probability. Near-reversal performance does not improve uniformly over holding.

JSON also contains lead0/30ms, post-publication reuse0/30/60ms, common-sample coverage, confidence strata, reversal strata and coarse direction agreement. Publication-to-first-control scheduling latency is not fully recorded; camera frame timestamps are not exposure-calibrated. No intervention or ground-contact simulation is performed.

Compiled candidate SHA-256: `1fd84e8d80ceeb1e41acb3ab8696d678d8c8d4bbace539193ca721f359375a1e`.
