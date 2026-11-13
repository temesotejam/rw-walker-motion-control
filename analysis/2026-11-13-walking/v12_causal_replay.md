# v12 causal eligibility and local-evidence review

This is a compiled-header replay on unchanged recorded trajectories, with the same source-selection helper as the runner. Recorded old commands and peaks force-update the ordinary controller and portable wheel-cycle guard; each hypothetical first candidate is independent. This does not predict the response to a new pulse or prove walking improvement. Remaining current/battery/hardware/driver guards and full branch-health context are not simulated.

The cumulative ablations are: v11 historical prediction /60ms cap /normal ETA; add local Returning evidence only; add80ms cap without truncating the ordinary proposal; add the bounded near-stall ETA route. All retain source quality, motion, one-attempt-per-peak and short-lead limits.

| Log | Extra availability delay | v11 | +Local /60 | +Local /80 | +Local /80 /stall |
|---|---:|---:|---:|---:|---:|
| 141282685 | 0ms | 2 | 2 | 2 | 3 |
| 141282685 | 15ms | 0 | 0 | 0 | 1 |
| 137267573 | 0ms | 5 | 5 | 5 | 5 |
| 137267573 | 15ms | 4 | 4 | 4 | 4 |
| 267557373 | 0ms | 1 | 1 | 1 | 1 |
| 267557373 | 15ms | 1 | 1 | 1 | 1 |
| 27908524 | 0ms | 2 | 2 | 2 | 2 |
| 27908524 | 15ms | 2 | 2 | 2 | 2 |
| 64040782 | 0ms | 1 | 1 | 1 | 1 |
| 64040782 | 15ms | 1 | 1 | 1 | 1 |
| 84815132 | 0ms | 4 | 4 | 4 | 5 |
| 84815132 | 15ms | 4 | 4 | 4 | 5 |
| 57814129 | 0ms | 0 | 0 | 0 | 0 |
| 57814129 | 15ms | 0 | 0 | 0 | 0 |
| 199015154 | 0ms | 9 | 10 | 10 | 10 |
| 199015154 | 15ms | 7 | 7 | 7 | 7 |

V12 uses a copy of the wheel guard to preview the same raw ordinary proposal; its selector receives the already-guarded width, rather than ignoring a later reduction or applying it twice. Every selected candidate also records a separate backend guard evaluation and width-preservation result. This preserves the existing wheel guard rather than bypassing it.

| Log | Delay | V12 full selector candidates | Wheel-width-preserved candidates |
|---|---:|---:|---:|
| 141282685 | 0ms | 3 | 3 |
| 141282685 | 15ms | 1 | 1 |
| 137267573 | 0ms | 5 | 5 |
| 137267573 | 15ms | 4 | 4 |
| 267557373 | 0ms | 1 | 1 |
| 267557373 | 15ms | 1 | 1 |
| 27908524 | 0ms | 2 | 2 |
| 27908524 | 15ms | 2 | 2 |
| 64040782 | 0ms | 1 | 1 |
| 64040782 | 15ms | 1 | 1 |
| 84815132 | 0ms | 5 | 5 |
| 84815132 | 15ms | 5 | 5 |
| 57814129 | 0ms | 0 | 0 |
| 57814129 | 15ms | 0 | 0 |
| 199015154 | 0ms | 10 | 10 |
| 199015154 | 15ms | 7 | 7 |

## Fixed failure window:141282685,27.622-27.958s

| Delay | First full candidate | Width | ETA | Old rescue advance |
|---|---:|---:|---:|---:|
| 0ms | 27.921s | 68ms | 263.5ms | 37ms |
| 15ms | 27.932s | 68ms | 295.9ms | 26ms |

At the same query near27.921s, v11 rejects PoseUncertain; local-only rejects Width for the guarded68ms ordinary proposal (raw70ms); width80 without stall rejects TooEarly because ETA exceeds250ms; all three changes permit a68ms candidate. The old15ms rescue at27.958s is not evidence that this hypothetical68ms action succeeds.

The wheel preview uses the last accepted negative pre-input378.91rpm and current1.81rpm at27.921s. The existing midpoint-bias guard reduces raw70ms to68ms (no extra peak-qualified reduction). The selector requests68ms and a separate evaluation of the same raw70ms returns68ms again, so width preservation succeeds. Sending raw70ms as the requested width would instead fail this existing guard.

A reconstructed v11 full-run count can differ from its3 real audits: sampled body times, publication scheduling and conservative old-command-first ties are not the exact runner. The failure-window comparison is performed identically across variants.

## Local prediction checks

Returning is evidence from three already-observed frames, not guaranteed current physical direction. Future camera frames are scorer-only and are not independent angular truth. Current-state point accuracy is mostly similar to holding; the purpose is to separate usable local observations from missing historical matches.

| Log | Local side margin<=.75: values | Local MAE | Hold MAE | Empirical margin coverage | Actual local-fallback values | Fallback MAE | Fallback linear MAE | Fallback max | Coarse opposite Returning |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 141282685 | 286 | 0.2504 | 0.2520 | 87.1% | 32 | 0.5701 | 0.5583 | 2.1281 | 2/14 |
| 137267573 | 202 | 0.2153 | 0.2186 | 91.6% | 10 | 0.4364 | 0.4039 | 1.0260 | 2/4 |
| 267557373 | 206 | 0.2527 | 0.2526 | 88.3% | 8 | 0.7033 | 0.5360 | 1.9405 | 3/7 |
| 27908524 | 245 | 0.1879 | 0.1905 | 93.9% | 20 | 0.4139 | 0.3747 | 1.6402 | 1/5 |
| 64040782 | 230 | 0.4681 | 0.4656 | 74.3% | 24 | 0.8870 | 0.6502 | 2.1637 | 0/10 |
| 84815132 | 327 | 0.1151 | 0.1173 | 96.0% | 10 | 0.4058 | 0.4054 | 2.0736 | 0/3 |
| 57814129 | 45 | 0.3468 | 0.3625 | 80.0% | 4 | 0.9261 | 0.7946 | 1.5131 | 1/2 |
| 199015154 | 301 | 0.4657 | 0.4696 | 76.1% | 30 | 1.2467 | 1.0838 | 3.6238 | 0/14 |

The local correction is limited to25ms and0.5degree instead of extrapolating the full observation age. Across all Returning strata it reduces mean error compared with old straight-line extrapolation; in the small actual-fallback strata, straight-line prediction can have lower MAE than the conservative local point estimate. No general accuracy superiority is claimed.

Margins are heuristic allowances, not calibrated error bounds. Coarse direction uses future camera central differences, which can span a reversal and share recognition error. Small fallback strata do not establish a reliable error probability. Historical score is preserved rather than replaced by invented confidence.

V11 logs use actual saved ready_ms; older logs use approximate delivered+image-processing+CAD times. Added delays are timing sensitivity, not measured v12 control deadlines. New local computation/publication overhead remains unmeasured. Header hashes and the compiled harness hash are in the JSON.
