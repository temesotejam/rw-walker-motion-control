# Foot support assist counterfactual eligibility

This is a compiled-estimator/selector replay on unchanged recorded camera and body trajectories. It is not a simulation of the physical response to a new motor command. Every candidate remains subject to runner timing, driver, battery, wheel, current and width-preservation guards that are not fully reproduced here.

Profile: default. Pose lead: 0 ms. Minimum confidence: 0.5. Tracker cadence: candidate. Shape anticipation: True.

Ordinary controller shadow is causally force-updated with recorded accepted old crossings, command widths/coast results and peaks. Later old data is still used after an earlier hypothetical candidate: candidates from different peaks are separate counterfactual eligibility checks, not one simulated new run.

| Log | Extra control delay | Controller-shadow candidates | Coast additions | Ordinary-drive relocations | Logged-stage mismatches |
|---|---:|---:|---:|---:|---:|
| 137267573 | 0 ms | 5 | 5 | 0 | 0/74 |
| 137267573 | 15 ms | 4 | 4 | 0 | 0/74 |
| 267557373 | 0 ms | 1 | 1 | 0 | 0/74 |
| 267557373 | 15 ms | 1 | 1 | 0 | 0/74 |
| 27908524 | 0 ms | 2 | 1 | 1 | 0/66 |
| 27908524 | 15 ms | 2 | 1 | 1 | 0/66 |

The final default shape evidence uses the Returning phase of the foot with the larger conservative forward angle, conservative forward >=1 degree, conservative inter-foot asymmetry >=1 degree, and incoming body peak <=8.5 degrees. It does not infer contact, a calibrated CAD transition, future deceleration or required mechanical work.

The strict camera confidence, margin and age gates remain. A coast exception requests 15 ms. Actual deceleration is separate causal evidence. The earlier two-consecutive-weak-body-peak idea was not adopted.

Primary timestamps below use the zero-extra-delay assumption. A reference width of zero is an old coast decision, not a delivered pulse.

| Log | Candidate time | Requested width | Old decision time | Old width | Advance |
|---|---:|---:|---:|---:|---:|
| 137267573 | 7.875 s | 15 ms | 7.963 s | 0 ms | 88 ms |
| 137267573 | 11.121 s | 15 ms | 11.201 s | 0 ms | 80 ms |
| 137267573 | 19.327 s | 15 ms | 19.407 s | 0 ms | 80 ms |
| 137267573 | 20.943 s | 15 ms | 21.026 s | 0 ms | 83 ms |
| 137267573 | 27.519 s | 15 ms | 27.586 s | 0 ms | 67 ms |
| 267557373 | 8.954 s | 15 ms | 9.036 s | 0 ms | 82 ms |
| 27908524 | 10.748 s | 15 ms | 10.835 s | 0 ms | 87 ms |
| 27908524 | 18.961 s | 46 ms | 19.014 s | 15 ms | 53 ms |

The former run 27908524 has no candidate immediately before its approximately26.5-second stopping episode in this replay. An eligibility result does not establish that this failure is prevented. Camera-frame timestamps are not calibrated exposure timestamps; publication/control latency remains approximate.
