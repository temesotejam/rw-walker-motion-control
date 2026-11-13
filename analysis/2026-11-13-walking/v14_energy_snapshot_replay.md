# v14 energy snapshot audit

Energy-v14 snapshot audit on recorded trajectories, not new closed-loop motion.

Each old decision supplies its preceding peak, recorded pre-output rate/current/
battery and previous same-side peak. Normal ZERO uses body angle0 as the real
v14 runner does; early opportunities use the saved support audit or latest past
body sample. Old
commands/peaks force-update only the wheel guard. The future response to a new
width, full branch eligibility and precise scheduler timing are not simulated.


The nominal rocker potential, conditional no-new-pulse arrival fit and inherited pulse-Q response are model estimates. Effective kinetic energy is not a measured body-inertia estimate; pulse-Q work is not measured shaft work. Potential still uses the fixed nominal rocker and mechanical angle0 reference; a moving-foot-pose-dependent potential minimum or energy-neutral angle is not implemented here. Old target stages are snapshots, not a replay of the new staged closed loop. See [passive-arrival audit](v14_passive_energy_audit.md) for the first-four-log training / latest-two-log held-out split.

| Log | Snapshots | Invalid | Drive | Coast | Cap-limited | Rate outside fit | Q outside fit | Any extrapolation | Wheel changes |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 97855607 | 118 | 0 | 82 | 36 | 46 | 61 | 25 | 111 | 9 |
| 241508524 | 120 | 0 | 107 | 13 | 78 | 65 | 36 | 118 | 21 |
| 611622951 | 123 | 0 | 108 | 15 | 77 | 77 | 38 | 118 | 0 |
| 141282685 | 113 | 0 | 88 | 25 | 54 | 74 | 24 | 108 | 10 |
| 43750941 | 108 | 0 | 95 | 13 | 64 | 70 | 25 | 103 | 0 |
| 137267573 | 115 | 0 | 50 | 65 | 12 | 58 | 2 | 86 | 8 |
| 267557373 | 116 | 0 | 63 | 53 | 22 | 53 | 16 | 98 | 16 |
| 27908524 | 100 | 0 | 69 | 31 | 38 | 49 | 19 | 92 | 14 |
| 64040782 | 132 | 0 | 114 | 18 | 80 | 88 | 38 | 125 | 0 |
| 84815132 | 93 | 0 | 82 | 11 | 61 | 67 | 26 | 92 | 0 |
| 57814129 | 122 | 0 | 57 | 65 | 15 | 48 | 2 | 86 | 6 |
| 199015154 | 77 | 0 | 73 | 4 | 39 | 62 | 47 | 72 | 23 |

## Normal / foot cap80 snapshots, separated from speculative cap15

| Log | Cap80 opportunities | Cap-limited | Coast | Q outside fit | Cap15 opportunities | Cap15 cap-limited |
|---|---:|---:|---:|---:|---:|---:|
| 97855607 | 50 | 10 | 4 | 25 | 68 | 36 |
| 241508524 | 53 | 27 | 0 | 36 | 67 | 51 |
| 611622951 | 57 | 25 | 1 | 38 | 66 | 52 |
| 141282685 | 51 | 15 | 4 | 24 | 62 | 39 |
| 43750941 | 48 | 19 | 1 | 25 | 60 | 45 |
| 137267573 | 42 | 0 | 6 | 2 | 73 | 12 |
| 267557373 | 43 | 1 | 1 | 16 | 73 | 21 |
| 27908524 | 36 | 6 | 0 | 19 | 64 | 32 |
| 64040782 | 60 | 28 | 1 | 38 | 72 | 52 |
| 84815132 | 41 | 20 | 1 | 26 | 52 | 41 |
| 57814129 | 43 | 2 | 1 | 2 | 79 | 13 |
| 199015154 | 77 | 39 | 4 | 47 | 0 | 0 |

The cap80 and cap15 limits are eligibility classes, not predictions of executed commands. Cap saturation is a residual energy deficit under the selected model. Q/rate extrapolation, transfer of a zero-cross fit to a finite body angle, and signed residual-current model gaps remain explicit.

## Model support

| Side | Passive intercept | Passive omega² coefficient | Passive rate band | Pulse-work rate band | Pulse-Q maximum |
|---|---:|---:|---:|---:|---:|
| -1 | 0.262475 mJ | 0.216457 mJ/(rad/s)² | 20.3130..35.5657 deg/s | 35.7375..91.4938 deg/s | 12.836790 mAs |
| 1 | 0.163016 mJ | 0.334588 mJ/(rad/s)² | 22.2512..37.5379 deg/s | 17.9508..92.2064 deg/s | 13.271380 mAs |

The positive passive intercept represents conditional stored-energy/coupling effects in past coast observations. Below20 deg/s it is scaled by (rate/20)² as an unvalidated extrapolation. The core accepts finite positive rates so low-speed rescue remains possible; the separate foot timing selector retains its own minimum rate. Above the observed passive band the same monotone quadratic is extrapolated, with flags. For the minus side the passive and pulse-work rate bands do not overlap, so joint rate support is absent.

## Matched-input sensitivity checks

Current angle0 deg, target8 deg, side+, battery7.5 V and I0=0 are held fixed. Both completed peak energies and the current rate actively alter the energy deficit and selected width. Common energy correction uses the mean of the two peaks; redistribution uses their signed imbalance. The incoming-to-arrival discrepancy remains a separate diagnostic and is not subtracted a second time.

| Input change | Rate | Same-side peak | Available | Peak correction | Request | Width | Coast |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 55 deg/s | 6 deg | 0.4713 mJ | 0.0467 mJ | 0.3088 mJ | 75 ms | False |
| weak_same_peak | 55 deg/s | 2 deg | 0.4713 mJ | 0.1002 mJ | 0.3623 mJ | 80 ms | False |
| large_same_peak | 55 deg/s | 11 deg | 0.4713 mJ | -0.0949 mJ | 0.1672 mJ | 55 ms | False |
| slow_rate | 25 deg/s | 6 deg | 0.2267 mJ | 0.0467 mJ | 0.5534 mJ | 80 ms | False |
| fast_rate | 80 deg/s | 6 deg | 0.8153 mJ | 0.0467 mJ | 0.0000 mJ | 0 ms | True |
| changed_incoming_peak | 55 deg/s | 6 deg | 0.4713 mJ | 0.0151 mJ | 0.2772 mJ | 71 ms | False |

## Foot-support snapshots from recent experiments

| Log / time | Old command | New energy width | Guarded | Available | Target | Peak correction | Request | Predicted work | Unmet after guard |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 97855607 / 12.143s | 15ms (True) | 0ms | 0ms | 0.5050 mJ | 0.3862 mJ | -0.0094 mJ | 0.0000 mJ | 0.0000 mJ | 0.0000 mJ |
| 97855607 / 15.399s | 15ms (True) | 0ms | 0ms | 0.4745 mJ | 0.3862 mJ | -0.0068 mJ | 0.0000 mJ | 0.0000 mJ | 0.0000 mJ |
| 97855607 / 18.625s | 47ms (True) | 0ms | 0ms | 0.5363 mJ | 0.4330 mJ | -0.0096 mJ | 0.0000 mJ | 0.0000 mJ | 0.0000 mJ |
| 97855607 / 28.894s | 65ms (True) | 70ms | 69ms | 0.5990 mJ | 0.7334 mJ | 0.0478 mJ | 0.1822 mJ | 0.1839 mJ | 0.0043 mJ |
| 241508524 / 15.715s | 0ms (False) | 80ms | 78ms | 0.4940 mJ | 0.7334 mJ | 0.0587 mJ | 0.2980 mJ | 0.2555 mJ | 0.0571 mJ |
| 611622951 / 9.561s | 28ms (True) | 31ms | 31ms | 0.5374 mJ | 0.5399 mJ | 0.0355 mJ | 0.0379 mJ | 0.0386 mJ | 0.0000 mJ |
| 611622951 / 12.759s | 0ms (False) | 73ms | 73ms | 0.5004 mJ | 0.6645 mJ | 0.0639 mJ | 0.2279 mJ | 0.2297 mJ | 0.0000 mJ |
| 611622951 / 23.661s | 51ms (True) | 80ms | 80ms | 0.3591 mJ | 0.7334 mJ | 0.0763 mJ | 0.4506 mJ | 0.2246 mJ | 0.2261 mJ |
| 611622951 / 25.359s | 0ms (False) | 80ms | 80ms | 0.2579 mJ | 0.7334 mJ | 0.0552 mJ | 0.5307 mJ | 0.2104 mJ | 0.3203 mJ |
| 611622951 / 28.858s | 56ms (True) | 80ms | 80ms | 0.3232 mJ | 0.7334 mJ | 0.0444 mJ | 0.4546 mJ | 0.2034 mJ | 0.2512 mJ |
| 141282685 / 7.429s | 15ms (True) | 0ms | 0ms | 0.5111 mJ | 0.3862 mJ | -0.0056 mJ | 0.0000 mJ | 0.0000 mJ | 0.0000 mJ |
| 141282685 / 9.034s | 15ms (True) | 0ms | 0ms | 0.5131 mJ | 0.3862 mJ | -0.0111 mJ | 0.0000 mJ | 0.0000 mJ | 0.0000 mJ |
| 141282685 / 10.652s | 15ms (True) | 0ms | 0ms | 0.5300 mJ | 0.3862 mJ | -0.0066 mJ | 0.0000 mJ | 0.0000 mJ | 0.0000 mJ |

This table compares eligibility/energy sizing at recorded decision opportunities. It does not establish how many new commands will execute, whether the gait improves, or physical energy conservation. The backend may reject stale/invalid telemetry, timing, direction or source changes. Signed residual current is audited but its coast-tail arrival contribution is unmodelled.

## Source snapshot

- `walking_energy_policy.h` SHA256 `ce3770bd04ab7c2585893ed891423ad6b50183cc3150f38aae9debb4c01b14e9`
- `lateral_energy_control.h` SHA256 `2004eca295b0215fd2fd16ca97ac3b7a3d03af6a99e2e58f292b431a11aa6d5a`
- `realtime_code.h` SHA256 `7181d44fd70a453cd4b2539e4ef3919315c4437e42e379044c561ef20003c927`
- `wheel_cycle_guard.h` SHA256 `5003755387958d714e74e4245e82d62cf34094470be7e35d2b3814827c8297b2`
