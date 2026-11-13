# v14 passive-arrival energy audit

Coefficient fitting uses only the first four trials; the two latest trials are excluded from that fit. All six trials were already inspected in an exploratory audit, so this comparison is not untouched or blinded model-selection validation.

`E_arrival[mJ] = offset + coefficient * omega[rad/s]^2`. Observed energy is the nominal, zero-CG-shift rocker `U(next peak)`.

| Side | Offset mJ | Coefficient mJ/(rad/s)^2 | Train n / rate dps | Train RMSE mJ | Held n / rate dps | Held RMSE mJ | Held max error mJ |
|---|---:|---:|---|---:|---|---:|---:|
| +1 | 0.163015668603 | 0.334588149898 | 40 / 22.2512..37.5379 | 0.00720832 | 9 / 25.6811..38.4680 | 0.00860403 | 0.01687048 |
| -1 | 0.262474859449 | 0.216456595216 | 12 / 20.3130..35.5657 | 0.01364325 | 4 / 21.0694..30.2227 | 0.01689086 | 0.02512085 |

Training trials: 137267573, 267557373, 141282685, 611622951. Held-out trials: 97855607, 241508524.

Selection: branch15 valid normal ZERO coast; first chronological accepted same-side peak 125..600ms later; no executed pulse from coast through next peak.

This fit is conditional on the old controller selecting coast. No new command does not eliminate stored wheel/foot energy exchange. Its positive intercept is not static kinetic energy or independently measured body inertia. The passive-arrival fit already includes losses; do not subtract the same loss again. Only arrival prediction is checked here, not actuator work calibration or new closed-loop walking.

The low-rate data do not identify behavior near zero or at high rates. Any connector to a legacy model needs continuous, monotone energy versus rate and explicit extrapolation labels. A naive 35..45 dps blend is nonmonotone for these fits. Training/held-out membership must remain fixed in future replays.

Reproduce: `python tools/audit_passive_energy_v14.py`. Detailed model metrics and the 65 selected rows are saved beside this report as JSON/CSV.
