# Camera receive comparison v30 / v31 (2026-11-17)

## Change and measurement boundary

v30 (`atoms3cam-foot-receive-diag`, `walking-foot-receive-diag-v30`) retains v29's sensor geometry, row copies and tracker. The HAL records ISR-to-task dispatch delay, short-frame rejection and event queue drops without printing text for every error in the receive path. Errors are counted, not accepted or hidden. `/status.json.camera_receive` exposes independent boot-lifetime atomic counters. Counter deltas wrap modulo 2^32; timing maxima include preemption and are not pure CPU execution cost. An 8-byte event replaces the 4-byte event in this profile; the queue remains 23 events backed by 24 physical DMA slots.

v31 (`atoms3cam-foot-receive-core1`, `walking-foot-receive-core1-v31`) additionally moves only the HAL task to Core1, priority3. IMU priority6 and control priority4 retain priority over it. The foot recognizer stays Core0, priority1. Sensor HB96, XCLK22.857MHz, raw320x236, logical row origin2, full-frame length75520 and energy controller v19 remain unchanged. Previous Core1 trials moved the recognizer, so they were a different experiment.

`short_start_delay_us` refers to the rejected frame's starting VSYNC, not the boundary that ends it. Driver timestamps still come from the existing HAL acquisition path; measurements are not calibrated exposure-to-control latency. Current status is sampled, not a record of every frame.

## Hardware conditions and a confounded comparison

USB-only supply was confirmed by the user; no motor/measurement-start command was sent. COM7 MAC F0:9E:9E:32:53:0C was checked before app-only writes. Every successful sample checks stopped motor, no pending command and unchanged boot/build. HTTP is polled at about5Hz with a simultaneous passive USB reader. Heavy diagnostic commands are not enabled. The normal Web idle polling can be faster than this bench; walking disables HTTP and has a different workload.

The user subsequently confirmed that the machine had been repositioned over several minutes, overlapping the first v31 window. Those windows cannot isolate a firmware effect. In particular, the initial drop in marker detection is NOT established as a v31 regression. All data are retained, and comparisons are repeated in the user's safer lying position. Zero-point readiness in that posture is distinct from marker detection and walking readiness.

| Initial window (45s, confounded across versions) | Receipt median / p95 / maximum ms | Both detected / sampled | Short frames / queue drops |
|---|---|---|---|
| v29 before edits |27.227 /45.452 /56.012|224/224|414/43 (USB text)|
| v30 first |27.038 /48.987 /61.298|225/225|353/49 (counter delta)|
| v30 warm |26.776 /34.120 /45.801|224/224|294/12 (counter delta)|
| v31 first, posture changing |25.116 /30.800 /35.936|201/211|694/6 (counter delta)|

The initial quiet period of v30 had fewer receive errors than the later polled window. That alone does not establish a logging improvement. USB error text is deliberately absent in v30/v31; compare the actual counters, not the absence of text.

## Stable lying repeats and final choice

| Window | Receipt median / p95 / maximum ms | Both detected / sampled | Short frames / queue drops | Captured-frame counter Hz |
|---|---|---|---|---|
| v31 lying, after posture confirmation |25.329 /29.681 /31.847|207/214|721/8|26.742|
| v30 lying, first after reflash |27.643 /45.143 /59.222|212/221|353/64|26.319|
| v30 lying, warm |26.737 /47.849 /62.676|202/203|304/43|26.795|

The first lying v30 window had initial low contrast and a preview transfer during sampling; exclude it from a controlled speed comparison. The warm window had no preview request. v31's first lying window and v30's warm window share the lying posture, but differ in boot, exposure history and tracker initialization. They are not randomized paired all-frame trials. Both retain some receive/recognition errors. v31 has shorter observed latency and fewer queue drops but more short-frame rejection and lower sampled marker detection. These measurements do not identify a single causal defect or prove equivalence of angular accuracy.

**Final hardware: v30, boot1797503538, stopped on USB.** Keep v31 as a comparison profile, not the selected walking firmware. The new receive telemetry and removal of per-error console work are retained in v30. Additional latency reduction relative to v29 is NOT demonstrated; the20ms target remains unmet. No motors ran. Zero calibration is not ready after reboot in the lying posture (`body_moving` gate); marker detection is separate and was202/203 in the last45s window. Do not infer walking-ready status from this bench. The gate was not bypassed.

The current bottleneck is still delivery of a complete image, around22–23ms median in these v30 windows, plus preparation/recognition/scheduling and control receipt. Receive dispatch tails reach tens of milliseconds under status traffic. Copy/offer duration includes preemption, so it must not be interpreted as pure memcpy cost. The next investigation should separate physical DMA boundary timing, queue age and interrupted copy ownership with complete-frame integrity checks; simply increasing event queue length beyond physical slots is not valid. The core1 variant also needs integrity/repeatability evidence before adoption.

Pages: selected v30 `http://127.0.0.1:8767/foot-receive-diag/`; comparison v31 `http://127.0.0.1:8767/foot-receive-core1/`; preserved v29 `http://127.0.0.1:8767/foot-overlap-always/`.

## Verification and artifacts

- Actual native HAL ownership/partial-frame tests with v30 tagged events: PASS, including one missing chunk rejection, own-frame start-delay attribution, ISR queue failure, timer wrap, and borrowed-buffer ownership. Legacy HAL and geometry probe tests pass. The probe fixture's generic source marker initially selected the new event type; its selection was made explicit and the test passed again.
- Actual task-creation wrapper for both core affinities: PASS; camera priority/stack, IMU/control pass-through and creation-failure handling are checked.
- Full foot/energy metadata fixtures pass; maximum v30 metadata1001546 and v31 metadata1001547 bytes of1048576. No energy control schema or behavior changes.
- Fifteen installer profiles pass; dedicated pages preserve prior firmware. Both new pages' eight served files match disk. Embedded Web check and linked HAL verification pass.
- Historical `test_camera_coexistence.py` stops at a pinned old hash for `src/experiment_runner.cpp`. That file is unchanged in this work; HEAD blob94473cbbe2d7d79178ea091a039d675a2c651acf differs from the historical expected58688977951627cdab038d73ca4d64b0c6b3d645. This legacy test is not reported as passing or as a newly introduced failure.

| Profile | Build | App bytes | SHA256 |
|---|---|---|---|
| v30 | SUCCESS168.911s |1951600|`8f985243245817653a6f180c77a81c11b53d4d80dfce5adf2a02a1489e94d141`|
| v31 | SUCCESS186.578s |1951600|`ab785f234b5b383a8a5eb7e308cc66493e1e933159b534507e481326162289cd`|

Local evidence is under `analysis/2026-11-17-camera-bench/`: `v29_receive_baseline*`, `v30_receive_first*`, `v30_receive_warm*`, `v31_receive_first*`, each version's build/flash/metadata files, and subsequent `*_lying_*` repeats. Raw data and binaries remain local.

Walking, repeated walking-log download and closed-loop deadline behavior are not tested by these stopped measurements. A shorter elapsed time for an accepted frame does not establish better walking or fewer missed observations.
