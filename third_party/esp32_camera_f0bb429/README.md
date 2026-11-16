# Pinned ESP32 camera HAL for the v21-v24 trials

Upstream: https://github.com/espressif/esp32-camera/tree/f0bb429

- `cam_hal.c`: `driver/cam_hal.c`, modified locally.
- `cam_hal.h`: `driver/private_include/cam_hal.h`, unmodified.
- `ll_cam.h`: `target/private_include/ll_cam.h`, unmodified.
- `LICENSE`: upstream Apache-2.0 license, unmodified.
- `UPSTREAM.json`: SHA256 of the downloaded upstream bytes before modification.

Local modifications publish complete GC0308 QVGA grayscale frames at the final
DMA EOF instead of waiting for the following VSYNC. They apply only to 20MHz
SDK initialization with internal DMA copying, 1-byte input/output pixels and exactly 76800
copied bytes. DMA stops before queue publication; the state remains IDLE until
the next VSYNC. Partial frames retain the upstream size check. JPEG, other
geometry/formats and direct-PSRAM DMA retain VSYNC publication. A shared helper
retains the upstream latest-frame replacement and buffer ownership policy.
Independent, atomic boot-lifetime counters report successful early/VSYNC
publications, queue discards and the last completed frame's driver-start age.

The isolated `FOOT_CAPTURE_V23` profile subsequently changes XCLK at boot to
PLL160 divided by7 (nominal22.857MHz). Its DMA layout keeps the same internal
allocation but halves each chunk, providing four physical slots and a queue of
three events. Layout checks require an aligned integral division of every frame;
they fail initialization on an unexpected layout. The production EOF loop and
complete-frame requirement remain unchanged. This reduces queue overflow in the
USB idle comparison; it does not eliminate all incomplete frames. See
`docs/FOOT_CAPTURE_V23.md` for measurements and limits.

The conditional `FOOT_ROW_OVERLAP_V24` path adds an independently owned,
single-slot upper-row mailbox. After each copied DMA chunk, the HAL offers the
available prefix only for the same internal-DMA QVGA grayscale path. Once that
prefix includes every required upper row, the producer copies 26 rows (8320
bytes) into a separate PSRAM buffer; it never lends an in-progress camera
buffer to the foot task. A frame timestamp is offered at most once. Atomic
`Free`/`Writing`/`Ready`/`Reading` ownership prevents the producer from overwriting
a slot being read. An unread `Ready` slot can be replaced by a newer frame; a
busy slot skips that frame without blocking the HAL. Counters distinguish
offers, busy skips, replacements and consumed slots.

`rw_camera_take_overlap` is used only by continuous acquisition. It checks the
complete-frame queue before speculative work, calls `rw_camera_overlap_poll`
in the consuming foot task, then checks the queue again before a one-tick wait.
These short waits share one elapsed-tick timeout budget; they do not repeatedly
call `cam_take` or reset DMA. At a full timeout it makes one final nonblocking
queue check. On ESP32-S3, an empty queue then uses the legacy single DMA reset
and one additional wait of the requested timeout. Consequently, the argument is
not a hard wall-clock upper bound: recovery can wait a second timeout, and a
running preparation callback is not interrupted at the deadline. Unsupported
formats or DMA modes fall back to `cam_take`; manual and boot acquisition retain
the original path.

The mailbox callback invalidates the prior complete-frame row cache, copies the
upper rows from the PSRAM mailbox into that cache's existing internal-RAM
storage as scratch, and prepares upper-marker image features there. Only the
foot task uses this scratch, and acquisition completes before the normal
complete-frame row capture refills it. This reuses the existing 21120-byte
internal allocation instead of allocating another upper-row buffer internally;
the foot task retains its 12288-byte stack. The callback neither updates
tracking history nor publishes a foot observation. The normal full
76800-byte frame must still arrive and pass frame/timestamp validation. Only
when its driver timestamp matches the prepared candidate does the consumer
commit those features through the tracker. Otherwise it processes the complete
frame's copied rows normally. The lower marker follows that normal path in both
cases. Preparation runs in the foot task, not in the ISR or the HAL task. This
overlaps image computation with reception; it does not change sensor readout,
the complete-frame publication rule, or establish a measured latency reduction.

Only `src/camera_frame_ready_hal.c` with `CAMERA_FRAME_READY_V21=1` includes this
source. The full HAL supplies the existing public symbols so the static SDK
archive's HAL object is not extracted. The SDK still supplies low-level DMA,
sensor and camera initialization code. Global SDK files are not modified.

The isolated `FOOT_NATIVE_SKIP_V25` profile admits GC0308 grayscale320x236
frames (75520 bytes) after configuring native row skipping. Its1280-byte DMA
chunks use30720 bytes of DMA storage. The raw geometry is distinct from the
logical320x240 tracker coordinates; all requested logical rows map to actual
received rows. Only a complete raw frame is published. A bounded single queued
frame discard at acquisition entry prevents old completed frames from adding
latency; live polling never calls the SDK timeout-reset path with zero timeout.

`FOOT_NATIVE_STREAM_V26` adds an independently owned lower-row mailbox. The HAL
task offers cumulative row snapshots at four progress points, never in the ISR.
The consumer computes image-only features and commits tracking only after the
complete raw frame and64-bit timestamp match. Ready snapshots may be replaced;
Reading/Writing snapshots cannot. Incomplete same-frame feature preparation
may finish from the consumer-owned complete row cache after delivery. This
preserves the normal rejection/fallback path and the1280-frame archive.

`CAMERA_GEOMETRY_PROBE_V25` is a separate camera-only bench and cannot coexist
with the native production profile. It allocates VGA capacity and returns only
received whole DMA prefixes at VSYNC, with event timing and failure counters.
A bench prefix is not asserted to be a complete physical image without separate
HREF geometry and length validation. See `docs/FOOT_NATIVE_V25.md` for measured
results, rejected settings and the actual-image orientation check.

This private ABI is pinned to Arduino 2.0.16 / IDF 4.4.7 and the camera archive
fingerprint in `tools/check_camera_hal_abi.py`. A different archive fails the
build rather than silently mixing private layouts. Re-verify headers, sdkconfig,
low-level EOF semantics, task priority wrapping and map ownership before any
SDK upgrade. Source/event tests do not establish board timing or image quality.
