# Pinned ESP32 camera HAL for the v21 trial

Upstream: https://github.com/espressif/esp32-camera/tree/f0bb429

- `cam_hal.c`: `driver/cam_hal.c`, modified locally.
- `cam_hal.h`: `driver/private_include/cam_hal.h`, unmodified.
- `ll_cam.h`: `target/private_include/ll_cam.h`, unmodified.
- `LICENSE`: upstream Apache-2.0 license, unmodified.
- `UPSTREAM.json`: SHA256 of the downloaded upstream bytes before modification.

Local modifications publish complete GC0308 QVGA grayscale frames at the final
DMA EOF instead of waiting for the following VSYNC. They apply only to 20MHz
XCLK with internal DMA copying, 1-byte input/output pixels and exactly 76800
copied bytes. DMA stops before queue publication; the state remains IDLE until
the next VSYNC. Partial frames retain the upstream size check. JPEG, other
geometry/formats and direct-PSRAM DMA retain VSYNC publication. A shared helper
retains the upstream latest-frame replacement and buffer ownership policy.
Independent, atomic boot-lifetime counters report successful early/VSYNC
publications, queue discards and the last completed frame's driver-start age.

Only `src/camera_frame_ready_hal.c` with `CAMERA_FRAME_READY_V21=1` includes this
source. The full HAL supplies the existing public symbols so the static SDK
archive's HAL object is not extracted. The SDK still supplies low-level DMA,
sensor and camera initialization code. Global SDK files are not modified.

This private ABI is pinned to Arduino 2.0.16 / IDF 4.4.7 and the camera archive
fingerprint in `tools/check_camera_hal_abi.py`. A different archive fails the
build rather than silently mixing private layouts. Re-verify headers, sdkconfig,
low-level EOF semantics, task priority wrapping and map ownership before any
SDK upgrade. Source/event tests do not establish board timing or image quality.
