# v20 downloadfix2: export-time restart investigation

2026-11-16. User reports walking motion, but RWLOG retrieval remains at zero
bytes and eventually reports the 120-second finalization timeout. The user
confirmed that neither power cycling nor switching devices occurred.

## Evidence and limits

`freefoot-diagnostics (5).json` identifies `walking-foot-fast-v20-downloadfix1`.
The browser started export at 05:00:10.455 UTC with boot ID 2760813921/run 1.
At 05:01:17.231 UTC the saved status has boot ID 1032406554, READY_TO_MEASURE,
run 0, samples 0, and export `empty`. Browser confirmed/cached/received bytes
are all zero. These observations and the user's confirmation establish a
device restart during retrieval. The screenshot's 120-second wait is not
evidence that one metadata build ran for 120 seconds: the old browser treated
the post-restart `empty` manifest as still building.

The current source log and browser partial data are absent in that diagnostic;
this attempt cannot be reconstructed from the JSON. HTTP status/manifest
read-only probes timed out and no USB port was present during this investigation.
No measurement, clear, restart or firmware write was performed by the agent.

The exact reset cause is **not yet established**: fix1 did not include the ESP
reset reason in the JSON. Metadata serialization had no blocking scheduler
break from its start until CRC calculation. The installed ESP32-S3 qio_opi SDK
configuration enables the CPU0 idle task watchdog with a five-second timeout
and panic. A long metadata build can therefore starve watchdog housekeeping.
This is a concrete defect and a plausible cause, not a captured watchdog report.
Old ELF entry frames inspected: export build 384, logger metadata 944,
foot metadata 1632 bytes, against a 12288-byte export task stack; this does not
by itself exclude a deeper stack fault, brownout or other reset cause.

## Change

- Only the export-owned PSRAM string has a progress callback. At append
  boundaries, after 2000 microseconds of elapsed work, it publishes progress and
  blocks for one RTOS tick. A plain scheduler yield would not allow the lower
  priority idle task to run. Callback is removed on both success and failure.
- Manifest includes boot ID and metadata byte progress. The browser distinguishes
  metadata construction from CRC calculation, and stops immediately on an empty
  export or a different boot ID. It preserves cached bytes but does not promise
  resumption when their source has disappeared.
- Status JSON includes `reset.reason`, `reset.name`, and previous RTC export
  phase/progress when a previous sample exists. The existing RTC journal is
  sampled during export even when USB reporting is disabled, without heap scans
  or USB output. This evidence is best effort and does not survive power loss.
- Walking energy policy, camera settings, motor limits and watchdog settings
  are unchanged. Old distributed installers are preserved.

## Verification and next step

Real C++ v19/v20 maximum-capacity serializers run with simulated elapsed time
through the production export callback, including timer wraparound. Their JSON
is byte-identical to serialization without the callback. The actual export
worker publishes a CRC-valid file; capacity failure publishes no chunks and
removes the callback. v20 retains 1280 foot rows, 192 energy rows, 96 support rows
within 985944 metadata bytes. These host checks are not board latency or reset
measurements.

Production browser JS passes empty/different/same boot tests, interrupted
download/resume, timeout/retry, CRC/token failure, cancel, and prior UI tests.
Bounded HTTP transport, embedded page consistency, and both installer profiles
pass. Build `atoms3cam-foot-fast-downloadfix2` succeeded in 184.895 seconds,
RAM 127620/327680, Flash 1924033/3342336. Binary: 1924400 bytes,
SHA256 `09b4be475f978dabefccfb50f8b53f21178796fe55a0dd24192bc39d4b7799f8`.
Eight installer files were checked over HTTP with exact byte equality; 32 files
in the previous four installers retain their hashes.

Installer: http://127.0.0.1:8767/foot-fast-downloadfix2/

Expected build ID: `walking-foot-fast-v20-downloadfix2`. The next hardware check
is a completed measurement followed by a CRC-verified RWLOG download with the
same boot ID. If the device resets again, save the new diagnostic JSON before
another power cycle so its reset reason and previous export evidence remain
available. Successful retrieval on hardware is still unverified.
