"""Prepare a USB-powered camera and RWLOG transfer fixture under .pio/.

From the project root run "python -B test/hardware/prepare_motorfree.py", then
"pio run -c .pio/motorfree-fixture/platformio.ini". Neither command uploads.
Pass --archive PATH to stage a byte-exact replay under .pio/motorfree-replay.
Add --measure to stage .pio/motorfree-measure. POST /bench/replay on this
image to append archived rows at their recorded times. Export normally, then
POST /clear for another measurement. Only the run ID and CRC vary across runs.
"""
import argparse
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / ".pio" / "motorfree-fixture"
SKIP = {"main.cpp", "roller485_manager.cpp", "experiment_runner.cpp", "imu_manager.cpp"}


def write_changed(path, data):
    if not path.exists() or path.read_bytes() != data:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)


def prepare(archive=None, target_free=22000, target_largest=12000, measure=False):
    if measure and not archive:
        raise ValueError("--measure needs a recorded --archive")
    out = OUT
    suffix = b"-motorfree-bench"
    if archive:
        out = ROOT / ".pio" / ("motorfree-measure" if measure else "motorfree-replay")
        suffix = b"-measure-bench" if measure else b"-archive-bench"
    firmware = ROOT / "src"
    files = {p.relative_to(firmware).as_posix() for p in firmware.rglob("*") if p.is_file()} - SKIP
    # The camera HAL refers back to ../../src/. Mirror both source trees under
    # one root so older benches' src/ copies cannot enter this build.
    stage = out / "staged"
    out_src = stage / "src"
    staged = {p.relative_to(out_src).as_posix() for p in out_src.rglob("*") if p.is_file()}
    unexpected = staged - files - {"main.cpp"}
    if archive:
        unexpected -= {"archive_layout.h", "archive_blob.S", "motorfree_archive.cpp"}
    if unexpected:
        raise RuntimeError(f"Unexpected source in isolated fixture directory: {sorted(unexpected)}")
    for name in sorted(files):
        source = ROOT / "src" / name
        data = source.read_bytes()
        if name == "runtime_diagnostics.h":
            data, count = re.subn(rb'"(walking-support-transfer-v39-walk[0-9]+(?:-logfix[0-9]+)?)"',
                                  rb'"\1' + suffix + b'"', data)
            if count != 2:
                raise RuntimeError("Update the fixture revision markers for this production build")
        if name == "web_ui.cpp":
            for route in (b"void WebUi::command(RunControlWorker::Command cmd) {",
                          b's.on("/stop", HTTP_POST, [this]() {'):
                if data.count(route) != 1:
                    raise RuntimeError(f"Unrecognized motion endpoint: {route.decode()}")
                guard = b'if (cmd != RunControlWorker::Command::Clear) ' if measure and route.startswith(b"void") else b""
                data = data.replace(route, route + b"\n    " + guard +
                                    b'{ server_->send(403, "text/plain", "motor_free_transport_bench"); return; }')
            if measure:
                # The ordinary walking session calls prepareCompletedExport() at finish.
                # The bench has its own start route; issue the same handoff on the HTTP owner.
                update = b"void WebUi::update() {"
                if data.count(update) != 1:
                    raise RuntimeError("Web update injection point changed")
                data = data.replace(update, update + b"""
  static uint32_t lastBenchFinishCheck;
  const uint32_t benchNow = millis();
  if (benchNow - lastBenchFinishCheck >= 50) {
    lastBenchFinishCheck = benchNow;
    if (export_->status().phase == ImmutableExport::Phase::Empty &&
        control_->commandsSettled() && control_->snapshot().downloadable)
      prepareCompletedExport();
  }""")
        if archive and name == "psram_logger.cpp":
            signature = b"bool PsramLogger::buildMetadataJson(PsramString& json) const {"
            if data.count(signature) != 1:
                raise RuntimeError("Archived metadata injection point changed")
            data = data.replace(signature, signature + b"\n  extern bool replayArchiveMetadata(PsramString&);"
                                b"\n  return replayArchiveMetadata(json);")
        write_changed(out_src / name, data)
    if archive:
        from archive_layout import pack_archive
        pack_archive(archive, out_src, target_free, target_largest, write_changed)
        source = Path(__file__).with_name("motorfree_archive.cpp")
        write_changed(out_src / source.name, source.read_bytes())
    write_changed(out_src / "main.cpp", Path(__file__).with_name("motorfree_main.cpp").read_bytes())
    for source in (ROOT / "third_party/esp32_camera_f0bb429").rglob("*"):
        if source.is_file():
            write_changed(stage / source.relative_to(ROOT), source.read_bytes())
    config = (ROOT / "platformio.ini").read_text(encoding="utf-8")
    if config.count("[platformio]") != 1:
        raise RuntimeError("Expected a single [platformio] section")
    relative = out.relative_to(ROOT).as_posix()
    src_dir = out_src.relative_to(ROOT).as_posix()
    config = config.replace("[platformio]", f"[platformio]\nsrc_dir = {src_dir}\n"
                            f"build_dir = {relative}/build\nlibdeps_dir = .pio/libdeps")
    if archive:
        flags = "  -DMOTORFREE_ARCHIVE=1\n" + ("  -DMOTORFREE_MEASURE=1\n" if measure else "")
        config = config.replace("build_flags =\n", "build_flags =\n" + flags, 1)
    write_changed(out / "platformio.ini", config.encode())
    print("Prepared", out / "platformio.ini")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, help="existing local RWLOG to replay")
    parser.add_argument("--measure", action="store_true", help="repeat acquisition at recorded row times on one boot")
    parser.add_argument("--target-free", type=int, default=22000, help="internal RAM headroom in bytes during replay")
    parser.add_argument("--target-largest", type=int, default=12000, help="largest internal allocation headroom in bytes")
    args = parser.parse_args()
    prepare(args.archive, args.target_free, args.target_largest, args.measure)
