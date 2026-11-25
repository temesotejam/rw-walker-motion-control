"""Prepare a USB-powered camera and RWLOG transfer fixture under .pio/.

From the project root run "python -B test/hardware/prepare_motorfree.py", then
"pio run -c .pio/motorfree-fixture/platformio.ini". Neither command uploads.
Pass --archive PATH to stage a byte-exact replay under .pio/motorfree-replay.
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


def prepare(archive=None, target_free=22000, target_largest=12000):
    out = ROOT / ".pio" / "motorfree-replay" if archive else OUT
    files = {p.name for p in (ROOT / "src").iterdir() if p.is_file()} - SKIP
    out_src = out / "src"
    unexpected = {p.name for p in out_src.glob("*") if p.is_file()} - files - {"main.cpp"}
    if archive:
        unexpected -= {"archive_layout.h", "archive_blob.S", "motorfree_archive.cpp"}
    if unexpected:
        raise RuntimeError(f"Unexpected source in isolated fixture directory: {sorted(unexpected)}")
    for name in sorted(files):
        source = ROOT / "src" / name
        data = source.read_bytes()
        if name == "runtime_diagnostics.h":
            suffix = b"-archive-bench" if archive else b"-motorfree-bench"
            data, count = re.subn(rb'"(walking-support-transfer-v39-walk5-logfix[0-9]+)"',
                                  rb'"\1' + suffix + b'"', data)
            if count != 1:
                raise RuntimeError("Update the fixture revision marker for this production build")
        if name == "web_ui.cpp":
            for route in (b"void WebUi::command(RunControlWorker::Command cmd) {",
                          b's.on("/stop", HTTP_POST, [this]() {'):
                if data.count(route) != 1:
                    raise RuntimeError(f"Unrecognized motion endpoint: {route.decode()}")
                data = data.replace(route, route + b'\n    server_->send(403, "text/plain", "motor_free_transport_bench"); return;')
        if archive and name == "psram_logger.cpp":
            signature = b"bool PsramLogger::buildMetadataJson(PsramString& json) const {"
            if data.count(signature) != 1:
                raise RuntimeError("Archived metadata injection point changed")
            data = data.replace(signature, signature + b"\n  extern bool replayArchiveMetadata(PsramString&);"
                                b"\n  return replayArchiveMetadata(json);")
        write_changed(out_src / name, data)
    if archive:
        from archive_layout import pack_archive
        pack_archive(archive, out, target_free, target_largest, write_changed)
        source = Path(__file__).with_name("motorfree_archive.cpp")
        write_changed(out_src / source.name, source.read_bytes())
    write_changed(out_src / "main.cpp", Path(__file__).with_name("motorfree_main.cpp").read_bytes())
    for source in (ROOT / "third_party/esp32_camera_f0bb429").rglob("*"):
        if source.is_file():
            write_changed(out / source.relative_to(ROOT), source.read_bytes())
    config = (ROOT / "platformio.ini").read_text(encoding="utf-8")
    if config.count("[platformio]") != 1:
        raise RuntimeError("Expected a single [platformio] section")
    relative = out.relative_to(ROOT).as_posix()
    config = config.replace("[platformio]", f"[platformio]\nsrc_dir = {relative}/src\n"
                            f"build_dir = {relative}/build\nlibdeps_dir = .pio/libdeps")
    if archive:
        config = config.replace("build_flags =\n", "build_flags =\n  -DMOTORFREE_ARCHIVE=1\n", 1)
    write_changed(out / "platformio.ini", config.encode())
    print("Prepared", out / "platformio.ini")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, help="existing local RWLOG to replay")
    parser.add_argument("--target-free", type=int, default=22000, help="internal RAM headroom in bytes during replay")
    parser.add_argument("--target-largest", type=int, default=12000, help="largest internal allocation headroom in bytes")
    args = parser.parse_args()
    prepare(args.archive, args.target_free, args.target_largest)
