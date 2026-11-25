"""Prepare a USB-powered camera and RWLOG transfer fixture under .pio/.

From the project root run "python -B test/hardware/prepare_motorfree.py", then
"pio run -c .pio/motorfree-fixture/platformio.ini". Neither command uploads.
"""
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / ".pio" / "motorfree-fixture"
SKIP = {"main.cpp", "roller485_manager.cpp", "experiment_runner.cpp", "imu_manager.cpp"}


def write_changed(path, data):
    if not path.exists() or path.read_bytes() != data:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)


def prepare():
    files = {p.name for p in (ROOT / "src").iterdir() if p.is_file()} - SKIP
    out_src = OUT / "src"
    unexpected = {p.name for p in out_src.glob("*") if p.is_file()} - files - {"main.cpp"}
    if unexpected:
        raise RuntimeError(f"Unexpected source in isolated fixture directory: {sorted(unexpected)}")
    for name in sorted(files):
        source = ROOT / "src" / name
        data = source.read_bytes()
        if name == "runtime_diagnostics.h":
            data, count = re.subn(rb'"(walking-support-transfer-v39-walk5-logfix[0-9]+)"',
                                  rb'"\1-motorfree-bench"', data)
            if count != 1:
                raise RuntimeError("Update the fixture revision marker for this production build")
        if name == "web_ui.cpp":
            for route in (b"void WebUi::command(RunControlWorker::Command cmd) {",
                          b's.on("/stop", HTTP_POST, [this]() {'):
                if data.count(route) != 1:
                    raise RuntimeError(f"Unrecognized motion endpoint: {route.decode()}")
                data = data.replace(route, route + b'\n    server_->send(403, "text/plain", "motor_free_transport_bench"); return;')
        write_changed(out_src / name, data)
    write_changed(out_src / "main.cpp", Path(__file__).with_name("motorfree_main.cpp").read_bytes())
    for source in (ROOT / "third_party/esp32_camera_f0bb429").rglob("*"):
        if source.is_file():
            write_changed(OUT / source.relative_to(ROOT), source.read_bytes())
    config = (ROOT / "platformio.ini").read_text(encoding="utf-8")
    if config.count("[platformio]") != 1:
        raise RuntimeError("Expected a single [platformio] section")
    config = config.replace("[platformio]", "[platformio]\nsrc_dir = .pio/motorfree-fixture/src\n"
                            "build_dir = .pio/motorfree-fixture/build\nlibdeps_dir = .pio/libdeps")
    write_changed(OUT / "platformio.ini", config.encode())
    print("Prepared", OUT / "platformio.ini")


if __name__ == "__main__":
    prepare()
