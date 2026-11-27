"""Exercise native v38 and legacy v24 camera tracking and row-buffer handoff."""
from pathlib import Path
import re
import subprocess
import tempfile

from _project import ROOT
from fixtures import camera_pipeline


V24_FLAGS = (
    "FOOT_LOW_LATENCY_V20", "FOOT_SUPPORT_ASSIST", "FOOT_SUPPORT_LOCAL", "WALKING_RETURN_ENERGY",
    "FOOT_IMAGE_ROWS_TEST_READS", "FOOT_SPEED_V22", "FOOT_SPEED_FIX1", "FOOT_CAPTURE_V23",
    "FOOT_ROW_OVERLAP_V24",
)
FLAGS = (
    *V24_FLAGS, "FOOT_NATIVE_SKIP_V25", "FOOT_NATIVE_STREAM_V26", "FOOT_EXPORT_WEBFIX",
    "FOOT_COPY_DELTA_V27", "FOOT_TEMPLATE_PRUNE_V28", "FOOT_OVERLAP_ALWAYS_V29", "FOOT_RECEIVE_DIAG_V30",
    "FOOT_HISTOGRAM_V32", "FOOT_WORKER26_V34", "FOOT_LIVE_STATUS_V35", "FOOT_STABLE40_V36",
    "FOOT_CADENCE20_V37", "FOOT_PERIODIC_V38",
)


def main():
    legacy = camera_pipeline.fixture()
    code = '#include "foot_latency_histogram.h"\n' + legacy
    code = code.replace(
        "assert(foot_fast_policy::kArchiveCapacity==1280);",
        "assert(foot_fast_policy::kArchiveCapacity==2048 && foot_fast_policy::kWorkerPeriodUs==20000);",
    )
    # 120 frames provide the fixture with the required 2 s dwell at the 20 ms cadence.
    indices = {"69": "119", "70": "120", "78": "128", "81": "131"}
    for signature in ("static std::vector<Plan> scenario()", "static void fullPipeline()"):
        section = camera_pipeline.function(code, signature)
        updated = re.sub(r"\b(69|70|78|81)\b", lambda match: indices[match[0]], section)
        code = code.replace(section, updated)
    with tempfile.TemporaryDirectory(prefix="foot-pipeline-") as temp:
        tmp = Path(temp)
        (tmp / "Arduino.h").write_text(camera_pipeline.ARDUINO, encoding="utf8")
        for name, flags, fixture in (("v38", FLAGS, code), ("v24", V24_FLAGS, legacy)):
            source = tmp / f"{name}.cpp"
            source.write_text(fixture, encoding="utf8")
            binary = tmp / f"{name}.exe"
            subprocess.run([
                "g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                *(f"-D{flag}=1" for flag in flags),
                "-I", tmp, "-I", ROOT / "src", source, ROOT / "src/white_marker_tracker.cpp",
                ROOT / "src/foot_angle_estimator.cpp", "-o", binary,
            ], check=True)
            subprocess.run([binary], check=True)
    print("PASS v38 observer at 20 ms cadence and v24 legacy row-buffer fallback")


if __name__ == "__main__":
    main()
