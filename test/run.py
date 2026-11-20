"""Run host-side checks for the active walking firmware and its offline tools."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from _project import ROOT
from fixtures.runtime_rwlog import write_fixture
from convert_rwlog_to_csv import parse_header


def checked(*command, **kwargs):
    subprocess.run(command, cwd=ROOT, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", default="node", help="Node.js executable (18 or newer)")
    parser.add_argument("--rwlog", type=Path, action="append", default=[],
                        help="Optional v37 walking recording for actual conversion and foot-angle replay")
    args = parser.parse_args()
    node = shutil.which(args.node)
    if not node or not shutil.which("g++"):
        parser.error("Put g++ and Node.js on PATH, or specify an installed Node.js executable with --node.")
    logs = [path.resolve(strict=True) for path in args.rwlog]
    with tempfile.TemporaryDirectory(prefix="walker-host-") as space:
        tmp = Path(space)
        py = [sys.executable, "-B"]
        trial_root = tmp / "recordings"
        converted_trials = []
        for rwlog in logs:
            run_start_us = parse_header(rwlog.read_bytes())["run_start_us"]
            converted = trial_root / str(run_start_us)
            checked(*py, ROOT / "tools/convert_rwlog_to_csv.py", rwlog, "--out", converted)
            converted_trials.append(converted)
        event_env = dict(os.environ, WALKER_TEST_TRIALS=str(trial_root)) if logs else None
        checked(*py, "-m", "unittest", "discover", "-s", "test", "-p", "test_event_foot*.py", env=event_env)
        for test in ("energy_admission_runner_v19", "energy_admission_metadata_v19", "half_cycle_energy_budget",
                     "foot_pipeline", "foot_native_geometry", "foot_cadence", "rwlog_v46_converter",
                     "foot_periodic40_flasher"):
            checked(*py, ROOT / "test" / f"test_{test}.py")
        for test in ("periodic_foot", "mekf_host", "fore_aft_motion_guard"):
            binary = tmp / (test + ".exe")
            source = [ROOT / "test" / f"test_{test}.cpp"]
            if test == "mekf_host":
                source.append(ROOT / "src/mekf6.cpp")
            checked("g++", "-std=c++17", "-O2", "-I", ROOT / "src", *source, "-o", binary)
            checked(binary, stdin=subprocess.DEVNULL)
        fixture = tmp / "synthetic-v56.rwlog"
        write_fixture(fixture)
        for test in ("runtime_web", "runtime_download", "runtime_export_retry", "live_status_web", "flasher_completion"):
            extra = [fixture] if test in ("runtime_download", "runtime_export_retry") else []
            checked(node, ROOT / "test" / f"test_{test}.js", *extra)
        for converted in converted_trials:
            checked(*py, ROOT / "tools/replay_periodic_foot.py", tmp / "periodic_foot.exe", converted)
    print("Active host checks passed.")


if __name__ == "__main__":
    main()
