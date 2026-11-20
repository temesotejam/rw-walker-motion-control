"""Exercise the control-core snapshot, bounded export, CSV path and legacy logs."""
import csv
import json
from pathlib import Path
import subprocess
import tempfile

from _project import ROOT
from convert_rwlog_to_csv import convert, write_gait_turn
from fixtures.runtime_rwlog import write_fixture


def main():
    with tempfile.TemporaryDirectory() as directory:
        work = Path(directory)
        exe = work / "gait_turn_run.exe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-DWALKING_TURN_OBSERVER=1", "-DFOOT_STABLE40_V36=1",
                        "-I", str(ROOT / "src"), "-I", str(ROOT / "test/fixtures/arduino"),
                        str(ROOT / "src/gait_turn_run.cpp"), str(ROOT / "test/fixtures/gait_turn_run.cpp"),
                        "-o", str(exe)], check=True)
        disabled, active, full, cleared, unavailable = [json.loads(v) for v in
            subprocess.check_output([str(exe)], text=True).splitlines()]
        assert disabled == cleared and disabled["gait_turn"]["enabled"] is False
        assert full == {"start": 0}  # An optional section cannot break the sealed log.
        assert unavailable["gait_turn"]["total"] == 2 and not unavailable["gait_turn"]["rows"]
        assert unavailable["gait_turn"]["confirmed_ms"] == 1300
        output = work / "converted"
        log = work / "turn.rwlog"
        write_fixture(log, active)
        convert(log, output)
        with (output / "gait_turn.csv").open(encoding="utf8") as stream:
            rows = list(csv.DictReader(stream))
        assert [r["reason"] for r in rows] == ["pending", "confirmed", "confirmed", "coverage"]
        assert rows[1]["weak_side"] == "1" and rows[1]["turn_sign"] == "-1" and rows[1]["samples"] == "24"
        assert rows[0]["pair_yaw_delta_deg"] == "" and float(rows[1]["pair_yaw_delta_deg"]) == -12
        assert rows[-1]["left_swing_deg"] == "" and active["gait_turn"]["confirmed_ms"] == 1900
        assert write_gait_turn({}, work) == 0 and not (work / "gait_turn.csv").exists()
        corrupt = dict(active["gait_turn"], exported=100)
        try:
            write_gait_turn({"gait_turn": corrupt}, work)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid exported count accepted")
    print("Gait turn cycle snapshot, limited archive, reset and RWLOG conversion passed")


if __name__ == "__main__":
    main()
