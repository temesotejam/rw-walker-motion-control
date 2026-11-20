"""Compile the real v39 snapshot, worker and export paths against a stepped host scheduler."""
import csv
import json
from pathlib import Path
import subprocess
import tempfile

from _project import ROOT
from convert_rwlog_to_csv import convert, write_support_transfer_shadow
from fixtures.runtime_rwlog import write_fixture


def main():
    runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf-8")
    begin = runner.index("void ExperimentRunner::observeSupportTransfer(")
    end = runner.index("\n#endif", begin)
    normal = runner[runner.index("void ExperimentRunner::updateEnergyControlAutonomousMotion("):]
    assert normal.index("observeSupportTransfer(t_test_ms") < normal.index("observeInwardStallRescue(")
    with tempfile.TemporaryDirectory() as temp:
        folder = Path(temp)
        (folder / "runner_snapshot.inc").write_text(runner[begin:end], encoding="utf-8")
        exe = folder / "audit.exe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-DSUPPORT_TRANSFER_V39=1", "-DFOOT_STABLE40_V36=1",
                        "-I", str(folder), "-I", str(ROOT / "src"), "-I", str(ROOT / "test/fixtures/arduino"),
                        str(ROOT / "test/fixtures/support_transfer_audit.cpp"), "-o", str(exe)], check=True)
        blocks = [json.loads(line) for line in subprocess.check_output([exe], text=True).splitlines()]
        assert len(blocks) == 6
        expected = [(5, 1, 3, 1), (1, 0, 0, 1), (1, 0, 1, 0), (1, 0, 0, 0)]
        for metadata, counts in zip(blocks, expected):
            b = metadata["support_transfer_shadow"]
            assert tuple(b[k] for k in ("submitted", "replaced", "completed", "abandoned")) == counts
            write_support_transfer_shadow(metadata, folder)
        first = blocks[0]["support_transfer_shadow"]
        raw = folder / "shadow.rwlog"
        write_fixture(raw, blocks[0])
        expanded = folder / "decoded"
        convert(raw, expanded)
        with (expanded / "support_transfer_shadow.csv").open() as stream:
            assert [int(r["packet"]) for r in csv.DictReader(stream)] == [2, 4, 5]
        rows = [dict(zip(first["columns"], row)) for row in first["rows"]]
        assert [r["packet"] for r in rows] == [2, 4, 5]
        assert [r["side"] for r in rows] == [1, -1, -1]
        assert [r["i0_ma"] for r in rows] == [7, -7, None]
        assert all(r["gate_mask"] == 0 for r in rows[:2])
        assert rows[1]["baseline_flags"] == 4 and rows[1]["lateral_deg"] < 0 and rows[1]["cap_ms"] == 80
        assert (rows[1]["raw_vbat_mv"],rows[1]["model_vbat_mv"],rows[1]["model_vbat_status"]) == (8350,8020,1)
        assert rows[1]["transfer_deg"] < rows[1]["lateral_deg"] and rows[1]["transfer_ms"] > 0
        assert rows[2]["gate_mask"] & (4 | 16 | 32 | 512) == 4 | 16 | 32 | 512
        assert rows[2]["reason"] == 0 and rows[2]["width_ms"] == 0
        assert rows[0]["ready_after_us"] >= 7000 and rows[0]["accel_dps2"] == 400
        assert blocks[3]["support_transfer_shadow"]["unavailable"] == 1
        zero_cap = blocks[5]["support_transfer_shadow"]
        zero_rows = [dict(zip(zero_cap["columns"], row)) for row in zero_cap["rows"]]
        assert [r["reason"] for r in zero_rows] == [2,5] and all(r["width_ms"] == 0 for r in zero_rows)
        assert [r["capture_us32"] for r in zero_rows] == [2**32 - 4, 4]
        assert [r["ready_after_us"] for r in zero_rows] == [7, 2]
        write_fixture(raw, blocks[5])
        convert(raw, expanded)
        with (expanded / "support_transfer_shadow.csv").open() as stream:
            assert [int(r["capture_us32"]) for r in csv.DictReader(stream)] == [2**32 - 4, 4]
        under_pressure = blocks[4]
        b = under_pressure["support_transfer_shadow"]
        assert b["completed"] == 1210 and b["stored"] == 1201
        assert 1 < b["exported"] < b["stored"]
        count = write_support_transfer_shadow(under_pressure, folder)
        with (folder / "support_transfer_shadow.csv").open() as stream:
            exported = list(csv.DictReader(stream))
        assert count == len(exported) and int(exported[-1]["packet"]) > 1000
        damaged = dict(b, completed=0)
        try:
            write_support_transfer_shadow({"support_transfer_shadow": damaged}, folder)
        except ValueError:
            pass
        else:
            raise AssertionError("inconsistent shadow counts accepted")
        assert write_support_transfer_shadow({}, folder) == 0
    print("PASS transfer snapshot, signs, gates, worker races, no motor interface, sealed budget and CSV")


if __name__ == "__main__":
    main()
