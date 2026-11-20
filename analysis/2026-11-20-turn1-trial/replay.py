"""Compare turn2 with turn1 and the ordinary pulses actually driven on the six recorded walks."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory

PREVIOUS = Path(__file__).resolve().parent.parent / "2026-11-20-turn-detection"
REASONS = ("waiting", "coverage", "gyro_unavailable", "below_asymmetry_gate", "below_yaw_gate",
           "pending", "confirmed", "phase")


def rows(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def keyed(records, column):
    return {(r["trial"], int(r[column])): r for r in records}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("converted", nargs="+", type=Path, help="RWLOG converter output directories")
    args = parser.parse_args()
    with TemporaryDirectory() as scratch:
        cycles, pulses = (Path(scratch) / name for name in ("cycles.csv", "ordinary.csv"))
        subprocess.run([sys.executable, "-B", PREVIOUS / "replay.py", *args.converted, "--out", cycles], check=True)
        subprocess.run([sys.executable, "-B", PREVIOUS / "replay_control.py", *args.converted,
                        "--cycles", cycles, "--out", pulses], check=True)
        new_cycles, new_pulses = rows(cycles), rows(pulses)

    old_cycles = keyed(rows(PREVIOUS / "cycles.csv"), "end_ms")
    old_pulses = keyed(rows(PREVIOUS / "ordinary-zero.csv"), "time_ms")
    for folder in args.converted:
        if not (folder / "gait_turn.csv").exists():
            continue
        label = str(json.loads((folder / "header.json").read_text(encoding="utf8"))["run_start_us"])
        for actual in rows(folder / "gait_turn.csv"):
            key = label, int(actual["end_ms"])
            old_cycles[key] = dict(actual, reason=str(REASONS.index(actual["reason"])))
        for actual in rows(folder / "walking_energy.csv"):
            if actual["executed"] == "True" and actual["optional"] == actual["return_event"] == "False":
                old_pulses[label, int(actual["time_ms"])] = {"new_ms": actual["guarded_ms"]}

    print("\nAdditional confirmations and changes from turn1 on recorded pre-command states")
    for r in new_cycles:
        key = r["trial"], int(r["end_ms"])
        old = old_cycles[key]
        if int(r["samples"]) != int(old["samples"]):
            raise ValueError(f"Foot observations differ from turn1 at {key}")
        for k in ("left_deg", "right_deg", "left_swing_deg", "right_swing_deg"):
            if abs(float(r[k]) - float(old[k])) > 0.001:
                raise ValueError(f"Foot measurement differs from turn1: {key} {k}")
        if old["reason"] != r["reason"]:
            print(*key, "reason", old["reason"], "->", r["reason"],
                  "paired_swing_asymmetry", r["paired_swing_asymmetry"])
    for r in new_pulses:
        key = r["trial"], int(r["time_ms"])
        before = int(old_pulses[key]["new_ms"])
        if before != int(r["new_ms"]):
            print(*key, "ordinary side", r["side"], "ms", before, "->", r["new_ms"],
                  "predicted work (J)", r["base_work_j"], "->", r["new_work_j"])


if __name__ == "__main__":
    main()
