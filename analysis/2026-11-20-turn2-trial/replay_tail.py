"""Compare the prior receiving-arc selection and a pivot certificate for the extra pulse time."""
import argparse
import csv
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / "2026-11-20-carry1-trials"
sys.path.insert(0, str(PREVIOUS))
from replay_aid import replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folders", type=Path, nargs="+", help="Converted RWLOG directories")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        exe = Path(temp) / "tail.exe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-DSUPPORT_TRANSFER_STALL_V3=1",
                        "-I", HERE.parents[1] / "src", PREVIOUS / "replay_aid.cpp", "-o", exe], check=True)
        rows = [r for folder in args.folders for r in replay(folder, exe, compare_pivot=True)]
    fields = ("trial time_ms observed_zero old_guarded old_pivot old_choice tail_proof "
              "shape_reason start_clearance_mm tail_clearance_mm end_clearance_mm horizon_s "
              "extra_j candidate_guarded reserve_ms new_ms new_work_j remaining_ms").split()
    with args.out.open("w", encoding="utf8", newline="") as file:
        writer = csv.DictWriter(file, fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        if row["old_choice"] != row["new_ms"] or row["tail_proof"] == "1":
            print(row["trial"], row["time_ms"], row["old_choice"], row["new_ms"], row["tail_proof"])
    print(f"Checked {len(rows)} rescue events")


if __name__ == "__main__":
    main()
