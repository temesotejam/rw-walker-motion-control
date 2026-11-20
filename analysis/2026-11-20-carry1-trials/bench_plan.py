"""Compare carry1 and the current planner on complete, CRC-checked recorded archives."""
import argparse
import json
import math
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from convert_rwlog_to_csv import parse_header, verify_crc

REFERENCE = "ab93f4dd06d39888b833a9f36f4b6dee8c402326"
FIELDS = ("peak_ms packet gate_mask side lateral_deg rate_dps accel_dps2 fore_aft_deg "
          "left_deg right_deg left_rate_dps right_rate_dps i0_ma goal_ma cap_ms").split()
FLOAT_FIELDS = ("transfer_ms", "transfer_deg", "terminal_deg", "terminal_ms",
                "coast_margin_j", "margin_j", "work_j")


def input_text(path):
    raw = path.read_bytes()
    h = parse_header(raw)
    if h["crc_offset"] + 4 != len(raw) or not verify_crc(raw, h):
        raise ValueError(f"Invalid CRC or length: {path}")
    block = json.loads(raw[h["header_size"]:h["samples_offset"]])["support_transfer_shadow"]
    if len(block["rows"]) != block["exported"] or block["exported"] != block["stored"]:
        raise ValueError(f"Incomplete archive: {path}")
    columns = [block["columns"].index(k) for k in FIELDS]
    lines = [str(block["rise_tau_s"])]
    for row in block["rows"]:
        lines.append(" ".join("nan" if row[i] is None else str(row[i]) for i in columns))
    return "\n".join(lines) + "\n"


def compare(path, binaries, rounds):
    payload = input_text(path)
    outputs, timings = {}, {name: [] for name in binaries}
    for i in range(rounds):
        order = list(binaries) if i % 2 == 0 else list(reversed(binaries))
        for name in order:
            text = subprocess.check_output([binaries[name]], input=payload, text=True)
            header, *rows = text.splitlines()
            if name in outputs and rows != outputs[name]:
                raise ValueError(f"Non-repeatable output for {name}")
            outputs[name] = rows
            timings[name].append(tuple(map(float, header.split())))
    ref, now = outputs["carry1"], outputs["current"]
    peak_diff = [0.] * len(FLOAT_FIELDS)
    changed = 0
    if len(ref) != len(now) or len(ref) != payload.count("\n") - 1:
        raise ValueError("Different trace lengths")
    for a, b in zip(ref, now):
        x, y = a.split(), b.split()
        if len(x) != 7+len(FLOAT_FIELDS) or len(x) != len(y):
            raise ValueError("Unexpected decision schema")
        changed += x[:7] != y[:7]
        for j, (u, v) in enumerate(zip(map(float, x[7:]), map(float, y[7:]))):
            if math.isnan(u) != math.isnan(v):
                raise ValueError(f"{FLOAT_FIELDS[j]} finite/NaN mismatch in {path.name}")
            if math.isinf(u) or math.isinf(v):
                raise ValueError(f"Unexpected infinite {FLOAT_FIELDS[j]} in {path.name}")
            if math.isfinite(u):
                peak_diff[j] = max(peak_diff[j], abs(u-v))
    result = {"rows": len(ref), "discrete_differences": changed, "max_abs": dict(zip(FLOAT_FIELDS, peak_diff))}
    for name, series in timings.items():
        full_us, receiving_us, counts = zip(*series)
        if len(set(counts)) != 1 or counts[0] <= 0:
            raise ValueError("Inconsistent or empty receiving group")
        result[name] = {"full_trace_us": round(statistics.median(full_us), 3),
                        "receiving_rows": int(counts[0]),
                        "receiving_us_per_row": round(statistics.median(receiving_us)/counts[0], 3)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rwlog", type=Path, nargs="+")
    parser.add_argument("--rounds", type=int, default=9)
    args = parser.parse_args()
    if args.rounds <= 0:
        parser.error("Positive --rounds required")
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        reference = subprocess.check_output(["git", "-c", f"safe.directory={ROOT.as_posix()}", "show",
                                             f"{REFERENCE}:src/support_transfer_plan.h"], cwd=ROOT)
        (tmp / "support_transfer_plan.h").write_bytes(reference)
        binaries = {}
        for name, include in (("carry1", tmp), ("current", ROOT/"src")):
            binaries[name] = tmp/(name+".exe")
            subprocess.run(["g++", "-O2", "-std=c++17", "-I", include, "-I", ROOT/"src",
                            HERE/"bench_plan.cpp", "-o", binaries[name]], check=True)
        for path in args.rwlog:
            print(path.name, json.dumps(compare(path, binaries, args.rounds)))


if __name__ == "__main__":
    main()
