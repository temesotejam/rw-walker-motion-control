"""Replay archived v39 worker inputs with the legacy and receiving-side selectors."""
import argparse
import csv
import json
import math
import struct
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from convert_rwlog_to_csv import convert_sample, expand_tables, parse_header, sample_format_for_version, verify_crc
from score_observed_transfers import command_overlap, gyro_samples, terminal_observation

INPUTS = ("peak_ms packet gate_mask side lateral_deg rate_dps accel_dps2 fore_aft_deg "
          "left_deg right_deg left_rate_dps right_rate_dps i0_ma goal_ma cap_ms").split()
OUTPUTS = ("old_reason old_delay old_width phase anchor new_reason new_delay new_width "
           "work_supported_new terminal_deg terminal_ms new_coast_j new_margin_j").split()
REASONS = ("invalid", "outside", "coast", "wait", "drive", "insufficient", "complete")


def read_log(path):
    data = path.read_bytes()
    h = parse_header(data)
    if (h["crc_offset"] + 4 != len(data) or not verify_crc(data, h)
            or h["header_size"] + h["metadata_json_size"] != h["samples_offset"]):
        raise ValueError(f"Invalid RWLOG length, offset, or CRC: {path}")
    m = expand_tables(json.loads(data[h["header_size"]:h["samples_offset"]]))
    a = m["support_transfer_shadow"]
    if a["revision"] != 1 or a["exported"] != a["stored"] or len(a["rows"]) != a["stored"]:
        raise ValueError(f"Expected full v39 revision-1 worker archive: {path}")
    epochs = {f["frame_us"] - f["measurement_time_us"] for f in m["foot_frames"]
              if f["state_id"] == 3 and f["measurement_time_us"] >= 0}
    if len(epochs) != 1:
        raise ValueError("Cannot recover a unique measurement clock")
    epoch = epochs.pop()
    version = h["format_version"]
    sample_struct = struct.Struct(sample_format_for_version(version))
    end = h["samples_offset"] + sample_struct.size * h["sample_count"]
    if sample_struct.size != h["log_sample_size"] or end > h["crc_offset"]:
        raise ValueError("Inconsistent IMU sample block")
    samples = (convert_sample(v, version) for v in sample_struct.iter_unpack(memoryview(data)[h["samples_offset"]:end]))
    return m, epoch, f"{struct.unpack_from('<I', data, h['crc_offset'])[0]:08x}", gyro_samples(m, samples, epoch)


def replay(binary, audit):
    if any(len(row) != len(audit["columns"]) for row in audit["rows"]):
        raise ValueError("Archived audit row width differs from schema")
    rows = [dict(zip(audit["columns"], row)) for row in audit["rows"]]
    payload = [str(audit["rise_tau_s"])]
    for r in rows:
        payload.append(" ".join("nan" if r[k] is None else str(r[k]) for k in INPUTS))
    run = subprocess.run([binary], text=True, input="\n".join(payload) + "\n",
                         stdout=subprocess.PIPE, check=True)
    lines = run.stdout.splitlines()
    if len(lines) != len(rows):
        raise ValueError("Replay output count differs from input")
    for r, line in zip(rows, lines):
        tokens = line.split()
        if len(tokens) != len(OUTPUTS):
            raise ValueError("Incomplete replay output")
        r.update(zip(OUTPUTS, [int(s) for s in tokens[:9]] + [float(s) for s in tokens[9:]]))
    return rows


def summarize(path, binary):
    meta, epoch, crc, motion = read_log(path)
    audit = meta["support_transfer_shadow"]
    rows = replay(binary, audit)
    peaks = meta["energy_control_autonomous_peak_events"]
    zero = {e["previous_peak_time_ms"]: e["zero_cross_time_ms"]
            for e in meta["energy_control_autonomous_zero_cross_events"]
            if e["event_kind"] == "energy_control_zero_cross"}
    outputs = [e for e in meta["walking_energy"]["records"] if e["executed"]]
    half_cycles = []
    for i, p in enumerate(peaks):
        peak = p["peak_time_ms"]
        crossed = peak in zero
        alternate = i + 1 < len(peaks) and peaks[i+1]["physical_peak_side"] == -p["physical_peak_side"]
        end = peaks[i+1]["peak_time_ms"] if i+1 < len(peaks) else meta["autonomous_duration_ms"]
        first = min((e for e in outputs if e["peak_ms"] == peak), key=lambda e: e["time_ms"], default=None)
        half = [r for r in rows if r["peak_ms"] == peak]
        before = min(zero.get(peak, end), first["time_ms"] if first else end)
        previous = None
        for r in half:
            capture = ((r["capture_us32"] - epoch) & 0xffffffff) / 1000
            r["capture_ms"] = round(capture, 3)
            r["old_ready_ms"] = round(capture + r["ready_after_us"] / 1000, 3)
            # Baseline flags record whether the half has already been acted on.
            # On four v19 early cycles no explicit ZERO was logged. The angle
            # sign also excludes samples already on the target side of zero.
            r["pre_baseline"] = (capture < before and r["time_ms"] < before and r["gate_mask"] == 0
                                 and r["baseline_flags"] == 0 and r["side"] * r["lateral_deg"] < 0)
            r["until_baseline_ms"] = round(before - capture, 3)
            # Retrospective confidence check only. The two snapshots must
            # describe the same terminal event with no intervening command.
            r["coast_pair"] = bool(
                previous and r["phase"] == previous["phase"] == 2
                and r["new_reason"] == previous["new_reason"] == 2
                and r["gate_mask"] == previous["gate_mask"] == 0
                and r["side"] == previous["side"] and r["packet"] == previous["packet"] + 1
                and r["terminal_deg"] == previous["terminal_deg"]
                and command_overlap(meta["walking_energy"], previous["capture_ms"], capture) == "no_recorded_command"
            )
            previous = r
            r.update(observed_terminal=None, observed_end_ms=None, observed_rate_dps=None,
                     intervening_command=None, observed_carry=None)
            if r["phase"] == 2 and math.isfinite(r["terminal_deg"]):
                # Captured state to the observed event, for retrospective scoring.
                # A new firmware completion time cannot be inferred from v39 timings.
                status, finish, speed = terminal_observation(*motion, capture, capture, end,
                                                            r["side"], r["terminal_deg"])
                r.update(observed_terminal=status, observed_end_ms=finish, observed_rate_dps=speed,
                         intervening_command=command_overlap(meta["walking_energy"], capture, finish))
                if speed is not None or status == "reversed":
                    r["observed_carry"] = speed is not None and speed >= 5
        seen = [r for r in half if r["phase"] == 2]
        proposals = [r for r in seen if r["new_reason"] == 4 and r["work_supported_new"]]
        pre = next((r for r in proposals if r["pre_baseline"]), None)
        half_cycles.append(dict(peak_ms=peak, side=-p["physical_peak_side"], zero_ms=zero.get(peak),
                                alternate_peak=int(alternate), explicit_or_alternate=int(crossed or alternate),
                                baseline_decision_ms=first["time_ms"] if first else None,
                                baseline_on_ms=first["output_start_ms"] if first else None,
                                baseline_width_ms=first["actual_elapsed_ms"] if first else None,
                                archived=len(half), receiving=len(seen), drive_supported=len(proposals),
                                pre_drive_ms=pre["capture_ms"] if pre else None,
                                pre_drive_width_ms=pre["new_width"] if pre else None,
                                pre_drive_headroom_ms=pre["until_baseline_ms"] if pre else None))
    old_mismatch = sum((r["old_reason"], r["old_delay"], r["old_width"])
                       != (r["reason"], r["delay_ms"], r["width_ms"]) for r in rows)
    print(f"{path.name}  CRC32={crc}  rows={len(rows)}  replaced={audit['replaced']}  "
          f"old_reason_delay_width_mismatch={old_mismatch}")
    for result in (1, 0):
        hc = [c for c in half_cycles if c["explicit_or_alternate"] == result]
        print(f"  explicit ZERO or opposite peak={result}  half_cycles={len(hc)}  "
              f"receiving={sum(c['receiving'] > 0 for c in hc)}  "
              f"receiving_supported_drive_before_baseline={sum(c['pre_drive_ms'] is not None for c in hc)}")
    print("  receiving reasons:", dict(Counter(REASONS[r["new_reason"]] for r in rows if r["phase"] == 2)))
    no_command = [r for r in rows if r.get("intervening_command") == "no_recorded_command"
                  and r["observed_carry"] is not None and r["new_reason"] in (2, 3, 4, 5)]
    print("  no-intervening-command snapshots (reason, observed carry >= 5 dps):",
          dict(Counter((REASONS[r["new_reason"]], r["observed_carry"]) for r in no_command)))
    coast = [r for r in no_command if r["new_reason"] == 2]
    print("  no-intervening-command coast (consecutive pair, observed carry >= 5 dps):",
          dict(Counter((r["coast_pair"], r["observed_carry"]) for r in coast)))
    return crc, rows, half_cycles


def write_csv(path, records):
    with path.open("w", encoding="utf8", newline="") as stream:
        w = csv.DictWriter(stream, fieldnames=records[0])
        w.writeheader()
        w.writerows(records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rwlog", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, help="Optional directory for per-snapshot and per-half-cycle CSV")
    args = parser.parse_args()
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="receiving-replay-") as tmp:
        binary = Path(tmp) / "audit.exe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-I", ROOT / "src",
                        HERE / "replay_audit.cpp", "-o", binary], check=True)
        for path in args.rwlog:
            crc, rows, cycles = summarize(path, binary)
            if args.out:
                write_csv(args.out / f"{crc}-snapshots.csv", rows)
                write_csv(args.out / f"{crc}-half-cycles.csv", cycles)


if __name__ == "__main__":
    main()
