"""Replay each archived generic-early and ZERO opportunity on its original trajectory."""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
EARLY = "angle_rate_early_trial"
ZERO = "energy_control_zero_cross"


def replay(folder, binary):
    data = json.loads((folder / "metadata.json").read_bytes())
    historical = data["diagnostic_build_id"] == "foot-support-energy-v14"
    if data["walking_energy"]["truncated"]:
        raise ValueError(f"{folder}: incomplete decision history")
    peaks = data["energy_control_autonomous_peak_events"]
    events = data["energy_control_autonomous_zero_cross_events"]
    energy = data["walking_energy"]["records"]
    assert events[0]["event_kind"] == "strong_start_kick" and len(events) == len(energy) + 1
    source = []
    decisions = []
    skips = Counter()
    for peak in peaks:
        matched = peak["pending_command_matched"] and (peak["pending_q_command_mA_s"] or 0) > 0
        source.append((peak["peak_telemetry_capture_time_ms"], 0, "P", peak["peak_time_ms"],
                       peak["physical_peak_side"], peak["peak_amplitude_deg"], int(matched)))
    for event, audit in zip(events[1:], energy):
        kind = event["event_kind"]
        t, side = audit["time_ms"], audit["side"]
        assert (audit["peak_ms"] == event["previous_peak_time_ms"]
                and 0 <= t - event["zero_cross_time_ms"] <= 5
                and audit["executed"] == event["output_executed"])
        if kind not in (EARLY, ZERO):
            skips[kind] += 1
            continue
        cap = event["work_brake_cap_ms"]
        if not cap:
            skips["no_recorded_cap_or_energy_input"] += 1
            continue
        assert event["pre_input_wheel_speed_valid"] and event["pre_input_wheel_speed_age_us"] <= 50000
        assert audit.get("fore_aft_allowed", True)
        same = [p for p in peaks if p["physical_peak_side"] == side and p["peak_telemetry_capture_time_ms"] <= t
                and t - p["peak_time_ms"] <= 3000]
        early = kind == EARLY
        angle = event["detector_angle_after_deg"]
        rate = event["zero_cross_abs_rate_dps"]
        lead_ms = 1000 * abs(angle) / rate if early else None
        if early:
            # Every archived optional decision in these four trials came from the body-motion gate.
            assert 20 < lead_ms < 30.001 and side * angle < 0
        source.append((t, 1, "D", t, side, int(early), rate, angle, event["startup_target_deg"],
                       event["previous_peak_amplitude_deg"], same[-1]["peak_amplitude_deg"] if same else -1,
                       event["aligned_initial_current_mA"], event["predicted_current_goal_mA"],
                       event["predicted_current_rise_tau_s"], event["pre_input_wheel_speed_rpm"],
                       cap, audit["requested_ms"], int(audit["executed"])))
        decisions.append((t, kind, audit, event, lead_ms))
    payload = "\n".join(" ".join(map(str, s[2:])) for s in sorted(source)) + "\n"
    rows = subprocess.check_output([binary], input=payload, text=True).splitlines()
    assert len(rows) == len(decisions)
    stats = Counter()
    first_change = None
    lead_times = []
    for (t, kind, audit, event, lead), line in zip(decisions, rows):
        valid, requested, guarded, admitted, charge, work = map(float, line.split())
        assert valid
        command = int(guarded) if admitted else 0
        if lead is not None:
            lead_times.append(lead)
        old = event["pulse_width_ms"] if event["output_executed"] else 0
        stats[kind] += 1
        stats["same_command_width"] += int(old == command)
        stats["input_width_matches"] += int(requested == audit["requested_ms"])
        if old != command:
            change = dict(time_ms=t, event=kind, logged_ms=old, baseline_ms=command)
            if first_change is None:
                first_change = change
        if historical:
            assert int(requested) == audit["requested_ms"] and int(guarded) == audit["guarded_ms"]
            assert old == command, (folder, t, old, command)
            assert abs(charge - audit["q_mas"]) < 0.0001 and abs(work - audit["predicted_work_j"]) < 2e-8
    return dict(build=data["diagnostic_build_id"], rows=len(rows), counts=dict(stats), excluded=dict(skips),
                early_lead_ms=[round(min(lead_times), 2), round(max(lead_times), 2)],
                first_command_change=first_change)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("converted", type=Path, nargs="+")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        binary = Path(tmp) / "selector.exe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-I", HERE.parents[1] / "src", HERE / "selector.cpp", "-o", binary], check=True)
        for folder in args.converted:
            print(folder.name, json.dumps(replay(folder, binary), ensure_ascii=False))


if __name__ == "__main__":
    main()
