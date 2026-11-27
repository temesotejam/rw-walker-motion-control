"""Compare return-aid decisions on the recorded trajectory of each v37 RWLOG."""
import argparse
import csv
import hashlib
import json
import tempfile
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from convert_rwlog_to_csv import convert
from event_foot_energy_candidate import decision_rows, evaluate_candidate, reachability_first
from event_foot_features import CausalFootFeatures
from event_foot_outcomes import RecordedOutcomes


DECISION_FIELDS = (
    "time_ms", "peak_ms", "side", "event_kind", "return_event", "return_evidence", "valid",
    "energy_admission_allowed", "fore_aft_allowed", "fore_aft_reason", "envelope_before_ms",
    "guarded_ms", "request_j", "target_j", "decision_i0_ma", "decision_goal_ma",
    "decision_tau_s", "decision_rate_dps",
)


def replay(source, trial_dir):
    # Convert the original file each time. The converter verifies CRC, length and layout.
    convert(source, trial_dir)
    metadata = json.loads((trial_dir / "metadata.json").read_text(encoding="utf8"))
    header = json.loads((trial_dir / "header.json").read_text(encoding="utf8"))
    energy = metadata["walking_energy"]
    if energy["revision"] != "walking_energy_admission_v19":
        raise ValueError(f"{source.name}: expected v19 energy decisions")
    if energy["truncated"] or len(energy["records"]) != energy["total"]:
        raise ValueError(f"{source.name}: incomplete decision history")
    features = CausalFootFeatures(metadata)
    outcomes = RecordedOutcomes(trial_dir, metadata)
    events = decision_rows(metadata)
    if len(events) != len(energy["records"]):
        raise ValueError(f"{source.name}: incomplete event pairing")
    rows = []
    for i, aid in enumerate(events):
        if not (aid["return_event"] and aid["executed"]):
            continue
        # This comparison is conditional on an actual aid decision. The current
        # pulse completion and its future trajectory cannot inform its alternative.
        now = aid["time_ms"]
        past_main = [
            e for e in events[:i]
            if not e["return_event"] and e["time_ms"] < now and (
                (e["executed"] and e["completion_valid"] and e["output_end_ms"] < now)
                or (not e["executed"] and e["valid"] and e["coast"])
            )
        ]
        context = features.at(aid)
        decision = {k: aid[k] for k in DECISION_FIELDS}
        candidate = evaluate_candidate(decision, context, past_main)
        crossing_first = reachability_first(decision)
        outcome = outcomes.for_aid(aid)
        rows.append({
            "trial": header["run_id"], "run_start_us": header["run_start_us"],
            "time_ms": now, "source_peak_ms": aid["peak_ms"], "side": aid["side"],
            "requested_ms": aid["requested_ms"], "guarded_ms": aid["guarded_ms"],
            "recorded_crossing_request_j": aid["request_j"],
            "lateral_rate_abs_dps": aid["decision_rate_dps"],
            "recorded_guarded_model_work_j": aid["output_work_pred_j"],
            "envelope_before_ms": aid["envelope_before_ms"],
            **context, **candidate, **crossing_first, **outcome,
        })
    info = {
        "source": source.name, "run_id": header["run_id"], "run_start_us": header["run_start_us"],
        "sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "crc_ok": True,
        "diagnostic_build_id": metadata["diagnostic_build_id"], "energy_revision": energy["revision"],
        "energy_neutral_model": energy["energy_neutral_model"], "energy_decisions": len(events),
        "executed_returns": len(rows), "terminal": metadata["terminal_state"]["last_error"],
    }
    return info, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rwlog", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    rows, inputs = [], []
    for source in args.rwlog:
        with tempfile.TemporaryDirectory() as tmp:
            info, trial_rows = replay(source, Path(tmp))
        inputs.append(info)
        rows.extend(trial_rows)
    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / "events.csv").open("w", encoding="utf8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
        writer.writeheader()
        writer.writerows(rows)
    (args.out / "inputs.json").write_text(
        json.dumps(inputs, ensure_ascii=False, indent=2) + "\n", encoding="utf8"
    )
    print(f"{len(rows)} recorded aids in {len(inputs)} trials")


if __name__ == "__main__":
    main()
