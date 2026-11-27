"""Match recorded return-aid decisions to the mechanical events in their own half-cycle."""

import json
from pathlib import Path


def _number(record, key):
    value = record.get(key)
    return None if value in (None, "") else float(value)


def _flag(record, key):
    value = record.get(key)
    if value in (True, "True", "true", "1"):
        return True
    if value in (False, "False", "false", "0"):
        return False
    return None


class RecordedOutcomes:
    """Read native metadata, which preserves signed detector samples omitted by the CSV."""

    def __init__(self, trial: Path, metadata=None):
        if metadata is None:
            metadata = json.loads((Path(trial) / "metadata.json").read_text(encoding="utf-8-sig"))
        walking = metadata["walking_energy"]
        self.reference_supported = (
            metadata.get("work_absolute_reference_enabled") is True
            and metadata.get("control_angle_origin_abs_deg") == 0
            and walking.get("mechanical_reference_deg") == 0
            and metadata.get("autonomous_input_peak_percent") == 0
            and metadata.get("autonomous_total_timing_projection_us") == 0
        )
        self.zeros = [e for e in metadata["energy_control_autonomous_zero_cross_events"]
                      if e["event_kind"] == "energy_control_zero_cross"]
        self.peaks = sorted(metadata["energy_control_autonomous_peak_events"],
                            key=lambda e: e["peak_time_ms"])
        self.mains = [r for r in walking["records"]
                      if r.get("rescue") is False and r.get("optional") is False]
        self.events_complete = (metadata.get("metadata_event_detail_truncated") is False
                                and metadata.get("energy_control_autonomous_event_overflow") is False)
        terminal = metadata.get("terminal_state", {})
        self.terminal_reason = terminal.get("last_error") or terminal.get("state")

    def for_aid(self, aid):
        """Return outcome labels only. The future ZERO, peak and main are hindsight data."""
        t, origin, side = (int(aid[k]) for k in ("time_ms", "peak_ms", "side"))
        if side not in (-1, 1):
            raise ValueError("aid side must be +1 or -1")
        next_peak = next((p for p in self.peaks if p["peak_time_ms"] > t), None)
        end = next_peak["peak_time_ms"] if next_peak else float("inf")
        zeros = [z for z in self.zeros if z["previous_peak_time_ms"] == origin
                 and z["physical_next_peak_side"] == side and t < z["zero_cross_time_ms"] < end]
        zero = None
        if len(zeros) == 1 and self.reference_supported:
            before = _number(zeros[0], "detector_angle_before_deg")
            after = _number(zeros[0], "detector_angle_after_deg")
            if (before is not None and after is not None
                    and before * side <= 0 <= after * side and before != after):
                zero = zeros[0]
        peak = (next_peak if next_peak and next_peak["physical_peak_side"] == side else None)
        main = None
        if zero:
            matches = [r for r in self.mains if r["peak_ms"] == origin and r["side"] == side
                       and zero["zero_cross_time_ms"] <= r["time_ms"] < end]
            if len(matches) == 1:
                main = matches[0]

        if not self.reference_supported:
            status = "unsupported_mechanical_reference"
        elif len(zeros) > 1 or (zeros and not zero):
            status = "unverified_zero"
        elif zero and peak:
            status = "zero_and_opposite_peak"
        elif not self.events_complete:
            status = "incomplete_event_archive"
        elif next_peak:
            status = "opposite_peak_without_verified_zero" if peak else "same_side_next_peak"
        else:
            status = "censored_after_zero" if zero else "censored_before_zero"

        aid_completed = _flag(aid, "executed") and _flag(aid, "completion_valid")
        main_completed = main and main.get("executed") and main.get("completion_valid")
        return {
            "outcome_status": status,
            "reference_supported": self.reference_supported,
            "zero_observed": True if zero else None,
            "zero_ms": zero["zero_cross_time_ms"] if zero else None,
            "zero_rate_dps": zero.get("zero_cross_rate_dps") if zero else None,
            "peak_observed": True if peak else (False if next_peak and self.events_complete else None),
            "peak_ms": peak["peak_time_ms"] if peak else None,
            "peak_amplitude_deg": peak.get("peak_amplitude_deg") if peak else None,
            "peak_target_deg": peak.get("target_peak_deg") if peak else None,
            # Peak energy is the firmware's potential-energy proxy at the observed extremum.
            "observed_peak_energy_j": peak.get("measured_energy_j") if peak else None,
            "peak_target_j": peak.get("target_energy_j") if peak else None,
            "main_executed": main.get("executed") if main else None,
            "main_request_j": main.get("request_j") if main and main.get("valid") else None,
            "main_guarded_ms": main.get("guarded_ms") if main else None,
            "main_elapsed_ms": main.get("actual_elapsed_ms") if main_completed else None,
            "main_model_unsupported_mask": (main.get("energy_unsupported_mask")
                                            if main and main.get("valid") else None),
            "observed_aid_elapsed_ms": _number(aid, "actual_elapsed_ms") if aid_completed else None,
            "terminal_reason": self.terminal_reason if next_peak is None else None,
        }
