#!/usr/bin/env python3
"""V7 Autonomous RWLOG metadata budget, JSON, CRC, and converter regressions."""
from __future__ import annotations

import csv
import importlib.util
import json
import struct
import tempfile
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESERVE_BYTES = 1024 * 1024
HARD_BUDGET_BYTES = 896 * 1024
TAIL_RESERVE_BYTES = 64 * 1024
MAX_EVENTS = 256


def peak_event(index: int) -> dict[str, object]:
    return {
        "peak_index": index, "peak_time_ms": 29_999, "physical_peak_side": -1,
        "peak_amplitude_deg": 12.34567, "detector_peak_angle_deg": -12.34567,
        "target_peak_deg": 8.0, "peak_error_deg": -4.34567, "phase": 3,
        "integral_plus_mA_s": 12.34567, "integral_minus_mA_s": -12.34567,
        "first_peak": False, "pending_command_matched": True,
        "pending_q_command_mA_s": 12.34567, "antiwindup_upper_hold": False,
        "antiwindup_lower_hold": False,
    }


def zero_event(index: int) -> dict[str, object]:
    return {
        "event_index": index, "event_kind": "energy_control_zero_cross",
        "zero_cross_time_ms": 29_999, "zero_cross_rate_dps": -123.4567,
        "zero_cross_abs_rate_dps": 123.4567, "detector_angle_before_deg": -12.34567,
        "detector_angle_after_deg": 12.34567, "detector_crossing_alpha": 0.99999,
        "interpolated_crossing_time_ms": 29_999.9999, "rate_support_diagnostic": False,
        "previous_peak_time_ms": 29_000, "previous_peak_side": 1,
        "previous_peak_amplitude_deg": 12.34567, "physical_next_peak_side": -1,
        "side_mismatch_diagnostic": False, "phase": 3,
        "free_next_peak_amplitude_deg": 12.34567,
        "free_model_revision": "P1_STEP_energy_alpha_0p870671664_Ec_0_20260828",
        "passive_energy_j": 0.01234567, "target_peak_deg": 8.0,
        "target_energy_j": 0.01234567, "delta_energy_required_j": 0.01234567,
        "q1_gain_deg_per_mA_s": 0.25455, "q_ff_energy_mA_s": 12.34567,
        "q_angle_diagnostic_mA_s": 12.34567, "integral_side_mA_s": 12.34567,
        "q_unclamped_mA_s": 12.34567, "q_command_mA_s": 12.34567,
        "q_effective_pred_mA_s": 12.34567, "q_available_mA_s": 12.34567,
        "q_gain_extrapolated": True, "predicted_next_peak_amplitude_deg": 12.34567,
        "predicted_energy_j": 0.01234567, "q_saturated_upper": True,
        "q_saturated_lower": False, "q_command_direction": 1, "command_matches_zero_cross_motion": True, "vbat_mV": 8100,
        "i0_estimated_mA": -299.9999, "solver_required_width_ms": 100.0,
        "solver_selected_integer_width_ms": 100, "command_current_mA": 300,
        "pulse_width_ms": 100, "pulse_start_ms": 29_999, "pulse_end_ms": 30_099,
        "output_executed": True, "valid": True, "reason": "NONE", "reason_code": 0,
    }


def render(peak_total: int, zero_total: int, *, budget: int = HARD_BUDGET_BYTES) -> bytes:
    base: dict[str, object] = {
        "format": "rwlog_energy_control_autonomous",
        "firmware_revision": "energy_control_autonomous_v7_side_response_correction_20260904",
        "measurement_mode": "energy_control_autonomous_v7_side_response_correction_rwlog30s",
        "energy_control_autonomous_revision": "V7",
        "energy_control_autonomous_base_revision": "V6",
        "normal_excitation_direction_policy": "same_as_zero_cross_roll_rate",
        "normal_direction_sign_fix": True,
        "start_kick_direction_changed": False,
        "physical_event_policy": "peak_zero_pulse_halfcycle_v5",
        "min_half_cycle_ms": 250, "min_zero_to_peak_ms": 125,
        "events_accepted_during_pulse": False, "autonomous_duration_ms": 30000,
        "metadata_json_budget_bytes": budget, "metadata_json_reserve_bytes": RESERVE_BYTES,
        "energy_control_autonomous_peak_events": [],
        "energy_control_autonomous_zero_cross_events": [],
    }
    truncated = False
    # Mirror the firmware policy: detail is append-only only while reserved
    # tail space remains. The final JSON always closes normally.
    for key, count, factory in (
        ("energy_control_autonomous_peak_events", peak_total, peak_event),
        ("energy_control_autonomous_zero_cross_events", zero_total, zero_event),
    ):
        for index in range(1, count + 1):
            trial = dict(base)
            trial[key] = list(base[key]) + [factory(index)]
            trial["metadata_event_detail_truncated"] = truncated
            encoded = json.dumps(trial, ensure_ascii=True, separators=(",", ":")).encode()
            if len(encoded) + TAIL_RESERVE_BYTES > budget:
                truncated = True
                break
            base[key].append(factory(index))
    base["metadata_event_detail_truncated"] = truncated
    base["energy_control_autonomous_peak_event_total_count"] = peak_total
    base["energy_control_autonomous_peak_event_rendered_count"] = len(base["energy_control_autonomous_peak_events"])
    base["energy_control_autonomous_zero_cross_event_total_count"] = zero_total
    base["energy_control_autonomous_zero_cross_event_rendered_count"] = len(base["energy_control_autonomous_zero_cross_events"])
    base["metadata_json_final_bytes"] = 0
    payload = json.dumps(base, ensure_ascii=True, separators=(",", ":")).encode()
    base["metadata_json_final_bytes"] = len(payload)
    payload = json.dumps(base, ensure_ascii=True, separators=(",", ":")).encode()
    base["metadata_json_final_bytes"] = len(payload)
    return json.dumps(base, ensure_ascii=True, separators=(",", ":")).encode()


def converter_module():
    spec = importlib.util.spec_from_file_location("rwlog_converter", ROOT / "tools" / "convert_rwlog_to_csv.py")
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_v44_fixture(path: Path, metadata: bytes) -> None:
    converter = converter_module()
    sample_format = converter.sample_format_for_version(44)
    sample_values = [0] * len(struct.unpack(sample_format, b"\0" * struct.calcsize(sample_format)))
    sample_values[0], sample_values[1], sample_values[2] = 1_000_000, 1_000, 3
    sample = struct.pack(sample_format, *sample_values)
    header_size = struct.calcsize(converter.HEADER_FORMAT)
    samples_offset = header_size + len(metadata)
    crc_offset = samples_offset + len(sample)
    header_values = [
        b"RWLOG01\0", 44, header_size, 1, 123456789, len(metadata), 1, 0, 0,
        len(sample), 0, 0, 20, 5, 20, 500, 1, 32, 1,
        samples_offset, crc_offset, crc_offset, crc_offset,
    ] + [0] * 8
    header = struct.pack(converter.HEADER_FORMAT, *header_values)
    payload = header + metadata + sample
    path.write_bytes(payload + struct.pack("<I", zlib.crc32(payload) & 0xFFFFFFFF))


def test_m1_to_m5_metadata_budget_and_truncation() -> None:
    normal = render(4, 4)
    parsed = json.loads(normal)
    assert not parsed["metadata_event_detail_truncated"]
    assert parsed["energy_control_autonomous_peak_event_rendered_count"] == 4
    worst = render(MAX_EVENTS, MAX_EVENTS)
    worst_parsed = json.loads(worst)
    assert len(worst) * 1.25 <= RESERVE_BYTES
    assert worst_parsed["metadata_json_final_bytes"] == len(worst)
    assert worst_parsed["energy_control_autonomous_peak_event_total_count"] == MAX_EVENTS
    constrained = render(MAX_EVENTS, MAX_EVENTS, budget=8 * 1024)
    constrained_parsed = json.loads(constrained)
    assert constrained_parsed["metadata_event_detail_truncated"]
    assert constrained_parsed["energy_control_autonomous_peak_event_rendered_count"] <= MAX_EVENTS
    assert constrained_parsed["energy_control_autonomous_zero_cross_event_rendered_count"] <= MAX_EVENTS


def test_m6_to_m8_rwlog_offset_crc_and_converter() -> None:
    metadata = render(1, 1)
    parsed = json.loads(metadata)
    with tempfile.TemporaryDirectory(prefix="v7_rwlog_") as temp:
        root = Path(temp)
        source, output = root / "v7.rwlog", root / "converted"
        write_v44_fixture(source, metadata)
        converter = converter_module()
        header = converter.parse_header(source.read_bytes())
        assert header["samples_offset"] == header["header_size"] + header["metadata_json_size"]
        assert converter.verify_crc(source.read_bytes(), header)
        converter.convert(source, output)
        converted_metadata = json.loads((output / "metadata.json").read_text(encoding="utf-8"))
        assert converted_metadata["metadata_event_detail_truncated"] == parsed["metadata_event_detail_truncated"]
        with (output / "timeseries.csv").open(newline="", encoding="utf-8") as handle:
            assert len(list(csv.DictReader(handle))) == 1
        with (output / "energy_control_autonomous_zero_cross_events.csv").open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        assert len(rows) == 1
        assert rows[0]["command_matches_zero_cross_motion"] == "True"


def test_source_contract() -> None:
    logger = (ROOT / "src" / "psram_logger.cpp").read_text(encoding="utf-8")
    for fragment in (
        "kMetadataJsonReserveBytes = 1024U * 1024U",
        "kMetadataJsonHardBudgetBytes = 896U * 1024U",
        "metadata_event_detail_truncated",
        "energy_control_autonomous_peak_event_rendered_count",
        "energy_control_autonomous_zero_cross_event_rendered_count",
        "metadata_json_final_bytes",
        "appendAutonomousDetail",
    ):
        assert fragment in logger


def main() -> None:
    test_m1_to_m5_metadata_budget_and_truncation()
    test_m6_to_m8_rwlog_offset_crc_and_converter()
    test_source_contract()
    print("PASS: V7 Autonomous metadata budget, JSON, RWLOG offset/CRC, and converter regressions")


if __name__ == "__main__":
    main()
