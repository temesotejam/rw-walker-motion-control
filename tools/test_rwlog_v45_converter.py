#!/usr/bin/env python3
"""Synthetic CRC/conversion test for the append-only RWLOG v45 current audit."""

import csv
import struct
import tempfile
import zlib
from pathlib import Path

import convert_rwlog_to_csv as converter


def write_fixture(path: Path, version: int) -> None:
    sample_format = (
        converter.SAMPLE_FORMAT_V45 if version == 45 else converter.SAMPLE_FORMAT_V42
    )
    values = [0] * len(struct.unpack(sample_format, b"\0" * struct.calcsize(sample_format)))
    values[0] = 1_000_000
    values[1] = 1_000
    values[2] = 3
    values[73:80] = [1234, -100, 34, 150, -250, 1, 1]
    if version == 45:
        values[80:90] = [1_234_567, 42, 123, 2, 7654, 8000, 7900, 5, 1, 1]
    sample = struct.pack(sample_format, *values)
    metadata = b"{}"
    header_size = struct.calcsize(converter.HEADER_FORMAT)
    samples_offset = header_size + len(metadata)
    crc_offset = samples_offset + len(sample)
    header_values = [
        b"RWLOG01\0", version, header_size, 1, 123456789, len(metadata), 1, 0, 0,
        len(sample), 0, 0, 20, 5, 20, 500, 1, 32, 1,
        samples_offset, crc_offset, crc_offset, crc_offset,
    ] + [0] * 8
    header = struct.pack(converter.HEADER_FORMAT, *header_values)
    payload = header + metadata + sample
    path.write_bytes(payload + struct.pack("<I", zlib.crc32(payload) & 0xFFFFFFFF))


def check(version: int) -> None:
    with tempfile.TemporaryDirectory(prefix=f"rwlog_v{version}_") as temp:
        root = Path(temp)
        source = root / f"fixture_v{version}.rwlog"
        output = root / "converted"
        write_fixture(source, version)
        data = source.read_bytes()
        header = converter.parse_header(data)
        assert converter.verify_crc(data, header), f"v{version} CRC verification failed"
        converter.convert(source, output)
        with (output / "timeseries.csv").open(newline="", encoding="utf-8") as f:
            row = next(csv.DictReader(f))
        assert row["physical_roll_abs_deg"] == "12.340"
        if version == 45:
            assert row["roller_current_sample_time_us"] == "1234567"
            assert row["roller_current_sequence"] == "42"
            assert row["roller_current_age_us"] == "123"
            assert row["roller_current_read_failure_count"] == "2"
            assert row["roller_q_meas_observed_mA_s"] == "7.654000"
            assert row["pulse_q_target_mA_s"] == "8.000000"
            assert row["pulse_q_pred_mA_s"] == "7.900000"
            assert row["roller_current_sample_count"] == "5"
            assert row["roller_current_valid"] == "1"
            assert row["roller_q_meas_observed_valid"] == "1"
        else:
            assert "roller_current_sample_time_us" not in row


if __name__ == "__main__":
    check(44)
    check(45)
    print("RWLOG v44 compatibility and v45 CRC/conversion checks passed")
