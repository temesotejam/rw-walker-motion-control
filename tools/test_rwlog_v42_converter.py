#!/usr/bin/env python3
"""Synthetic CRC and version-compatibility checks for the Current Roll RWLOG v42 extension."""
import csv
import struct
import tempfile
import zlib
from pathlib import Path

import convert_rwlog_to_csv as converter


def write_fixture(path: Path, version: int) -> None:
    sample_format = converter.SAMPLE_FORMAT_V42 if version == 42 else converter.SAMPLE_FORMAT_V35
    sample_values = [0] * len(struct.unpack(sample_format, b"\0" * struct.calcsize(sample_format)))
    sample_values[0] = 1_000_000
    sample_values[1] = 1_000
    sample_values[2] = 3
    if version == 42:
        sample_values[73] = 1234
        sample_values[74] = -100
        sample_values[75] = 34
        sample_values[76] = 150
        sample_values[77] = -250
        sample_values[78] = 1
        sample_values[79] = 1
    sample = struct.pack(sample_format, *sample_values)
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
        if version == 42:
            assert row["physical_roll_abs_deg"] == "12.340"
            assert row["current_roll_deg"] == "-1.000"
            assert row["physical_roll_rate_dps"] == "0.3400"
            assert row["static_confirmed"] == "1"
            assert row["target_roll_deg"] == "1.500"
            assert row["target_error_deg"] == "-2.500"
            assert row["ready"] == "1"
        else:
            assert "physical_roll_abs_deg" not in row


if __name__ == "__main__":
    check(41)
    check(42)
    print("RWLOG v41 compatibility and v42 CRC/conversion checks passed")
