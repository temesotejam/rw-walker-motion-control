#!/usr/bin/env python3
"""Synthetic CRC/conversion test for append-only RWLOG v46 MEKF diagnostics."""
import csv
import struct
import tempfile
import zlib
from pathlib import Path
import convert_rwlog_to_csv as converter


def write_fixture(path: Path, version: int) -> None:
    if version == 52:
        sample_format = converter.SAMPLE_FORMAT_V52
    elif version == 51:
        sample_format = converter.SAMPLE_FORMAT_V51
    elif version == 50:
        sample_format = converter.SAMPLE_FORMAT_V50
    elif version == 49:
        sample_format = converter.SAMPLE_FORMAT_V49
    elif version == 48:
        sample_format = converter.SAMPLE_FORMAT_V48
    elif version == 47:
        sample_format = converter.SAMPLE_FORMAT_V47
    elif version == 46:
        sample_format = converter.SAMPLE_FORMAT_V46
    elif version == 45:
        sample_format = converter.SAMPLE_FORMAT_V45
    else:
        sample_format = converter.SAMPLE_FORMAT_V42
    values = [0] * len(struct.unpack(sample_format, b"\0" * struct.calcsize(sample_format)))
    values[0] = 1_000_000
    values[1] = 1_000
    values[2] = 3
    values[73:80] = [1234, -100, 34, 150, -250, 1, 1]
    if version >= 45:
        values[80:90] = [1_234_567, 42, 123, 2, 7654, 8000, 7900, 5, 1, 1]
    if version >= 46:
        values[90:103] = [1250, 1300, 1275, 9999, 10, -20, 30, 11, -22, 33, 8750, 250, 40]
        values[103:107] = [5000, 321, 1, 1]
    if version >= 47:
        values[107:113] = [100, 200, 300, 1100, 1200, 1300]
        values[113:116] = [111111, 222222, 333333]
    if version >= 48:
        values[116:118] = [450, 1450]
        values[118] = 444444
    if version >= 52:
        values[119:] = [123456, 55, 224, 252, 64, 6, 168, 253, 21, 106]
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
        assert converter.verify_crc(data, header)
        assert header["log_sample_size"] == struct.calcsize(converter.sample_format_for_version(version))
        converter.convert(source, output)
        with (output / "timeseries.csv").open(newline="", encoding="utf-8") as f:
            row = next(csv.DictReader(f))
        assert row["physical_roll_abs_deg"] == "12.340"
        if version >= 52:
            assert header["log_sample_size"] == 274
            assert row["mag_sample_us"] == "123456" and row["mag_sequence"] == "55"
            assert row["mag_raw_x"] == "-100" and row["mag_rhall"] == "6789"
            assert row["mag_factory_ok"] == "0" and row["mag_value_valid"] == "0" and row["mag_norm_uT"] == ""
        if version >= 45:
            assert row["roller_current_sequence"] == "42"
        if version >= 46:
            assert row["pitch_mekf_control_deg"] == "12.500"
            assert row["pitch_mekf_abs_deg"] == "13.000"
            assert row["pitch_madgwick_dynamic_abs_deg"] == "12.750"
            assert row["mekf_q_w"] == "0.999900"
            assert row["mekf_accel_confidence"] == "0.87500"
            assert row["mekf_accel_residual_deg"] == "2.500"
            assert row["mekf_accel_mag_error_g"] == "0.0400"
            assert row["imu_update_dt_us"] == "5000"
            assert row["imu_sample_age_us"] == "321"
            assert row["mekf_accel_used"] == "1"
            assert row["attitude_filter_adopted"] == "1"
            if version >= 47:
                assert row["pitch_mekf_start_sync_relative_deg"] == "1.000"
                assert row["pitch_mekf_measurement_relative_deg"] == "2.000"
                assert row["pitch_mekf_trial_relative_deg"] == "3.000"
                assert row["mekf_start_sync_zero_abs_deg"] == "11.000"
                assert row["mekf_measurement_zero_abs_deg"] == "12.000"
                assert row["mekf_trial_zero_abs_deg"] == "13.000"
                assert row["mekf_start_sync_zero_sample_us"] == "111111"
                assert row["mekf_measurement_zero_sample_us"] == "222222"
                assert row["mekf_trial_zero_sample_us"] == "333333"
                if version >= 48:
                    assert row["pitch_mekf_detector_relative_deg"] == "4.500"
                    assert row["mekf_detector_zero_predicted_abs_deg"] == "14.500"
                    assert row["mekf_detector_zero_sample_us"] == "444444"
        else:
            assert "pitch_mekf_control_deg" not in row


def check_compact_angles(version: int) -> None:
    fields = converter.FIELDS_V54 if version == 54 else converter.FIELDS_V53
    values = [0] * len(fields)
    names = [field[1] for field in fields]
    values[names.index('pitch_mekf_abs_deg')] = 1050
    if version == 54:
        values[names.index('roll_mekf_abs_deg')] = -1234
    packed = struct.pack(converter.sample_format_for_version(version), *values)
    assert len(packed) == (114 if version == 54 else 112)
    row = converter.convert_sample(struct.unpack(converter.sample_format_for_version(version), packed), version)
    assert row['pitch_mekf_abs_deg'] == 10.5
    assert row.get('roll_mekf_abs_deg') == (-12.34 if version == 54 else None)
    assert list(row) == converter.csv_columns_for_version(version)[1:] + ['time_s']


if __name__ == "__main__":
    check(44)
    check(45)
    check(46)
    check(47)
    check(48)
    check(49)
    check(50)
    check(51)
    check(52)
    check_compact_angles(53)
    check_compact_angles(54)
    print("RWLOG v44-v54 compatibility including MEKF roll and pitch passed")
