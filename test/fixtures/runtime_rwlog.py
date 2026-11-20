"""Small deterministic RWLOG v56 for browser CRC, resume, CSV and wheel-work tests."""
import json
import struct
import zlib

from convert_rwlog_to_csv import FIELDS_V56, HEADER_FIELDS, HEADER_FORMAT, SAMPLE_FORMAT_V56


def write_fixture(path):
    # More than two 4 KiB chunks so the browser exercises resume at byte 8192.
    frames = [{"sequence": n, "left_deg": n % 11, "right_deg": -(n % 7)} for n in range(768)]
    metadata = json.dumps({"foot_frames": frames}, separators=(",", ":")).encode()
    sensor = {"state_id": 3, "roller_speed_sequence": 1, "roller_speed_valid": 1,
              "roller_speed_sample_time_us": 124456}
    sample = struct.pack(SAMPLE_FORMAT_V56, *(sensor.get(name, 0) for _, name, _ in FIELDS_V56))
    header_size = struct.calcsize(HEADER_FORMAT)
    start = header_size + len(metadata)
    end = start + len(sample)
    fields = {
        "magic": b"RWLOG01\0", "format_version": 56, "header_size": header_size, "run_id": 1,
        "run_start_us": 123456, "metadata_json_size": len(metadata), "sample_count": 1,
        "log_sample_size": len(sample), "flags": 1, "samples_offset": start,
        "summaries_offset": end, "events_offset": end, "crc_offset": end,
    }
    header = struct.pack(HEADER_FORMAT, *(fields.get(name, 0) for name in HEADER_FIELDS))
    payload = header + metadata + sample
    path.write_bytes(payload + struct.pack("<I", zlib.crc32(payload)))
