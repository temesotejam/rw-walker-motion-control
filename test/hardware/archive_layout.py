"""Create test-only independent DEFLATE blocks from a validated current-format RWLOG."""
import hashlib
from pathlib import Path
import struct
import zlib


HEADER = struct.Struct("<8sHHIQ4I9H5I8I")
BLOCK_BYTES = 16384


def pack_archive(path, out_src, target_free, target_largest, write):
    data = Path(path).read_bytes()
    if len(data) < HEADER.size + 4:
        raise ValueError("Truncated RWLOG")
    h = HEADER.unpack_from(data)
    start, count, row_size = h[4], h[6], h[9]
    meta_len = h[5]
    sample_at, summary_at, events_at, crc_at = h[19:23]
    crc, = struct.unpack_from("<I", data, len(data) - 4)
    if (h[:3] != (b"RWLOG01\0", 56, HEADER.size) or not 0 < h[3] <= 65535 or
            not start or not count or row_size != 172 or h[7:9] != (0, 0) or
            sample_at != HEADER.size + meta_len or crc_at != sample_at + count * row_size or
            summary_at != crc_at or events_at != crc_at or crc_at + 4 != len(data) or
            zlib.crc32(data[:-4]) != crc):
        raise ValueError("RWLOG identity, section bounds or whole-file CRC did not validate")
    if not (12000 <= target_free <= 100000 and 4096 <= target_largest <= target_free):
        raise ValueError("Expected target-free 12000..100000 and target-largest 4096..target-free bytes")
    blob = bytearray()

    def add_section(section, block_length):
        for at in range(0, len(section), block_length):
            block = section[at:at + block_length]
            codec = zlib.compressobj(level=6, wbits=-15)
            encoded = codec.compress(block) + codec.flush()
            if zlib.decompress(encoded, wbits=-15) != block:
                raise ValueError("Archive block round trip failed")
            blob.extend(struct.pack("<HH", len(encoded), len(block)))
            blob.extend(encoded)

    metadata, samples = data[HEADER.size:sample_at], data[sample_at:crc_at]
    add_section(metadata, BLOCK_BYTES)
    split = len(blob)
    add_section(samples, BLOCK_BYTES // row_size * row_size)
    asset = out_src.parent / "archive.deflate"
    write(asset, blob)
    constants = dict(BlockBytes=BLOCK_BYTES, RunId=h[3], MetadataSize=meta_len,
                     MetadataCrc=zlib.crc32(metadata), SamplesCrc=zlib.crc32(samples),
                     SampleBlockOffset=split, TargetFree=target_free, TargetLargest=target_largest)
    header = ("#pragma once\n#include <stdint.h>\nnamespace replay_layout {\n"
              "constexpr uint8_t kHeader[] = {" + ",".join(map(str, data[:HEADER.size])) + "};\n")
    for key, value in constants.items():
        header += f"constexpr uint32_t k{key} = {value}u;\n"
    write(out_src / "archive_layout.h", (header + "}\n").encode())
    if '"' in asset.as_posix():
        raise ValueError("Assembly asset path must not contain double quotes")
    # An incbin dependency alone is not tracked by every SCons assembler.
    # Make replacing an existing input invalidate the object on incremental builds.
    asm = (f'/* archived block SHA256 {hashlib.sha256(blob).hexdigest()} */\n'
           '.section .rodata.motorfree_archive,"a",@progbits\n.balign 4\n'
           '.global replay_blob_start\n.global replay_blob_end\nreplay_blob_start:\n'
           f'.incbin "{asset.as_posix()}"\nreplay_blob_end:\n')
    write(out_src / "archive_blob.S", asm.encode())
    print(f"Archived {len(data)} bytes (run {h[3]}, start {start}, {count} samples); "
          f"{len(blob)} bytes in test-only image; CRC32 {crc:08x}; SHA256 {hashlib.sha256(data).hexdigest()}")
