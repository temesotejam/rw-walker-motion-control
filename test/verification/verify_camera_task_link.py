"""Verify the precompiled camera actually calls our RTOS allocation wrapper."""
import argparse
from pathlib import Path
import re
import struct
import subprocess

p = argparse.ArgumentParser()
p.add_argument("elf", type=Path)
p.add_argument("--objdump", required=True)
args = p.parse_args()
data = args.elf.read_bytes()
if data[:6] != b"\x7fELF\x01\x01":
    raise SystemExit("Expected little-endian ELF32")
section_offset = struct.unpack_from("<I", data, 32)[0]
section_size, section_count = struct.unpack_from("<HH", data, 46)
sections = [struct.unpack_from("<10I", data, section_offset + i * section_size)
            for i in range(section_count)]

def word(address):
    for s in sections:
        if s[1] == 1 and s[3] <= address and address + 4 <= s[3] + s[5]:
            return struct.unpack_from("<I", data, s[4] + address - s[3])[0]
    raise ValueError("Unmapped literal address: " + hex(address))

symbols = subprocess.check_output([args.objdump, "-t", str(args.elf)], text=True)
wrapper = re.search(r"^([0-9a-f]+)\s+.*\s__wrap_xTaskCreatePinnedToCore$", symbols, re.M)
if not wrapper:
    raise SystemExit("Missing camera task allocation wrapper")
wrapper_address = int(wrapper[1], 16)
assembly = subprocess.check_output(
    [args.objdump, "-d", "--disassemble=cam_config", str(args.elf)], text=True).splitlines()
for i, line in enumerate(assembly):
    literal = re.search(r"\bl32r\s+(a\d+),\s*([0-9a-f]+)", line)
    direct = re.search(r"\bcall\d*\s+([0-9a-f]+)\s+<__wrap_xTaskCreatePinnedToCore>", line)
    indirect = (literal and word(int(literal[2], 16)) == wrapper_address
                and i + 1 < len(assembly)
                and re.search(r"\bcallx\d*\s+" + literal[1] + r"\b", assembly[i + 1]))
    if indirect or direct:
        print("PASS: linked SDK cam_config calls __wrap_xTaskCreatePinnedToCore")
        break
else:
    raise SystemExit("SDK cam_config bypasses the task allocation wrapper")
