"""Verify firmware gzip framing on a PC using a supplied upstream miniz v1.14 source."""
import argparse
import gzip
import hashlib
import struct
import subprocess
import tempfile
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROBE = r'''
#include "export_gzip.h"
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
int main(int argc, char** argv) {
  if (argc!=4) return 1;
  const unsigned chunks=std::stoul(argv[3]);
  if (!chunks || chunks>export_protocol::kStreamChunks) return 1;
  std::ifstream file(argv[1],std::ios::binary);
  std::vector<uint8_t> input{std::istreambuf_iterator<char>(file),{}};
  std::ofstream out(argv[2],std::ios::binary);
  auto codec=std::make_unique<tdefl_compressor>();
  const uint32_t block=export_protocol::kChunkBytes, window=block*chunks;
  std::vector<uint8_t> packet(block+16);
  for (size_t pos=0; pos<input.size();pos+=window) {
    const uint32_t n=std::min<size_t>(window,input.size()-pos);
    std::vector<uint8_t> encoded(export_protocol::packetBytes(n));
    auto fill=[&](uint32_t at,uint32_t length) {
      const uint32_t h[]={export_protocol::kChunkMagic,at,length,export_protocol::crc32(0,input.data()+at,length)};
      memcpy(packet.data(),h,sizeof(h));
      memcpy(packet.data()+sizeof(h),input.data()+at,length);
      return true;
    };
    size_t used=export_gzip::encode(*codec,encoded.data(),encoded.size(),packet.data(),pos,n,fill);
    uint32_t wire[]={n,static_cast<uint32_t>(used)};
    out.write(reinterpret_cast<const char*>(wire),sizeof(wire));
    if (used) out.write(reinterpret_cast<const char*>(encoded.data()),used);
    else for (uint32_t i=0;i<n;i+=block) {
      const auto count=std::min(block,n-i);
      fill(pos+i,count);
      out.write(reinterpret_cast<const char*>(packet.data()),count+16);
    }
  }
  return !out.good();
}
'''


def verify(file, wire):
    source = file.read_bytes()
    at = payload = windows = compressed = 0
    while payload < len(source):
        n, size = struct.unpack_from('<II', wire, at)
        at += 8
        expected_plain = n + 16*((n+4095)//4096)
        length = size or expected_plain
        body = wire[at:at+length]
        plain = gzip.decompress(body) if size else body
        assert len(plain) == expected_plain and (not size or size < expected_plain)
        packet = 0
        while packet < len(plain):
            magic, offset, count, crc = struct.unpack_from('<IIII', plain, packet)
            data = plain[packet+16:packet+16+count]
            assert magic == 0x31484346 and offset == payload and 0 < count <= 4096
            assert len(data) == count and zlib.crc32(data) == crc and data == source[payload:payload+count]
            payload += count
            packet += 16+count
        windows += 1
        compressed += bool(size)
        at += length
    assert at == len(wire) and payload == len(source)
    return dict(raw_bytes=payload, packet_bytes=payload+16*((payload+4095)//4096),
                wire_body_bytes=at-8*windows, windows=windows, gzip_windows=compressed)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('rwlog', type=Path)
    parser.add_argument('--miniz', type=Path, required=True, help='miniz.c from the upstream v114 tag')
    parser.add_argument('--window-chunks', type=int, choices=(1, 2, 4, 16), default=4,
                        help='number of 4 KiB packets in each HTTP response (default: 4)')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        source = args.miniz.read_text(encoding='utf-8-sig')
        a, b = '#define TDEFL_LESS_MEMORY 0', '#define TDEFL_LESS_MEMORY 1'
        assert source.count(a) == 1  # match compressor configuration in the ESP32-S3 SDK
        (d/'miniz.c').write_text(source.replace(a,b),encoding='utf-8')
        included = d/'esp32s3/rom'; included.mkdir(parents=True)
        (included/'miniz.h').write_text('#define MINIZ_HEADER_FILE_ONLY\n#include "miniz.c"\n',encoding='utf-8')
        (d/'probe.cpp').write_text(PROBE,encoding='utf-8')
        subprocess.run(['gcc','-O2','-w','-DMINIZ_NO_ARCHIVE_APIS','-c',d/'miniz.c','-o',d/'miniz.o'],check=True)
        exe = d/'probe.exe'
        subprocess.run(['g++','-std=c++17','-O2','-I',d,'-I',ROOT/'src',d/'probe.cpp',d/'miniz.o','-o',exe],check=True)
        random_file = d/'incompressible.bin'
        random_file.write_bytes(hashlib.shake_256(b'raw fallback').digest(64001))
        for path in (args.rwlog, random_file):
            wire = d/'response.bin'
            subprocess.run([exe,path,wire,str(args.window_chunks)],check=True)
            stats = verify(path,wire.read_bytes())
            if path == random_file:
                assert stats['gzip_windows'] == 0
            print(path.name, stats)
