#!/usr/bin/env python3
"""Verify one locally staged or deployed camera diagnostic Web Serial flasher."""
import argparse
import hashlib
import json
import time
from pathlib import Path
from urllib.request import Request, urlopen

PIN = "2fdac86a87f529cffbe2219dfaaf07318b0575c7"
EXPECTED = {"firmware/bootloader.bin": 0, "firmware/partitions.bin": 0x8000,
            "firmware/boot_app0.bin": 0xe000, "firmware/firmware.bin": 0x10000}


def verify(read, mode):
    html = read("index.html").decode("utf-8")
    manifest = json.loads(read("manifest.json"))
    assert manifest.get("new_install_prompt_erase") is True
    assert manifest.get("new_install_improv_wait_time") == 0
    assert manifest.get("version") == f"camera-compare-mode-{mode}-{PIN[:12]}"
    assert html.index('../../flasher-completion.js') < html.index('esp-web-tools@10.1.1')
    assert 'Erase device' in html
    builds = manifest['builds']
    assert len(builds) == 1 and builds[0]['chipFamily'] == 'ESP32-S3'
    parts = builds[0]['parts']
    assert len(parts) == 4 and {p['path']: p['offset'] for p in parts} == EXPECTED
    identity = read('BUILD.txt').decode('utf-8')
    assert f'mode={mode}' in identity.splitlines() and f'pr_head={PIN}' in identity.splitlines()
    hashes = {}
    for line in read('SHA256SUMS.txt').decode('utf-8').splitlines():
        digest, name = line.split(maxsplit=1)
        hashes[name.lstrip('*')] = digest
    for path in EXPECTED:
        data = read(path)
        # Bound every write to its known region in the stable 8 MiB layout.
        # In particular, the partition table must end before NVS at 0x9000.
        limits = {'firmware/bootloader.bin': 0x8000,
                  'firmware/partitions.bin': 0x1000,
                  'firmware/boot_app0.bin': 0x2000,
                  'firmware/firmware.bin': 0x330000}
        assert data and len(data) <= limits[path], (path, len(data))
        assert hashlib.sha256(data).hexdigest() == hashes[path], path
    print(f'Camera mode {mode}: firmware identity, layout, no-erase fallback and SHA-256 match')


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--path', type=Path)
    group.add_argument('--url')
    parser.add_argument('--mode', type=int, choices=(0, 1, 2), required=True)
    args = parser.parse_args()
    if args.path:
        verify(lambda name: (args.path / name).read_bytes(), args.mode)
        return
    def read(name):
        req = Request(args.url.rstrip('/') + '/' + name + '?verify=' + PIN,
                      headers={'Cache-Control': 'no-cache'})
        with urlopen(req, timeout=30) as resp:
            return resp.read()
    for attempt in range(8):
        try:
            verify(read, args.mode)
            return
        except Exception as exc:
            if attempt == 7:
                raise
            print('Retrying published camera verification:', exc, flush=True)
            time.sleep(10)


if __name__ == '__main__':
    main()
