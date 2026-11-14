"""Prepare the v16 transport-only repair without replacing the original trial."""
import json
import shutil
import argparse
from pathlib import Path

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--psram', action='store_true', help='Prepare webfix2 with a PSRAM diagnostic archive')
args = parser.parse_args()
suffix = '2' if args.psram else ''
build = root / ('.pio/build/atoms3cam-energy-pulse-lifecycle-webfix' + suffix)
identity = 'energy-pulse-lifecycle-v16-webfix2' if args.psram else 'energy-pulse-lifecycle-v16-webfix1'
firmware = build / 'firmware.bin'
if not firmware.is_file() or identity.encode() not in firmware.read_bytes():
    raise SystemExit('Build ' + build.name + ' successfully first.')
files = {name: build / name for name in ['bootloader.bin', 'partitions.bin', 'firmware.bin']}
files['boot_app0.bin'] = Path.home() / '.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/partitions/boot_app0.bin'
if any(not path.is_file() for path in files.values()):
    raise SystemExit('Missing binary part')
dest = root / ('flasher/energy-pulse-lifecycle-webfix' + suffix)
(dest / 'firmware').mkdir(parents=True, exist_ok=True)
for name, path in files.items():
    shutil.copyfile(path, dest / 'firmware' / name)
for name in ['flasher-completion.js', 'soft-boot.js']:
    shutil.copyfile(root / 'flasher' / name, dest / name)
page = (root / 'flasher/index.html').read_text(encoding='utf8').replace('angle-early-weak-zero-v2', identity)
(dest / 'index.html').write_text(page, encoding='utf8')
manifest = json.loads((root / 'flasher/manifest.json').read_text(encoding='utf8'))
manifest['name'] = 'RW Walker v16 web transport repair'
manifest['version'] = identity
(dest / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf8')
print('Prepared http://127.0.0.1:8767/energy-pulse-lifecycle-webfix' + suffix + '/')
