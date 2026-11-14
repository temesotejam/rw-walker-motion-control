"""Prepare the v17 installer while preserving every comparison installer."""
import json
import shutil
from pathlib import Path

root = Path(__file__).resolve().parents[1]
build = root / '.pio/build/atoms3cam-return-energy'
identity = 'walking-return-energy-v17'
firmware = build / 'firmware.bin'
if not firmware.is_file() or identity.encode() not in firmware.read_bytes():
    raise SystemExit('Build atoms3cam-return-energy successfully first.')
parts = {name: build / name for name in ['bootloader.bin', 'partitions.bin', 'firmware.bin']}
parts['boot_app0.bin'] = Path.home() / '.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/partitions/boot_app0.bin'
if any(not p.is_file() for p in parts.values()):
    raise SystemExit('Missing binary part')
dest = root / 'flasher/return-energy'
(dest / 'firmware').mkdir(parents=True, exist_ok=True)
for name, path in parts.items():
    shutil.copyfile(path, dest / 'firmware' / name)
for name in ['flasher-completion.js', 'soft-boot.js']:
    shutil.copyfile(root / 'flasher' / name, dest / name)
page = (root / 'flasher/index.html').read_text(encoding='utf8').replace('angle-early-weak-zero-v2', identity)
(dest / 'index.html').write_text(page, encoding='utf8')
manifest = json.loads((root / 'flasher/manifest.json').read_text(encoding='utf8'))
manifest.update(name='RW Walker return energy trial', version=identity)
(dest / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf8')
print('Prepared http://127.0.0.1:8767/return-energy/')
