"""Prepare a separate local web installer after the early rescue trial builds."""
import json
import shutil
from pathlib import Path

root = Path(__file__).resolve().parents[1]
build = root / '.pio' / 'build' / 'atoms3cam-early-rescue'
firmware = build / 'firmware.bin'
build_id = 'angle-early-rescue-v5'
if not firmware.is_file() or build_id.encode() not in firmware.read_bytes():
    raise SystemExit('The early rescue trial firmware is missing or has the wrong build ID; build it first.')
source = {
    'bootloader.bin': build / 'bootloader.bin',
    'partitions.bin': build / 'partitions.bin',
    'boot_app0.bin': Path.home() / '.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/partitions/boot_app0.bin',
    'firmware.bin': firmware,
}
missing = [str(p) for p in source.values() if not p.is_file()]
if missing:
    raise SystemExit('Missing firmware files: ' + ', '.join(missing))
dest = root / 'flasher' / 'early-rescue'
files = dest / 'firmware'
files.mkdir(parents=True, exist_ok=True)
for name, path in source.items():
    shutil.copyfile(path, files / name)
for name in ('flasher-completion.js', 'soft-boot.js'):
    shutil.copyfile(root / 'flasher' / name, dest / name)
page = (root / 'flasher' / 'index.html').read_text(encoding='utf-8')
page = page.replace('angle-early-weak-zero-v2', build_id)
(dest / 'index.html').write_text(page, encoding='utf-8')
manifest = json.loads((root / 'flasher' / 'manifest.json').read_text(encoding='utf-8'))
manifest['version'] = build_id
(dest / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('Prepared http://127.0.0.1:8767/early-rescue/ (' + build_id + ')')
