"""Prepare a local web installer for the shadow comparison build."""
import json
import shutil
from pathlib import Path

root = Path(__file__).resolve().parents[1]
build = root / '.pio' / 'build' / 'atoms3cam-fast-crossing-shadow'
build_id = 'angle-fast-crossing-shadow-v9'
firmware = build / 'firmware.bin'
if not firmware.is_file() or build_id.encode() not in firmware.read_bytes():
    raise SystemExit('Shadow firmware is missing or has the wrong build ID; build it first.')
sources = {
    'bootloader.bin': build / 'bootloader.bin',
    'partitions.bin': build / 'partitions.bin',
    'boot_app0.bin': Path.home() / '.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/partitions/boot_app0.bin',
    'firmware.bin': firmware,
}
missing = [str(path) for path in sources.values() if not path.is_file()]
if missing:
    raise SystemExit('Missing firmware files: ' + ', '.join(missing))
dest = root / 'flasher' / 'fast-crossing-shadow'
files = dest / 'firmware'
files.mkdir(parents=True, exist_ok=True)
for name, path in sources.items():
    shutil.copyfile(path, files / name)
for name in ('flasher-completion.js', 'soft-boot.js'):
    shutil.copyfile(root / 'flasher' / name, dest / name)
page = (root / 'flasher' / 'index.html').read_text(encoding='utf-8')
(dest / 'index.html').write_text(page.replace('angle-early-weak-zero-v2', build_id), encoding='utf-8')
manifest = json.loads((root / 'flasher' / 'manifest.json').read_text(encoding='utf-8'))
manifest['version'] = build_id
(dest / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('Prepared http://127.0.0.1:8767/fast-crossing-shadow/ (' + build_id + ')')
