"""Prepare the separate v20 foot-recognition installer after a successful build.

Retain v19 walking energy control and all older installer directories. --dry-run
validates templates without requiring binaries or creating an installer.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = 'atoms3cam-foot-fast'
IDENTITY = 'walking-foot-fast-v20'
SUFFIX = 'foot-fast'
EXPECTED_PARTS = [('firmware/bootloader.bin', 0), ('firmware/partitions.bin', 32768),
                  ('firmware/boot_app0.bin', 57344), ('firmware/firmware.bin', 65536)]


def settings(profile=None):
    result = {'environment':ENVIRONMENT, 'identity':IDENTITY, 'suffix':SUFFIX,
              'title':'歩行器 v20 足角度認識の低遅延版',
              'name':'RW Walker foot recognition low latency v20 trial'}
    result.update(profile or {})
    return result


def installer_text(root=ROOT, *, profile=None):
    """Render and validate source templates without writing files."""
    selected = settings(profile)
    page = (root / 'flasher/index.html').read_text(encoding='utf8')
    replacements = {
        'angle-early-weak-zero-v2': selected['identity'],
        '<title>歩行器ファームウェア書き込み</title>': '<title>' + selected['title'] + '</title>',
        '<h1>歩行器ファームウェア書き込み</h1>': '<h1>' + selected['title'] + '</h1>',
    }
    if any(page.count(old) != 1 for old in replacements) or page.count('<esp-web-install-button ') != 1:
        raise ValueError('Unexpected base installer template')
    for old, new in replacements.items():
        page = page.replace(old, new, 1)
    description = (
        '<p>足角度認識の低遅延版です。歩行のエネルギー制御はv19を継承します。</p>\n  '
        '<p>実機での認識速度と精度は未確認です。書込み後は、まずモータ測定を開始せずに'
        '足角度と更新を確認してください。</p>\n  '
        '<p>比較用のv19書込みページは保持しています。</p>\n  '
    )
    page = page.replace('<esp-web-install-button ', selected.get('description', description) + '<esp-web-install-button ', 1)
    manifest = json.loads((root / 'flasher/manifest.json').read_text(encoding='utf8'))
    builds = manifest.get('builds', [])
    if (len(builds) != 1 or builds[0].get('chipFamily') != 'ESP32-S3' or
            [(part.get('path'), part.get('offset')) for part in builds[0].get('parts', [])] != EXPECTED_PARTS):
        raise ValueError('Unexpected ESP32-S3 installer layout')
    manifest.update(name=selected['name'], version=selected['identity'])
    return page, json.dumps(manifest, indent=2, ensure_ascii=False) + '\n'


def dry_run(root=ROOT, *, profile=None):
    selected = settings(profile)
    page, manifest = installer_text(root, profile=selected)
    return {'dry_run': True, 'environment': selected['environment'], 'identity': selected['identity'],
            'url': 'http://127.0.0.1:8767/' + selected['suffix'] + '/',
            'page_bytes': len(page.encode('utf8')), 'manifest': json.loads(manifest)}


def prepare(root=ROOT, boot_app=None, *, profile=None):
    selected = settings(profile)
    build = root / '.pio/build' / selected['environment']
    firmware = build / 'firmware.bin'
    blob = firmware.read_bytes() if firmware.is_file() else b''
    pattern = rb'(?<![A-Za-z0-9_-])' + re.escape(selected['identity'].encode()) + rb'(?![A-Za-z0-9_-])'
    if re.search(pattern, blob) is None:
        raise SystemExit('Build ' + selected['environment'] + ' successfully first (v20 identity required).')
    parts = {name: build / name for name in ['bootloader.bin', 'partitions.bin', 'firmware.bin']}
    parts['boot_app0.bin'] = Path(boot_app) if boot_app is not None else (
        Path.home() / '.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/partitions/boot_app0.bin')
    support = [root / 'flasher' / name for name in ['flasher-completion.js', 'soft-boot.js']]
    invalid = [str(path) for path in list(parts.values()) + support if not path.is_file() or not path.stat().st_size]
    if invalid:
        raise SystemExit('Missing or empty installer file(s): ' + ', '.join(invalid))
    page, manifest = installer_text(root, profile=selected)
    # Validate identity, templates, layout and every source before writing.
    dest = root / 'flasher' / selected['suffix']
    (dest / 'firmware').mkdir(parents=True, exist_ok=True)
    for name, path in parts.items():
        shutil.copyfile(path, dest / 'firmware' / name)
    for path in support:
        shutil.copyfile(path, dest / path.name)
    (dest / 'index.html').write_text(page, encoding='utf8')
    (dest / 'manifest.json').write_text(manifest, encoding='utf8')
    return {'environment': selected['environment'], 'identity': selected['identity'],
            'url': 'http://127.0.0.1:8767/' + selected['suffix'] + '/',
            'parts': {name: {'bytes': path.stat().st_size,
                             'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                      for name, path in parts.items()}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='Validate templates without writing an installer')
    args = parser.parse_args()
    print(json.dumps(dry_run() if args.dry_run else prepare(), indent=2, ensure_ascii=False))
