"""Prepare only the separately identified v18 energy-transfer installer.

Run after `platformio run -e atoms3cam-energy-transfer` succeeds. The v17 and
earlier installer directories are preserved. Generated installer files remain
local; this script is their reproducible source.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = 'atoms3cam-energy-transfer'
IDENTITY = 'walking-energy-transfer-v18'
SUFFIX = 'energy-transfer'


def installer_text(root=ROOT):
    """Render the source templates without requiring or copying a binary."""
    page = (root / 'flasher/index.html').read_text(encoding='utf8')
    old_identity = 'angle-early-weak-zero-v2'
    if page.count(old_identity) != 1 or page.count('<esp-web-install-button ') != 1:
        raise ValueError('Unexpected base installer template')
    page = page.replace(old_identity, IDENTITY)
    page = page.replace('<title>歩行器ファームウェア書き込み</title>',
                        '<title>歩行器 v18 エネルギー入力試験版</title>')
    page = page.replace('<h1>歩行器ファームウェア書き込み</h1>',
                        '<h1>歩行器 v18 エネルギー入力試験版</h1>')
    description = (
        '<p>戻り救済の要求エネルギー（J）と、中心通過時の通常入力を分けて扱います。'
        '救済後も、中心通過時に必要な入力を再評価します。</p>\n  '
        '<p>起動キックを除き、1半周期の合計入力時間は80ms以内の実験枠です。'
        '新しい版の実機歩行は未確認です。</p>\n  '
    )
    page = page.replace('<esp-web-install-button ', description + '<esp-web-install-button ', 1)
    manifest = json.loads((root / 'flasher/manifest.json').read_text(encoding='utf8'))
    manifest.update(name='RW Walker energy transfer v18 trial', version=IDENTITY)
    expected_parts = [('firmware/bootloader.bin', 0), ('firmware/partitions.bin', 32768),
                      ('firmware/boot_app0.bin', 57344), ('firmware/firmware.bin', 65536)]
    builds = manifest.get('builds', [])
    if (len(builds) != 1 or builds[0].get('chipFamily') != 'ESP32-S3' or
            [(part.get('path'), part.get('offset')) for part in builds[0].get('parts', [])] != expected_parts):
        raise ValueError('Unexpected ESP32-S3 installer layout')
    return page, json.dumps(manifest, indent=2, ensure_ascii=False) + '\n'


def prepare(root=ROOT, boot_app=None):
    build = root / '.pio/build' / ENVIRONMENT
    firmware = build / 'firmware.bin'
    blob = firmware.read_bytes() if firmware.is_file() else b''
    # The ID may be concatenated directly with JSON punctuation in firmware.
    pattern = rb'(?<![A-Za-z0-9_-])' + re.escape(IDENTITY.encode()) + rb'(?![A-Za-z0-9_-])'
    if re.search(pattern, blob) is None:
        raise SystemExit('Build ' + ENVIRONMENT + ' successfully first (v18 identity required).')
    parts = {name: build / name for name in ['bootloader.bin', 'partitions.bin', 'firmware.bin']}
    parts['boot_app0.bin'] = Path(boot_app) if boot_app is not None else (
        Path.home() / '.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/partitions/boot_app0.bin')
    missing = [str(path) for path in parts.values() if not path.is_file()]
    support = [root / 'flasher' / name for name in ['flasher-completion.js', 'soft-boot.js']]
    missing += [str(path) for path in support if not path.is_file()]
    if missing:
        raise SystemExit('Missing installer file(s): ' + ', '.join(missing))
    page, manifest = installer_text(root)
    # Validate all inputs before writing any destination file.
    dest = root / 'flasher' / SUFFIX
    (dest / 'firmware').mkdir(parents=True, exist_ok=True)
    for name, path in parts.items():
        shutil.copyfile(path, dest / 'firmware' / name)
    for path in support:
        shutil.copyfile(path, dest / path.name)
    (dest / 'index.html').write_text(page, encoding='utf8')
    (dest / 'manifest.json').write_text(manifest, encoding='utf8')
    report = {'environment': ENVIRONMENT, 'identity': IDENTITY,
              'url': 'http://127.0.0.1:8767/' + SUFFIX + '/',
              'parts': {name: {'bytes': path.stat().st_size,
                               'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                        for name, path in parts.items()}}
    return report


if __name__ == '__main__':
    print(json.dumps(prepare(), indent=2, ensure_ascii=False))
