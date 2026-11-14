"""Prepare the separate v19 energy-admission installer after a successful build.

The v18 and earlier installers are preserved. --dry-run checks the source
templates and manifest without creating an installer or requiring a binary.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = 'atoms3cam-energy-admission'
IDENTITY = 'walking-energy-admission-v19'
SUFFIX = 'energy-admission'


def installer_text(root=ROOT):
    """Render and validate the installer templates without writing files."""
    page = (root / 'flasher/index.html').read_text(encoding='utf8')
    old_identity = 'angle-early-weak-zero-v2'
    if page.count(old_identity) != 1 or page.count('<esp-web-install-button ') != 1:
        raise ValueError('Unexpected base installer template')
    page = page.replace(old_identity, IDENTITY)
    page = page.replace('<title>歩行器ファームウェア書き込み</title>',
                        '<title>歩行器 v19 エネルギー入力試験版</title>')
    page = page.replace('<h1>歩行器ファームウェア書き込み</h1>',
                        '<h1>歩行器 v19 エネルギー入力試験版</h1>')
    description = (
        '<p>戻り救済の後、実ZEROクロスで主入力を再計算します。'
        '救済と主入力は半周期の共有80ms実験枠を使います。</p>\n  '
        '<p>前倒し候補のモデル支持を確認し、範囲外はZEROで再評価します。'
        '前後の持続逸脱では入力を停止します。転倒防止を保証するものではありません。</p>\n  '
        '<p>この版の実機歩行は未確認です。v18の比較用ページは保持しています。</p>\n  '
    )
    page = page.replace('<esp-web-install-button ', description + '<esp-web-install-button ', 1)
    manifest = json.loads((root / 'flasher/manifest.json').read_text(encoding='utf8'))
    manifest.update(name='RW Walker energy admission v19 trial', version=IDENTITY)
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
    # Reject another version or an identity embedded in a longer identifier.
    pattern = rb'(?<![A-Za-z0-9_-])' + re.escape(IDENTITY.encode()) + rb'(?![A-Za-z0-9_-])'
    if re.search(pattern, blob) is None:
        raise SystemExit('Build ' + ENVIRONMENT + ' successfully first (v19 identity required).')
    parts = {name: build / name for name in ['bootloader.bin', 'partitions.bin', 'firmware.bin']}
    parts['boot_app0.bin'] = Path(boot_app) if boot_app is not None else (
        Path.home() / '.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/partitions/boot_app0.bin')
    missing = [str(path) for path in parts.values() if not path.is_file()]
    support = [root / 'flasher' / name for name in ['flasher-completion.js', 'soft-boot.js']]
    missing += [str(path) for path in support if not path.is_file()]
    if missing:
        raise SystemExit('Missing installer file(s): ' + ', '.join(missing))
    page, manifest = installer_text(root)
    # Finish validation before creating any destination file.
    dest = root / 'flasher' / SUFFIX
    (dest / 'firmware').mkdir(parents=True, exist_ok=True)
    for name, path in parts.items():
        shutil.copyfile(path, dest / 'firmware' / name)
    for path in support:
        shutil.copyfile(path, dest / path.name)
    (dest / 'index.html').write_text(page, encoding='utf8')
    (dest / 'manifest.json').write_text(manifest, encoding='utf8')
    return {'environment': ENVIRONMENT, 'identity': IDENTITY,
            'url': 'http://127.0.0.1:8767/' + SUFFIX + '/',
            'parts': {name: {'bytes': path.stat().st_size,
                             'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                      for name, path in parts.items()}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='Validate templates without writing an installer')
    args = parser.parse_args()
    if args.dry_run:
        page, manifest = installer_text()
        result = {'dry_run': True, 'environment': ENVIRONMENT, 'identity': IDENTITY,
                  'url': 'http://127.0.0.1:8767/' + SUFFIX + '/',
                  'page_bytes': len(page.encode('utf8')), 'manifest': json.loads(manifest)}
    else:
        result = prepare()
    print(json.dumps(result, indent=2, ensure_ascii=False))
