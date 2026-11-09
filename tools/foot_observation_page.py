"""Package and verify the pinned, independent foot-observation flasher."""
import argparse
import hashlib
import json
import re
import shutil
import urllib.request
from pathlib import Path

COMMIT = 'a50c80e3f9c697ad2767b356dfd1a2a96c8d4f1d'
BUILD_ID = 'foot-control-observation-v1'
PARTS = {'firmware/bootloader.bin': 0, 'firmware/partitions.bin': 32768,
         'firmware/boot_app0.bin': 57344, 'firmware/firmware.bin': 65536}


def digest(data):
    if not data:
        raise ValueError('empty flash part')
    return hashlib.sha256(data).hexdigest()


def manifest():
    return {'name': 'Foot control observation diagnostic',
            'version': BUILD_ID + '-' + COMMIT[:12],
            'new_install_prompt_erase': True,
            'new_install_improv_wait_time': 0,
            'builds': [{'chipFamily': 'ESP32-S3', 'parts': [
                {'path': path, 'offset': offset} for path, offset in PARTS.items()]}]}


def verify(read):
    info = json.loads(read('BUILD.json'))
    if set(info) != {'commit', 'diagnostic_build_id', 'firmware_sha256', 'bundle_sha256'} or info['commit'] != COMMIT or info['diagnostic_build_id'] != BUILD_ID:
        raise ValueError('unexpected build identity')
    if not all(re.fullmatch(r'[0-9a-f]{64}', info[k]) for k in ('firmware_sha256', 'bundle_sha256')):
        raise ValueError('invalid build hashes')
    prefix = 'builds/' + COMMIT + '-' + info['firmware_sha256'] + '-' + info['bundle_sha256'] + '/'
    if json.loads(read(prefix + 'manifest.json')) != manifest():
        raise ValueError('unexpected manifest')
    page = read('index.html').decode('utf8')
    if COMMIT not in page or BUILD_ID not in page or 'manifest="' + prefix + 'manifest.json"' not in page:
        raise ValueError('page identity mismatch')
    sums = {}
    for line in read('SHA256SUMS.txt').decode('ascii').splitlines():
        match = re.fullmatch(r'([0-9a-f]{64})  (firmware/[a-z0-9_.-]+)', line)
        if not match or match[2] in sums:
            raise ValueError('invalid hash list')
        sums[match[2]] = match[1]
    if set(sums) != set(PARTS):
        raise ValueError('missing or extra flash part')
    if sums['firmware/firmware.bin'] != info['firmware_sha256']:
        raise ValueError('firmware identity mismatch')
    calculated_bundle = hashlib.sha256(''.join(sums[path] for path in PARTS).encode('ascii')).hexdigest()
    if calculated_bundle != info['bundle_sha256']:
        raise ValueError('bundle identity mismatch')
    for path in PARTS:
        if digest(read(prefix + path)) != sums[path]:
            raise ValueError('hash mismatch: ' + path)


def package(source, stable, dest):
    if dest.exists():
        raise ValueError('destination already exists')
    if (source / 'source-sha.txt').read_text(encoding='ascii').strip() != COMMIT:
        raise ValueError('wrong source checkout')
    for name in ('bootloader.bin', 'partitions.bin', 'boot_app0.bin'):
        if digest((source / 'firmware' / name).read_bytes()) != digest((stable / name).read_bytes()):
            raise ValueError('different shared flash part: ' + name)
    hashes = {name: digest((source / name).read_bytes()) for name in PARTS}
    firmware_sha = hashes['firmware/firmware.bin']
    bundle_sha = hashlib.sha256(''.join(hashes[name] for name in PARTS).encode('ascii')).hexdigest()
    prefix = 'builds/' + COMMIT + '-' + firmware_sha + '-' + bundle_sha + '/'
    dest.mkdir(parents=True)
    for name in PARTS:
        target = dest / prefix / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / name, target)
    (dest / prefix / 'manifest.json').write_text(json.dumps(manifest(), indent=2) + '\n', encoding='utf8')
    (dest / 'BUILD.json').write_text(json.dumps({'commit': COMMIT, 'diagnostic_build_id': BUILD_ID, 'firmware_sha256': firmware_sha, 'bundle_sha256': bundle_sha}, indent=2) + '\n', encoding='utf8')
    sums = ''.join(hashes[name] + '  ' + name + '\n' for name in PARTS)
    (dest / 'SHA256SUMS.txt').write_text(sums, encoding='ascii')
    page = f'''<!doctype html><html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>足フレーム時刻診断</title><script src="../flasher-completion.js"></script>
<script type="module" src="https://unpkg.com/esp-web-tools@10.1.1/dist/web/install-button.js?module"></script>
<style>body{{font:16px/1.7 system-ui,sans-serif;max-width:760px;margin:3rem auto;padding:0 1rem}}code{{word-break:break-all}}button{{font-size:1.1rem;padding:.8rem 1rem}}</style></head>
<body><h1>足フレーム時刻診断</h1><p>歩行中に制御側が足フレームを取得した時刻を記録する診断版です。通常の歩行制御が動作します。</p>
<p>Chrome または Edge で機体を USB 接続し、書き込むポートを選択してください。消去の確認が表示された場合、「Erase device」にチェックを入れないでください。</p>
<esp-web-install-button manifest="{prefix}manifest.json"><button slot="activate">診断版を書き込む</button><span slot="unsupported">Chrome または Edge を使用してください。</span></esp-web-install-button>
<p>診断 ID: <code>{BUILD_ID}</code><br>ソース: <code>{COMMIT}</code></p>
<p>このページからの書き込みは、ブラウザと実機での動作確認がまだ済んでいません。</p>
<p>書き込み後、機体のステータスまたは RWLOG メタデータで診断 ID を確認してください。取得できなかったフレーム時刻は不明です。</p>
<p><a href="../">通常版に戻る</a>・<a href="../camera/">カメラ診断に戻る</a></p></body></html>'''
    (dest / 'index.html').write_text(page, encoding='utf8')
    verify(lambda name: (dest / name).read_bytes())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--package', action='store_true')
    group.add_argument('--path', type=Path)
    group.add_argument('--url')
    parser.add_argument('--source', type=Path)
    parser.add_argument('--stable', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.package:
        if not all((args.source, args.stable, args.output)):
            parser.error('--package needs --source, --stable and --output')
        package(args.source, args.stable, args.output)
    elif args.path:
        verify(lambda name: (args.path / name).read_bytes())
    else:
        base = args.url.rstrip('/') + '/'
        def read(name):
            request = urllib.request.Request(base + name + '?verify=' + COMMIT,
                                             headers={'Cache-Control': 'no-cache'})
            with urllib.request.urlopen(request, timeout=30) as response:
                return response.read()
        verify(read)
