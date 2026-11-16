"""Validate the v20 environment/identity and isolated installer preparation.

No firmware build, serial access, real installer mutation or network is used.
Temporary byte fixtures exercise copy/hash and rejection paths, not bootability.
"""
import configparser
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('prepare_foot_fast', ROOT / 'tools/prepare_foot_fast_flasher.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


def snapshot(root):
    return {str(path.relative_to(root)): path.read_bytes() for path in root.rglob('*') if path.is_file()}


def rejected(call, root, expected):
    before = snapshot(root)
    try:
        call()
    except expected:
        pass
    else:
        raise AssertionError('Invalid installer input accepted')
    assert snapshot(root) == before, 'Rejection changed installer or source files'


def check_environment():
    config = configparser.ConfigParser(interpolation=None)
    config.read(ROOT / 'platformio.ini', encoding='utf8')
    target = config['env:atoms3cam-foot-fast']
    assert target['extends'] == 'env:atoms3cam-energy-admission'
    assert target['build_flags'].split() == ['${env:atoms3cam-energy-admission.build_flags}', '-DFOOT_LOW_LATENCY_V20=1']

    def flags(section):
        return re.sub(r'\$\{([^}]+)\.build_flags\}', lambda m: flags(m[1]), config[section]['build_flags'])

    previous = flags('env:atoms3cam-energy-admission').split()
    assert flags('env:atoms3cam-foot-fast').split() == previous + ['-DFOOT_LOW_LATENCY_V20=1']
    for env, identity in [('atoms3cam-foot-fast', helper.IDENTITY),
                          ('atoms3cam-energy-admission', 'walking-energy-admission-v19'),
                          ('atoms3cam-energy-transfer', 'walking-energy-transfer-v18'),
                          ('atoms3cam-return-energy', 'walking-return-energy-v17-rangefix1')]:
        definitions = [flag for flag in flags('env:' + env).split() if flag.startswith('-D')]
        result = subprocess.run(['g++', '-E', '-dM', '-x', 'c++', '-I' + str(ROOT / 'src'),
                                 '-include', 'runtime_diagnostics.h'] + definitions + ['-'],
                                input='', text=True, capture_output=True, check=True)
        assert '#define RUNTIME_DIAGNOSTIC_BUILD_ID ' + json.dumps(identity) in result.stdout
    print('v20 environment inherits v19 flags; native v17/v18/v19/v20 identities PASS')


def check_installer():
    with tempfile.TemporaryDirectory(prefix='foot_fast_flasher_') as folder:
        root = Path(folder); (root / 'flasher').mkdir()
        for name in ['index.html', 'manifest.json', 'flasher-completion.js', 'soft-boot.js']:
            shutil.copyfile(ROOT / 'flasher' / name, root / 'flasher' / name)
        # Old pages and firmware are sentinels: the new generator must preserve
        # all their bytes, including on rejected input and repeated preparation.
        for suffix in ['energy-admission', 'energy-transfer', 'return-energy-rangefix']:
            old = root / 'flasher' / suffix; (old / 'firmware').mkdir(parents=True)
            (old / 'index.html').write_text('preserve ' + suffix, encoding='utf8')
            (old / 'firmware/firmware.bin').write_bytes(b'old\x00' + suffix.encode())
        old_snapshots = {suffix: snapshot(root / 'flasher' / suffix) for suffix in
                         ['energy-admission', 'energy-transfer', 'return-energy-rangefix']}
        before = snapshot(root)
        dry = helper.dry_run(root)
        assert snapshot(root) == before and not (root / '.pio').exists()
        assert dry['dry_run'] and dry['identity'] == helper.IDENTITY and dry['environment'] == helper.ENVIRONMENT
        assert dry['url'] == 'http://127.0.0.1:8767/foot-fast/'
        page, manifest_text = helper.installer_text(root)
        assert '足角度認識の低遅延版' in page and '歩行のエネルギー制御はv19を継承' in page
        assert '認識速度と精度は未確認' in page and 'まずモータ測定を開始せずに' in page and '足角度と更新を確認' in page
        assert page.count('<esp-web-install-button ') == 1 and helper.IDENTITY in page
        manifest = json.loads(manifest_text)
        assert manifest['version'] == helper.IDENTITY
        assert [(p['path'], p['offset']) for p in manifest['builds'][0]['parts']] == helper.EXPECTED_PARTS
        build = root / '.pio/build' / helper.ENVIRONMENT; build.mkdir(parents=True)
        firmware = build / 'firmware.bin'; boot_app = root / 'boot_app0.bin'
        call = lambda: helper.prepare(root, boot_app)
        rejected(call, root, SystemExit)  # No successful build present.
        for bad in [b'walking-energy-admission-v19\x00', b'walking-foot-fast-v200\x00',
                    b'walking-foot-fast-v20-debug\x00', b'prefix-walking-foot-fast-v20\x00']:
            firmware.write_bytes(bad); rejected(call, root, SystemExit)
        firmware.write_bytes(b'\x00' + helper.IDENTITY.encode() + b'"}\x00')
        rejected(call, root, SystemExit)  # Correct ID, missing remaining parts.
        for name in ['bootloader.bin', 'partitions.bin']:
            (build / name).write_bytes(b'fixture-' + name.encode())
        boot_app.write_bytes(b'')
        rejected(call, root, SystemExit)  # Empty input is not an installer part.
        boot_app.write_bytes(b'fixture-boot-app')
        base_manifest = json.loads((root / 'flasher/manifest.json').read_text(encoding='utf8'))
        variants = []
        for key, value in [('chipFamily', 'ESP32'), ('chipFamily', None)]:
            bad = copy.deepcopy(base_manifest); bad['builds'][0][key] = value; variants.append(bad)
        bad = copy.deepcopy(base_manifest); bad['builds'][0]['parts'][3]['offset'] = 0; variants.append(bad)
        bad = copy.deepcopy(base_manifest); bad['builds'][0]['parts'][0]['path'] = '../bootloader.bin'; variants.append(bad)
        bad = copy.deepcopy(base_manifest); bad['builds'][0]['parts'].pop(); variants.append(bad)
        bad = copy.deepcopy(base_manifest); bad['builds'].append(copy.deepcopy(bad['builds'][0])); variants.append(bad)
        manifest_path = root / 'flasher/manifest.json'
        for bad in variants:
            manifest_path.write_text(json.dumps(bad), encoding='utf8'); rejected(call, root, ValueError)
        manifest_path.write_text(json.dumps(base_manifest), encoding='utf8')
        page_path = root / 'flasher/index.html'; base_page = page_path.read_text(encoding='utf8')
        page_path.write_text(base_page.replace('angle-early-weak-zero-v2', 'wrong-template'), encoding='utf8')
        rejected(call, root, ValueError); page_path.write_text(base_page, encoding='utf8')
        report = call(); destination = root / 'flasher/foot-fast'
        assert report['identity'] == helper.IDENTITY and report['url'] == dry['url']
        for name, part in report['parts'].items():
            source = boot_app if name == 'boot_app0.bin' else build / name
            output = destination / 'firmware' / name
            assert output.read_bytes() == source.read_bytes()
            assert part == {'bytes': source.stat().st_size, 'sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
        assert (destination / 'index.html').read_text(encoding='utf8') == page
        assert json.loads((destination / 'manifest.json').read_text(encoding='utf8')) == manifest
        for name in ['flasher-completion.js', 'soft-boot.js']:
            assert (destination / name).read_bytes() == (root / 'flasher' / name).read_bytes()
        for suffix, old in old_snapshots.items():
            assert snapshot(root / 'flasher' / suffix) == old
        # Preserve an already generated v20 installer when a later input fails.
        firmware.write_bytes(b'walking-energy-admission-v19\x00'); rejected(call, root, SystemExit)
    print('isolated dry-run, page guidance, four-part copy/hash, identity/layout/template rejection and older installers PASS')


def main():
    check_environment(); check_installer()
    result = subprocess.run([sys.executable, str(ROOT / 'tools/prepare_foot_fast_flasher.py'), '--dry-run'],
                            text=True, encoding='utf8', capture_output=True, check=True)
    assert json.loads(result.stdout)['dry_run']
    print('actual CLI --dry-run PASS; no real firmware or installer prepared')


if __name__ == '__main__':
    main()
