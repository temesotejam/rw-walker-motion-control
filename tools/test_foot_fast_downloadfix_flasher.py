"""Check the hotfix identity/inherited flags and isolation using temporary bytes."""
import configparser
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

import prepare_foot_fast_downloadfix_flasher as fix
import prepare_foot_fast_flasher as base

ROOT = Path(__file__).resolve().parents[1]


def snapshot(path):
    return {str(p.relative_to(path)):p.read_bytes() for p in path.rglob('*') if p.is_file()}


def main():
    config=configparser.ConfigParser(interpolation=None)
    config.read(ROOT/'platformio.ini',encoding='utf8')
    section=config['env:'+fix.ENVIRONMENT]
    assert section['extends']=='env:'+base.ENVIRONMENT

    def flags(name):
        return re.sub(r'\$\{([^}]+)\.build_flags\}',lambda m:flags(m[1]),config[name]['build_flags'])

    original=flags('env:'+base.ENVIRONMENT).split()
    assert flags('env:'+fix.ENVIRONMENT).split()==original+['-DEXPORT_RETRY_FIX=1']
    for name,identity in [(base.ENVIRONMENT,base.IDENTITY),(fix.ENVIRONMENT,fix.IDENTITY)]:
        definitions=[f for f in flags('env:'+name).split() if f.startswith('-D')]
        output=subprocess.run(['g++','-E','-dM','-x','c++','-I'+str(ROOT/'src'),'-include','runtime_diagnostics.h']+
            definitions+['-'],input='',capture_output=True,text=True,check=True).stdout
        assert '#define RUNTIME_DIAGNOSTIC_BUILD_ID '+json.dumps(identity) in output
    with tempfile.TemporaryDirectory(prefix='downloadfix_') as folder:
        root=Path(folder);(root/'flasher').mkdir()
        for name in ['index.html','manifest.json','flasher-completion.js','soft-boot.js']:
            shutil.copyfile(ROOT/'flasher'/name,root/'flasher'/name)
        old=root/'flasher/foot-fast';(old/'firmware').mkdir(parents=True)
        (old/'index.html').write_bytes(b'previous page');(old/'firmware/firmware.bin').write_bytes(b'previous firmware')
        previous=snapshot(old);before=snapshot(root)
        dry=fix.dry_run(root);assert snapshot(root)==before
        assert dry['identity']==fix.IDENTITY and dry['url'].endswith('/foot-fast-downloadfix/')
        page,_=base.installer_text(root,profile=fix.PROFILE)
        assert '通信再試行' in page and '先に「RWLOGを取得・再開」で保存' in page
        build=root/'.pio/build'/fix.ENVIRONMENT;build.mkdir(parents=True)
        for name in ['bootloader.bin','partitions.bin']:(build/name).write_bytes(name.encode())
        boot=root/'boot_app0.bin';boot.write_bytes(b'boot')
        firmware=build/'firmware.bin';firmware.write_bytes(b'\0'+base.IDENTITY.encode()+b'\0')
        before=snapshot(root)
        try:fix.prepare(root,boot)
        except SystemExit:pass
        else:raise AssertionError('Old v20 firmware accepted as hotfix')
        assert snapshot(root)==before
        firmware.write_bytes(b'\0'+fix.IDENTITY.encode()+b'\0payload')
        result=fix.prepare(root,boot);dest=root/'flasher'/fix.SUFFIX
        assert (dest/'firmware/firmware.bin').read_bytes()==firmware.read_bytes()
        assert result['parts']['firmware.bin']['sha256']==hashlib.sha256(firmware.read_bytes()).hexdigest()
        manifest=json.loads((dest/'manifest.json').read_text(encoding='utf8'))
        assert manifest['version']==fix.IDENTITY
        assert [(p['path'],p['offset']) for p in manifest['builds'][0]['parts']]==base.EXPECTED_PARTS
        assert snapshot(old)==previous
    print('Downloadfix installer PASS: v20 flags retained, distinct ID/layout, old identity rejected, all old bytes retained')


if __name__=='__main__':
    main()
