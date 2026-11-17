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
import prepare_foot_fast_downloadfix2_flasher as fix2
import prepare_foot_frame_ready_flasher as ready
import prepare_foot_speed_flasher as speed
import prepare_foot_speed_fix_flasher as speed_fix
import prepare_foot_capture_flasher as capture
import prepare_foot_overlap_flasher as overlap
import prepare_foot_native_flasher as native
import prepare_foot_native_stream_flasher as native_stream
import prepare_foot_export_webfix_flasher as export_fix
import prepare_foot_copy_delta_flasher as copy_delta
import prepare_foot_template_prune_flasher as template_prune
import prepare_foot_fast_flasher as base

ROOT = Path(__file__).resolve().parents[1]


def snapshot(path):
    return {str(p.relative_to(path)):p.read_bytes() for p in path.rglob('*') if p.is_file()}


def check(profile, parent, additions, page_terms):
    fix = profile
    config=configparser.ConfigParser(interpolation=None)
    config.read(ROOT/'platformio.ini',encoding='utf8')
    section=config['env:'+fix.ENVIRONMENT]
    assert section['extends']=='env:'+parent.ENVIRONMENT

    def flags(name):
        return re.sub(r'\$\{([^}]+)\.build_flags\}',lambda m:flags(m[1]),config[name]['build_flags'])

    original=flags('env:'+base.ENVIRONMENT).split()
    assert flags('env:'+fix.ENVIRONMENT).split()==original+additions
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
        parent_path=root/'flasher'/parent.SUFFIX
        if parent_path != old:
            (parent_path/'firmware').mkdir(parents=True)
            (parent_path/'index.html').write_bytes(b'parent page')
            (parent_path/'firmware/firmware.bin').write_bytes(b'parent firmware')
        previous=snapshot(old);previous_parent=snapshot(parent_path);before=snapshot(root)
        dry=fix.dry_run(root);assert snapshot(root)==before
        assert dry['identity']==fix.IDENTITY and dry['url'].endswith('/'+fix.SUFFIX+'/')
        page,_=base.installer_text(root,profile=fix.PROFILE)
        assert all(term in page for term in page_terms)
        build=root/'.pio/build'/fix.ENVIRONMENT;build.mkdir(parents=True)
        for name in ['bootloader.bin','partitions.bin']:(build/name).write_bytes(name.encode())
        boot=root/'boot_app0.bin';boot.write_bytes(b'boot')
        firmware=build/'firmware.bin'
        for rejected in dict.fromkeys([base.IDENTITY,parent.IDENTITY]):
            firmware.write_bytes(b'\0'+rejected.encode()+b'\0')
            before=snapshot(root)
            try:fix.prepare(root,boot)
            except SystemExit:pass
            else:raise AssertionError('Parent/old firmware accepted as '+fix.IDENTITY)
            assert snapshot(root)==before
        firmware.write_bytes(b'\0'+fix.IDENTITY.encode()+b'\0payload')
        result=fix.prepare(root,boot);dest=root/'flasher'/fix.SUFFIX
        assert (dest/'firmware/firmware.bin').read_bytes()==firmware.read_bytes()
        assert result['parts']['firmware.bin']['sha256']==hashlib.sha256(firmware.read_bytes()).hexdigest()
        manifest=json.loads((dest/'manifest.json').read_text(encoding='utf8'))
        assert manifest['version']==fix.IDENTITY
        assert [(p['path'],p['offset']) for p in manifest['builds'][0]['parts']]==base.EXPECTED_PARTS
        assert snapshot(old)==previous
        assert snapshot(parent_path)==previous_parent
    print(f'{fix.IDENTITY} installer PASS: v20 flags retained, distinct ID/layout, old identity rejected, all old bytes retained')


def main():
    check(fix, base, ['-DEXPORT_RETRY_FIX=1'], ['通信再試行', '先に「RWLOGを取得・再開」で保存'])
    check(fix2, fix, ['-DEXPORT_RETRY_FIX=1', '-DEXPORT_COOPERATIVE_FIX=1'], ['小分けに実行', '再起動理由'])
    check(ready, fix2, ['-DEXPORT_RETRY_FIX=1', '-DEXPORT_COOPERATIVE_FIX=1', '-DCAMERA_FRAME_READY_V21=1'], ['全画素', '実機では未確認'])
    check(speed, ready, ['-DEXPORT_RETRY_FIX=1', '-DEXPORT_COOPERATIVE_FIX=1', '-DCAMERA_FRAME_READY_V21=1', '-DFOOT_SPEED_V22=1'], ['行間待ち時間', '短縮時間は未確認'])
    check(speed_fix, speed, ['-DEXPORT_RETRY_FIX=1', '-DEXPORT_COOPERATIVE_FIX=1', '-DCAMERA_FRAME_READY_V21=1', '-DFOOT_SPEED_V22=1', '-DFOOT_SPEED_FIX1=1'], ['カメラ初期化', 'まず測定を開始せず'])
    check(capture, speed_fix, ['-DEXPORT_RETRY_FIX=1', '-DEXPORT_COOPERATIVE_FIX=1', '-DCAMERA_FRAME_READY_V21=1', '-DFOOT_SPEED_V22=1', '-DFOOT_SPEED_FIX1=1', '-DFOOT_CAPTURE_V23=1'], ['カメラ取得', 'まず測定を開始せず'])
    check(overlap, capture, ['-DEXPORT_RETRY_FIX=1','-DEXPORT_COOPERATIVE_FIX=1','-DCAMERA_FRAME_READY_V21=1','-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1'], ['画像の受信中', 'まず測定を開始せず'])
    check(native, overlap, ['-DEXPORT_RETRY_FIX=1','-DEXPORT_COOPERATIVE_FIX=1','-DCAMERA_FRAME_READY_V21=1','-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1','-DCAMERA_NATIVE_HB=96'], ['生画像236行', '測定を始めず'])
    check(native_stream, native, ['-DEXPORT_RETRY_FIX=1','-DEXPORT_COOPERATIVE_FIX=1','-DCAMERA_FRAME_READY_V21=1','-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1','-DCAMERA_NATIVE_HB=96','-DFOOT_NATIVE_STREAM_V26=1'], ['全画像の到着', '測定を始めず'])
    check(export_fix, native_stream, ['-DEXPORT_RETRY_FIX=1','-DEXPORT_COOPERATIVE_FIX=1','-DCAMERA_FRAME_READY_V21=1','-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1','-DCAMERA_NATIVE_HB=96','-DFOOT_NATIVE_STREAM_V26=1','-DFOOT_EXPORT_WEBFIX=1'], ['webfix1', 'USB'])
    check(copy_delta, export_fix, ['-DEXPORT_RETRY_FIX=1','-DEXPORT_COOPERATIVE_FIX=1','-DCAMERA_FRAME_READY_V21=1','-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1','-DCAMERA_NATIVE_HB=96','-DFOOT_NATIVE_STREAM_V26=1','-DFOOT_EXPORT_WEBFIX=1','-DFOOT_COPY_DELTA_V27=1'], ['webfix1', 'USB'])
    check(template_prune, copy_delta, ['-DEXPORT_RETRY_FIX=1','-DEXPORT_COOPERATIVE_FIX=1','-DCAMERA_FRAME_READY_V21=1','-DFOOT_SPEED_V22=1','-DFOOT_SPEED_FIX1=1','-DFOOT_CAPTURE_V23=1','-DFOOT_ROW_OVERLAP_V24=1','-DFOOT_NATIVE_SKIP_V25=1','-DCAMERA_NATIVE_HB=96','-DFOOT_NATIVE_STREAM_V26=1','-DFOOT_EXPORT_WEBFIX=1','-DFOOT_COPY_DELTA_V27=1','-DFOOT_TEMPLATE_PRUNE_V28=1'], ['webfix1', 'USB'])


if __name__=='__main__':
    main()
