"""Prepare v33 HB80 sensor timing comparison firmware."""
import argparse
import json
import prepare_foot_fast_flasher as base
import prepare_foot_histogram_flasher as parent
ENVIRONMENT='atoms3cam-foot-hblank80'
IDENTITY='walking-foot-hblank80-v33'
SUFFIX='foot-hblank80'
PROFILE=dict(parent.PROFILE,environment=ENVIRONMENT,identity=IDENTITY,suffix=SUFFIX,
             title='歩行器 v33 HB80比較版・不採用',name='RW Walker v33 HB80 comparison only',
             description='<p>実機で画像の白化とマーカー検出不能を確認したため、この版は不採用です。通常の測定には使用しないでください。</p>\n  '
                         '<p>画像サイズと認識範囲を保ち、行間待ちをHB96からHB80へ短縮した比較版です。webfix1を含みます。</p>\n  '
                         '<p>バッテリー給電を切り離してUSBで書き込み、USB接続中はモータを動かさないでください。</p>\n  ')
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),ensure_ascii=False,indent=2))
