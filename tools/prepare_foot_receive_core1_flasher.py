"""Prepare v31 separate-core camera reception comparison firmware."""
import argparse
import json
import prepare_foot_receive_diag_flasher as parent
import prepare_foot_fast_flasher as base
ENVIRONMENT = 'atoms3cam-foot-receive-core1'
IDENTITY = 'walking-foot-receive-core1-v31'
SUFFIX = 'foot-receive-core1'
PROFILE = dict(parent.PROFILE, environment=ENVIRONMENT, identity=IDENTITY, suffix=SUFFIX,
               title='歩行器 v31 カメラ受信コア分離版', name='RW Walker v31 separate camera receive core',
               description='<p>カメラ受信をWi-Fiと別コアへ移し、IMUと制御より低い優先度で処理します。webfix1を含みます。</p>\n  '
                           '<p>書込み後は左右マーカーと画像を確認してください。バッテリーを切り離してUSBで書き込み、USB中はモータを動かさないでください。</p>\n  ')
def dry_run(root=base.ROOT): return base.dry_run(root, profile=PROFILE)
def prepare(root=base.ROOT, boot_app=None): return base.prepare(root, boot_app, profile=PROFILE)
if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),ensure_ascii=False,indent=2))
