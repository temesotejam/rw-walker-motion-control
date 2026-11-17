"""Prepare v37 sensor-cadence observer with full 2048-row archive."""
import argparse,json
import prepare_foot_fast_flasher as base
import prepare_foot_stable40_flasher as parent
ENVIRONMENT='atoms3cam-foot-cadence20'
IDENTITY='walking-foot-cadence20-v37'
SUFFIX='foot-cadence20'
PROFILE=dict(parent.PROFILE,environment=ENVIRONMENT,identity=IDENTITY,suffix=SUFFIX,
             title='歩行器 v37 20ms観測周期版',name='RW Walker v37 cadence20',
             description='<p>センサーに近い20ms観測周期で更新の抜けを減らす比較版です。遅延20msの保証ではありません。足2048行・更新間隔計測・webfix1・軽量Web更新を含みます。</p>\n  '
                         '<p>バッテリー給電を切り離してUSBで書き込み、USB接続中はモータを動かさないでください。</p>\n  ')
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),ensure_ascii=False,indent=2))
