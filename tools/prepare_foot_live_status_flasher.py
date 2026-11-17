"""Prepare v35 compact status comparison; v34 capture and logs unchanged."""
import argparse
import json
import prepare_foot_fast_flasher as base
import prepare_foot_worker26_flasher as parent
ENVIRONMENT='atoms3cam-foot-live-status'
IDENTITY='walking-foot-live-status-v35'
SUFFIX='foot-live-status'
PROFILE=dict(parent.PROFILE,environment=ENVIRONMENT,identity=IDENTITY,suffix=SUFFIX,
             title='歩行器 v35 Web通信軽量化版',name='RW Walker v35 live status',
             description='<p>画面の通常更新を軽量化し、カメラ処理との通信競合を減らします。詳細診断は保存時に取得します。v34の26ms周期・足1600行・webfix1を維持します。</p>\n  '
                         '<p>バッテリー給電を切り離してUSBで書き込み、USB接続中はモータを動かさないでください。</p>\n  ')
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),ensure_ascii=False,indent=2))
