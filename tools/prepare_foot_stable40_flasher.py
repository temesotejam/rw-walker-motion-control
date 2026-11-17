"""Prepare v36 headroom above 40Hz with cadence evidence and full logs."""
import argparse,json
import prepare_foot_fast_flasher as base
import prepare_foot_live_status_flasher as parent
ENVIRONMENT='atoms3cam-foot-stable40'
IDENTITY='walking-foot-stable40-v36'
SUFFIX='foot-stable40'
PROFILE=dict(parent.PROFILE,environment=ENVIRONMENT,identity=IDENTITY,suffix=SUFFIX,
             title='歩行器 v36 40Hz検証版',name='RW Walker v36 stable40',
             description='<p>観測周期23msで40Hz以上の余裕を確保し、有効更新間隔と5秒ごとの検出回数を記録します。実際の安定性は測定で確認します。足1760行・webfix1・軽量Web更新を含みます。</p>\n  '
                         '<p>バッテリー給電を切り離してUSBで書き込み、USB接続中はモータを動かさないでください。</p>\n  ')
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),ensure_ascii=False,indent=2))
