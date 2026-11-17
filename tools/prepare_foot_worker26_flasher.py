"""Prepare v34 26ms observer comparison with sufficient full-run archive."""
import argparse
import json
import prepare_foot_fast_flasher as base
import prepare_foot_histogram_flasher as parent
ENVIRONMENT='atoms3cam-foot-worker26'
IDENTITY='walking-foot-worker26-v34'
SUFFIX='foot-worker26'
PROFILE=dict(parent.PROFILE,environment=ENVIRONMENT,identity=IDENTITY,suffix=SUFFIX,
             title='歩行器 v34 足観測周期26ms版',name='RW Walker v34 worker26',
             description='<p>センサーHB96を維持し、足観測の周期を26msに短縮します。同期を含む全記録用に足1600行とメタデータ容量を確保し、webfix1を含みます。</p>\n  '
                         '<p>バッテリー給電を切り離してUSBで書き込み、USB接続中はモータを動かさないでください。</p>\n  ')
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),ensure_ascii=False,indent=2))
