"""Prepare the isolated native sensor and incremental tracker v26 installer."""
import argparse
import json
import prepare_foot_fast_flasher as base
ENVIRONMENT='atoms3cam-foot-native-stream'
IDENTITY='walking-foot-native-stream-v26'
SUFFIX='foot-native-stream'
PROFILE={'environment':ENVIRONMENT,'identity':IDENTITY,'suffix':SUFFIX,
         'title':'歩行器 v26 足角度の受渡し短縮試験',
         'name':'RW Walker v26 native incremental tracking',
         'description':'<p>センサーの読出しと左右マーカーの認識を並行して進めます。全画像の到着と同じフレームであることを確認して足角度を更新します。</p>\n  '
                       '<p>書込み後は測定を始めず、左右の足角度と画像更新を確認してください。</p>\n  '}
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true')
    args=p.parse_args();print(json.dumps(dry_run() if args.dry_run else prepare(),indent=2,ensure_ascii=False))
