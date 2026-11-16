"""Prepare the isolated native sensor-readout v25 installer."""
import argparse
import json
import prepare_foot_fast_flasher as base
ENVIRONMENT='atoms3cam-foot-native'
IDENTITY='walking-foot-native-v25'
SUFFIX='foot-native'
PROFILE={'environment':ENVIRONMENT,'identity':IDENTITY,'suffix':SUFFIX,
         'title':'歩行器 v25 センサー読出しの短縮試験',
         'name':'RW Walker v25 native sensor readout',
         'description':'<p>センサー内部の行間引きと受信中の認識を使います。足角度の横座標と探索範囲を保ち、生画像236行を従来の座標へ対応させます。</p>\n  '
                       '<p>書込み後は測定を始めず、左右の足角度と画像更新を確認してください。</p>\n  '}
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true')
    args=p.parse_args();print(json.dumps(dry_run() if args.dry_run else prepare(),indent=2,ensure_ascii=False))
