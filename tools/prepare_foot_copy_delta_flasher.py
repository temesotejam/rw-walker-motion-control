"""Prepare v27 incremental row-copy candidate; user performs browser flashing."""
import argparse
import json
import prepare_foot_fast_flasher as base
ENVIRONMENT='atoms3cam-foot-copy-delta'
IDENTITY='walking-foot-native-stream-v27'
SUFFIX='foot-copy-delta'
PROFILE={'environment':ENVIRONMENT,'identity':IDENTITY,'suffix':SUFFIX,
         'title':'歩行器 v27 足認識のコピー削減版',
         'name':'RW Walker v27 incremental row copy',
         'description':'<p>同じ画像の行を繰り返しコピーする処理と、画像を待っていない間の準備処理を減らす比較版です。ログ取得修正webfix1を含みます。</p>\n  '
                       '<p>認識速度の実機改善は未確認です。書き込み後は測定を始めず、左右の足角度と画像を確認してください。</p>\n  '
                       '<p>バッテリー給電を切り離してからUSBで書き込み、USB接続中はモータを動かさないでください。</p>\n  '}
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true')
    args=p.parse_args();print(json.dumps(dry_run() if args.dry_run else prepare(),indent=2,ensure_ascii=False))
