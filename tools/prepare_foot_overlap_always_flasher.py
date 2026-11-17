"""Prepare v29 continuous-overlap comparison firmware."""
import argparse
import json
import prepare_foot_fast_flasher as base
ENVIRONMENT='atoms3cam-foot-overlap-always'
IDENTITY='walking-foot-overlap-always-v29'
SUFFIX='foot-overlap-always'
PROFILE={'environment':ENVIRONMENT,'identity':IDENTITY,'suffix':SUFFIX,
         'title':'歩行器 v29 足認識の事前準備継続版',
         'name':'RW Walker v29 continuous overlap',
         'description':'<p>画像取得待ち以外も事前準備を継続する比較版です。v27のコピー削減・v28の不要探索削減・ログ取得修正webfix1を含みます。</p>\n  '
                       '<p>書込み後は左右の足角度と画像を確認してください。バッテリー給電を切り離してUSBで書き込み、USB中はモータを動かさないでください。</p>\n  '}
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),ensure_ascii=False,indent=2))
