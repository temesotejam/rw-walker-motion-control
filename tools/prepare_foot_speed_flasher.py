"""Prepare the isolated v22 sensor timing and recognition speed trial."""
import argparse
import json
import prepare_foot_fast_flasher as base
ENVIRONMENT='atoms3cam-foot-speed'
IDENTITY='walking-foot-speed-v22'
SUFFIX='foot-speed'
PROFILE={'environment':ENVIRONMENT,'identity':IDENTITY,'suffix':SUFFIX,
         'title':'歩行器 v22 足認識の速度優先試験版','name':'RW Walker v22 sensor timing speed trial',
         'description':'<p>カメラの行間待ち時間と露光上限を短くし、認識の重複計算を減らす試験版です。'
         '実機での短縮時間は未確認です。暗所で画像が暗くなる可能性があります。</p>\n  '
         '<p>未保存ログを先に保存してください。書込み後はまず測定を開始せず、足角度とプレビューを確認してください。'
         '比較用のv21ページも保持しています。</p>\n  '}
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),indent=2,ensure_ascii=False))
