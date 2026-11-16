"""Prepare isolated v24 reception/recognition overlap installer."""
import argparse
import json
import prepare_foot_fast_flasher as base
ENVIRONMENT='atoms3cam-foot-overlap'
IDENTITY='walking-foot-overlap-v24'
SUFFIX='foot-overlap'
PROFILE={'environment':ENVIRONMENT,'identity':IDENTITY,'suffix':SUFFIX,
         'title':'歩行器 v24 足画像の受信・認識並行化',
         'name':'RW Walker v24 camera recognition overlap',
         'description':'<p>画像の受信中にマーカー候補を計算し、正常な画像全体と一致を確認してから足角度へ反映します。</p>\n  '
                       '<p>書込み後はまず測定を開始せず、左右の足角度と画像更新を確認してください。</p>\n  '}
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true')
    args=p.parse_args();print(json.dumps(dry_run() if args.dry_run else prepare(),indent=2,ensure_ascii=False))
