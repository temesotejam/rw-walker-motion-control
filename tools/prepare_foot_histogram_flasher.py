"""Prepare v32 on-device receipt histogram firmware, HB96 baseline."""
import argparse
import json
import prepare_foot_fast_flasher as base
ENVIRONMENT='atoms3cam-foot-histogram'
IDENTITY='walking-foot-histogram-v32'
SUFFIX='foot-histogram'
PROFILE={'environment':ENVIRONMENT,'identity':IDENTITY,'suffix':SUFFIX,
         'title':'歩行器 v32 遅延集計版', 'name':'RW Walker v32 receipt histogram',
         'description':'<p>受取り遅延を本体内で集計し、通信負荷を抑えて比較する版です。センサーはHB96、webfix1を含みます。</p>\n  '
                       '<p>バッテリー給電を切り離してUSBで書き込み、USB接続中はモータを動かさないでください。</p>\n  '}
def dry_run(root=base.ROOT):return base.dry_run(root,profile=PROFILE)
def prepare(root=base.ROOT,boot_app=None):return base.prepare(root,boot_app,profile=PROFILE)
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(),ensure_ascii=False,indent=2))
