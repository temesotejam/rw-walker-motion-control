"""Prepare the isolated v22 camera startup correction installer."""
import argparse
import json
import prepare_foot_fast_flasher as base

ENVIRONMENT = 'atoms3cam-foot-speed-fix1'
IDENTITY = 'walking-foot-speed-v22-fix1'
SUFFIX = 'foot-speed-fix1'
PROFILE = {'environment': ENVIRONMENT, 'identity': IDENTITY, 'suffix': SUFFIX,
           'title': '歩行器 v22 カメラ初期化修正版',
           'name': 'RW Walker v22 camera startup correction',
           'description': '<p>カメラ初期化の修正と足認識の高速化を含む版です。'
                          '歩行のエネルギー制御はv19を継承します。</p>\n  '
                          '<p>未保存ログを先に保存してください。書込み後はまず測定を開始せず、'
                          '左右の足角度と画像更新を確認してください。</p>\n  '}


def dry_run(root=base.ROOT):
    return base.dry_run(root, profile=PROFILE)


def prepare(root=base.ROOT, boot_app=None):
    return base.prepare(root, boot_app, profile=PROFILE)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()
    print(json.dumps(dry_run() if args.dry_run else prepare(), indent=2, ensure_ascii=False))
