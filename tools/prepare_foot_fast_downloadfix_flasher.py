"""Prepare the isolated v20 download retry installer, preserving all older pages."""
import argparse
import json
import prepare_foot_fast_flasher as base

ENVIRONMENT = 'atoms3cam-foot-fast-downloadfix'
IDENTITY = 'walking-foot-fast-v20-downloadfix1'
SUFFIX = 'foot-fast-downloadfix'
PROFILE = {'environment':ENVIRONMENT, 'identity':IDENTITY, 'suffix':SUFFIX,
           'title':'歩行器 v20 ログ取得の通信修正版',
           'name':'RW Walker v20 download retry fix1',
           'description':'<p>ログの確定待ちと取得中の通信再試行を修正した版です。'
           '足角度認識とエネルギー制御はv20と同じです。</p>\n  '
           '<p>書込みは本体を再起動します。機体に未保存の測定がある場合は、'
           '先に「RWLOGを取得・再開」で保存してください。</p>\n  '}


def dry_run(root=base.ROOT):
    return base.dry_run(root, profile=PROFILE)


def prepare(root=base.ROOT, boot_app=None):
    return base.prepare(root, boot_app, profile=PROFILE)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    print(json.dumps(dry_run() if args.dry_run else prepare(), indent=2, ensure_ascii=False))
