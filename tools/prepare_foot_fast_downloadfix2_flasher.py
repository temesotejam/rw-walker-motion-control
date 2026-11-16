"""Prepare the isolated cooperative export fix; preserve older installers."""
import argparse
import json
import prepare_foot_fast_flasher as base

ENVIRONMENT = 'atoms3cam-foot-fast-downloadfix2'
IDENTITY = 'walking-foot-fast-v20-downloadfix2'
SUFFIX = 'foot-fast-downloadfix2'
PROFILE = {'environment': ENVIRONMENT, 'identity': IDENTITY, 'suffix': SUFFIX,
           'title': '歩行器 v20 ログ生成・再起動対策版',
           'name': 'RW Walker v20 cooperative export fix2',
           'description': '<p>ログ付加情報の生成を小分けに実行し、通信・異常監視の処理時間を確保します。'
           '再起動理由と生成進捗を診断に追加します。足認識・エネルギー制御の設定はv20を継承します。</p>\n  '
           '<p>書込みは本体を再起動します。未保存のログが残っている場合は先に保存してください。'
           'この版の実機での取得成功は、書込み後に確認してください。</p>\n  '}

def dry_run(root=base.ROOT):
    return base.dry_run(root, profile=PROFILE)

def prepare(root=base.ROOT, boot_app=None):
    return base.prepare(root, boot_app, profile=PROFILE)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    print(json.dumps(dry_run() if args.dry_run else prepare(), indent=2, ensure_ascii=False))
