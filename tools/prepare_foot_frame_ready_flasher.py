"""Prepare the isolated full-frame DMA publication trial."""
import argparse
import json
import prepare_foot_fast_flasher as base

ENVIRONMENT = 'atoms3cam-foot-frame-ready'
IDENTITY = 'walking-foot-frame-ready-v21'
SUFFIX = 'foot-frame-ready'
PROFILE = {'environment': ENVIRONMENT, 'identity': IDENTITY, 'suffix': SUFFIX,
           'title': '歩行器 v21 画像受渡し待ち短縮試験版',
           'name': 'RW Walker v21 complete frame publication trial',
           'description': '<p>画像の全画素が揃った時点で足角度認識へ渡す試験版です。'
           '短縮時間と画質は実機では未確認です。書込み後は、まず測定を開始せず、'
           '足角度の更新とプレビューを確認してください。</p>\n  '
           '<p>未保存ログは先に保存してください。比較用のv20-downloadfix2ページも保持しています。</p>\n  '}

def dry_run(root=base.ROOT):
    return base.dry_run(root, profile=PROFILE)

def prepare(root=base.ROOT, boot_app=None):
    return base.prepare(root, boot_app, profile=PROFILE)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    print(json.dumps(dry_run() if args.dry_run else prepare(), indent=2, ensure_ascii=False))
