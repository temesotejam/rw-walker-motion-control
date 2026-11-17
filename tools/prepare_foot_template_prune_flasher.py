"""Prepare isolated v28 exact template-pruning candidate for user flashing."""
import argparse
import json
import prepare_foot_fast_flasher as base

ENVIRONMENT = 'atoms3cam-foot-template-prune'
IDENTITY = 'walking-foot-template-prune-v28'
SUFFIX = 'foot-template-prune'
PROFILE = {
    'environment': ENVIRONMENT, 'identity': IDENTITY, 'suffix': SUFFIX,
    'title': '歩行器 v28 足認識の不要探索削減版',
    'name': 'RW Walker v28 exact template pruning',
    'description': '<p>最大コントラストが検出基準に届かない候補行だけ、後段の探索を省きます。探索範囲と検出基準は同じです。v27のコピー削減とログ取得修正webfix1を含みます。</p>\n  '
                   '<p>速度改善は実機未測定です。書込み後、まず手で足を動かして左右の足角度が追従することを確認してください。</p>\n  '
                   '<p>バッテリー給電を切り離してからUSBで書き込み、USB接続中はモータを動かさないでください。</p>\n  ',
}

def dry_run(root=base.ROOT):
    return base.dry_run(root, profile=PROFILE)

def prepare(root=base.ROOT, boot_app=None):
    return base.prepare(root, boot_app, profile=PROFILE)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    print(json.dumps(dry_run() if args.dry_run else prepare(), indent=2, ensure_ascii=False))
