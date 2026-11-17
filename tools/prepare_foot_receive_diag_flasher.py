"""Prepare v30 nonblocking camera receive diagnostics firmware."""
import argparse
import json
import prepare_foot_fast_flasher as base
ENVIRONMENT = 'atoms3cam-foot-receive-diag'
IDENTITY = 'walking-foot-receive-diag-v30'
SUFFIX = 'foot-receive-diag'
PROFILE = {'environment': ENVIRONMENT, 'identity': IDENTITY, 'suffix': SUFFIX,
           'title': '歩行器 v30 カメラ受信改善版',
           'name': 'RW Walker v30 camera receive diagnostics',
           'description': '<p>受信エラーは全件をカウンタに残し、受信処理内の連続した文字出力を削減します。webfix1とv29の画像認識を含みます。</p>\n  '
                          '<p>書込み後は左右マーカーと画像を確認してください。バッテリーを切り離してUSBで書き込み、USB中はモータを動かさないでください。</p>\n  '}

def dry_run(root=base.ROOT): return base.dry_run(root, profile=PROFILE)
def prepare(root=base.ROOT, boot_app=None): return base.prepare(root, boot_app, profile=PROFILE)
if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()
    print(json.dumps(dry_run() if a.dry_run else prepare(), ensure_ascii=False, indent=2))
