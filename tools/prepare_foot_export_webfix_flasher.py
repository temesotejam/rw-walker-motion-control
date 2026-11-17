"""Prepare the isolated v26 webfix1 installer; never flash hardware."""
import argparse
import json
import prepare_foot_fast_flasher as base

ENVIRONMENT = 'atoms3cam-foot-export-webfix'
IDENTITY = 'walking-foot-native-stream-v26-webfix1'
SUFFIX = 'foot-export-webfix'
PROFILE = {'environment': ENVIRONMENT, 'identity': IDENTITY, 'suffix': 'foot-export-webfix',
           'title': '歩行器 v26 ログ取得改善版 webfix1',
           'name': 'RW Walker v26 export webfix1',
           'description': '<p>ログ生成時のメモリ負荷と失敗検出、カメラ停止処理を改善した版です。足認識の低遅延化と現在のエネルギー制御を引き継ぎます。</p>\n  '
                          '<p>バッテリー給電を切り離してからUSBを接続して書き込んでください。USB接続中は測定を開始しないでください。</p>\n  '
                          '<p>書き込み後は状態表示と足角度を確認してください。ログ取得の実機での再発防止は未検証です。</p>\n  '}

def dry_run(root=base.ROOT): return base.dry_run(root, profile=PROFILE)
def prepare(root=base.ROOT, boot_app=None): return base.prepare(root, boot_app, profile=PROFILE)

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()
    print(json.dumps(dry_run() if args.dry_run else prepare(), indent=2, ensure_ascii=False))
