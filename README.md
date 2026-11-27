# RW Walker Motion Control

リアクションホイールで左右に揺動し、受動脚を交互に振り出す歩行器です。現在の通常歩行版は **`walking-support-transfer-v39-walk7-logfix1`** です。

## 歩行実験で使うもの

1. **書き込み**　バッテリーを外しUSB給電にして、[書き込みページ](http://127.0.0.1:8767/support-transfer/?v=walking-support-transfer-v39-walk7-logfix1)を開きます。このPCでページが起動していないときは [open-flasher.cmd](open-flasher.cmd) を実行します。
2. **測定とログ保存**　USBを外してバッテリー給電に切り替え、機体のWi-Fiに接続します。[測定画面](http://192.168.4.1/)で「歩行を開始（±8°・30秒）」を押し、終了後は同じ画面の「RWLOGを取得・再開」で保存します。

保存したログをCSVに展開するときは `python tools/convert_rwlog_to_csv.py run.rwlog --out analysis/local/my-run` を実行します。直近で実機確認した版・結果・未確認点は[研究の引き継ぎ](CURRENT_STATUS_20261109.md)にあります。

## 本体の動作を変えるとき

| 変えたい動作 | 最初に読むファイル |
| --- | --- |
| 足と胴体の状態からホイールへの入力を決める | [歩行時の入力処理](src/walking_energy_runner.inc) |
| カメラ画像から左右の足角を求める | [足角観測](src/foot_observer.cpp) |
| ブラウザーの表示・RWLOGの取得を変える | [測定画面の処理](web/runtime.js) |

実機の入口は [src/main.cpp](src/main.cpp)、通常ビルドの設定は [platformio.ini](platformio.ini) の1環境です。ローカルの依存パッケージと `.pio/` の生成物はPlatformIOが管理します。新しいチェックアウトから書き込みページを準備する手順は次のとおりです。

```powershell
pio run
python tools/prepare_foot_periodic40_flasher.py
```

姿勢推定、定周期の足角発行、支持移行の予測記録、モータに電流指令を渡す経路は[本体ソースの案内](src/README.md)にまとめています。Web画面のHTML・JavaScriptは [web/](web/) からビルド時に取り込みます。

## 検証と研究

| 場所 | 使う場面 |
| --- | --- |
| [test/](test/README.md) | 変更後の回帰試験。PCから `python -B test/run.py` を実行します。無駆動の実機通信試験には専用ファームを使います。 |
| [analysis/](analysis/README.md) | 実測ログの解析、支持移行・失速の再生、過去の試行との比較。 |
| [docs/](docs/SUPPORT_TRANSFER_V39.md) | 歩行制御モデル、入力の判断根拠、実測との照合方法。 |

## 旧版・実験データの保管先

整理直前の作業フォルダー全体を、このPCに同じ階層で保存しました。実測データ、生成済みの通常版・試験版、Git履歴も含みます。過去の解析を当時のプログラムと照合するときは、このコピーかGitHubの固定版を開いてください。[PCの全体コピー](C:/Users/arika/Desktop/walker-backups/rw-walker-motion-control_20261127_before_project_cleanup_481a221/project) · [GitHubの整理直前の版](https://github.com/temesotejam/rw-walker-motion-control/tree/481a221e22352d8744a6accf8d1ce4670b6e9bcc)
