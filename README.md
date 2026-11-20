# RW Walker Motion Control

リアクションホイールで左右に揺動し、受動脚を交互に振り出す歩行器のファームウェアと実験用ツールです。実機確認済みの基準版は **v38**。次の試験用の **v39**（`walking-support-transfer-v39`）は、同じv19エネルギー制御で歩行しながら、次の支持側へ移る前の通電時点・幅を予測し、RWLOGに記録します。胴体姿勢はMEKF、左右足角はカメラ観測から25 ms周期で推定します。[予測方法と検証手順](docs/SUPPORT_TRANSFER_V39.md)

## ビルドと書き込み

1. PlatformIOでこのフォルダーを開き、**Build** を選ぶか、ルートで `pio run` を実行します。[platformio.ini](platformio.ini) のビルド環境は `atoms3cam-foot-periodic40` の1つです。
2. このPCでは [open-flasher.cmd](open-flasher.cmd) を開くと、v39の書き込みページ `http://127.0.0.1:8767/support-transfer/` を表示します。実機確認済みのv38へ戻すリンクもページ内にあります。機体のWi-Fiに接続した後の測定画面は `http://192.168.4.1/` です。
3. **自分でビルドした版を配布する場合**は、ビルド成功後に `python tools/prepare_foot_periodic40_flasher.py` を実行して、書き込みページに4個のバイナリを用意します。新しくクローンしたPCでもこの手順でページを準備できます。

## プログラムの入口

| 用途 | 読み始める場所 |
|---|---|
| センサー、タスク構成、制御の流れ | [ファームウェア構成](src/README.md) |
| 必要仕事の算定、電流パルスの採否と開始 | [歩行時の入力処理](src/walking_energy_runner.inc)、[v19の判断条件](docs/ENERGY_ADMISSION_V19.md) |
| カメラの足角と40 Hzの定周期推定 | [足角観測](src/foot_observer.cpp)、[定周期の推定器](src/periodic_foot_state.h) |
| 測定画面とRWLOG保存 | [Web画面](web/index.html)、[画面の処理](web/runtime.js)、[RWLOG変換](tools/convert_rwlog_to_csv.py) |
| 次の荷重移行を見越した制御の研究 | [v39のイベント予測](docs/SUPPORT_TRANSFER_V39.md)、[研究の現在地](CURRENT_STATUS_20261109.md)、[足姿勢とエネルギー中立点](docs/ENERGY_NEUTRAL_V14.md) |

測定画面のソースを編集した場合は、`python tools/embed_runtime_web.py` でファームウェア用ヘッダーを更新し、`python tools/embed_runtime_web.py --check` で一致を確認します。

制御、足角の推定、Web画面、ログ変換は、PC上で `python -B test/run.py` を実行して一括確認できます。必要なソフトと、実測ログを加えて再生する手順は[検証手順](test/README.md)にあります。

## 測定ログの扱い

```powershell
python tools/convert_rwlog_to_csv.py run.rwlog --out analysis/my-run
python tools/replay_event_foot_energy.py run1.rwlog run2.rwlog --out analysis/my-replay
```

前者は元ログのCRCを確認して時系列と診断値を展開します。後者は、v19の戻り補助を判断した時点で入手済みの情報を使い、開始時刻を固定して幅を比較します。開始時刻も選ぶv39の再生には `python tools/replay_support_transfer.py run1.rwlog run2.rwlog --out analysis/my-transfer` を使います。

## 旧版・実験データの保管先

2026-11-20の整理前のGit管理ファイルは[コミット ce7d018](https://github.com/temesotejam/rw-walker-motion-control/tree/ce7d01871b3b31cd5e061fc813d8166bb65d4e13)にあります。Git管理外の実測データ、書き込みイメージ、ビルド成果物、Git履歴を含む全体のコピーは、このPCの `C:\Users\arika\Desktop\walker-backups\rw-walker-motion-control_20261120_before_cleanup\project.zip` です。同じ場所の `retired` では、作業フォルダーから移した旧ファイルを元の相対パスで参照できます。全体を復元する場合はZIPを別の空フォルダーに展開してください。
