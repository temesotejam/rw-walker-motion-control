# RW Walker Motion Control

リアクションホイールで左右に揺動し、受動脚を交互に振り出す歩行器のファームウェアと実験用ツールです。歩行版 **v39-walk6** は、左への戻りを補助したあとも、足角と胴体の減速から中心に届くかを判定します。30秒の測定中に片側に傾いたまま静止したときは、過去に中心を通過した速度を参考に、中心へ揺り戻す入力を選びます。胴体姿勢はMEKFで推定し、足角は25 ms周期で配信します。測定後のRWLOGは圧縮して転送し、途中で切れた場合は検証済みの位置から取得を再開します。[制御と実測の比較](docs/SUPPORT_TRANSFER_V39.md)

## ビルドと書き込み

このPCでは [open-flasher.cmd](open-flasher.cmd) で書き込みページを開き、「歩行プログラムを書き込む」を押します。歩行実験は機体のWi-Fiに接続し、測定画面の「歩行を開始（±8°・30秒）」から始めます。[書き込みページ](http://127.0.0.1:8767/support-transfer/) · [機体の測定画面](http://192.168.4.1/)

新しくクローンした場合や、変更後のファームウェアを書き込む場合は、先にルートで `pio run`、次に `python tools/prepare_foot_periodic40_flasher.py` を実行して書き込みページを準備します。[PlatformIOのビルド設定](platformio.ini)

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
