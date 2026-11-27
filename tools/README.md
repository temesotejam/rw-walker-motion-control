# PC側のツール

| やりたいこと | 入口 |
|---|---|
| 測定後にPCへ直接RWLOGを保存 | 機体のWi-Fiに接続して [download-rwlog.cmd](../download-rwlog.cmd) を開く。同じツールでもう一度開くと、PCのダウンロードフォルダーに残した、このツール自身の検証済み部分から再開する。 |
| 測定したRWLOGをCSV・JSONへ展開 | [convert_rwlog_to_csv.py](convert_rwlog_to_csv.py)。元ログの形式、長さ、CRCを確認する。 |
| 荷重移行に向けたエネルギー配分を比較 | [replay_event_foot_energy.py](replay_event_foot_energy.py)。`event_foot_features.py` が当時使えた観測を取り出し、`event_foot_energy_candidate.py` が仕事を計算し、`event_foot_outcomes.py` が後から実測結果を対応付ける。 |
| v39の開始時点・幅を評価 | [replay_support_transfer.py](replay_support_transfer.py) はv37の実測軌道上でファームと共通の予測器を再生する。[score_observed_transfers.py](score_observed_transfers.py) は、新たに測ったv39ログを独立した支持移行の観測と照合する。[操作手順](../docs/SUPPORT_TRANSFER_V39.md) |
| v38の足角定周期推定を記録済み試行で確認 | [replay_periodic_foot.py](replay_periodic_foot.py) と [実機と同じ推定器を呼ぶホスト側のコード](../test/test_periodic_foot.cpp)。カメラの観測そのものは [audit_walking_foot_measurements.py](audit_walking_foot_measurements.py) で照合する。 |
| 足角によるエネルギーの谷の移動を調べる | [analyze_energy_neutral_v14.py](analyze_energy_neutral_v14.py)。提供されたCADを入力にする。`generate_cad_support.py` はCADから頂点ヘッダーを再生成するときに使う。 |
| Web測定画面をファームへ埋め込む | [embed_runtime_web.py](embed_runtime_web.py)。ビルド時に `web/` から各ビルドの `generated/freefoot_runtime_web.h` を自動生成。 |
| ビルド成果物から書き込みページを用意・照合 | [prepare_foot_periodic40_flasher.py](prepare_foot_periodic40_flasher.py)。`--check` で配布中の4ファイルを検査する。 |

`platformio.ini` の3つのビルドフックは、BMI270、姿勢推定ライブラリ、ESP32カメラHALのバージョンと互換性を扱う。`fixtures/adafruit_ahrs_2_4_0/` はその照合用のソース。`verify_*_link.py` はコンパイル後のELF・リンクマップをチェックするためのスクリプト。制御・カメラ・ブラウザ・追試用データの回帰テストは [test/](../test/) にまとめている。
