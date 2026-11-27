# 歩行測定とビルドに使うツール

| 操作 | 入口 |
|---|---|
| 測定後のログをPCへ保存する | 機体のWi-Fiに接続して [download-rwlog.cmd](../download-rwlog.cmd) を開く。同じ方法で、保存済み部分から再開できる。 |
| RWLOGをCSVとJSONに展開する | [convert_rwlog_to_csv.py](convert_rwlog_to_csv.py) が形式、長さ、CRCを確認する。 |
| 書き込みページを準備する | [open-flasher.cmd](../open-flasher.cmd) を開く。ビルドした4ファイルの準備と照合には [prepare_foot_periodic40_flasher.py](prepare_foot_periodic40_flasher.py) を使う。 |

```powershell
python -B tools/convert_rwlog_to_csv.py "$env:USERPROFILE/Downloads/energy_control_autonomous_run_1_44453706.rwlog" --out analysis/local/44453706
```

[platformio.ini](../platformio.ini) が呼ぶビルド用スクリプトは、測定画面の埋め込み、BMI270と姿勢推定ライブラリの調整、カメラHALの互換性の確認を行う。`fixtures/` は依存ライブラリの照合に使う。

計測値や歩行制御の評価は [研究用プログラム](../analysis/README.md)、モータを使わない通信試験とビルド後のELF・リンクマップの検査は [検証用プログラム](../test/verification/)、回帰試験の実行手順は [テスト手順](../test/README.md) にまとめている。

## ファイル別の役割

手動で使うプログラムと、ビルド時にライブラリを照合するためのファイルを含みます。

<details>
<summary>ログ操作とビルド用ツール（8件）</summary>

| ファイル | 役割 |
| --- | --- |
| [README.md](README.md) | 日常の取得・変換・書き込み補助とビルド時のツールを案内する。 |
| [check_camera_hal_abi.py](check_camera_hal_abi.py) | ビルド前に外部カメラHALとSDKの組み合わせに互換性があるか照合する。 |
| [convert_rwlog_to_csv.py](convert_rwlog_to_csv.py) | RWLOGの形式・長さ・CRCを検査して、各計測のCSVとメタデータを展開する。 |
| [download_rwlog.py](download_rwlog.py) | PCから機体のRWLOGを分割取得し、中断時の続きから保存する。 |
| [embed_runtime_web.py](embed_runtime_web.py) | 測定画面のHTMLとJavaScriptを、ビルドで組み込むヘッダーへまとめる。 |
| [patch_bmi270_timing.py](patch_bmi270_timing.py) | BMI270ライブラリの原本を照合し、IMU読出しタイミング用の変更をビルド時に適用する。 |
| [patch_madgwick_hot.py](patch_madgwick_hot.py) | ライブラリ原本を照合し、Madgwick計算の対象関数に実行場所・最適化の指定を加える。 |
| [prepare_foot_periodic40_flasher.py](prepare_foot_periodic40_flasher.py) | 通常版の書き込みパッケージを構成・照合し、ローカルサーバーで配信する。 |

</details>

<details>
<summary>依存ライブラリの照合用原本（4件）</summary>

| ファイル | 役割 |
| --- | --- |
| [fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_FusionInterface.h](fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_FusionInterface.h) | 姿勢推定ライブラリの共通インターフェースの原本をビルド時の照合に使う。 |
| [fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_Madgwick.cpp](fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_Madgwick.cpp) | Madgwick姿勢推定の上流実装を、使用するライブラリの照合に使う。 |
| [fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_Madgwick.h](fixtures/adafruit_ahrs_2_4_0/Adafruit_AHRS_Madgwick.h) | Madgwick姿勢推定の上流ヘッダーを、照合とビルド時の修正に使う。 |
| [fixtures/adafruit_ahrs_2_4_0/provenance.json](fixtures/adafruit_ahrs_2_4_0/provenance.json) | 姿勢推定ライブラリ原本の版・出典と照合用ハッシュを記録する。 |

</details>
