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
