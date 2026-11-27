# RW Walker Motion Control

リアクションホイールで左右に揺動し、受動脚を交互に振り出す歩行器です。現在の通常歩行版は **`walking-support-transfer-v39-walk7-logfix1`** です。

通信・制御・認識などの担当から探す場合は、[本体136ファイルの内訳](src/README.md#ファイル別の役割)または[全291ファイルの案内](#ファイル別の役割)を開いてください。

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

## ファイル別の役割

一覧は通常歩行版 `walk7-logfix1` の構成に対応します。全291件を、ファイル名から実装や試験に直接移動できるようにまとめました。ビルド生成物と各自の測定データは集計の対象外です。

| 場所 | 件数 | どこで使うか |
| --- | ---: | --- |
| [src/](src/README.md#ファイル別の役割) | 137 | 機体が動くときの自作コード136件と案内1件。歩行制御・足角認識・通信などに分けた内訳があります。 |
| web/ | 4 | 機体が配信し、PCやスマートフォンのブラウザーで動く測定画面。 |
| flasher/ | 7 | PCで開く書き込みページと配布設定。 |
| third_party/ | 6 | カメラ用の外部ドライバー、出典、ライセンス。 |
| [tools/](tools/README.md#ファイル別の役割) | 12 | PCでのログ取得・変換、書き込み準備、ビルド時の処理。 |
| [test/](test/README.md#ファイル別の役割) | 75 | 回帰試験、模擬機器、無駆動の実機通信試験。 |
| [analysis/](analysis/README.md#ファイル別の役割) | 39 | 測定後の研究、シミュレータ、CADによる形状検証。 |
| docs/ | 3 | 支持移行とエネルギー制御の仕様・仮定。 |
| ルート直下 | 8 | 設定、起動コマンド、引き継ぎ、ライセンス等。 |

ルート直下、測定画面、書き込みページ、研究仕様、外部ドライバーの各ファイルは以下を展開して確認できます。

<details>
<summary>ルートの入口・設定・案内（8件）</summary>

| ファイル | 役割 |
| --- | --- |
| [.gitattributes](.gitattributes) | 外部ライブラリの照合に使う原本3ファイルの改行形式を固定する。 |
| [.gitignore](.gitignore) | ビルド生成物やローカルの測定データをGit管理から除く。 |
| [CURRENT_STATUS_20261109.md](CURRENT_STATUS_20261109.md) | 実機の版、直近の検証結果と未検証事項、歩行研究の次の目標を記録する。 |
| [README.md](README.md) | 書き込み・測定の入口と、本体の変更・検証・研究で開く場所を案内する。 |
| [THIRD_PARTY_BMM150_LICENSE.txt](THIRD_PARTY_BMM150_LICENSE.txt) | BMM150に関する外部コードの利用条件を記載する。 |
| [download-rwlog.cmd](download-rwlog.cmd) | Windowsから単独のRWLOG取得プログラムを起動する。 |
| [open-flasher.cmd](open-flasher.cmd) | WindowsでPC側のローカル書き込みページを起動する。 |
| [platformio.ini](platformio.ini) | 対象基板、ライブラリ、機能設定とビルド前処理を指定する。 |

</details>

<details>
<summary>機体が配信する測定画面（4件）</summary>

| ファイル | 役割 |
| --- | --- |
| [web/index.html](web/index.html) | 機体から配信する測定画面の表示項目、ボタン、各スクリプトの組み込み位置を定義する。 |
| [web/pose_comparison.js](web/pose_comparison.js) | 保存した静止姿勢の画像・IMUサンプルをブラウザー側で比較する。 |
| [web/runtime.js](web/runtime.js) | ブラウザーで機体状態を表示し、測定開始・停止やRWLOGの取得・再開・保存を扱う。 |
| [web/wheel_work.js](web/wheel_work.js) | 保存ログからホイールの反作用による仕事をブラウザーなどで計算する。 |

</details>

<details>
<summary>PCで開く書き込みページ（7件）</summary>

| ファイル | 役割 |
| --- | --- |
| [flasher/foot-periodic40/index.html](flasher/foot-periodic40/index.html) | 以前の書き込みページのURLから現行ページへ案内する。 |
| [flasher/index.html](flasher/index.html) | 書き込みサーバーのトップから現行ページへ案内する。 |
| [flasher/manifest.json](flasher/manifest.json) | 旧URLからも現行版の書き込み対象とフラッシュ上の配置を参照できるようにする。 |
| [flasher/support-transfer/flasher-completion.js](flasher/support-transfer/flasher-completion.js) | USB書き込みの進捗を追い、完了時に表示を更新する。 |
| [flasher/support-transfer/index.html](flasher/support-transfer/index.html) | 通常歩行版をUSBで書き込むためのブラウザー画面を構成する。 |
| [flasher/support-transfer/manifest.json](flasher/support-transfer/manifest.json) | 書き込む通常版、4つのイメージのURL、各書き込み先を定義する。 |
| [flasher/support-transfer/soft-boot.js](flasher/support-transfer/soft-boot.js) | 対応する機体へUSB経由で書き込み準備コマンドを送る。 |

</details>

<details>
<summary>研究仕様（3件）</summary>

| ファイル | 役割 |
| --- | --- |
| [docs/ENERGY_ADMISSION_V19.md](docs/ENERGY_ADMISSION_V19.md) | v19で補助入力を許可・保留する条件と、ログで確かめる項目を説明する。 |
| [docs/ENERGY_NEUTRAL_V14.md](docs/ENERGY_NEUTRAL_V14.md) | 足の姿勢と位置エネルギーの谷、機構上の中立点の関係とモデルの仮定を説明する。 |
| [docs/SUPPORT_TRANSFER_V39.md](docs/SUPPORT_TRANSFER_V39.md) | 歩行時の通電と支持移行予測、その限界、ログでの照合方法を説明する。 |

</details>

<details>
<summary>同梱するカメラドライバーと出典（6件）</summary>

| ファイル | 役割 |
| --- | --- |
| [third_party/esp32_camera_f0bb429/LICENSE](third_party/esp32_camera_f0bb429/LICENSE) | 取り込んだESP32カメラコードの利用条件を記載する。 |
| [third_party/esp32_camera_f0bb429/README.md](third_party/esp32_camera_f0bb429/README.md) | 取り込んだカメラドライバーと、フレーム受け渡しに加えた変更を説明する。 |
| [third_party/esp32_camera_f0bb429/UPSTREAM.json](third_party/esp32_camera_f0bb429/UPSTREAM.json) | 取り込んだカメラコードの出典と、変更前の照合用ハッシュを記録する。 |
| [third_party/esp32_camera_f0bb429/cam_hal.c](third_party/esp32_camera_f0bb429/cam_hal.c) | DMAで届いたカメラ画像をフレームとして引き渡す本体のカメラHALを実装する。 |
| [third_party/esp32_camera_f0bb429/cam_hal.h](third_party/esp32_camera_f0bb429/cam_hal.h) | カメラHALの型・状態・関数を宣言する。 |
| [third_party/esp32_camera_f0bb429/ll_cam.h](third_party/esp32_camera_f0bb429/ll_cam.h) | カメラの低層DMA実装と接続するための関数を宣言する。 |

</details>
