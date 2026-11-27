# ホスト側での動作確認

リポジトリのルートで次のコマンドを実行します。Python 3.9 以降、C++17 対応の `g++`、Node.js 18 以降を使います。すべての処理は PC 内で完結します。

```text
python -B test/run.py
```

## ファイル別の役割

PC上の回帰試験は [run.py](run.py) から実行します。無駆動の実機通信試験は [hardware/](hardware/) に、本体の設定で有効になる制御試験・補正は [src/control_trials/](../src/control_trials/) にあります。

<details>
<summary>PC上の回帰試験と入口（40件）</summary>

| ファイル | 役割 |
| --- | --- |
| [README.md](README.md) | PC上の回帰試験とモータを使わない実機通信試験の手順を案内する。 |
| [_project.py](_project.py) | 回帰試験から解析・ログ変換用の同じPythonモジュールを読み込めるようにする。 |
| [run.py](run.py) | 本体、Web、ログ変換、利用可能なシミュレータと実測再生の回帰試験を一括実行する。 |
| [test_download_rwlog.py](test_download_rwlog.py) | PC側のログ取得で、中断再開・測定ID・全バイトとCRCの照合を試す。 |
| [test_energy_admission_metadata_v19.py](test_energy_admission_metadata_v19.py) | 補助入力の許可・保留と前後傾斜保護が実際のJSON・CSVに記録されるか試す。 |
| [test_energy_admission_runner_v19.py](test_energy_admission_runner_v19.py) | v19の実コードで補助判断、中心通過時の通電、電流指令と停止を試す。 |
| [test_event_foot_energy_candidate.py](test_event_foot_energy_candidate.py) | オフラインのエネルギー候補評価で、仕事の配分と履歴の扱いを検査する。 |
| [test_event_foot_features.py](test_event_foot_features.py) | 入力判断時に実際に届いていた足角だけで足の位相を復元することを検査する。 |
| [test_event_foot_outcomes.py](test_event_foot_outcomes.py) | 同じ半周期の実際の中心通過・反対側ピークを正しく入力判断に結び付けるか試す。 |
| [test_export_transport_recovery.cpp](test_export_transport_recovery.cpp) | 確定済みログの転送中に起きるWi-Fi不調の回復時刻や進捗の扱いを検査する。 |
| [test_flasher_completion.js](test_flasher_completion.js) | 書き込み画面のポート選択・確認と完了表示のタイミングを試す。 |
| [test_foot_cadence.py](test_foot_cadence.py) | 足角の有効更新間隔と5秒区間ごとの集計を検査する。 |
| [test_foot_native_geometry.py](test_foot_native_geometry.py) | カメラの画素配置やプレビューの境界を検査し、マーカー追跡結果を比較する。 |
| [test_foot_periodic40_flasher.py](test_foot_periodic40_flasher.py) | 通常版の配布URL・イメージ・版の一致と旧版や失敗時の扱いを試す。 |
| [test_foot_pipeline.py](test_foot_pipeline.py) | 通常版と従来版のカメラ追跡、画像の行バッファ引き渡しを試す。 |
| [test_fore_aft_motion_guard.cpp](test_fore_aft_motion_guard.cpp) | 前後傾斜の監視が姿勢の変化に応じて通電を抑え、停止を判定するか試す。 |
| [test_gait_turn_observer.cpp](test_gait_turn_observer.cpp) | 左右の半周期の組み合わせ、ジャイロによる向き、観測の鮮度とリセットを試す。 |
| [test_gait_turn_run.py](test_gait_turn_run.py) | 旋回観測を制御側から記録・CSV化する経路と旧ログ互換を試す。 |
| [test_half_cycle_energy_budget.py](test_half_cycle_energy_budget.py) | 半周期内の補助・主入力に配分した時間と、仕事から求める通電幅を検査する。 |
| [test_http_client_queue.py](test_http_client_queue.py) | 待機中のHTTP接続があるときのサーバーの受付と処理順を試す。 |
| [test_live_status_web.js](test_live_status_web.js) | 状態の定期読出し、足角などの表示更新、接続失敗・再起動時の扱いを試す。 |
| [test_mekf_host.cpp](test_mekf_host.cpp) | IMUから機体座標への変換とMEKF姿勢推定を既知の静止・回転入力で検査する。 |
| [test_observed_transfers.py](test_observed_transfers.py) | 支持移行予測の採点と、動画のLED同期・接地や離地の区間前後の実測・左右の符号・欠測を検査する。 |
| [test_offline_run_export.cpp](test_offline_run_export.cpp) | 測定終了、途中停止、ログ確定、再接続・次の測定への遷移を試す。 |
| [test_periodic_foot.cpp](test_periodic_foot.cpp) | 定周期足角推定器に観測を与え、時刻・失効・推定出力を検査する。 |
| [test_return_history.cpp](test_return_history.cpp) | 左向きの戻り判断に過去の成功履歴を採用できる条件を検査する。 |
| [test_return_recheck.cpp](test_return_recheck.cpp) | 補助終了後の実測例を使い、戻りの減速と再評価の条件を検査する。 |
| [test_right_arrival_energy.cpp](test_right_arrival_energy.cpp) | 右向きの到達に要するエネルギーと通電幅候補の判定を試す。 |
| [test_runtime_download.js](test_runtime_download.js) | ブラウザーのログ取得を中断・再開し、内容・CRC・足角CSVを照合する。 |
| [test_runtime_export_retry.js](test_runtime_export_retry.js) | ブラウザーの取得処理に通信中断や壊れたパケットを与え、再試行を試す。 |
| [test_runtime_page_encoding.cpp](test_runtime_page_encoding.cpp) | HTTP要求に応じてページをgzip・非圧縮で返す選択を検査する。 |
| [test_runtime_web.js](test_runtime_web.js) | 測定ページの歩行開始条件、足角の表示、プレビュー取得とエラー表示を検査する。 |
| [test_rwlog_v46_converter.py](test_rwlog_v46_converter.py) | 模擬v46ログでCRC検証とMEKF診断のCSV展開を試す。 |
| [test_settled_walking_restart.py](test_settled_walking_restart.py) | 自立停止後の再始動について、実際の指令方向・開始・停止処理を試す。 |
| [test_small_arc_return.cpp](test_small_arc_return.cpp) | 小さい足裏円弧へ移る局面での補助入力判断を過去の観測条件で試す。 |
| [test_support_transfer_audit.py](test_support_transfer_audit.py) | 本体の支持移行予測タスク、記録の引き渡し・上限・CSV出力を試す。 |
| [test_support_transfer_plan.cpp](test_support_transfer_plan.cpp) | 支持移行の接近・到達・反転、仕事と通電時間の制約を計画器で試す。 |
| [test_support_transfer_replay.py](test_support_transfer_replay.py) | 支持移行を再生する際の観測時刻、極性、到達の採点を検査する。 |
| [test_walking_trial_request.py](test_walking_trial_request.py) | Webからの歩行要求について既定モードと開始許可条件を検査する。 |
| [test_wifi_transport_restart.cpp](test_wifi_transport_restart.cpp) | Wi-FiとHTTPを再起動する条件や再試行間隔を検査する。 |

</details>

<details>
<summary>模擬機器・入力データ（10件）</summary>

| ファイル | 役割 |
| --- | --- |
| [fixtures/arduino/Arduino.h](fixtures/arduino/Arduino.h) | PC上で本体のC++を試すためにArduinoの型や機能を模擬する。 |
| [fixtures/arduino/esp_heap_caps.h](fixtures/arduino/esp_heap_caps.h) | ESP32のメモリ確保APIをPC上で模擬する。 |
| [fixtures/camera_pipeline.py](fixtures/camera_pipeline.py) | カメラと時刻を模擬し、本体の画像取得・マーカー追跡をPCで実行する。 |
| [fixtures/energy/metadata.py](fixtures/energy/metadata.py) | 実際の記録処理をテスト用C++へ組み込み、エネルギー関連のメタデータ生成を試す。 |
| [fixtures/energy/metadata_fixture.cpp](fixtures/energy/metadata_fixture.cpp) | 記録試験用の模擬データと本体のログ処理を差し込む場所を定義する。 |
| [fixtures/energy/runner.py](fixtures/energy/runner.py) | 本体の入力制御処理と機器の模擬実装を組み合わせて試験用コードを生成する。 |
| [fixtures/energy/runner_fixture.cpp](fixtures/energy/runner_fixture.cpp) | センサー・駆動系・時刻を模擬して歩行制御をPC上で試せるようにする。 |
| [fixtures/gait_turn_run.cpp](fixtures/gait_turn_run.cpp) | 模擬の歩行サイクルを与え、左右差や旋回観測の記録を試す。 |
| [fixtures/runtime_rwlog.py](fixtures/runtime_rwlog.py) | ブラウザーの再開・CRC・CSV試験に使う再現可能な模擬RWLOGを作る。 |
| [fixtures/support_transfer_audit.cpp](fixtures/support_transfer_audit.cpp) | 模擬タスクと時刻を用意し、支持移行の比較記録をPCで試す。 |

</details>

<details>
<summary>シミュレータの回帰試験（11件）</summary>

| ファイル | 役割 |
| --- | --- |
| [simulator/__init__.py](simulator/__init__.py) | シミュレータの研究モジュールをPythonの試験探索から読み込めるようにする。 |
| [simulator/test_action_model.py](simulator/test_action_model.py) | シミュレータの電流立ち上がり・残流、入力時刻と折り返しの扱いを検査する。 |
| [simulator/test_add_early_watch.py](simulator/test_add_early_watch.py) | 監視位相を増やした評価で、見逃しと判断保留を母数に残すことを確認する。 |
| [simulator/test_aid_current_delivery.py](simulator/test_aid_current_delivery.py) | 補助電流の符号付き積分、欠測と電流指令境界の扱いを検査する。 |
| [simulator/test_foot_kinematics.py](simulator/test_foot_kinematics.py) | 足の角速度計算で重複発行、欠測、失効、未来の値を正しく扱うか検査する。 |
| [simulator/test_loss_observer.py](simulator/test_loss_observer.py) | 損失補正が判断前の観測を使い、減速と不足の継続を反映するか検査する。 |
| [simulator/test_post_zero_carry.py](simulator/test_post_zero_carry.py) | 中心通過後の評価で、同じ試行の結果や別入力の結果を予測に混ぜないことを確認する。 |
| [simulator/test_receiving_carry.py](simulator/test_receiving_carry.py) | 受け側での到達と折り返しを、その時点の観測・入力から区別できるか検査する。 |
| [simulator/test_return_history_measurement.py](simulator/test_return_history_measurement.py) | 戻り履歴を用いた入力変更、実際の通電幅、その後の主入力の記録を検査する。 |
| [simulator/test_return_model.py](simulator/test_return_model.py) | 惰性で戻るモデルの足角受領時刻、イベント境界、試行の独立性を検査する。 |
| [simulator/test_right_candidates.py](simulator/test_right_candidates.py) | 右向き入力候補が別試行の比較例とホイール制限に基づくことを検査する。 |

</details>

<details>
<summary>無駆動の通信試験（5件）</summary>

| ファイル | 役割 |
| --- | --- |
| [hardware/archive_layout.py](hardware/archive_layout.py) | 検査済みのRWLOGを、通信試験版に埋め込める独立した圧縮ブロックにする。 |
| [hardware/motorfree_archive.cpp](hardware/motorfree_archive.cpp) | 試験版で保存済み測定を展開し、実測時刻に合わせた記録・転送を再現する。 |
| [hardware/motorfree_main.cpp](hardware/motorfree_main.cpp) | モータドライバーを含まないカメラ・ログ転送試験版を起動する。 |
| [hardware/prepare_motorfree.py](hardware/prepare_motorfree.py) | 通常ソースから無駆動試験版の独立したソースとビルド設定を生成する。 |
| [hardware/replay_transfer.cjs](hardware/replay_transfer.cjs) | 実機の無駆動試験版に対し、ブラウザーで記録・中断再開・保存・全バイト照合を繰り返す。 |

</details>

<details>
<summary>機体通信・ビルド結果の検査（9件）</summary>

| ファイル | 役割 |
| --- | --- |
| [verification/probe_foot_idle.py](verification/probe_foot_idle.py) | 停止中の機体をHTTPで照会して、カメラ更新間隔・観測遅延を標本化する。 |
| [verification/probe_http_stability.py](verification/probe_http_stability.py) | 停止中の機体のWeb、状態、画像の分割読出しを繰り返し、CRCと応答を確認する。 |
| [verification/verify_camera_task_link.py](verification/verify_camera_task_link.py) | カメラのRTOSタスク生成が指定ラッパーを経由することをリンク結果で確かめる。 |
| [verification/verify_core_isolation_link.py](verification/verify_core_isolation_link.py) | タイマー割り込みから通知までの経路の配置をリンク結果で確かめる。 |
| [verification/verify_frame_ready_link.py](verification/verify_frame_ready_link.py) | カメラHALだけが差し替わり、指定の低層DMA実装が残ることをリンクマップで確かめる。 |
| [verification/verify_imu_transport_link.py](verification/verify_imu_transport_link.py) | IMUの割り込み転送・完了処理が組み込まれていることをリンク結果で確かめる。 |
| [verification/verify_log_encoder_link.py](verification/verify_log_encoder_link.py) | RWLOGサンプルの符号化関数と共用の量子化関数をELF上で特定し、その数と機械語の合計サイズを検査する。 |
| [verification/verify_motion_acquisition_link.py](verification/verify_motion_acquisition_link.py) | 運動制御・IMU取得の指定関数の配置と合計サイズを旧版の予算と照合する。 |
| [verification/verify_realtime_code_link.py](verification/verify_realtime_code_link.py) | 姿勢推定やログ処理など時間制約のある関数の配置とサイズを検査する。 |

</details>

Node.js が PATH にない場合は `--node "C:\path\to\node.exe"` で指定できます。実測ログが手元にあれば、次のように追加できます。

```text
python -B test/run.py --rwlog "C:\Users\arika\Downloads\energy_control_autonomous_run_1_46492793.rwlog" --rwlog "C:\Users\arika\Downloads\energy_control_autonomous_run_2_201405726.rwlog"
```

この2試行を指定すると、実測ログの CRC を検査して CSV とメタデータに変換し、イベント時点までに制御側へ届いていた足角度と、40 Hz の足角度出力を再生します。変換先は一時ディレクトリです。引数を省略した場合も、ユーザーの `Downloads` に上記の2ファイルがそろっていれば自動で使います。見つからなければ実測ログ照合をスキップした旨を表示します。

| 対象 | 確認すること |
| --- | --- |
| エネルギー制御 | v19 の入力を許可・保留する判定、半周期のエネルギー収支と記録 |
| カメラと姿勢推定 | v38 の画像取得周期、マーカーの判定、40 Hz の角度出力、MEKF、前後傾斜のガード |
| イベント再生 | 入力候補の評価に、そのイベントより後に得た情報が混じらないこと |
| v39支持移行予測 | CAD上の移行点、移行後のエネルギー余裕、電流の立ち上がり、観測用タスクとログ出力。映像照合では計算結果の到着時刻と実際の支持足・方向を検査する。 |
| 測定と書き込み画面 | Web の表示と開始条件、RWLOG の再取得と CRC、旧形式の変換、書き込み完了の表示と配布物の整合 |

`fixtures/camera_pipeline.py` と `fixtures/arduino/` は、現行ソースを PC 上の模擬カメラ・Arduino 環境で動かすために使います。`fixtures/energy/` の `runner_fixture.cpp` はセンサーと駆動系を、`metadata_fixture.cpp` は測定レコードを模擬します。隣の Python モジュールは、各テンプレート内の目印に本体ソースから読み取った制御・記録処理を差し込みます。

NumPy をインストールした環境ではシミュレーターの回帰試験も実行します。個別に実行するコマンドは `python -B -m unittest discover -s test/simulator -t test` です。

Web のログ取得テストでは `fixtures/runtime_rwlog.py` が変換ツールと同じ v56 ヘッダー・サンプル定義から、768件の架空の足観測と模擬ホイール応答を含む入力を毎回生成します。中断位置は 8192 バイトです。再開後は全バイト・CRC・CSV の行数を照合します。

## 停止中の実機で通信が続くか確認する

専用Wi-Fiで機体に接続し、`--source` にそのアダプターのIPv4アドレスを指定します。

```powershell
python -B test/verification/probe_http_stability.py --source 192.168.4.2 --seconds 180 --min-bytes 2200000 --heap
```

約2秒ごとに状態とモータ指令を読み、同じ起動番号であることを確認しながら、測定ページと停止中のカメラ画像を取得します。画像はRWLOGと同じ4 KiB単位の送信処理を通り、各ブロックと画像全体のCRCが一致した分だけを転送量に数えます。`--heap` は5秒ごとに内部RAMの残量を記録します。結果には最初と最後の状態取得時刻、成功回数、失敗した処理と時刻が含まれます。歩行終了後のログ確定から実ファイルの保存までは、別途歩行試行で確認します。

`--frozen` を付けると、最初に取得した1枚を同じトークンで繰り返し読みます。転送中に失敗した場合は、その位置から読み直します。各失敗は結果に残るため、初回ですべて通った試行と、再試行して完了した試行を区別できます。

測定終了からRWLOGの保存、カメラ再開まで繰り返すには、モータドライバーを含まない通信試験版を使います。次の例は実測RWLOGを元の時間間隔で再生し、通常3回、別画面を開いたまま1回、通信中断1回、ページ再読み込み1回を比較します。各回の保存内容を全バイト照合します。

```powershell
python -B test/hardware/prepare_motorfree.py --measure --archive .\recording.rwlog --target-free 19588 --target-largest 11252
pio run -c .pio/motorfree-measure/platformio.ini
```

USB給電でこの試験版を書き込み、PCを機体のWi-Fiに接続してから実行します。保存先には新しいディレクトリ名を指定します。PCにEdgeとPlaywrightが必要です。歩行時は通常版に書き戻します。

```powershell
node test/hardware/replay_transfer.cjs .\recording.rwlog .\transfer-results 3 --observer --interrupt --reload
```
