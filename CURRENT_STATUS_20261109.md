# 歩行制御の引き継ぎ

各チャットでこのファイルを更新し、ローカルの変更を保護して GitHub の `origin/main` と通常同期する。共通の作業履歴はワークスペースの `WORKSPACE_CHANGE_LOG.md` に追記する。

ユーザーは、書き込みを依頼するときは毎回バッテリーを外し、USBだけでPCに接続していると明示した。以後の書き込み依頼ではこれを前提とし、USBではモータを動かさない。

## 使用中の機体と測定

通常歩行版は `walking-support-transfer-v39-walk7-logfix1`、歩行設定は `left_transfer_carry_v7`。左右揺動・足の出方・ホイールの残留電流から必要な仕事と通電を決める。開始は測定画面の「歩行を開始（±8°・30秒）」。腰の揺動が続く時間と、左右の一歩が前進につながった時間は区別して記録する。[歩行制御の仕様](docs/SUPPORT_TRANSFER_V39.md)

11月27日にUSB給電で通常版を書き込み、起動、待機、0標本、モータ指令0 mA、左右足角の更新を確認した。書き込み済みのアプリは2,115,104バイト、SHA-256 `34f8e27b13ef6af7af00a1147a181fc2d75fbb203d282fa6c2e1b3faccca4d9f`。通常版の床上歩行直後の転送と、その版での連続歩行は未確認。[機体の測定画面](http://192.168.4.1/) · [書き込みページ](http://127.0.0.1:8767/support-transfer/?v=walking-support-transfer-v39-walk7-logfix1)

通信は、モータドライバーを含まない実機試験版で1回2,893,231バイトの模擬ログを連続6回保存し、全バイト・CRCを照合済み。通常22.0～24.6秒、別画面を開いて25.5秒、要求を中断して29.1秒、ページを再読み込みして26.2秒。6回とも次の測定に戻り、カメラはPSRAMで有効画像を毎秒48.7～51.3枚取得した（短時間のフレーム数差）。正常に完了した旧版2回は17.7秒と19.7秒。測定終了時はカメラの12,800バイトを通信に返し、再確保できるまでPSRAMで認識を続ける。[検証レポート](C:/Users/arika/Desktop/walker-backups/rw-walker-motion-control_20261127_before_transfer_cleanup/verification/final-six-runs.json)

### 実機に書き込んだ walk7-logfix1 の内訳

11月27日のCOM7での書き戻しは `0x10000` に通常歩行アプリ `firmware.bin` を1本書き込み、ハッシュを検証した。書き込み直後に機体から同じビルド名と `READY_TO_MEASURE` を取得している。保存済みアプリのSHA-256は上記の `34f8e27b…`。アプリ内部のELF識別値も、解析したELF全体のSHA-256 `fa6ae9b40621ea28a4fb48f35c207ae3b41d0d1adde78e7d7387f80c3e516ddc` と一致した。[実機の書き込み記録](C:/Users/arika/Desktop/walker-backups/rw-walker-motion-control_20261127_before_transfer_cleanup/verification/normal-flash.txt) · [起動後に実機が返した状態](C:/Users/arika/Desktop/walker-backups/rw-walker-motion-control_20261127_before_transfer_cleanup/verification/normal-ready.json) · [書き込み版のリンクマップ](C:/Users/arika/Desktop/walker-backups/rw-walker-motion-control_20261127_before_project_cleanup_481a221/project/.pio/build/atoms3cam-foot-periodic40/firmware.map)

| 主な用途 | 当時の自作 `src` ファイル数 | アプリに占めるバイト数 | コンパイルした実装ファイル |
| --- | ---: | ---: | --- |
| 歩行入力・エネルギー制御 | 73 | 211,768 | `experiment_runner.cpp`（歩行・再始動処理の `.inc` を取り込む）、`lateral_cg_store.cpp` |
| ホイール入出力 | 2 | 4,321 | `roller485_manager.cpp` |
| カメラ画像取得 | 13 | 13,256 | `camera_coexistence.cpp`、`camera_frame_ready_hal.c`、`camera_row_overlap.cpp`、`camera_task_priority_patch.cpp` |
| 白マーカー認識・足角推定 | 25 | 51,706 | `white_marker_tracker.cpp`、`foot_angle_estimator.cpp`、`foot_observer.cpp`、`periodic_foot_stream.cpp` |
| IMU・胴体姿勢推定 | 21 | 30,533 | `imu_i2c_transport.cpp`、`imu_manager.cpp`、`mekf6.cpp`、`mekf_attitude_diagnostics.cpp`、`upright_reference_store.cpp` |
| 歩容観測・支持移行候補の比較 | 10 | 14,424 | `gait_turn_run.cpp`、`gyro_heading_diagnostic.cpp`、`support_transfer_audit.cpp` |
| Web画面・HTTP・ログ転送 | 12 | 272,014 | `web_ui.cpp`、`bounded_web_server.cpp`、`immutable_export.cpp` |
| ログ記録・符号化 | 11 | 464,900 | `psram_logger.cpp`、`log_sample_encoder.cpp`、`log_quantization.cpp` |
| 起動・実行管理・診断 | 15 | 35,531 | `main.cpp`、`control_latency.cpp`、`control_work_profile.cpp`、`runtime_diagnostics.cpp` |
| SDK・外部ライブラリ・共有領域・配置情報など | — | 1,016,651 | Wi-Fi・カメラ等の下位ドライバー、OSなど |
| **合計** | **182** | **2,115,104** | **自作のコンパイル単位29本** |

ファイル数は書き込み版のコンパイラ依存記録を使い、C/C++実装29件・ヘッダー151件・取り込み用 `.inc` 2件を主な役割で1回ずつ数えた。容量はアプリに格納された命令と定数・初期値をリンクマップのオブジェクト単位で分類した値で、複数の機能が同居する実装は主な担当に計上した。Web欄には配信用の測定画面189,168バイト（平文とgzip版の合計）が含まれる。測定で発生する記録データはPSRAMに置く。CPU時間はこの集計の対象外。[書き込み時のソース](https://github.com/temesotejam/rw-walker-motion-control/tree/481a221e22352d8744a6accf8d1ce4670b6e9bcc/src)

## 次の研究目標

足の前振れで接触形状と荷重配分が変化する。荷重移行直後に次の支持側へ進む揺動速度が不足すると予測した半周期で、減速が大きくなる前にホイール反作用を加える。イベント駆動型MPCで比較する操作量は入力開始位相と通電幅。支持が移った後の揺動速度、受動脚の振り出し、左右交互の一歩と前進を評価する。足角からの接地・荷重推定と、変更した入力に対する実機の歩行継続は未検証。

## 11月27日の整理と検証

現在の±8°歩行は [開始条件](src/walking/start.h)、[半周期の入力判断](src/walking/walking_energy_runner.inc)、[通電の開始・終了](src/experiment_runner.cpp) の順に追える。直前の44件を38件に統合・配置し、実行条件が両立しない旧ピークMPCは[研究用](analysis/simulator/peak_mpc.h)に移した。歩行入力・エネルギー制御に分類したファイルは、書き込み済みの版で73件、前回整理版で48件、今回のソースで41件。現在の通常ビルドの自作コードは129件、Git管理全体は285件。[全体の案内](README.md#ファイル別の役割) · [本体の内訳](src/README.md#ファイル別の役割) · [今回の整理直前の全体コピー](C:/Users/arika/Desktop/walker-backups/rw-walker-motion-control_20261127_before_walk_control_layout_4db7b13/project)

製作者によると11月以降は概ね同じ形状で、足裏にはテクスチャを追加した。追加日と摩擦係数は未確定。提供資料の重心高128 mmと制御の名目値120 mmは一致しない。現機の重心高が未測定のため、係数は維持した。実通電に使う幾何・仕事の係数、記録専用の推定、試行ごとに初期化する通過履歴の出所と検証条件を[歩行制御の資料](docs/SUPPORT_TRANSFER_V39.md#形状変更後に検証する量通常のwalk7)にまとめた。

今回の通常版は2,113,056バイト、SHA-256 `d55f068b6ebdb5bcad09c6338d7732804588c140c7ba1014e119a5fabae5280a`。コンパイラの依存記録で本体129件を照合した。リンク後の旧ピークMPC呼び出し関数は1件から0件となり、歩行入力の判断と足観測時の判定は引き続き存在する。統合した44ファイルの処理本体を整理前のコピーと照合した。実測44453706と38871004を同じイベント配分設定で再生した全結果が整理前後で一致した（戻り補助の比較対象は各7、21区間）。実機への書き込みは行っていない。機体と書き込みページは従来の実機確認済みアプリ（`34f8e27b…`）。

ホスト回帰一式が完走し、実測46492793・201405726の双方で8,192バイトの位置から再開したRWLOGの内容とCRCが一致した。無駆動の通信試験版も33863489を使って新しい配置から生成・ビルドし、ELFにモータ駆動と歩行入力関数が含まれず、試験用の記録再生が含まれることを確認した。

### 同日、先に行ったリポジトリ全体の整理

11月27日の整理で、通常の操作を[書き込みと測定の2つの入口](README.md)にまとめた。通常のビルド設定は `platformio.ini` の1環境。実際の通電経路、同じ測定中に比較用の予測を記録する経路、機体に組み込まないCAD検証の所在は[本体ソースの案内](src/README.md)に記載した。回帰試験は [test/](test/README.md)、実測ログから入力を再生する研究は [analysis/](analysis/README.md) にある。

Git管理対象は481件から291件。通常のコンパイラが参照する自作の本体コードは182件から136件になった。整理前に9,044ファイルを複製し、過去の解析と生成物はバックアップから引ける。歩行の通電と再始動を決める2本の実装、モータへの電流指令、MEKF、測定画面、通常ビルド設定は原本とバイト単位で一致した。実測2走の中断再開では全バイトとCRCが一致し、10走に対するシミュレータの全評価結果も整理前と一致した。

Git管理ファイルだけを一時ディレクトリに展開し、実測2走を指定した `python -B test/run.py` が完走した（シミュレータ46件を含む）。別に、試行33863489からモータドライバーを含まない通信試験版を再生成・ビルドした。専用の再生経路が通信試験版だけに入ることも照合した。

通常版を再ビルドしたアプリのSHA-256は `cc13c39a9ecc905ad700777ecf1a29a71e4823b451349485f4bd077230ccd1a8`。29個の本体オブジェクトから逆アセンブルした183,372命令は、短縮命令・整列・分岐先の番地などを正規化すると一致し、並びが変わった定数・文字列も参照先の内容が一致した。書き込みページには整理前に実機確認した `34f8e27b…` のイメージを引き続き配布し、ページ経由のバイト列も照合した。整理後ビルドの書き込み用パッケージはPC内で別途検証した。整理後イメージでの実機の挙動・タイミングは未測定。

旧版1.0.23用の任意検査で、運動・IMU取得に使う指定関数群は整理前後とも29,132バイト。当時の予算25,600バイトを超え、残るリンク監査6項目は両版で通過した。[当時の予算と検査対象](test/verification/verify_motion_acquisition_link.py)

## 過去の測定と判断

11月9日～27日の時系列、個別試行、過去の書き込み結果は、整理直前の引き継ぎ全文から参照できる。元の実測データ・ビルド・Git履歴を含むPCのコピーでも同じ相対パスを使用できる。[以前の引き継ぎ全文](https://github.com/temesotejam/rw-walker-motion-control/blob/481a221e22352d8744a6accf8d1ce4670b6e9bcc/CURRENT_STATUS_20261109.md) · [PCの全体コピー](C:/Users/arika/Desktop/walker-backups/rw-walker-motion-control_20261127_before_project_cleanup_481a221/project)
