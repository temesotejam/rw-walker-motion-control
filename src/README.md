# 現行歩行ファームウェア

通常の±8°測定は [platformio.ini](../platformio.ini) の `atoms3cam-foot-periodic40` でビルドします。[main.cpp](main.cpp) の `controlStep()` が機体の状態を読み、歩行制御を更新します。

| 変更したいこと | 最初に開くファイル | 現行版の処理 |
|---|---|---|
| 歩行入力のタイミングと強さ | [walking_energy_runner.inc](walking/walking_energy_runner.inc) | `tryWalkingEnergyFoot()` が揺れと足姿勢を調べ、`tryWalkingEnergy()` が必要な仕事と半周期で使える時間から通電を判断します。静止後の再始動は [settled_walking_restart_runner.inc](walking/recovery/settled_walking_restart_runner.inc) にあります。 |
| 足角の認識と発行 | [foot_observer.cpp](foot_observer.cpp) | 画像から左右の実測角・有効性・時刻を取得します。[periodic_foot_stream.cpp](periodic_foot_stream.cpp) が観測に基づく足角を25 ms周期で発行します。 |
| 測定データの記録と確定 | [psram_logger.cpp](psram_logger.cpp) | 測定中にPSRAMへ記録します。[immutable_export.cpp](immutable_export.cpp) が確定後のログを転送用に保持します。 |
| 測定ページとダウンロード | [web_ui.cpp](web_ui.cpp) | 本体で状態照会とログ取得のHTTP要求を受け付けます。取得・再開を行うブラウザ側の処理は [runtime.js](../web/runtime.js) にあります。 |

胴体の角度は [imu_manager.cpp](imu_manager.cpp) で取得したIMUデータから [MEKF](mekf6.cpp) が推定します。歩行制御が採用した電流指令は [roller485_manager.cpp](roller485_manager.cpp) を通ってホイールへ渡ります。

## 歩行制御を追う順番

±8°歩行の開始条件とモードは [start.h](walking/start.h)、その半周期に補助・主入力・惰行を選ぶ処理は [walking_energy_runner.inc](walking/walking_energy_runner.inc)、通電の開始・終了は [experiment_runner.cpp](experiment_runner.cpp) にあります。

旧版を「歩行入力・エネルギー制御」と広く分類した73ファイルは、前回の整理で48件、今回は41件になりました。内訳は歩行14件、モデル10件、開始設定2件、試験・補正12件、統括と設定保存3件です。通常歩行でも進む向きと戻りの減速を見て補助を選び、静止時には再始動を評価します。共通の出力制限は前後傾斜やホイールの回転に応じて働きます。

`models/` は物理量や入力候補を計算する場所です。固定脚の実測で同定し、固定脚制御の通電選択に使うホイール仕事モデルもここにあります。`control_trials/` は本体の機能・設定で有効になる試験や補正をまとめています。12°固定脚の試験はその一例です。PC側で行う検証は [test/](../test/README.md) にあります。

## 支持移行の予測が使われる場所

- **実際の通電**では、[return_observation_policy.h](walking/support/return_observation_policy.h) が戻り途中の実測足角と胴体の減速を判定し、[return_crossing_work.h](walking/support/return_crossing_work.h) と [receiving_arc_aid.h](walking/support/receiving_arc_aid.h) が支持移行を越えるために加える仕事を計算します。
- **通常測定中の比較記録**では、[support_transfer_plan.h](support_transfer_plan.h) の `select()` が開始時刻と幅の候補を評価し、[support_transfer_audit.cpp](support_transfer_audit.cpp) が結果を記録します。同じヘッダーの `supportGap()` と `arcEntryDeg()` は実通電の仕事計算でも使います。RWLOGには比較候補と実行した入力の両方が残ります。[制御と評価の詳細](../docs/SUPPORT_TRANSFER_V39.md)
- **CADによる形状検証**は [analysis/cad-shadow](../analysis/cad-shadow/) にあります。専用ビルドで `CAD_SUPPORT_SHADOW` を有効にすると、CAD形状から求めた接地候補を比較できます。足裏形状・座標の出典は[支持移行の仕様](../docs/SUPPORT_TRANSFER_V39.md)にあります。

## ファイル別の役割

通常ビルドが参照する本体コード129件を主な役割で1回ずつ数えました。内訳はヘッダー98件、C/C++の実装29件、歩行処理を取り込む `.inc` 2件です。設定・分岐・入力によって実行される経路は変わります。

カメラから制御には、画像の時刻・有効性を伴う足角の実測と、それを使って25 ms周期で発行する推定値の両方が届きます。通電を決めた後は機体内のI²CでRoller485へ電流を指令します。PCとの測定操作・ログ転送はWi-Fi上のHTTPが担当します。[制御ループ](main.cpp) · [ホイールへの入出力](roller485_manager.cpp) · [HTTP要求の受付](web_ui.cpp)

<details>
<summary>通常歩行の入力・支持移行・再始動（14件）</summary>

| ファイル | 役割 |
| --- | --- |
| [walking/fore_aft_motion_guard.h](walking/fore_aft_motion_guard.h) | 胴体の前後傾斜・運動から通電を許容できる範囲を判定し、絶対傾斜の停止条件も扱う。 |
| [walking/pulse.h](walking/pulse.h) | 励振方向からモータ指令の符号を求め、ホイールの回転偏りによる幅制限と実通電時間に基づく終了処理を扱う。 |
| [walking/recovery/settled_walking_restart.h](walking/recovery/settled_walking_restart.h) | 足角・胴体の履歴から揺れが片側に収束した条件を検出し、再始動入力の候補を計算する。 |
| [walking/recovery/settled_walking_restart_runner.inc](walking/recovery/settled_walking_restart_runner.inc) | 停止検出と再始動計画を歩行制御に接続し、再始動入力の実行・結果確認を扱う。 |
| [walking/start.h](walking/start.h) | ±8°歩行で使う制御を選び、開始時の目標角の立上げと最初の運動の確認条件を定める。 |
| [walking/support/gait_turn_energy.h](walking/support/gait_turn_energy.h) | 右足の振れと旋回傾向を観測した条件で、エネルギー目標や通電幅の微調整候補を計算する。 |
| [walking/support/receiving_arc_aid.h](walking/support/receiving_arc_aid.h) | 既に採用された戻り補助について、次の支持側の足裏円弧へ届くための延長と主入力の余地を評価する。 |
| [walking/support/return_crossing_work.h](walking/support/return_crossing_work.h) | 過去に支持側へ到達した半周期と現在の速度を比べ、中心通過の前後へ配分する仕事を計算する。 |
| [walking/support/return_motion.h](walking/support/return_motion.h) | 戻り途中の胴体の減速や失速を検出し、足の前振れ・観測の有効性と合わせて早めの補助入力を判定する。 |
| [walking/support/return_observation_policy.h](walking/support/return_observation_policy.h) | 胴体の減速・足の観測時刻と姿勢から、戻り中に不足が出る区間と補助の必要性を判定する。 |
| [walking/support/right_arrival_energy.h](walking/support/right_arrival_energy.h) | 右に向かうゼロ通過で、実測範囲内の到達エネルギーに基づき入力の維持・縮小・惰行を評価する。 |
| [walking/support/walking_return_energy.h](walking/support/walking_return_energy.h) | 戻りの半周期で目標に不足する仕事を予測し、補助と主入力に使える通電時間・エネルギーを管理する。 |
| [walking/walking_energy_policy.h](walking/walking_energy_policy.h) | 足裏の位置エネルギー、角速度、通電で増える推定仕事から候補を評価し、補助通電の成立条件を判定する。 |
| [walking/walking_energy_runner.inc](walking/walking_energy_runner.inc) | 現在の足・胴体・ホイール状態を使い、前倒し補助・ゼロ通過時の主入力・惰行の実行を決める。 |

</details>

<details>
<summary>エネルギー・予測の計算（10件）</summary>

| ファイル | 役割 |
| --- | --- |
| [models/control_math_cache.h](models/control_math_cache.h) | 電圧・電流モデルの入力が同一のときだけ計算結果を再利用する。 |
| [models/direct_q_solver.h](models/direct_q_solver.h) | 次のピークに必要な電流の時間積分をモデルから逆算し、通電時間の候補に変換する。 |
| [models/free_peak_prediction.h](models/free_peak_prediction.h) | ゼロ通過時の角速度と直前のピーク・揺動方向から、自由揺動で届くピークの予測を補正する。 |
| [models/lateral_cg_energy.h](models/lateral_cg_energy.h) | 等価的な左右重心ずれを含む位置エネルギーと到達角を計算し、左右差の補正量を扱う。 |
| [models/lateral_energy_control.h](models/lateral_energy_control.h) | 足裏円弧の幾何・角速度・電流の時間積分から左右のエネルギーや必要入力を計算する。 |
| [models/low_peak_energy.h](models/low_peak_energy.h) | ±8°試験の正側ピークで到達エネルギーの予測誤差を追い、対象条件の通電幅を微調整する。 |
| [models/peak_error_adaptation.h](models/peak_error_adaptation.h) | 同じ側・指令条件・ホイール速度域の観測から、次ピークのエネルギー予測誤差を補正する。 |
| [models/walking_energy_loss.h](models/walking_energy_loss.h) | 自由脚で惰行した実測から、通電しない半周期に減る有効エネルギーやモデル適用範囲を見積もる。 |
| [models/wheel_work_energy_control.h](models/wheel_work_energy_control.h) | 固定脚で同定したホイール仕事と次ピークのモデルを使い、電流・パルス幅の候補を比較する。 |
| [models/work_target_policy.h](models/work_target_policy.h) | 固定脚の仕事制御用に、目標角・入力条件・開始時の入力候補やモデル適用範囲を定める。 |

</details>

<details>
<summary>開始設定（2件）</summary>

| ファイル | 役割 |
| --- | --- |
| [control_setup/foot_support_start.h](control_setup/foot_support_start.h) | 足ゼロ点・観測の有効性など、足角を必要とする測定の開始条件を画面と本体で共用する。 |
| [control_setup/run_settings.h](control_setup/run_settings.h) | 開始要求から目標揺動角・電流・入力割合・制動時間を読み取り、設定範囲を確認する。 |

</details>

<details>
<summary>設定で有効になる制御試験・補正（12件）</summary>

| ファイル | 役割 |
| --- | --- |
| [control_trials/fixed_foot/one_peak_energy_inverse.h](control_trials/fixed_foot/one_peak_energy_inverse.h) | 固定脚・負側300 mAの実測モデルで、各パルス幅の到達エネルギーを予測し、目標に近い制動時間を選ぶ。 |
| [control_trials/fixed_foot/target12_state_work.h](control_trials/fixed_foot/target12_state_work.h) | 固定脚12°の実測モデルで、電池電圧・回転数・指令候補から仕事と到達エネルギーを予測する。 |
| [control_trials/foot_balance_adaptive.h](control_trials/foot_balance_adaptive.h) | 各半周期で観測した左右足の振れ量から、歩行試験用の左右補正を調整する。 |
| [control_trials/foot_energy_balance.h](control_trials/foot_energy_balance.h) | 身体に対する左右足の角度振幅を使い、歩行試験で左右のエネルギー調整の効果を追う。 |
| [control_trials/foot_phase_cap.h](control_trials/foot_phase_cap.h) | 揺動位相に対応づけた足の振れを使い、試験時の正側通電候補に上限をかける。 |
| [control_trials/foot_span_candidate.h](control_trials/foot_span_candidate.h) | 足の角度振幅から提案された正側パルスを、元と同じ電流・エネルギーモデルで評価し直す。 |
| [control_trials/foot_span_control.h](control_trials/foot_span_control.h) | 複数周期にわたる左右足の角度振幅・左右差を集計し、試験用の入力補正を求める。 |
| [control_trials/gyro_steering.h](control_trials/gyro_steering.h) | ジャイロによる相対方位の積算と、左右の揺動目標差を用いる操舵試験の計算を定義する。 |
| [control_trials/negative_cap_recovery.h](control_trials/negative_cap_recovery.h) | 負側パルス幅の上限を条件付きで試す補正と、結果を通常の足補正の学習から外す判定。 |
| [control_trials/tilt_priority.h](control_trials/tilt_priority.h) | 起動時の傾きや左右の揺れに応じて試験中の立上げ目標・入力候補を調整する。 |
| [control_trials/walking_side_assist.h](control_trials/walking_side_assist.h) | 正側や揺れの弱い側を補助する試験用候補を、左右の足・胴体の観測から計算する。 |
| [control_trials/work_probes.h](control_trials/work_probes.h) | 仕事モデルの同定用試験で、ゼロ通過ごとに負側パルス幅・正側電流などの条件を割り当てる。 |

</details>

<details>
<summary>測定・駆動の統括、設定の保存（3件）</summary>

| ファイル | 役割 |
| --- | --- |
| [experiment_runner.cpp](experiment_runner.cpp) | 測定・制御の状態を進め、IMUの更新、ピーク・ゼロ通過の検出、通電の開始・終了を取りまとめる。 |
| [experiment_runner.h](experiment_runner.h) | 測定状態、半周期ごとの制御状態、推定器・記録器と各制御処理の接続を定義する。 |
| [lateral_cg_store.cpp](lateral_cg_store.cpp) | 左右重心ずれの設定を本体の不揮発メモリへ保存・復元する。 |

</details>

<details>
<summary>ホイールへの指令・読出し（2件）</summary>

| ファイル | 役割 |
| --- | --- |
| [roller485_manager.cpp](roller485_manager.cpp) | I²CでRoller485へ電流・停止指令を送り、実電流・回転数・電源電圧などを読み取る。 |
| [roller485_manager.h](roller485_manager.h) | ホイールの電流指令・停止・テレメトリ読出しのインターフェース。 |

</details>

<details>
<summary>カメラ画像の取得（11件）</summary>

| ファイル | 役割 |
| --- | --- |
| [camera_capture_dma.h](camera_capture_dma.h) | センサーから画像を取り込むDMAバッファの分割長・スロット数を計算する。 |
| [camera_coexistence.cpp](camera_coexistence.cpp) | カメラの初期化、クロック、単発・連続取得、フレームの返却と停止を実装する。 |
| [camera_coexistence.h](camera_coexistence.h) | カメラの単発・連続取得を開始・停止し、状態を参照するためのインターフェース。 |
| [camera_frame_ready.h](camera_frame_ready.h) | カメラドライバーからフレームの到着、割り込み、破棄数や遅延を取得するための宣言。 |
| [camera_frame_ready_hal.c](camera_frame_ready_hal.c) | プロジェクトに保存したESP32カメラの低水準ドライバーを対応ビルドに組み込む。 |
| [camera_lifecycle.h](camera_lifecycle.h) | カメラの初期化・終了処理を指定コアに依頼し、完了結果を呼び出し側へ返す。 |
| [camera_row_overlap.cpp](camera_row_overlap.cpp) | 受信済みの画像上段・下段を先に取り出し、フレーム受信中にマーカー認識の前処理を進める。 |
| [camera_row_overlap.h](camera_row_overlap.h) | カメラドライバーと足認識側の間で、部分画像と前処理済みデータを受け渡す。 |
| [camera_sensor_profile.h](camera_sensor_profile.h) | GC0308の読出しタイミングや行の間引きなど、ビルドごとの撮像設定を定める。 |
| [camera_task_priority_patch.cpp](camera_task_priority_patch.cpp) | カメラの内部タスクを作る際、使用コア・優先度・スタックを実機向け設定に調整する。 |
| [camera_task_priority_patch.h](camera_task_priority_patch.h) | カメラ内部タスクの設定・スタック使用状況を確認するインターフェース。 |

</details>

<details>
<summary>足マーカー認識・足角推定（16件）</summary>

| ファイル | 役割 |
| --- | --- |
| [foot_angle_estimator.cpp](foot_angle_estimator.cpp) | 左右の白マーカーの横位置を校正表とゼロ位置に照らし、胴体基準の足角に変換する。 |
| [foot_angle_estimator.h](foot_angle_estimator.h) | 足角の値・有効性・換算係数を表す結果型と推定処理のインターフェース。 |
| [foot_calibration.h](foot_calibration.h) | 左右足の画像位置と角度の対応表、範囲外の扱い、ゼロ点を確定する条件をまとめる。 |
| [foot_calibration_diagnostics.h](foot_calibration_diagnostics.h) | 足角の校正表と判定範囲を、Web診断・画像情報・ログ用JSONに出力する。 |
| [foot_fast_policy.h](foot_fast_policy.h) | 画像取得・認識の処理量や足観測の保存量を制限し、遅延の分布を記録する。 |
| [foot_image_rows.h](foot_image_rows.h) | 撮像時の行配置を認識用の画像座標に対応させ、上段・下段の必要な行を保持する。 |
| [foot_motion_prediction.h](foot_motion_prediction.h) | 遅れて届く足角の履歴から、現在の姿勢と前振り・折り返し・戻りの位相を予測する。 |
| [foot_observer.cpp](foot_observer.cpp) | 画像取得から左右マーカー追跡・足角算出までを実行し、観測時刻・有効性と一緒に公開する。 |
| [foot_observer.h](foot_observer.h) | 足画像の観測レコード、左右足の状態、プレビューとカメラタスクのインターフェース。 |
| [foot_tracking_config.h](foot_tracking_config.h) | 足マーカーを探す画像領域と品質判定しきい値、左右の足角の座標規約を定める。 |
| [marker_identity_policy.h](marker_identity_policy.h) | 白マーカー候補が大きく移動して品質も落ちたとき、同じマーカーとみなすか判定する。 |
| [periodic_foot_state.h](periodic_foot_state.h) | 左右足の実測履歴から定周期時刻の角度を算出し、保持・短時間外挿・欠測を区別する。 |
| [periodic_foot_stream.cpp](periodic_foot_stream.cpp) | 足観測を受け、25 ms周期で左右足の角度・出典時刻・品質を発行する。 |
| [periodic_foot_stream.h](periodic_foot_stream.h) | 制御ループで定周期の足角を発行し、最新値と記録用の列を読み出すための宣言。 |
| [white_marker_tracker.cpp](white_marker_tracker.cpp) | 必要な画像行から白いマーカー候補を探し、位置・形状・コントラスト・追跡履歴で左右足の観測を選ぶ。 |
| [white_marker_tracker.h](white_marker_tracker.h) | 白マーカーの位置・品質・欠測理由、候補追跡と画像前処理に使う型とインターフェース。 |

</details>

<details>
<summary>IMU・胴体姿勢推定（16件）</summary>

| ファイル | 役割 |
| --- | --- |
| [beta_phase_controller.h](beta_phase_controller.h) | 比較用の姿勢推定で、通電後の角速度・切り返しに応じてフィルターのβを切り替える。 |
| [bmi270_timing_reader.h](bmi270_timing_reader.h) | BMI270の更新状態を確認し、必要な加速度・ジャイロのレジスタを読み、時刻を付ける。 |
| [imu_acquisition_audit.h](imu_acquisition_audit.h) | 測定開始境界でのIMU連番・取りこぼしや、センサー読出し時間の統計を記録する。 |
| [imu_i2c_transport.cpp](imu_i2c_transport.cpp) | BMI270専用のI²Cバスを初期化し、割り込みを使うドライバーでレジスタを読む。 |
| [imu_i2c_transport.h](imu_i2c_transport.h) | BMI270用のI²Cドライバーを初期化し、準備状態を確認して読み取るための宣言。 |
| [imu_manager.cpp](imu_manager.cpp) | IMUを初期化し、専用タスクで加速度・ジャイロを取得して時刻・連番付きで制御に渡す。 |
| [imu_manager.h](imu_manager.h) | IMU読出し結果の形式、センサー取得タスク、制御側が最新データを読むための宣言。 |
| [imu_poll_timer.h](imu_poll_timer.h) | ハードウェアタイマーでIMUの読出しタスクを定周期に起こす。 |
| [madgwick_pitch.h](madgwick_pitch.h) | 比較用Madgwickフィルターのクォータニオンから必要なピッチ角だけを取り出す。 |
| [mekf6.cpp](mekf6.cpp) | ジャイロで姿勢を予測し、加速度で補正する乗法型拡張カルマンフィルターを実装する。 |
| [mekf6.hpp](mekf6.hpp) | 姿勢クォータニオン、ジャイロバイアス、MEKFの設定・状態・演算インターフェース。 |
| [mekf_attitude_diagnostics.cpp](mekf_attitude_diagnostics.cpp) | MEKFの姿勢・バイアスなどを制御側で診断用のスナップショットに複製する。 |
| [mekf_attitude_diagnostics.h](mekf_attitude_diagnostics.h) | コピーしたMEKF状態の形式と、Webで確認する姿勢診断JSONの生成を定義する。 |
| [upright_pose_guide.h](upright_pose_guide.h) | 加速度で見た直立の目安と、起動時の機体の立て方を知らせるLEDタイミングを定める。 |
| [upright_reference.h](upright_reference.h) | 明示的な直立姿勢の取得、記録値の妥当性確認、保存基準を用いた相対角の計算を定義する。 |
| [upright_reference_store.cpp](upright_reference_store.cpp) | 直立姿勢として保存した基準を本体の不揮発メモリに読み書きする。 |

</details>

<details>
<summary>歩容の観測・支持移行候補の比較（8件）</summary>

| ファイル | 役割 |
| --- | --- |
| [foot_phase_observer.h](foot_phase_observer.h) | 胴体ピークと足の画像時刻を対応づけ、半周期・一往復での足角の変化量を集計する。 |
| [gait_turn_observer.h](gait_turn_observer.h) | 同じ揺動位相で左右足の振れと胴体方位の変化を比べ、旋回の傾向を判定する。 |
| [gait_turn_run.cpp](gait_turn_run.cpp) | 胴体ピークごとに旋回観測を集計し、完了した周期の結果をログ用JSONへ書き出す。 |
| [gait_turn_run.h](gait_turn_run.h) | 歩行中に旋回観測・足フレームを受け取り、周期単位で結果を保持するインターフェース。 |
| [gyro_heading_diagnostic.cpp](gyro_heading_diagnostic.cpp) | ジャイロのバイアスと軸向きを補正し、独立した胴体方位の診断値を更新する。 |
| [support_transfer_audit.cpp](support_transfer_audit.cpp) | 制御状態を別タスクで読み、支持移行の入力候補と結果を比較用に計算・記録する。 |
| [support_transfer_audit.h](support_transfer_audit.h) | 比較用の支持移行計算へ渡す状態と、実際のモータ指令から独立した記録処理を定義する。 |
| [support_transfer_plan.h](support_transfer_plan.h) | 足裏形状と運動状態から通電開始・幅の候補を比較する。支持側の高さの差や円弧の入口を求める幾何計算は、実際の補助入力の判定でも使う。 |

</details>

<details>
<summary>Wi-Fi・Web通信・ログ転送（11件）</summary>

| ファイル | 役割 |
| --- | --- |
| [bounded_web_server.cpp](bounded_web_server.cpp) | 大きなページなどのHTTP応答を小分けに書き、送信の停滞や未完了を検出する。 |
| [bounded_web_server.h](bounded_web_server.h) | 送信量を制限するHTTPサーバーと、完了・未完了・停滞を報告するインターフェース。 |
| [export_gzip.h](export_gzip.h) | RWLOGの転送パケットをgzipにまとめ、転送量を減らせる場合に圧縮応答を生成する。 |
| [export_protocol.h](export_protocol.h) | RWLOG転送の範囲、パケット長、CRC、複数の記録領域からのコピーを扱う。 |
| [export_transport_recovery.h](export_transport_recovery.h) | ログ取得の進捗と通信停止時間を追跡し、HTTP待受けやWi-Fiを復旧する条件を判定する。 |
| [immutable_export.cpp](immutable_export.cpp) | 停止後のRWLOGを確定し、同じ内容を位置指定で繰り返し読めるようにする。転送時の圧縮も受け持つ。 |
| [immutable_export.h](immutable_export.h) | 確定済みログの状態、識別トークン、範囲読出し、転送用タスクのインターフェース。 |
| [offline_run_session.h](offline_run_session.h) | Webの開始要求から測定中・終了後の復帰まで、HTTP処理と測定の状態遷移を管理する。 |
| [runtime_page_encoding.h](runtime_page_encoding.h) | ブラウザーのAccept-Encodingを読み、測定ページをgzipまたは無圧縮で返せるか判定する。 |
| [web_ui.cpp](web_ui.cpp) | 機体のHTTPルートを実装し、状態表示、測定開始、カメラ診断、RWLOG取得・再開を処理する。 |
| [web_ui.h](web_ui.h) | 測定ページ、機体状態、ログ転送と復旧を管理するWebサーバー側の状態と操作を宣言する。 |

</details>

<details>
<summary>測定ログの記録・符号化（11件）</summary>

| ファイル | 役割 |
| --- | --- |
| [compact_json_table.h](compact_json_table.h) | ログ確定時に、各レコードの項目名と値を表形式のJSONに書き出す。 |
| [log_quantization.cpp](log_quantization.cpp) | ログ用の浮動小数点値を、決められた丸め方と上限・下限で整数に符号化する。 |
| [log_quantization.h](log_quantization.h) | RWLOG各項目に用いるスケーリング・飽和つき整数変換のインターフェース。 |
| [log_sample_encoder.cpp](log_sample_encoder.cpp) | 同時点の制御状態とホイールの読み値を、RWLOGの1サンプルに符号化する。 |
| [log_sample_encoder.h](log_sample_encoder.h) | 制御・ホイールのスナップショットを所定のログ行に変換する宣言。 |
| [log_types.h](log_types.h) | RWLOGのファイルヘッダー、サンプル行や各種記録のバイナリ形式を定義する。 |
| [psram_archive.h](psram_archive.h) | 診断レコード用の配列を起動時にPSRAMへ確保し、測定中の動的確保を避ける。 |
| [psram_logger.cpp](psram_logger.cpp) | 測定サンプルとイベントをPSRAMに記録し、足観測などの記録を含むRWLOGのメタデータを停止後に組み立てる。 |
| [psram_logger.h](psram_logger.h) | 測定・イベントの記録形式と、記録の開始・追記・読出しのインターフェース。 |
| [psram_string.h](psram_string.h) | 大きなログメタデータ用にPSRAM上で文字列を組み立て、容量不足を検出する。 |
| [solver_audit.h](solver_audit.h) | 制御計算の入力・候補・決定値を記録し、測定終了後の比較に使う。 |

</details>

<details>
<summary>実行管理・設定・稼働診断（13件）</summary>

| ファイル | 役割 |
| --- | --- |
| [config.h](config.h) | 基板、センサー周期、制御条件、計測時間、同期LEDなどの共通設定を集める。 |
| [control_latency.cpp](control_latency.cpp) | IMUの読出しから制御側での受領・処理までの所要時間を計測し集計する。 |
| [control_latency.h](control_latency.h) | 制御側への到着遅延、I/O区間、進行回数を記録する型と測定点を定める。 |
| [control_work_profile.cpp](control_work_profile.cpp) | 制御タスク内の処理別の所要時間を診断JSONにまとめる。 |
| [control_work_profile.h](control_work_profile.h) | 制御タスク内を段階に分け、通電中・それ以外の時間統計を収集する。 |
| [diagnostic_journal.h](diagnostic_journal.h) | 再起動をまたぐRTC診断記録、スタック検査の頻度、USB診断の制御を扱う。 |
| [experiment_status.h](experiment_status.h) | 姿勢・角速度、測定状態、制御指令などを共有・表示・記録する状態データを定義する。 |
| [main.cpp](main.cpp) | 起動時に機体とタスクを初期化し、足・姿勢・歩行制御・ログ・Webを接続する。 |
| [realtime_code.h](realtime_code.h) | 時間制約の厳しい関数に適用する内部RAM配置やコンパイラ最適化の指定をまとめる。 |
| [run_control_worker.h](run_control_worker.h) | Webから受けた開始・クリア・校正などの要求を制御側に渡し、画面が読める状態コピーを作る。 |
| [runtime_diagnostics.cpp](runtime_diagnostics.cpp) | 起動段階、停止箇所、タスクの生存と空きメモリを記録し、USBなどから診断できるようにする。 |
| [runtime_diagnostics.h](runtime_diagnostics.h) | 制御・IMU・カメラ・HTTP・ホイール・転送の各診断項目と記録用の呼び出しを定義する。 |
| [timing_deadline.h](timing_deadline.h) | 処理時間が設定予算を超えた回数などを、制御を妨げにくい整数カウンタで記録する。 |

</details>

<details>
<summary>本体ソースの案内（1件）</summary>

| ファイル | 役割 |
| --- | --- |
| [README.md](README.md) | 通常歩行の入口、実通電の経路、比較記録とCAD検証の場所を案内する。 |

</details>
