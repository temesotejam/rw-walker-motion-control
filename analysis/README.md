# 実測データと歩行制御の研究

新しい試行を受け取ったら、[歩行シミュレータ](simulator/simulate.py) で胴体・足の応答を評価し、[walk7の入力再生](simulator/replay_walk7.py) で同じ観測から実機の判定を再現できる。ここで比較できる範囲は、新しい入力を最初に選ぶ時点までの実測軌道に限られる。入力を変えた後の歩幅・進行方向は別の実測で確認する。

## 新しいRWLOGを評価する

リポジトリのルートから実行する。シミュレータはPython 3とNumPyを使う。実機と同じC++の判定を再生するには `g++` もPATHに必要。

```powershell
$raw = "$env:USERPROFILE/Downloads"
$trial = "$raw/energy_control_autonomous_run_1_44453706.rwlog"
python -B analysis/simulator/simulate.py --raw-dir $raw --evaluate-run $trial --json-out analysis/local/evaluation.json
python -B analysis/simulator/replay_walk7.py --after-first --event-allocation $trial
```

最初のPythonコマンドでは `--raw-dir` から参照8走（69501075、224021399、69728952、205922296、50784949、52400264、45820669、239670034）を読み、評価用の試行を別に指定する。CRC検査と展開は一時ディレクトリで行う。続く入力再生には指定したRWLOGだけを使う。対象は必要な電流・姿勢・足の時系列を保存した版。全体の再現には上記8走と33125639、216664125を使う。`--part floor` は同梱した床上の計測表だけで実行できる。[シミュレータの評価方法・モデルの仮定](https://github.com/temesotejam/rw-walker-motion-control/blob/481a221e22352d8744a6accf8d1ce4670b6e9bcc/analysis/2026-11-22-walking-simulator/README.md) · [walk7再生の実測結果](https://github.com/temesotejam/rw-walker-motion-control/blob/481a221e22352d8744a6accf8d1ce4670b6e9bcc/analysis/2026-11-25-walk6-replay/README.md)

## 調べたい現象から選ぶ

| 見たいもの | 入口 |
|---|---|
| 動画で次の足が着く前と旧支持足が離れた後の胴体角速度 | [撮影とLED同期の手順](../docs/SUPPORT_TRANSFER_V39.md#動画とrwlogで支持が移る前後の角速度を測る)、[左右別の測定](measurement/measure_transfer_speeds.py) |
| 支持移行に向けた通電の開始・幅と、実測された到達点 | [予測器の再生](support/replay_support_transfer.py)、[実測の採点](support/score_observed_transfers.py)、[操作手順](../docs/SUPPORT_TRANSFER_V39.md) |
| 足の出方・戻り方と半周期の不足仕事 | [イベントごとのエネルギー再生](support/replay_event_foot_energy.py) |
| 画像取得、足角度の計算・発行、制御への到着 | [歩行中の足角度計測](measurement/audit_walking_foot_measurements.py)、[時刻の内訳](measurement/analyze_foot_latency.py) |
| 実測ログ上での定周期足角度 | [定周期推定器の再生](measurement/replay_periodic_foot.py) |
| 足を振ったときの足裏形状とエネルギー的な中立点 | [中立点の計算](measurement/analyze_energy_neutral_v14.py)、[CADの診断実装](cad-shadow/)、[形状・重心の仮定](../docs/ENERGY_NEUTRAL_V14.md) |

## 過去の試行を再現する

整理時点でGitに保存していた試行別の解析コード、図表、CSV、旧版の再現手順は日付ごとに確認できる。[過去の研究一覧](https://github.com/temesotejam/rw-walker-motion-control/tree/481a221e22352d8744a6accf8d1ce4670b6e9bcc/analysis) · [試行44453706と38871004の図](https://github.com/temesotejam/rw-walker-motion-control/tree/481a221e22352d8744a6accf8d1ce4670b6e9bcc/analysis/2026-11-25-walk6-logfix1-trial)

このPCの整理前コピーは `C:\Users\arika\Desktop\walker-backups\rw-walker-motion-control_20261127_before_project_cleanup_481a221\project` にある。Git管理外で旧作業ツリーに展開していたデータもコピーしてある。元のRWLOGと動画は提供時の場所にあり、新しい変換結果や図は `analysis/local/` に保存できる。

## ファイル別の役割

実測の再生、支持移行の評価、足角の精度・遅延、CAD形状の照合に用いるファイルを挙げます。

<details>
<summary>研究の案内（1件）</summary>

| ファイル | 役割 |
| --- | --- |
| [README.md](README.md) | 実測ログの展開後に実行する研究プログラムと、旧試行の保管場所を案内する。 |

</details>

<details>
<summary>実測に基づくシミュレーション（23件）</summary>

| ファイル | 役割 |
| --- | --- |
| [simulator/action_model.py](simulator/action_model.py) | ログから角速度の変化を学習し、正側ピークから中心へ戻る区間で別の電流入力を試算する。 |
| [simulator/add_early_watch.py](simulator/add_early_watch.py) | 中心へ戻る途中に早めの監視角を加えた場合の検出・見逃しを比較する。 |
| [simulator/aid_current_delivery.py](simulator/aid_current_delivery.py) | 左向きの補助入力で、中心通過または折り返しまでに届いた電流を積分する。 |
| [simulator/arrival_calibration.cpp](simulator/arrival_calibration.cpp) | 右へ揺れる際の到達エネルギーと入力選択の本体コードをPCから呼び出す。 |
| [simulator/arrival_calibration.py](simulator/arrival_calibration.py) | 過去の中心通過と到達点にモデルを合わせ、右向き入力の本体判定を再生する。 |
| [simulator/floor_check.py](simulator/floor_check.py) | 中心通過から次のピークまでに足が床基準でどの程度動いたかを計測表で比較する。 |
| [simulator/floor_observations.csv](simulator/floor_observations.csv) | 動画から得た左右の足の前進量と横移動量を床の格子単位で保持する。 |
| [simulator/floor_video.py](simulator/floor_video.py) | 床格子に合わせた動画から左右の足を追跡し、画面座標を床座標へ写す。 |
| [simulator/foot_kinematics.py](simulator/foot_kinematics.py) | 定周期足角の観測履歴から、判断時点で分かる左右の足角・角速度を取り出す。 |
| [simulator/foot_phase_check.py](simulator/foot_phase_check.py) | 両足の角速度を加えたときに中心への到達予測が改善するかを調べる。 |
| [simulator/legacy_check.py](simulator/legacy_check.py) | 旧v14の2試行を、受領時刻・校正・符号が確認できる範囲で新しい基準と比較する。 |
| [simulator/loss_observer.py](simulator/loss_observer.py) | IMUで観測した減速を使う損失補正を、補助後と惰性で戻る区間で再生する。 |
| [simulator/peak_mpc.h](simulator/peak_mpc.h) | 過去9走で同定した通電幅と次ピーク・角速度・回転数のモデルを、PCで過去の制御候補と比較するために保管する。 |
| [simulator/phase_dynamics.py](simulator/phase_dynamics.py) | 足・胴体・エネルギーの情報から左向きの戻り軌道を予測し、試行を分けて実測と比較する。 |
| [simulator/post_zero_carry.py](simulator/post_zero_carry.py) | 実際に中心を通過した後、受け側足裏の円弧に届くまでの運動を再生する。 |
| [simulator/receiving_carry.py](simulator/receiving_carry.py) | 左向きの入力判断と受け側足裏の到達候補を、その後の実測運動と対応付ける。 |
| [simulator/replay_walk7.py](simulator/replay_walk7.py) | 保存した実測軌道にwalk7などの補助・再始動判定を適用し、最初に判断が変わる点を調べる。 |
| [simulator/return_history_replay.py](simulator/return_history_replay.py) | 当時までの履歴を使い、左向きの補助入力の再判断を測定と同じ時点で再生する。 |
| [simulator/return_history_selector.cpp](simulator/return_history_selector.cpp) | 戻り履歴、仕事から通電時間への逆算、ホイール制限の本体実装をPCで呼ぶ。 |
| [simulator/return_model.py](simulator/return_model.py) | 正側ピークから惰性で戻る実測例から、中心に届くか途中で折り返すかを予測する。 |
| [simulator/right_candidates.py](simulator/right_candidates.py) | 右向きの中心通過で選ぶ通電幅を、過去に近い条件で実行した例と比較する。 |
| [simulator/simulate.py](simulator/simulate.py) | 基準走行を読み込み、各モデルの再生や新しいRWLOGの評価をまとめて実行する。 |
| [simulator/walk7_detectors.cpp](simulator/walk7_detectors.cpp) | walk7の本体側観測判定を、実測データでPC上から動かす。 |

</details>

<details>
<summary>支持移行とエネルギーの再生（7件）</summary>

| ファイル | 役割 |
| --- | --- |
| [support/event_foot_energy_candidate.py](support/event_foot_energy_candidate.py) | 中心への到達に必要な仕事と反対側ピークへ向けた仕事を比較し、入力候補を試算する。 |
| [support/event_foot_features.py](support/event_foot_features.py) | 入力イベント時点までに制御側が受領した足角から、足の出方・戻り方を復元する。 |
| [support/event_foot_outcomes.py](support/event_foot_outcomes.py) | 補助入力の判断を、同じ半周期内で起きた中心通過やピークに対応付ける。 |
| [support/replay_event_foot_energy.py](support/replay_event_foot_energy.py) | v37の実測ログで、足角を使う補助入力候補と当時の判断を比較する。 |
| [support/replay_support_transfer.py](support/replay_support_transfer.py) | イベント単位の支持移行計画を実測ログで再生し、幾何学的な到達目安と比較する。 |
| [support/score_observed_transfers.py](support/score_observed_transfers.py) | 支持移行予測と、動画の注釈または実測の胴体運動を照合する。 |
| [support/support_transfer_replay.cpp](support/support_transfer_replay.cpp) | 本体の定周期足角推定と支持移行の計画計算をPCで呼び出す。 |

</details>

<details>
<summary>足角・計測の分析（5件）</summary>

| ファイル | 役割 |
| --- | --- |
| [measurement/analyze_energy_neutral_v14.py](measurement/analyze_energy_neutral_v14.py) | 足角を固定して重心高さの谷と左右の接触が切り替わる角度を比較する。 |
| [measurement/analyze_foot_latency.py](measurement/analyze_foot_latency.py) | ログにあるカメラの取得・処理・制御受領の時刻から各段階の遅れを集計する。 |
| [measurement/audit_walking_foot_measurements.py](measurement/audit_walking_foot_measurements.py) | 歩行中の足角の有効率、欠測間隔や観測時刻を実測ログで調べる。 |
| [measurement/replay_periodic_foot.py](measurement/replay_periodic_foot.py) | 過去の足角が制御側に届いた時刻に合わせて定周期推定器を再生する。 |
| [measurement/measure_transfer_speeds.py](measurement/measure_transfer_speeds.py) | 動画のLEDとRWLOGを同期し、観察した接地・離地の区間前後で胴体角速度、進入時の左右足角、通電状況を取り出す。 |

</details>

<details>
<summary>CAD形状による比較（5件）</summary>

| ファイル | 役割 |
| --- | --- |
| [cad-shadow/README.md](cad-shadow/README.md) | CAD診断で使う座標・角度の仮定と、照合できる範囲を記す。 |
| [cad-shadow/cad_support.h](cad-shadow/cad_support.h) | CAD足裏頂点から左右の最低高さと支持の切り替わり候補を求める。 |
| [cad-shadow/cad_support_vertices.h](cad-shadow/cad_support_vertices.h) | 提供CADから抽出した左右足裏の頂点座標を保持する。 |
| [cad-shadow/foot_pose_prediction.h](cad-shadow/foot_pose_prediction.h) | 足の連続観測からCAD診断に渡す前振れ角を予測する。 |
| [cad-shadow/generate_cad_support.py](cad-shadow/generate_cad_support.py) | 機体のGLBファイルから足裏頂点ヘッダーを再生成する。 |

</details>
