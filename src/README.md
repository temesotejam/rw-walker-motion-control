# 実機ファームウェアの読み方

現在の歩行版 v39-walk4 は、左足が受け側にあるとき、中心まで届くために必要な仕事を、中心通過後の主入力のために時間を確保した場合に出せる仕事と比較します。通常の±8°開始で選ぶ制御は `walking_trial::forRun()` にあります。`main.cpp` の `setup()` が制御タスクやカメラを初期化し、`controlStep()` が歩行時の更新を進めます。Arduinoの `loop()` はWebとUSB診断を処理します。[中心前の仕事配分](return_crossing_work.h) · [左補助後の再判断](return_observation_policy.h) · [右主入力](right_arrival_energy.h) · [半周期の仕事と時間](half_cycle_energy_budget.h)

| 確認したいこと | 読む場所 | 処理の役割 |
|---|---|---|
| 胴体姿勢 | [imu_manager.cpp](imu_manager.cpp)、[mekf6.cpp](mekf6.cpp) | IMUを取得し、MEKFで姿勢を更新する |
| 足の実測角 | [foot_observer.cpp](foot_observer.cpp)、[foot_angle_estimator.cpp](foot_angle_estimator.cpp) | カメラ画像からマーカーを検出して左右の角度・有効性を求める。`controlStep()` が初回の制御受取り時刻を記録する |
| 定周期の足角 | [periodic_foot_stream.cpp](periodic_foot_stream.cpp)、[periodic_foot_state.h](periodic_foot_state.h) | 受け取った観測を使い25 ms周期で角度を発行する。観測時刻と発行時刻を保持し、古すぎる足角を無効にする |
| 入力の開始時点 | [experiment_runner.cpp](experiment_runner.cpp) の `updateEnergyControlAutonomousMotion()` と [walking_energy_runner.inc](walking_energy_runner.inc) の `tryWalkingEnergyFoot()` | ZEROクロス、前倒し主入力、戻り補助の候補を判定する。戻り補助は [return_observation_policy.h](return_observation_policy.h) が実測足角と胴体の減速から判断する |
| 通電時間と実際の出力 | [walking_energy_runner.inc](walking_energy_runner.inc) の `tryWalkingEnergy()`、[half_cycle_energy_budget.h](half_cycle_energy_budget.h) | 必要な仕事から候補幅を選び、前後姿勢・ホイール速度・半周期の残り時間などを照合する。採用時は `ExperimentRunner` から [roller485_manager.cpp](roller485_manager.cpp) に電流指令を渡す |
| 記録・取得 | [psram_logger.cpp](psram_logger.cpp)、[immutable_export.cpp](immutable_export.cpp)、[web_ui.cpp](web_ui.cpp) | 実測と制御判断を蓄積し、終了した測定をRWLOGとしてWebから取得できるようにする |

### 次の荷重移行制御との関係

[歩行周期の判定](gait_turn_observer.h)は、同一画像の左右足角を一往復単位で集計し、相対ジャイロ方位の変化と照合します。turn2は二周期に共通する足の偏りからも[通常ZEROで補正する仕事](gait_turn_energy.h)を決めます。RWLOGを展開すると `gait_turn.csv` の確認時刻を `walking_energy.csv` の `turn_cycle_end_ms` と照合できます。[仕事モデルと保護条件](../docs/SUPPORT_TRANSFER_V39.md)

v39-carry2では、[歩行時の入力処理](walking_energy_runner.inc)が戻り補助を採用した時点で[左受け足の追加仕事](receiving_arc_aid.h)を調べ、条件を満たせば実通電の幅を延長します。[支持切替予測器](support_transfer_plan.h)による開始時刻と幅の候補比較は記録用です。[記録タスク](support_transfer_audit.cpp)で、電流指令を切ってからの経過と実測電流も照合して、惰性の判定を記録します。[前提と評価方法](../docs/SUPPORT_TRANSFER_V39.md)

v38は、各エネルギー判断に対応する定周期足角の発行番号を記録します。現行v19の戻り補助が採用する足姿勢の根拠は、制御側に届いた実測です。ここで実測、定周期推定、実行したパルスを区別すると、判断時点で何が分かっていたかをログからたどれます。

次の実機試験では、予測した移行の時刻と向き、移行後に残る揺動、反対側の受動脚の振り出しと前進を照合します。[研究の引き継ぎ](../CURRENT_STATUS_20261109.md)
