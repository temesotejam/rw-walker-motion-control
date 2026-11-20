# 実機ファームウェアの読み方

現在使っている構成は、左右の足角を25 ms周期で供給するv38と、ホイールの入力幅をエネルギーから選ぶv19です。歩行時の処理は [main.cpp](main.cpp) の `controlStep()` からたどれます。`setup()` が制御タスクやカメラを初期化し、Arduinoの `loop()` がWebとUSB診断を処理します。

| 確認したいこと | 読む場所 | 処理の役割 |
|---|---|---|
| 胴体姿勢 | [imu_manager.cpp](imu_manager.cpp)、[mekf6.cpp](mekf6.cpp) | IMUを取得し、MEKFで姿勢を更新する |
| 足の実測角 | [foot_observer.cpp](foot_observer.cpp)、[foot_angle_estimator.cpp](foot_angle_estimator.cpp) | カメラ画像からマーカーを検出して左右の角度・有効性を求める。`controlStep()` が初回の制御受取り時刻を記録する |
| 定周期の足角 | [periodic_foot_stream.cpp](periodic_foot_stream.cpp)、[periodic_foot_state.h](periodic_foot_state.h) | 受け取った観測を使い25 ms周期で角度を発行する。観測時刻と発行時刻を保持し、古すぎる足角を無効にする |
| 入力の開始時点 | [experiment_runner.cpp](experiment_runner.cpp) の `updateEnergyControlAutonomousMotion()` と [walking_energy_runner.inc](walking_energy_runner.inc) の `tryWalkingEnergyFoot()` | ZEROクロス、前倒し主入力、戻り補助の候補を判定する。戻り補助は [return_observation_policy.h](return_observation_policy.h) が実測足角と胴体の減速から判断する |
| 通電時間と実際の出力 | [walking_energy_runner.inc](walking_energy_runner.inc) の `tryWalkingEnergy()`、[half_cycle_energy_budget.h](half_cycle_energy_budget.h) | 必要な仕事から候補幅を選び、前後姿勢・ホイール速度・半周期の残り時間などを照合する。採用時は `ExperimentRunner` から [roller485_manager.cpp](roller485_manager.cpp) に電流指令を渡す |
| 記録・取得 | [psram_logger.cpp](psram_logger.cpp)、[immutable_export.cpp](immutable_export.cpp)、[web_ui.cpp](web_ui.cpp) | 実測と制御判断を蓄積し、終了した測定をRWLOGとしてWebから取得できるようにする |

### 次の荷重移行制御との関係

v38は、各エネルギー判断に対応する定周期足角の発行番号を記録します。現行v19の戻り補助が採用する足姿勢の根拠は、制御側に届いた実測です。ここで実測、定周期推定、実行したパルスを区別すると、判断時点で何が分かっていたかをログからたどれます。

今後は荷重が移る前に入力開始位相と幅を比較し、移行後に残る揺動と、反対側の受動脚が振り出せるかを評価します。このイベント駆動型MPCは設計段階です。現行機の振る舞いと実験の根拠は [研究の引き継ぎ](../CURRENT_STATUS_20261109.md) にあります。
