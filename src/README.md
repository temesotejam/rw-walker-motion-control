# 現行歩行ファームウェア

通常の±8°測定は [platformio.ini](../platformio.ini) の `atoms3cam-foot-periodic40` でビルドします。[main.cpp](main.cpp) の `controlStep()` が機体の状態を読み、歩行制御を更新します。

| 変更したいこと | 最初に開くファイル | 現行版の処理 |
|---|---|---|
| 歩行入力のタイミングと強さ | [walking_energy_runner.inc](walking_energy_runner.inc) | `tryWalkingEnergyFoot()` が揺れと足姿勢を調べ、`tryWalkingEnergy()` が必要な仕事と半周期で使える時間から通電を判断します。静止後の再始動は [settled_walking_restart_runner.inc](settled_walking_restart_runner.inc) にあります。 |
| 足角の認識と発行 | [foot_observer.cpp](foot_observer.cpp) | 画像から左右の実測角・有効性・時刻を取得します。[periodic_foot_stream.cpp](periodic_foot_stream.cpp) が観測に基づく足角を25 ms周期で発行します。 |
| 測定データの記録と確定 | [psram_logger.cpp](psram_logger.cpp) | 測定中にPSRAMへ記録します。[immutable_export.cpp](immutable_export.cpp) が確定後のログを転送用に保持します。 |
| 測定ページとダウンロード | [web_ui.cpp](web_ui.cpp) | 本体で状態照会とログ取得のHTTP要求を受け付けます。取得・再開を行うブラウザ側の処理は [runtime.js](../web/runtime.js) にあります。 |

胴体の角度は [imu_manager.cpp](imu_manager.cpp) で取得したIMUデータから [MEKF](mekf6.cpp) が推定します。歩行制御が採用した電流指令は [roller485_manager.cpp](roller485_manager.cpp) を通ってホイールへ渡ります。

## 支持移行の予測が使われる場所

- **実際の通電**では、[return_observation_policy.h](return_observation_policy.h) が戻り途中の実測足角と胴体の減速を判定し、[return_crossing_work.h](return_crossing_work.h) と [receiving_arc_aid.h](receiving_arc_aid.h) が支持移行を越えるために加える仕事を計算します。
- **通常測定中の比較記録**では、[support_transfer_plan.h](support_transfer_plan.h) が開始時刻と幅の候補を評価し、[support_transfer_audit.cpp](support_transfer_audit.cpp) が結果を記録します。実行された入力と予測候補はRWLOGで照合できます。[制御と評価の詳細](../docs/SUPPORT_TRANSFER_V39.md)
- **CADによる形状検証**は [analysis/cad-shadow](../analysis/cad-shadow/) にあります。専用ビルドで `CAD_SUPPORT_SHADOW` を有効にすると、CAD形状から求めた接地候補を比較できます。
