# 研究用データの入口

| 場所 | 残した理由 |
|---|---|
| [2026-11-20-support-transfer](2026-11-20-support-transfer/) | v37の2試行でv39の開始時刻と通電幅の候補を再計算した結果。`half_cycles.csv` は各半周期、`summary.json` は試行別の件数と入力ファイルのハッシュ。[予測と実測の読み方](../docs/SUPPORT_TRANSFER_V39.md) |
| `2026-11-19-event-replay/` | v37の実測2試行の戻り補助10件を、判断時点の足姿勢・仕事・後続イベントで比較した基準結果。再現手順は[解析報告](../docs/EVENT_FOOT_ENERGY_REPLAY_20261119.md)にある。 |
| `2026-11-17-walking/46492793`、`201405726` | 上記2試行のローカル展開データ。足角の制御受取時刻、実ZERO、反対側ピークを確認できる。 |
| `2026-11-17-walking/38091117`、`117625571` と `2026-11-13-walking/172594684`、`294107213` | 過去試行との歩容・停止・足振れ比較用。ファームとカメラ条件は各試行の `metadata.json` で確認する。 |
| `2026-11-18-periodic40/` | v38を直立待機させて、Web画面の有無・設置姿勢と定周期の足角発行を比較したローカル記録。 |
| `2026-11-11-walking/cad_support_geometry.*`、`2026-11-13-walking/v14_energy_neutral*` | 足姿勢によって支持位置と位置エネルギーの谷が動くという仮説を検討するためのローカル幾何計算結果。[足裏の計算](2026-11-11-walking/analyze_cad_support.py)と[中立点の計算](../tools/analyze_energy_neutral_v14.py)で提供CADから再生成できる。重心・床・足の仮定は[計算の根拠](../docs/ENERGY_NEUTRAL_V14.md)に記載。 |

元のRWLOG・動画・CADは提供時の場所にある。ローカル展開データが必要な場合は `python tools/convert_rwlog_to_csv.py <元ログ> --out <展開先>` で作成できる。退避した試行は[整理前の保管先](../README.md#旧版実験データの保管先)から参照できる。
