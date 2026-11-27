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
| 支持移行に向けた通電の開始・幅と、実測された到達点 | [予測器の再生](support/replay_support_transfer.py)、[実測の採点](support/score_observed_transfers.py)、[操作手順](../docs/SUPPORT_TRANSFER_V39.md) |
| 足の出方・戻り方と半周期の不足仕事 | [イベントごとのエネルギー再生](support/replay_event_foot_energy.py) |
| 画像取得、足角度の計算・発行、制御への到着 | [歩行中の足角度計測](measurement/audit_walking_foot_measurements.py)、[時刻の内訳](measurement/analyze_foot_latency.py) |
| 実測ログ上での定周期足角度 | [定周期推定器の再生](measurement/replay_periodic_foot.py) |
| 足を振ったときの足裏形状とエネルギー的な中立点 | [中立点の計算](measurement/analyze_energy_neutral_v14.py)、[CADの診断実装](cad-shadow/)、[形状・重心の仮定](../docs/ENERGY_NEUTRAL_V14.md) |

## 過去の試行を再現する

整理時点でGitに保存していた試行別の解析コード、図表、CSV、旧版の再現手順は日付ごとに確認できる。[過去の研究一覧](https://github.com/temesotejam/rw-walker-motion-control/tree/481a221e22352d8744a6accf8d1ce4670b6e9bcc/analysis) · [試行44453706と38871004の図](https://github.com/temesotejam/rw-walker-motion-control/tree/481a221e22352d8744a6accf8d1ce4670b6e9bcc/analysis/2026-11-25-walk6-logfix1-trial)

このPCの整理前コピーは `C:\Users\arika\Desktop\walker-backups\rw-walker-motion-control_20261127_before_project_cleanup_481a221\project` にある。Git管理外で旧作業ツリーに展開していたデータもコピーしてある。元のRWLOGと動画は提供時の場所にあり、新しい変換結果や図は `analysis/local/` に保存できる。
