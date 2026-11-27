# ホスト側での動作確認

リポジトリのルートで次のコマンドを実行します。Python 3.9 以降、C++17 対応の `g++`、Node.js 18 以降を使います。すべての処理は PC 内で完結します。

```text
python -B test/run.py
```

Node.js が PATH にない場合は `--node "C:\path\to\node.exe"` で指定できます。実測ログが手元にあれば、次のように追加できます。

```text
python -B test/run.py --rwlog "C:\Users\arika\Downloads\energy_control_autonomous_run_1_46492793.rwlog" --rwlog "C:\Users\arika\Downloads\energy_control_autonomous_run_2_201405726.rwlog"
```

この2試行を指定すると、実測ログの CRC を検査して CSV とメタデータに変換し、イベント時点までに制御側へ届いていた足角度と、40 Hz の足角度出力を再生します。変換先は一時ディレクトリです。指定しない場合、イベントと足角度の照合には `analysis/2026-11-17-walking/` にある同じ2試行の変換済みデータを使います。データがそろっていない場合はこの照合だけをスキップします。

| 対象 | 確認すること |
| --- | --- |
| エネルギー制御 | v19 の入力を許可・保留する判定、半周期のエネルギー収支と記録 |
| カメラと姿勢推定 | v38 の画像取得周期、マーカーの判定、40 Hz の角度出力、MEKF、前後傾斜のガード |
| イベント再生 | 入力候補の評価に、そのイベントより後に得た情報が混じらないこと |
| v39支持移行予測 | CAD上の移行点、移行後のエネルギー余裕、電流の立ち上がり、観測用タスクとログ出力。映像照合では計算結果の到着時刻と実際の支持足・方向を検査する。 |
| 測定と書き込み画面 | Web の表示と開始条件、RWLOG の再取得と CRC、旧形式の変換、書き込み完了の表示と配布物の整合 |

`fixtures/camera_pipeline.py` と `fixtures/arduino/` は、現行ソースを PC 上の模擬カメラ・Arduino 環境で動かすために使います。`fixtures/energy/` には、v19 の判定・記録テストが読み込む既存のシナリオが入っています。ファイル名の旧版番号はシナリオの由来を示します。

Web のログ取得テストでは `fixtures/runtime_rwlog.py` が変換ツールと同じ v56 ヘッダー・サンプル定義から、768件の架空の足観測と模擬ホイール応答を含む入力を毎回生成します。中断位置は 8192 バイトです。再開後は全バイト・CRC・CSV の行数を照合します。

## 停止中の実機で通信が続くか確認する

専用Wi-Fiで機体に接続し、`--source` にそのアダプターのIPv4アドレスを指定します。

```powershell
python -B tools/probe_http_stability.py --source 192.168.4.2 --seconds 180 --min-bytes 2200000 --heap
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
