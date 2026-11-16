# 足認識の速度優先試験版 v22（2026-11-16）

**実機検証後の更新:** 元v22は設定反映前の読戻しで起動失敗し、HB64は
長時間で白化した。使用する修正版は [v22-fix1](FOOT_SPEED_FIX1.md)。
以下は元v22の準備時点の記録であり、実機検証済みの推奨設定ではない。

ユーザーの「早くすることを第一に改善」に対応し、撮影側の待ち時間と
認識の重複計算を短縮する派生版を準備した。実機速度・画質・歩行は未確認。

- ID: `walking-foot-speed-v22`
- 環境: `atoms3cam-foot-speed`（v21を継承）
- 書込み: http://127.0.0.1:8767/foot-speed/
- 比較版: http://127.0.0.1:8767/foot-frame-ready/

## 撮影の変更と根拠

v21の2試行は、全画素完成時点で受渡しが実行されても画像受取約50ms、
制御反映約61msだった。今回はGC0308の水平blanking（各行の間の待ち）を
306から64へ短縮する。画像320×240、センサー窓648×488、1/2 subsampling、
XCLK20MHz、vflip/hmirror、足角度の画素座標対応は継承する。

根拠は[使用SDKのGC0308設定](https://github.com/espressif/esp32-camera/blob/f0bb429/sensors/private_include/gc0308_settings.h)
と[Linux GC0308ドライバ](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/gc0308.c)
（2026-11-16閲覧）。Linux側のHB既定値は64、下限は33。
同実装の行時間式 `HB + sample_hold_delay + window_width + 4` と、
現行のdelay42/window648を用いると1000→758の比になる。
露光が撮影周期を延ばさず、同じクロック関係で動くという仮定なら、
旧実測50msから**37.9ms程度**を見込む。これは絶対クロックを実測した
計算でも、新版の測定結果でもない。

自動露光の4段階上限を264/396/396/396行、anti-flicker stepを132行に設定し、
初期露光を264行にする。露光が伸びて速度改善を相殺しないことを狙う。
132行は上記推定の行時間75.8usで約10msに相当する候補であり、照明の
ちらつきや実露光時間は実機で確認する。暗い場所で従来より暗くなる可能性がある。

設定は起動中、カメラ用I2C0を一時的に所有している間だけ行う。
対象sensor PID、既存HB/VB/窓寸法/delay/clock divideを検査し、AECを一時停止して
書込み・読戻し、AEC復帰を確認する。不一致・通信失敗時はカメラをdeinitし
`camera_speed_timing_readback_failed` とする。未知設定へ無検証で上書きしない。
実行中にモータ/IMU用I2Cへ設定通信を追加しない。

## 認識と受取り

左右のマーカー5行＋背景3行、上下17位置、全66行cacheは維持する。
隣の探索位置へ移る際に、出入りする行だけを足し引きして整数contrastを更新する。
背景との差分、候補の順序、曖昧さ判定、再捕捉判定は従来と同じ。
行配置と探索刻みが一致しない構成はstatic_assertで拒否する。
固定極細1行や狭い範囲への限定による認識仕様の変更は行っていない。

認識時の内部RAM画素読取は両足87040→46080 bytes（47.1%減）。
外部フレームからの必要行コピーは21120 bytesで同じ。
worker最短周期は40→33.333ms（最大約30Hz）、3種類の予測器の受理可能な
画像間隔の下限は40→25msとする。上限200ms、速度/跳躍/古さ判定は継承。
1280行archiveは最短周期で40秒を保存できる。エネルギー選択の式や上限は
変更していないが、入力する観測の時刻・頻度が変わる影響は実機評価が必要。

## 検証

- 実認識器と固定旧実装の合成画像比較：v22/旧経路それぞれ13256比較で一致。
  座標/候補/無効理由/追跡状態/17位置/全8理由/バッファ寿命を含む。
- PC上のcache＋認識benchmark：旧0.114790ms、v22 0.057838ms/両足。
  ホスト上での参考値で、ESP32実測8.1msが半分になる保証ではない。
- 実センサー設定関数：全read/write位置の故障、不一致、未知設定拒否を検証。
  3予測器の38ms入力Readyと2ms間隔拒否、archive容量検査PASS。
- v20/v22実foot pipelineの所有権/ゼロ校正/時刻/周期/無効化、旧予測回帰PASS。
  pipelineテストのidle期間延長時に残った旧期待値1箇所を訂正して再実行PASS。
- 実v19〜v22 metadata生成・CRC/export/変換PASS。v22最大fixture986514 bytes、
  足1280/energy192/support96を保存し1MiB上限内。メモリ断片化は実機未検証。
- 4版installer分離試験PASS、ELFのtask priority wrapperとproject HAL使用を確認。
- ビルド成功：RAM127644/Flash1926013 bytes。
  firmware1926384 bytes、SHA256
  `c1a370211e4d0884141e08e9adce28df46e116e3b39f7a39586aee107ce7737f`。
- 配信8ファイルHTTP200/全バイト一致、旧配信208ファイル不変。

## 次の実機評価

今回は未書込み。未保存ログを保存後にv22を書き込み、まず測定開始前に
左右の角度更新とプレビューを確認する。v22で認識できる明るさを確保し、
同じ照明条件で前版と比較する。カメラ初期化失敗なら診断JSONで理由を確認する。

RWLOGの新profile情報と既存 `frame_us/delivered_us/recognition_us/ready_us/
control_observed_us` を確認し、約50msの画像取得が短縮したか、検出欠落や
control latencyが増えていないかを評価する。30秒FINISHEDだけで歩行成功とはしない。
ローカル検証資料：`analysis/2026-11-16-foot-speed/` の `tracker_test.json`、
`metadata_test.json`、`installer_verification.json`。
