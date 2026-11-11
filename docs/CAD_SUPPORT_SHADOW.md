# CAD足姿勢予測の診断版 v10

ビルド環境：`atoms3cam-cad-support-shadow`、識別名：`cad-support-shadow-v10`。
v6系の制御を継承し、CADに基づく足姿勢予測と幾何計算を診断として追加する。
追加トルク・早期入力の新規発動は行わない。CAD/カメラの原点と機体姿勢の対応が未同定のため、計算値を出力許可に使わない。

## 実装

- `foot_pose_prediction.h`：連続する2個の有効足フレームの差から左右の角速度を求め、カメラ処理時点から30 ms先を予測する。取得から処理までの画像年齢も予測時間へ加える。30 msは診断用の仮定で、実測したモータ遅延ではない。
- ユーザー確認：左右とも、Web表示のプラス角度が足の前方振れ。速度がプラスなら前へ振れ、マイナスなら戻る。CADのX軸回転符号・絶対中立との対応は別途必要。
- 画像年齢180 ms超、未来/逆順時刻、フレーム間隔40〜200 ms外、角度差8°超または推定速度80°/s超、欠測/校正範囲外/run不一致では予測を無効化する。これらは診断の暫定範囲。
- 新しい無効フレームが届いた場合に古い有効フレームを流用しない。run変更で履歴をリセットする。画像取得が約100 ms間隔のため、接地直後の急変を正確に予測できる保証はない。
- `cad_support_vertices.h` は提供されたGLBの左右足裏頂点を上側X軸基準へ変換して生成。入力ハッシュはヘッダ先頭に保存。再生成は `python tools/generate_cad_support.py`（numpyと元GLBが必要）。
- `cad_support.h`：両足の最低高さが一致する機体角を±15°の範囲で求める。左右の予測角度をそのまま／両方反転した2仮説を記録。機体の前後姿勢は仮に0°。実際の前後姿勢の未変換値も記録する。
- 計算はCore0の既存カメラタスク内でカメラバッファ解放後に行う。制御Core1に幾何計算を追加しない。処理時間 `compute_us` を残し、実機で追加負荷を確認する。PSRAMのフレーム保存領域が診断分だけ増える。

## RWLOG

バイナリ形式v56と既存フィールドは維持し、メタデータに `cad_support_shadow` を追加。
停止後のcompact table export。フレームsequenceで元の `foot_frames` と結合する。
出力には `output_enabled=false`、`mapping_calibrated=false` を明記。
メタデータの余裕が不足する場合は後半レコードを省略し、`stored/total/truncated` を記録する。

`python tools/convert_rwlog_to_csv.py input.rwlog --out converted` で
`cad_support_shadow.csv` が追加出力される。古いログにはこのCSVを作らない。

主な列：`sequence`, `reason`, `age_ms`, `horizon_ms`,
`left_rate_dps`, `right_rate_dps`, `predicted_left_deg`, `predicted_right_deg`,
`switch_sign_plus_deg`, `switch_sign_minus_deg`, `body_fore_aft_raw_deg`,
`geometry_valid`, `compute_us`。予測の対象時刻はフレーム時刻+horizon。
現在角・未来角・予測無効を混同せず、無効な予測値はnullとする。

## 検証

`python tools/test_cad_support_shadow.py`：予測の時刻・run切り替え・無効画像・急変の破棄、CAD中立対称性と既知の接触計算例、compact metadataからCSVへの結合を確認する。

`python tools/replay_cad_support_shadow.py <converted-dir> --out replay.json` は同じC++をホストで実行する。取得後に届いた次の有効画像を線形補間した値との比較は診断に限り、独立した角度真値ではない。
今回の27908524では399フレーム中237件で予測可能、比較432値の平均絶対差約0.474°、最大約3.956°。実機処理時刻を厳密には再現していない。予測が不安定な場面を調べるための結果であり、介入の改善を意味しない。

## 書き込みと次の確認

ビルド：`platformio run -e atoms3cam-cad-support-shadow`。
成功後：`python tools/prepare_cad_support_flasher.py`。
専用ページ：`http://127.0.0.1:8767/cad-support-shadow/`。

この版で新しいログを取得し、予測誤差・処理時間・支持位置の仮説を実際の足の動きと照合する。
静止時の中立姿勢と左右各足を前へ振った姿勢で、カメラゼロとCAD角・前後姿勢を合わせる。
その後、足姿勢と勢いに応じて出力を変える版へ進む。今回の版自体は歩行改善の介入版ではない。
