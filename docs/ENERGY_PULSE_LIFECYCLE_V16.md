# v16：入力時間の即終了不具合の修正と完了記録

2026-11-13。環境 `atoms3cam-energy-pulse-lifecycle`、build ID `energy-pulse-lifecycle-v16`、branch21。エネルギー選択モデルはv15のまま `walking_early_energy_main_v15`、完了量の予測は `actual_elapsed_pulse_completion_v16` と分ける。

## 修正した不具合

v15は入力開始時刻をキュー受理後へ更新した一方、同じループの終了判定に開始前の古い時刻を渡していた。unsigned差分が巨大になり、多数の要求入力が直後に停止した。実機2試行では起動以外69/58件中50/41件、前倒し18/14件すべてで継続中の指令sampleが得られなかった。

v16は `updateEnergyControlAutonomousPulse` で現在の `millis()` を読み直してから終了を判定する。時刻の大小を単純比較する方式にせず、カウンタの折り返しを含めて検証した。v14/v15の環境は比較用として保持する。

## 完了量を予定量と分ける

入力採用時に初期電流・目標電流・時定数・角速度と判断の識別子を保存する。終了時にはソフトウェア指令の実経過時間から次を計算する。

```
T = actual_elapsed_ms / 1000
Q = Ig*T + (I0-Ig)*tau*(1-exp(-T/tau))
I_end = Ig + (I0-Ig)*exp(-T/tau)
W = inherited_response(rate, Q)
unmet = max(0, request-W)
```

Qは符号を保持し、負のnet Qを絶対値で正の励振仕事へ変換しない。仕事の適用条件を満たさなければW/到達/未達は無効と記録する。実経過123msなら123msを計算し、80/100msへ切り詰めない。幅・速度・Qのモデル範囲外を別に表示する。

残留電流モデルの終了値/時刻を実経過時間へ合わせ、同じトリガーと到達側に対応するpendingのQ/予測到達も更新する。通常終了・途中停止・ESTOP・run終了の経路を区別する。停止要求を先にキューへ入れてから完了量の計算を行う。

完了処理は一回だけ行う。監査保存枠192件が満杯でも、別に保持した進行中入力の状態で残留電流/pending更新を継続する。制御をログ容量に依存させない。

## 記録の意味

`walking_energy` に予定幅と `output_start_ms` / `output_end_ms` / `actual_elapsed_ms`、完了理由、予定時間完了フラグ、実経過による予測Q/終了電流/W/未達を追加した。

完了理由は0未観測、1時間到達、2中断、3ESTOP、4run終了。`completion_reason=0` の未完了と、観測した経過0msを区別する。予定入力のモデル値は残し、`elapsed_*` の完了予測と混ぜない。converterは追加列を `walking_energy.csv` へ保存する。

ここでの実経過は**制御側のキュー受理から停止要求までのソフトウェア指令時間**である。`setCurrentMa` 成功はキュー受理で、RS485適用や電流の立ち上がりを直接示さない。Q/Wも実経過時間を入力したモデル予測であり、測定された機械仕事ではない。ドライバ適用シャドウと電流readbackは既存timeseriesに別に残る。

## エネルギー制御と比較基準

不足Jに到達する最小整数幅、保護後に前倒しを見送って通常機会を残す処理、現在の80ms実験範囲/別の15ms救済はv15を維持する。エネルギー係数・基準・段階目標・中立点モデルを今回同時に変更していない。

ユーザーが評価した、前回v14の「遅く小さい歩幅でもきれいに歩けた」状態を比較基準とする。足角振幅や8度到達だけを改善の指標にしない。v16ではv15で止まっていた入力が予定幅を継続するため、実際の入力量が増える場面がある。時計の修正だけで滑らかさが戻るとは保証せず、実測した継続時間と歩行を次に比較する。

## 検証

- 実begin→実updateを同じループ順で接続。0/1/7msの受付遅延、古い判断時計、カウンタ折り返し、書込み失敗を検証。v15の15条件で誤停止を再現、v16は72条件すべてで全幅を保持、v14参照も72条件正常。
- 実際の入力採用→監査登録→begin→周期更新→停止→完了を接続。幅直前まで継続し、終了一回、途中停止/遅い終了、192枠超過でも状態更新が残ることを確認。
- 実完了関数・実監査構造体で42条件ほか、0/17/80/123/1000ms、負Q、pendingの時刻/側不一致、終了重複、ログ枠なしを検証。
- 実C++ metadata/compact JSON/9桁J/CSVで、完了・中断・無効仕事・未完了・無出力・0ms・123msを区別して保存。

対応コードは `test_energy_pulse_lifecycle_v16.py`、`test_energy_main_lifecycle_v16.py`、`test_energy_pulse_completion.py`、`test_energy_pulse_completion_runner_v16.py`、`test_energy_pulse_lifecycle_metadata_v16.py`。独立レビューは `ENERGY_PULSE_LIFECYCLE_V16_REVIEW.md`。

## 配信と未確認事項

PlatformIO SUCCESS、RAM166780/327680 bytes、Flash1879405/3342336 bytes。firmware.bin1879776 bytes、SHA256 `f41dc856fc00d43d15d49f90c32e790b8947a296454daf9dc047c00a051a36c2`。

専用ページ `http://127.0.0.1:8767/energy-pulse-lifecycle/`、再準備 `python tools/prepare_energy_pulse_lifecycle_flasher.py`。ページ/manifest/4partsのHTTP200と全バイト一致を確認。小さい検証結果は `analysis/2026-11-13-walking/v16_build_verification.json`。v14比較版の配信firmwareも以前のSHA256のまま保持されていることをHTTPで確認した。

今回は実機へ書き込んでいない。新制御での実電流/機械仕事/連続歩行、足姿勢で動く中立点の能動補正は未検証である。
