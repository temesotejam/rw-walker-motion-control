# 2026-11-17 ログ取得不通と再測定の比較

## 確認結果

ユーザーが電源を入れ直した後、同じv26でrun 1の取得は成功し、run 2では再び状態問い合わせとログ取得が不通になった。直接原因は未特定。バッテリー給電中のUSB追加接続はユーザーが電気的懸念から希望しないため実施せず、以降は保存済みデータとソースだけで調査する。モータ、再起動、書込み、消去の操作はしていない。

| 項目 | 診断(6) / run 1 | 診断(7) / run 2 |
|---|---|---|
| boot_id | 3427627879 | 3427627879 |
| 最後に受信した状態 | FINISHED / 30秒 / 9996 samples | FINISHED / 30秒 / 10076 samples |
| export | ready | building |
| manifest metadata_bytes | 716383 | 519786 |
| 転送済みbytes | 2435809 | 0 |
| 再試行 | 0 | 4回、5回目にFailed to fetch |
| 最後の失敗要求 | なし | /export/manifest |
| メモリ計測 | sampled=false | sampled=false |

run 1のRWLOGは長さとCRC一致。末尾CRC・manifest CRC・末尾を除くファイルの計算CRCは483075946で一致する。全ファイル（末尾CRCを含む）のCRCとmanifestのCRCを比較してはならない。

診断保存は `web/runtime.js::diagnosticSnapshot()` で最後の状態 `latest` と転送記録を結合する。保存時に本体へ新たに問い合わせる処理ではない。したがって診断(7)のonline/building/MEKF freshは最後の受信時点の情報であり、保存時点で動作している証明ではない。最後のmanifestはメタデータ生成途中を示すが、519786 bytesで実際に処理が停止したとは断定しない。CRC計算・最初のチャンク取得へ進んだことは確認できない。

両診断のboot_idとBROWNOUTは同一。少なくともそれぞれの最後の状態受信の間には起動IDの変化はない。このBROWNOUTを「2回目の取得中に電圧低下で再起動した証拠」として扱わない。不応答後の再起動の有無は未確認。メモリ値0も未計測を意味し、空きRAMゼロではない。

## ソースとの照合

- HTTPは測定中に休止するがAP/DHCPは停止しない。復帰時はlistenerを検査し、失敗なら1秒おきに再試行する。run 2でFINISHEDとbuildingのmanifestを取得できているため、単純なHTTP再開漏れでは説明できない。
- `ImmutableExport::reset()` はBuilding中のリセットを拒否し、それ以外ではmetadata/statusをclearする。生成開始でもmetadataをclearする。前回JSONへ単純に追記して倍増する実装ではない。固定PSRAMバッファを再利用し、生成途中は協調yieldする。
- **再現済みの欠陥:** `CompactJsonTable` の一時Stringへの追加失敗が検出されない。値追加だけを故障注入で失敗させると、`{"a":123}` が `{"fields":["a"],"rows":[[]]}` となり、append/finishはtrue。この欠陥は今回の無線不通と同じ現象を再現したものではない。
- **候補:** 足フレーム等のJSON化で通常Stringを大量に作り直す。大きな最終出力はPSRAM固定でも中間文字列にはその保証がない。初回不通時のUSB診断には内部RAM最小4452 bytesがあるが、今回の診断(6)(7)ではメモリを計測しておらず、反復による枯渇・断片化は未証明。
- **別の候補:** カメラ停止後のキュー清掃 `cam_take(0)` が空キューでDMA resetを起こす。Wi-Fiへの影響は未確認。receiver_activeのガードがあるため停止後20 msごとに繰り返す処理ではない。

優先修正案は、中間文字列の失敗検出と確保回数削減、停止時の副作用のないキュー清掃、ログ生成段階/メモリ/最終HTTP成功時刻を残す診断の追加。タイムアウト延長だけで解消したとは扱わない。今回、ファームウェア修正・書込みは行っていない。

## 取得できたrun 1の足観測・停止

`energy_control_autonomous_run_1_38091117.rwlog`、v26、30秒、CRC正常。ユーザー確認: 約18秒以降は自立したまま足が停止。

- 測定中855画像で両足855/855有効、制御範囲拒否0。カメラ記録時刻から制御側初回受取りまで中央値27.400 ms、p95 32.256 ms、最大39.005 ms。比較対象41601702は中央値50.357 msで、約45.6%短縮。20 msは未達。これはドライバ記録時刻基準で、物理露光時刻の校正値ではない。
- 更新平均28.526 Hz。フレーム間隔中央値40.628 ms、最大326.242 msで、60 ms超の間隔が16件。受取済み画像の遅延が短くても更新の途切れは残る。
- 最終ZERO 16.660秒、最終ピーク16.919秒・+11.174°。最後の補助入力は17.027〜17.090秒の63 ms。その後も29.943秒まで足観測あり。25〜30秒の胴体角2.41〜2.44°、左足1.439〜1.457°、右足0.006〜0.045°。認識停止ではなく観測上も足が静止している。
- この半周期の入力枠80 msのうち63 ms使用、17 msが残る。`half_cycle_energy_budget.h::allowAid()` はaid_usedで同じ半周期の2度目の補助を禁止し、return観測側にも同ピークの消費状態がある。ゼロを越えないまま停止した後に、残り枠を使った追加補助へ進めない制約と整合する。追加17 msで越えられることや必要エネルギー量はまだ保証できない。
- 最終補助の予測仕事は約0.2155 mJ、局所要求約0.2182 mJ。ただしwork_rate_supported=falseで外挿。局所要求充足の予測はZERO到達や次ピーク到達の保証ではない。

run 2動画 `WIN_20261117_11_59_40_Pro.mp4` は43.665秒。抽出画像では序盤の足姿勢変化と移動があり、約29〜43秒では自立したほぼ同じ姿勢が続く。run 2のRWLOGがないため、動画時刻と制御イベントの同期や停止原因は確定しない。

## 検証・成果物

ブラウザ再試行、HTTP休止/復帰、送信制限、実serializer全容量v25/v25native/v26の既存テストPASS。全容量テストのworker期待値だけがCore1のまま残っていたため現行Core0/priority1へ訂正した。通常条件のホスト試験であり、実機無線・ヒープ枯渇・反復測定を再現した試験ではない。

ローカル証拠:
- `analysis/2026-11-17-log-recovery/`: diagnostics_6.json、diagnostics_7.json、diagnostic_comparison.json、USB診断、故障注入再現、run2_video_metadata.json、run2_video_contact.jpg。
- `analysis/2026-11-17-walking/38091117/`: header.json、metadata.json、timeseries.csvほか変換CSV、speed_trial_review.json、latest_foot_at_energy.csv、energy_half_cycles.csv、foot_gaps.csv、speed_trial_overview.png、speed_trial_final_cycle.png。

生データ・動画・バイナリはGitへ追加しない。
