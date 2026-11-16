# v22 カメラ起動修正と速度調整（2026-11-16）

対象は `walking-foot-speed-v22-fix1` / `atoms3cam-foot-speed-fix1`。
USB接続中はモータを動かさないというユーザー指定に従い、COM7で
書込み・カメラ・足認識・通信だけを確認した。歩行のエネルギー制御はv19を継承する。
書込みページは `http://127.0.0.1:8767/foot-speed-fix1/`。

## カメラを停止させていた判定

元v22は `READY_TO_MEASURE` でMEKF/Webは正常だったが、カメラと足認識の
起動に失敗していた。診断版で、GC0308のレジスタ0x01へ0x40を書いた直後、
読戻しが変更前の0x32だったことを実測した。書込みAPIは成功を返しており、
この即時不一致を異常としてカメラをdeinitしていた。

修正版は設定後の読戻しを5ms間隔、各レジスタ最大100msで確認する。
待機は一時SCCB I2C0を所有する起動中のみ。値が一致すれば即終了し、
通信エラーや不一致が残る場合は異常として扱う。認識ループや制御周期へ
待機を追加していない。Web診断に失敗段階・レジスタ・期待値・読戻し値・
反映待機時間を追加し、Webの目標更新上限もv22の30Hzへ合わせた。

## HB64で確認された白化

初期化修正後のHB64は約38msで画像を取得し、起動直後には左右を認識した。
しかし撮影条件を変えていない状態で白化し、マーカーのコントラストが消失した。
従来版v21を同じ機体に戻すと109/109標本で左右を検出し、約3分後も検出した。

自動露光をSDK既定値に戻してもHB64では再発したため、独自AEC設定だけを
原因とはしない。モータ・制御・Webを初期化する前のカメラ単独診断でも再現。
保存できたHB64の112.5〜128.6秒では、画像の標本平均が228→234、最小が
203→229へ上昇する一方、露光は4行、ゲインは低い値まで下がっていた。
単純な露光時間の過大や足認識計算の誤りだけでは説明できない。
センサー内部のどの回路・補正が白化を起こすかは未特定。

同じ診断でセンサーをSDK設定へリセットしてHB128を試すと、130秒間、
取得時間中央値41.354ms、画像標本平均84〜107（初期過渡後は約85）、
最小3〜9程度を維持し、白化なし。露光480行、AECの明度レジスタ37〜38で安定した。
HB64の序盤はUSB再列挙で受信が切れ、112.5秒以降の9標本だけを保存した。
HB128は65標本を保存。診断時の設定切替に伴う部分フレームはHALが破棄した。

## 最終設定

水平blankingを306→128に短縮。垂直blanking12、sample-hold delay42、
窓648×488、320×240画像、20MHz XCLK、反転設定・画素座標校正は維持する。
自動露光・ゲイン・anti-flicker・露光上限はSDK既定値を保持し、書き換えない。
v22の整数contrast再利用、全17位置探索、33.333ms最短worker周期、
予測器25ms間隔対応を維持する。診断専用の260秒待機は通常版から除去した。

レジスタの定義は[GC0308データシート](https://download.kamami.pl/p1197045-GC0308.pdf)
と[Linuxドライバ](https://github.com/torvalds/linux/blob/master/drivers/media/i2c/gc0308.c)を照合。
LinuxのHB64という単独値は、この機体の全設定での安定性を保証しない。
同ドライバのsample-hold値は80で、今回の42とは異なる。
行時間比による約41.1msは見積りで、実測は別に記録する。

## 検証と再現用ファイル

通常版をCOM7へ書込み・再起動した後、180秒間のHTTP標本化で
**976/976の異なる画像で左右を検出**。981応答・通信エラー0・起動ID一定、
ゼロ点確定、全応答で非動作/モータ指令0。最終プレビューも左右マーカーを
視認し、階調5〜254、平均83.33で白化なし。Webは113624 bytesを完全受信。
通常版で起動後約240秒以降も左右検出を確認した。

同じUSB接続機体・照明・姿勢での静止中央値（旧版は20秒109標本、新版180秒976標本）:

| 指標 | v21 | v22-fix1 |
|---|---:|---:|
| driver画像時刻→画像受取り | 50.22ms | 41.33ms（17.7%短縮） |
| driver画像時刻→認識・予測結果ready | 64.42ms | 53.29ms（17.3%短縮） |
| 足認識の計算 | 8.937ms | 6.172ms（30.9%短縮） |
| 瞬間的な画像更新fpsの標本中央値 | 19.84 | 24.16 |
| フレームカウンタ差分 / 経過時間 | 14.11Hz | 16.71Hz |

新版の画像受取りp95は42.97ms、最大49.72ms。ready p95は59.73ms、
最大79.84msで、常に中央値どおりではない。HTTPからの抜き取り標本であり、
全フレーム精度や歩行中の遅延ではない。撮影露光の物理時刻・動的角度精度は未検証。

### 追加確認で残った問題

ユーザーの「今までのところがうまくいくか」の依頼で、同じ起動のまま60秒間再確認した。
294応答・292の異なる画像はすべて左右検出、ゼロ確定、MEKF valid/fresh、制御heartbeat fresh。
カメラ取得失敗・左右検出失敗の累積カウンタは増えず、モータ指令0・run0・samples0を維持した。
画像取得41.35ms、認識ready53.43ms、認識計算6.017ms（いずれも標本中央値）で高速化は再現した。
Chrome実画面でも `walking-foot-speed-v22-fix1`、接続中、READY_TO_MEASURE、左右角度、
ゼロ点確定、約24fpsを確認。8°ボタンは有効だが押していない。

ただし、3秒timeoutの診断HTTP GETが1回timeoutし、再取得で復帰した。
294応答のうち7応答は `frame_stale`（判定時の画像年齢200ms超）だった。
最初の180秒記録を再集計すると、こちらにも981応答中27応答のstaleがある。
「両マーカーを検出できた」と「常に制御に使える新しい画像がある」は別であり、
安定性を完全に確認できたとはしない。旧v21も111応答中5応答がstaleだった。

画面のfpsは直前2画像の受取間隔の逆数であり、その標本中央値24.16fpsを持続更新率と
解釈しない。カウンタ差分を同一起動のIMU時刻差で割ると、最初の180秒は16.71Hz、
追加60秒は17.31Hz。旧v21比較は14.11Hzだった。間欠的な遅れを含む値を併記する。
Webを開いた待機状態での観測であり、運転中はWeb休止・previewコピー省略という違いがある。
この記録だけではカメラ更新待ち、task実行待ち、HTTP生成中のsnapshot経過の寄与を分離できず、
Web負荷を原因と断定しない。モータ禁止を維持し、歩行中の鮮度・追従・実RWLOG取得は未確認。

追加記録は `recheck_idle_60s.json`、`recheck_detailed_60s.summary.json`、
`recheck_original_180s.summary.json`、`recheck_v21.summary.json`。
`tools/probe_foot_idle.py` にstale理由・累積カウンタ差分・持続更新率を追加し、
保存済みrawの再集計、HTTP間隔/timeout指定も可能にした。元のraw/summaryは保持する。

さらに診断GETを応答後1秒間隔・timeout8秒へ変更し、Chromeは開いたまま60秒確認した。
56/56応答が左右検出・制御開始鮮度Ready・MEKF valid/fresh、通信error0、
カメラ/検出失敗カウンタ増加0、同じboot・motor0。受取41.26ms/ready53.185ms、
カウンタ差分から22.07Hz（1313frames/59.499秒）。直前の頻回GET条件より改善したが、
観測間隔も異なるので「staleを完全に解消」とはしない。記録 `recheck_light_poll_60s.json`
と `.summary.json`。今後は追加診断通信の負荷を測定条件に含め、待ち時間の原因を切り分ける。

起動後1105.95秒の追加確認で、同じboot ID1572963981、修正版ID、モータ指令0、
制御コマンド投入0・run0・samples0・足Readyを確認した。実機Web113624bytes全体を受信し、
プレビュー19200bytesの各chunk/full CRCも一致。目視で左右の白マーカーを確認、
画素min2/max254/mean81.01で白化なし。`recheck_preview.json` / `.png`、
`recheck_web_preview.json`、`recheck_final_status.json` に保存。新たな書込みは行っていない。

最終build SUCCESS、RAM127740/Flash1928729 bytes。
firmware1929088 bytes、SHA256
`fd1c1fcb76614d6eb1543b8ac74fa1ef361d8d0277fe67a4314aed910aa28bf0`。
ROM書込みハッシュ検証成功。配信8ファイルHTTP200/全バイト一致、旧216ファイル不変。
最大metadata fixture986692 bytes、足1280/energy192/support96を保存し1MiB以内。
以下の回帰テストと、ELFの独自HAL・camera task wrapperリンク検証に成功した。

- `tools/test_foot_speed_profile.py`: 即時反映、50ms遅延、永久不一致、各通信故障、
  未知profile拒否、AEC/ゲイン/露光レジスタを変更しないこと、38ms予測入力を検証。
- `tools/test_foot_fast_pipeline.py`: 画像所有権・早期返却・ゼロ校正・無効化・予測・容量。
- `tools/test_foot_fast_metadata.py`: v19〜v22/fix1の最大ログ生成、CRC、変換、容量。
- `tools/test_foot_fast_downloadfix_flasher.py`: 5版のIDと配信分離。
- `tools/probe_foot_idle.py`: HTTP GETのみで静止状態の最新フレームを標本化。
  全フレームのログでも、歩行中の制御反映遅延の測定でもない。
- `tools/analyze_camera_boot_audit.py`: カメラ単独診断のレジスタと明度を集計。

生データ・途中版・USBログ・プレビューはローカル
`analysis/2026-11-16-foot-speed-fix/` に保存し、GitHubへ送信しない。
`diagnostic_registers.json` は起動失敗の直接証拠、`idle_preview.png` は白化画像、
`v21_idle_B.json` は従来版、`boot_audit_usb_live.txt` と `boot_audit_summary.json`
はカメラ単独比較。`final_idle_180s.json` は不採用のHB64/SDK-AEC中間版であり、
ファイル名のfinalを完成版の意味として扱わない。完成版の記録はHB128を明記する。
完成版は `hb128_final_idle_180s.json` / `.summary.json`、`hb128_comparison.json`、
`hb128_final_status.json`、`hb128_final_preview.png`、`hb128_firmware.json`、
`installer_hb128_verified.json`。各試行はモータを動かさず、歩行測定ログは作っていない。

書込みはesptool4.5.1のROM経由でapp0（0x10000）のみを更新。
通常stubは応答しなかった。`ENTER DOWNLOAD`のRTCビットは読取りで1を確認し、
`write_mem 0x6000812c 0 1`でそのビットだけを解除。NVS・パーティションは保持。
ユーザーによる手動操作以外の動的追従・暗所・別機体・歩行中の評価は別途必要。
