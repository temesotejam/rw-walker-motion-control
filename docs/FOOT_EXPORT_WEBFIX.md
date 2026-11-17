# v26 webfix1：ログ取得改善版

対象環境は `atoms3cam-foot-export-webfix`、識別名は `walking-foot-native-stream-v26-webfix1`。ユーザーがブラウザで書き込むための準備版であり、こちらから実機への書込み・起動・モータ操作は行わない。

書き込みページ: <http://127.0.0.1:8767/foot-export-webfix/>。

バッテリー給電を切り離した後にUSBでPCへ接続し、いつもの書き込みボタンでポートを選ぶ。USB接続中は測定を開始しない。書き込み後、表示される識別名と左右の足角度を確認する。歩行試験はUSBを外しバッテリー給電へ切り替えてユーザーが行う。

## 変更

- CompactJsonTableは、行ごとに列名・値・保存用列名のStringを作る方式を廃止。列名を出力済みPSRAM内の位置で照合し、値を直接出力する。出力の長さが増えなかった場合は失敗状態を保持し、不完全な表を成功として返さない。
- 足フレーム・予測の行作成は、8 KiBのPSRAM作業バッファを1回確保して再利用する。容量超過・確保失敗ではログ生成を失敗扱いにする。小さい数値文字列などの一時Stringはまだ残るため、全処理が無確保になったという意味ではない。
- 状態JSONは16 KiBの固定PSRAMバッファをHTTP所有タスクが再利用する。最終出力の再確保をなくし、バッファ未確保・容量超過時は503を返す。
- カメラ停止時のキュー清掃は、空キューでDMAリセットするcam_take(0)ではなく、キューの取り出しとフレーム返却だけを行う。v21以降の組込みHALで有効。受信中の認識処理やセンサー設定は変更しない。
- manifestにその時点の内部RAM空き・最小空き・最大連続領域を追加。ブラウザ診断に最後の状態受信時刻と経過msを追加し、キャッシュされたonline表示と現在の疎通を区別できるようにする。

歩行のv19エネルギー制御、補助入力の回数・上限、センサーのnative設定、Core0/priority1は維持する。先に確認した「同じ半周期で再補助できない」制約は、この通信修正版では変更していない。

## 検証

- 文字列追加を各箇所で1回だけ失敗させても、append/finishが成功しないことを確認。列の欠落・順序変更・不正終端も拒否。
- 実際の最大件数serializerでv25/v25 native/v26 webfix1を検証。足1280件・energy192件・support96件保持。v26 webfix1のメタデータ1001430 bytes、1 MiB以内。
- 同じImmutableExportでclear/buildを3回反復。CRCと全体長が一致し、新tokenだけを受け付け、古いtokenを拒否することを確認。ホスト試験であり実機無線やヒープ断片化の再現ではない。
- 実HALのキュー清掃をnull/空/複数フレーム/反復で検証。DMA操作なし。既存native HALの画像長・バッファ所有権・受信待ち試験もPASS。
- ブラウザ診断の鮮度、ログ再試行・再開・CRC/token拒否、HTTP休止/復帰、送信上限の試験PASS。書き込みページ10構成の識別名・配置・旧版保持試験PASS。
- Webソースから `tools/embed_runtime_web.py` で埋込みHTMLを更新し、`--check`で一致確認。ビルド済みバイナリにも新しい診断キーが含まれることを配信前に検査する。

実機での再発防止・足認識遅延・連続歩行は未検証。診断(7)の通信不通の直接原因を特定済みとは扱わない。

最終ビルドSUCCESS（39.425秒）、静的RAM131484 bytes、app1947392 bytes。SHA256は `1f1df6ed50d296d1c1f3ec98c004a96d49f45b31e075d66df95d30ca9f8df315`。localhostからページ・manifest・全4パーツを取得しビルドとの一致を確認した。最初のビルドで埋込みWeb診断の更新漏れを検出し、HTML再生成・再ビルド後のこのハッシュを最終成果物とする。旧v26のSHA256 `7fc287d9d290ca8767cecb4d54a24306ca285c64d1f9a6abdbb054668fb90ed6` も保持を確認。

## 再生成と配信

1. `python tools/embed_runtime_web.py`
2. `python -m platformio run -e atoms3cam-foot-export-webfix`
3. `python tools/prepare_foot_export_webfix_flasher.py`
4. flasherをルートに、`python -m http.server 8767 --bind 127.0.0.1`

配信はローカルPC限定。旧 `flasher/foot-native-stream/` は保持する。ビルド・配信照合結果はlocal `analysis/2026-11-17-log-recovery/webfix_*`。バイナリや実験生データはGitへ追加しない。
