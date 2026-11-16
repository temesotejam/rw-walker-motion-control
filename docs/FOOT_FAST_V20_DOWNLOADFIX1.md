# v20：ログ取得のabort調査とdownloadfix1

## 機体の区別

2026-11-16、ユーザーが40505704.rwlogは試しに作った別機体で同じプログラムを実行した記録と訂正した。続く診断JSON (3)/(4)は同じ機体での2回の記録と説明された。40505704の起動停止・足角度の小変化を、今回の機体の挙動へ一般化しない。旧試行と40505704の速度比較も機体を揃えた検証ではない。前報告と引継ぎに訂正を追加した。

## 診断JSONで確認できたこと

|項目|freefoot-diagnostics (3).json|freefoot-diagnostics (4).json|
|---|---:|---:|
|実行ID|walking-foot-fast-v20|walking-foot-fast-v20|
|状態・経過|FINISHED・30000 ms|FINISHED・30000 ms|
|samples|10127|10071|
|export_phase|building|building|
|last_error|空|空|
|最後のfps表示|20.0525|20.1572|
|最後のframe→ready|59.26 ms|57.84 ms|
|足archive件数／overflow|783／false|785／false|

原本SHA256は(3) `ecb098974e472c4ebd7709d04f0c9784eaa7c08d3ff43ace0daa86b775a69754`、(4) `2885c46049fe6e461c06de7260c8738eeb4be50e4ccc0ba8b4a73b7f1029f178`。原本は変更していない。

2件とも測定は終了しており、診断採取時にはログのメタデータ／CRCを確定する処理が進行中だった。各数値は末尾snapshotで、測定中の中央値や全期間の認識精度ではない。右/左のmarker failureカウンタはbeginRunでresetしない起動後累計で、archive件数を分母に試行失敗率を計算しない。

camera.receiver_active=false／xclk_active=false、画像が約5〜6秒古い状態は、FootObserverがFINISHED/ESTOP後にカメラを止める仕様と整合する。これを測定中の認識停止と断定しない。右足は生のin_range=falseでも許容外挿約0.20°、right_control_range_ok=trueであり、今回の保存失敗を範囲拒否で説明しない。memory.sampled=falseの0は未測定で、メモリ枯渇の証拠ではない。

診断JSONに歩行時系列・接地・前進量は含まれず、今回の歩行の成否や停止原因は確定できない。

## ブラウザ側の問題と未確定事項

旧 `web/runtime.js` の `request()` は、応答ヘッダと本文の読取りを含め3秒後に理由を指定せずAbortControllerをabortする。エンジンの英語メッセージをそのまま表示していた。chunkは再試行するが、`POST /export/prepare` と `GET /export/manifest` は1回の失敗でdownload全体を終了する構造だった。

したがって、一時的な応答遅延で「signal is aborted without reason」となり、ログ準備が本体で継続していても画面は取得を中断する問題がある。実コードのヘッダ前・本文読取中abortを注入して再現した。ただし今回の診断にはブラウザの失敗先・段階がなく、実際にprepare/manifest/chunkのどれが失敗したか、なぜ3秒を超えたかは特定できない。メタデータ作成時間・無線・HTTP待ちのどれかを根本原因と断定しない。「保存済みの部分は保持」の文言だけでは、何bytes保存されたかも分からない。

実機GET確認はproxyなしでstatus0.062秒、manifest0.031秒。現在接続先はboot_id2265249153／READY_TO_MEASURE／run0／samples0／export emptyで、診断のboot_id2230865637・2229272991とは異なる。現在の接続先から当時のRWLOGを回収できなかった。prepare/start/clear/reboot/flashは実行していない。

## 修正

- request自身のタイムアウトを通信先・秒数付き日本語の `RequestTimeoutError` にする。外からの一時停止と区別する。
- prepare/manifest/chunkとも1要求8秒・最大5回。4xxは408/429以外再試行せず、ログ識別不一致を黙って継続しない。POST prepareは本体の既存idempotent仕様を使う。
- buildingを500ms間隔で確認し、成功応答が続いても120秒を超えて待ち続けない。上限はソフトウェア上の待ち予算で、実機確定時間の計測から導いた値ではない。
- 「一時停止」は進行中fetchも中止する。検証済みチャンクと既存の完成ファイルを失敗で上書きしない。同じtokenのcacheをCRC確認して再使用し、別tokenを混ぜない。
- 診断JSONに `browser_transfer` を追加。段階、失敗先、試行回数、最後のmanifest、今回確認したbytes、再利用bytes、最後の例外を保存する。機体状態のJSON項目は維持する。
- v20のraw足frameに保存した全control_observed_usを、ブラウザCSVのsparse64件補助表の展開時に消していた点も修正。旧版の補助表だけのファイルも扱う。

認識探索・周期・カメラ構成・姿勢推定・エネルギー係数・入力幅・制御guardは変更していない。変更したC++実行内容は識別IDのみ。

## 検証と配信

実JSでprepare abort・manifest body abort→building→ready、失敗5回終了、8192bytesからの再開、旧完成データ保持、一時停止、export error、120秒待ち、chunk CRC不一致、409の即停止、新token、診断保存、v20全receiptの保持を確認した。旧download/runtime/preview/start/v20/v19 UI、v19実runnerのJ/幅/guard、新旧installerのID・flag・隔離もPASS。

現HTML112912bytesでproduction socket loopのRAM/flash・slow/partial/retry/rollover/idle/total/fatalを確認した。最初の成功fixtureは37B/5tickの速度で15秒予算を超えたため失敗した。成功ケースを約12kB/s以上のpeerへ修正し、遅い進捗の打切り・無進捗限界の試験は維持した。production transportの15秒total／3秒idleは変更していない。

環境 `atoms3cam-foot-fast-downloadfix`、ID `walking-foot-fast-v20-downloadfix1`。初回build244.458秒後、最終画面で依存再build SUCCESS35.551秒。RAM127604/327680、Flash1921001/3342336、firmware1921360bytes、SHA256 `617ba429ae118a595eb08cfdc0579c238ed5cbc5e628775c0753b9e800a1f587`。最終HTML全bytesをfirmware内で照合した。

専用ページ `http://127.0.0.1:8767/foot-fast-downloadfix/`。page/manifest/補助JS2件/4partsの8件すべてHTTP200・ローカル全bytes一致。ESP32-S3/offset0,32768,57344,65536。既存v20/v19/v18ページ・部品24件のSHAは保持した。生成は `python tools/prepare_foot_fast_downloadfix_flasher.py`。

診断2件のコピー・解析JSON・build/HTTP検証はローカル `analysis/2026-11-16-download-abort/` に保存しGitへ追加しない。今回は実機書込み・新測定を行っていない。実際の無線／大きなログ確定処理での取得成功は未確認で、次回RWLOG取得と新しいbrowser_transfer診断で確認する。
