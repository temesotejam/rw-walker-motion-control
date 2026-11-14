# v16-webfix2：診断履歴をPSRAMへ移す

2026-11-14、v16-webfix1でも操作画面の読み込みが終わらないという録画 `レコーディング 2026-11-14 123809.mp4` を確認した。約131秒の録画は初期文言の操作画面で始まり、60秒・130秒ではChromeの `ERR_CONNECTION_TIMED_OUT` が見える。前回の「HTMLを一回全長取得できた」という結果だけでは、継続使用の復旧確認になっていなかった。

その後のUSB診断では、同じ起動のまま `energy-pulse-lifecycle-v16-webfix1` がREADYで動作し、制御・カメラ・HTTP heartbeatは継続、AP接続1台・Wi-Fiイベント数4も変わらなかった。それでもPCのHTTP/ARPは到達不能になった。内部RAMは空き8992バイト、最小2004バイト、最大連続3060バイト、PSRAMは約1.70MB空いていた。再起動やアプリのHTTPループ停止では説明できず、通信に必要な内部RAMの余裕不足を有力な要因として対策する。単独原因と確定するには更新後の実機比較が必要である。

`ExperimentRunner` の192件分の `WalkingEnergyAudit` 配列は39936バイトを内部RAMに静的確保していた。専用ビルド `WALKING_AUDIT_PSRAM=1` では、これを起動時の `ps_malloc` で確保する。動作中の記録保存・終了更新にメモリ再確保はない。実入力の完了量・残留電流・pendingを更新する独立したactiveコピーは内部RAMに置いたままにする。

PSRAM確保に失敗した場合は履歴の保存件数を0に保ち、完了処理は独立activeコピーで継続する。metadataの `archive_storage`、`archive_available`、`archive_bytes` で欠損を識別する。保存容量192件や完了Q/Wの定義、歩行のエネルギー選択、MEKF、入力電流・幅の制約は変えない。記録用配列の配置を変える修正であり、歩行改善や実機の制御期限維持を検証済みとするものではない。

検証は `python tools/test_walking_audit_psram.py`。実v16の採用→保存→開始→停止→完了経路をPSRAM型でコンパイルし、確保成功・確保失敗・192件満杯・二重完了・pending/残留の整合、動作中に再確保しないことを確認する。実metadata serializerでも保存成功と確保失敗/空を検証する。従来v16の連結テストとmetadata/CSV回帰もPASS。

専用環境は `atoms3cam-energy-pulse-lifecycle-webfix2`、識別は `energy-pulse-lifecycle-v16-webfix2`。PlatformIO build SUCCESSで静的RAMは166780→126844バイト（39936バイト減）、Flashは1872921/3342336バイト。firmwareは1873280バイト、SHA256 `9e6d35d4ea9f7170f75b978b0bbbcc44427afc731952e844c1ee121a0ba485c9`。

`python tools/prepare_energy_pulse_lifecycle_webfix_flasher.py --psram` で `http://127.0.0.1:8767/energy-pulse-lifecycle-webfix2/` を準備する。page/manifest/4partsはHTTP200・ローカル全バイト一致を確認した。元v16/webfix1の配信物は保持する。実機では書込み後の識別・空き内部RAM・繰返しHTML/状態取得・接続維持を別途確認する。

## 実機の書込みと継続通信確認

同日、ユーザーがこちらでの書込みと通信確認を承認した。COM24（ESP32-S3 rev0.2、MAC末尾6a:d4）のREADYをUSBで確認してから、既存の `ENTER DOWNLOAD` を使用した。ビルドで使った `.platformio/packages/tool-esptoolpy@1.40501.0/esptool.py`（4.5.1）で `--no-stub write_flash -z`、DIO/80MHz/8MB、app0の0x10000だけを書いた。NVS・パーティション・ブートローダーは書き換えていない。1873280バイトの書込みとハッシュ検証が成功した。

書込み後、ソフトdownload用のRTC `FORCE_DOWNLOAD_BOOT` が残っていた。SDKの `RTC_CNTL_OPTION1_REG=0x6000812c`、bit0を確認し、ROMの `read_mem` で値1を確認した後、`write_mem 0x6000812c 0x0 0x1` でその一時ビットだけを消して再起動した。flash/NVS/eFuseの消去ではない。この経路で再び書き込む場合も、ROMに残っていないかを起動識別で確認する必要がある。最初のツール選択は別版4.9系の依存不足で接続前に失敗しており、実際の書込みは上記4.5.1だけで行った。

USB/HTTPの新識別を確認し、起動直後の内部RAM空き61728バイト・最小水位61340バイトを観測した。書込み用リセット後の理由はTASK_WDTと記録されたが、下記試験中に起動IDの変化はなかった。

181.5秒、ブラウザに近いHTTPヘッダーで229要求（HTML36回・状態193回）を実行し、全て成功した。HTMLは毎回95778バイト、SHA256 `bdf5416b9df799dd05c413cbcdade8a8a0ebf362fe49f8b70e9a756ac496ae01` と一致した。最後の3接続同時取得も成功、最大応答時間は1.093秒、HTTPのboot_idは4216065150で一定、状態はREADY・制御状態fresh・samples0を維持した。

USB診断180サンプルでは、観測した内部RAM空きの最小42824バイト、起動以来の最小水位38632バイト、最大連続領域の最小30708バイト。試験終了時は空き57492バイトだった。診断は試験後OFFへ戻した。ユーザーもCtrl+F5後に「状態・数値が表示された」と確認した。今回の範囲では通信と表示が復帰したが、歩行や長時間の制御期限を検証した試験ではない。

ローカル検証記録は `analysis/2026-11-14-web/flash_webfix2/summary_180s.json`、`http_180s.jsonl`、`usb_180s.txt`、`status_after.json`、書込みログ一式。生ログはGitHubへ送っていない。
