# v16-webfix2：診断履歴をPSRAMへ移す

2026-11-14、v16-webfix1でも操作画面の読み込みが終わらないという録画 `レコーディング 2026-11-14 123809.mp4` を確認した。約131秒の録画は初期文言の操作画面で始まり、60秒・130秒ではChromeの `ERR_CONNECTION_TIMED_OUT` が見える。前回の「HTMLを一回全長取得できた」という結果だけでは、継続使用の復旧確認になっていなかった。

その後のUSB診断では、同じ起動のまま `energy-pulse-lifecycle-v16-webfix1` がREADYで動作し、制御・カメラ・HTTP heartbeatは継続、AP接続1台・Wi-Fiイベント数4も変わらなかった。それでもPCのHTTP/ARPは到達不能になった。内部RAMは空き8992バイト、最小2004バイト、最大連続3060バイト、PSRAMは約1.70MB空いていた。再起動やアプリのHTTPループ停止では説明できず、通信に必要な内部RAMの余裕不足を有力な要因として対策する。単独原因と確定するには更新後の実機比較が必要である。

`ExperimentRunner` の192件分の `WalkingEnergyAudit` 配列は39936バイトを内部RAMに静的確保していた。専用ビルド `WALKING_AUDIT_PSRAM=1` では、これを起動時の `ps_malloc` で確保する。動作中の記録保存・終了更新にメモリ再確保はない。実入力の完了量・残留電流・pendingを更新する独立したactiveコピーは内部RAMに置いたままにする。

PSRAM確保に失敗した場合は履歴の保存件数を0に保ち、完了処理は独立activeコピーで継続する。metadataの `archive_storage`、`archive_available`、`archive_bytes` で欠損を識別する。保存容量192件や完了Q/Wの定義、歩行のエネルギー選択、MEKF、入力電流・幅の制約は変えない。記録用配列の配置を変える修正であり、歩行改善や実機の制御期限維持を検証済みとするものではない。

検証は `python tools/test_walking_audit_psram.py`。実v16の採用→保存→開始→停止→完了経路をPSRAM型でコンパイルし、確保成功・確保失敗・192件満杯・二重完了・pending/残留の整合、動作中に再確保しないことを確認する。実metadata serializerでも保存成功と確保失敗/空を検証する。従来v16の連結テストとmetadata/CSV回帰もPASS。

専用環境は `atoms3cam-energy-pulse-lifecycle-webfix2`、識別は `energy-pulse-lifecycle-v16-webfix2`。PlatformIO build SUCCESSで静的RAMは166780→126844バイト（39936バイト減）、Flashは1872921/3342336バイト。firmwareは1873280バイト、SHA256 `9e6d35d4ea9f7170f75b978b0bbbcc44427afc731952e844c1ee121a0ba485c9`。

`python tools/prepare_energy_pulse_lifecycle_webfix_flasher.py --psram` で `http://127.0.0.1:8767/energy-pulse-lifecycle-webfix2/` を準備する。page/manifest/4partsはHTTP200・ローカル全バイト一致を確認した。元v16/webfix1の配信物は保持する。実機では書込み後の識別・空き内部RAM・繰返しHTML/状態取得・接続維持を別途確認する。
